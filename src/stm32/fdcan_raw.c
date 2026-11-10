// Raw CAN FD frame bridge on the STM32H5 FDCAN1 (Core One L AC Controller bus)
//
// The xBuddy-extension (H503) is the Cyphal master of the Core One L's CAN FD
// bus, on which the mains-bed "AC Controller" lives. This module only moves
// raw frames: the host (klippy/extras/ac_controller.py + cyphal_can.py) owns
// all of Cyphal (framing, CRC, node allocation, heartbeat, DSDL).
//
// Safety model: the master heartbeat and every bed target come from the host.
// If the host stops, no heartbeat is sent and the AC Controller times out. On
// MCU shutdown the controller is forced into INIT (off the bus) and the bus
// enable line is released, so a shut-down MCU can never keep the bus alive.
//
// Bus timing matches Prusa's xBuddy-extension firmware: CAN FD frames without
// bit-rate switching at 125 kbit/s, 87.5% sample point, from a 48 MHz kernel
// clock (PLL1Q).
//
// A 64-byte frame does not fit one Klipper message, so frames travel in
// chunks: bit 0 of "flags" = more chunks follow.
//
// Copyright (C) 2026
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h>       // memcpy
#include "autoconf.h"     // CONFIG_MACH_STM32H5
#include "basecmd.h"      // oid_alloc
#include "board/irq.h"    // irq_save
#include "board/misc.h"   // timer_read_time, timer_from_us, timer_is_before
#include "command.h"      // DECL_COMMAND, sendf
#include "generic/armcm_boot.h" // armcm_enable_irq
#include "gpio.h"         // gpio_out_setup
#include "internal.h"     // gpio_peripheral, GPIO_FUNCTION
#include "sched.h"        // DECL_TASK, DECL_SHUTDOWN

#if !CONFIG_MACH_STM32H5
#error "fdcan_raw.c targets the STM32H5 FDCAN1 only"
#endif

#define FDR_CHUNK   32          // bytes per host<->MCU message chunk
#define FDR_RX_RING 16          // frames (power of two)
#define FDR_TX_RING 8           // frames (power of two)
#define FDR_PACE_US 1000        // min spacing of forwarded rx frames

// FDCAN message RAM (fixed layout on G0/G4/H5: 72-byte elements)
struct fdcan_fifo {
    uint32_t id_section;
    uint32_t dlc_section;
    uint32_t data[64 / 4];
};
struct fdcan_msg_ram {
    uint32_t FLS[28];
    uint32_t FLE[16];
    struct fdcan_fifo RXF0[3];
    struct fdcan_fifo RXF1[3];
    uint32_t TEF[6];
    struct fdcan_fifo TXFIFO[3];
};
#define MSG_RAM (*(volatile struct fdcan_msg_ram *)SRAMCAN_BASE)

#define ID_XTD (1u << 30)
#define DLC_FDF (1u << 21)

struct fdr_frame {
    uint32_t id;
    uint8_t len;
    uint8_t data[64];
};

static struct fdr {
    struct gpio_out enable;
    uint8_t oid, configured, enable_on;
    // tx staging from chunked host commands
    uint8_t stage[64];
    uint8_t stage_len;
    struct fdr_frame tx[FDR_TX_RING];
    uint8_t tx_head, tx_tail;
    struct fdr_frame rx[FDR_RX_RING];
    volatile uint8_t rx_head;
    uint8_t rx_tail;
    uint32_t rx_overflow, tx_overflow, bad_cmd;
    // rx pacing: console_sendf drops on a full usb buffer, so frames are
    // forwarded at most one per FDR_PACE_US
    struct timer pace_timer;
    uint32_t next_send;
    uint8_t pace_active, rx_seq;
} fdr;

static struct task_wake fdr_wake;

static const uint8_t dlc_len[16] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64
};

static int
len_to_dlc(uint8_t len)
{
    int i;
    for (i = 0; i < 16; i++)
        if (dlc_len[i] == len)
            return i;
    return -1;
}

// Move queued tx frames into the hardware tx fifo
static void
fdr_tx_kick(void)
{
    while (fdr.tx_tail != fdr.tx_head) {
        uint32_t txfqs = FDCAN1->TXFQS;
        if (txfqs & FDCAN_TXFQS_TFQF)
            return;
        uint32_t idx = (txfqs & FDCAN_TXFQS_TFQPI) >> FDCAN_TXFQS_TFQPI_Pos;
        struct fdr_frame *f = &fdr.tx[fdr.tx_tail % FDR_TX_RING];
        volatile struct fdcan_fifo *hw = &MSG_RAM.TXFIFO[idx];
        hw->id_section = (f->id & 0x1fffffff) | ID_XTD;
        hw->dlc_section = ((uint32_t)len_to_dlc(f->len) << 16) | DLC_FDF;
        uint32_t words[16];
        memset(words, 0, sizeof(words));
        memcpy(words, f->data, f->len);
        int i;
        for (i = 0; i < (f->len + 3) / 4; i++)
            hw->data[i] = words[i];
        barrier();
        FDCAN1->TXBAR = 1u << idx;
        fdr.tx_tail++;
    }
}

void
FDCAN1_IRQHandler(void)
{
    uint32_t ir = FDCAN1->IR;
    if (ir & FDCAN_IR_RF0N) {
        FDCAN1->IR = FDCAN_IR_RF0N;
        for (;;) {
            uint32_t rxf0s = FDCAN1->RXF0S;
            if (!(rxf0s & FDCAN_RXF0S_F0FL))
                break;
            uint32_t idx = (rxf0s & FDCAN_RXF0S_F0GI) >> FDCAN_RXF0S_F0GI_Pos;
            volatile struct fdcan_fifo *hw = &MSG_RAM.RXF0[idx];
            uint32_t ids = hw->id_section;
            uint8_t len = dlc_len[(hw->dlc_section >> 16) & 0x0f];
            uint8_t head = fdr.rx_head;
            uint8_t full = (uint8_t)(head - fdr.rx_tail) >= FDR_RX_RING;
            if (full || !(ids & ID_XTD)) {
                if (ids & ID_XTD)
                    fdr.rx_overflow++;
            } else {
                struct fdr_frame *f = &fdr.rx[head % FDR_RX_RING];
                uint32_t words[16];
                int i;
                for (i = 0; i < (len + 3) / 4; i++)
                    words[i] = hw->data[i];
                f->id = ids & 0x1fffffff;
                f->len = len;
                memcpy(f->data, words, len);
                fdr.rx_head = head + 1;
            }
            barrier();
            FDCAN1->RXF0A = idx;
        }
        sched_wake_task(&fdr_wake);
    }
    if (ir & FDCAN_IR_TFE) {
        FDCAN1->IR = FDCAN_IR_TFE;
        sched_wake_task(&fdr_wake);
    }
    // Error/bus-off flags are polled via fdcan_raw_query; just ack here.
    FDCAN1->IR = ir & ~(FDCAN_IR_RF0N | FDCAN_IR_TFE);
}

static void
fdr_hw_init(uint32_t rx_pin, uint32_t tx_pin)
{
    // Kernel clock = PLL1Q (48 MHz, enabled by stm32h5.c clock setup)
    RCC->CCIPR5 = (RCC->CCIPR5 & ~RCC_CCIPR5_FDCANSEL_Msk)
                  | RCC_CCIPR5_FDCANSEL_0;
    RCC->APB1HENR |= RCC_APB1HENR_FDCANEN;
    (void)RCC->APB1HENR;

    gpio_peripheral(rx_pin, GPIO_FUNCTION(9), 1);
    gpio_peripheral(tx_pin, GPIO_FUNCTION(9), 0);

    FDCAN1->CCCR &= ~FDCAN_CCCR_CSR;
    while (FDCAN1->CCCR & FDCAN_CCCR_CSA)
        ;
    FDCAN1->CCCR |= FDCAN_CCCR_INIT;
    while (!(FDCAN1->CCCR & FDCAN_CCCR_INIT))
        ;
    FDCAN1->CCCR |= FDCAN_CCCR_CCE;
    FDCAN_CONFIG->CKDIV = 0;   // kernel clock / 1

    // CAN FD, no bit-rate switching, automatic retransmission on
    FDCAN1->CCCR = (FDCAN1->CCCR & ~(FDCAN_CCCR_BRSE | FDCAN_CCCR_DAR))
                   | FDCAN_CCCR_FDOE | FDCAN_CCCR_PXHD;

    // 48 MHz / 3 = 16 MHz tq; 1+111+16 = 128 tq -> 125 kbit/s, SP 87.5%
    FDCAN1->NBTP = ((16 - 1) << FDCAN_NBTP_NSJW_Pos)
                   | ((3 - 1) << FDCAN_NBTP_NBRP_Pos)
                   | ((111 - 1) << FDCAN_NBTP_NTSEG1_Pos)
                   | ((16 - 1) << FDCAN_NBTP_NTSEG2_Pos);
    // Data phase (unused without BRS, kept equal to Prusa's): 500 kbit/s
    FDCAN1->DBTP = ((4 - 1) << FDCAN_DBTP_DSJW_Pos)
                   | ((3 - 1) << FDCAN_DBTP_DBRP_Pos)
                   | ((27 - 1) << FDCAN_DBTP_DTSEG1_Pos)
                   | ((4 - 1) << FDCAN_DBTP_DTSEG2_Pos);

    // No filters: accept all extended frames into FIFO0, reject standard
    // and remote frames (Cyphal uses 29-bit data frames only).
    FDCAN1->RXGFC = (0 << FDCAN_RXGFC_ANFE_Pos) | (2 << FDCAN_RXGFC_ANFS_Pos)
                    | FDCAN_RXGFC_RRFE | FDCAN_RXGFC_RRFS;
    FDCAN1->TXBC = 0;           // tx fifo mode

    FDCAN1->CCCR &= ~FDCAN_CCCR_CCE;
    FDCAN1->CCCR &= ~FDCAN_CCCR_INIT;

    armcm_enable_irq(FDCAN1_IRQHandler, FDCAN1_IT0_IRQn, 2);
    FDCAN1->ILS = 0;            // all interrupt groups -> line 0
    FDCAN1->ILE = FDCAN_ILE_EINT0;
    FDCAN1->IE = FDCAN_IE_RF0NE | FDCAN_IE_TFEE;
}

void
command_config_fdcan_raw(uint32_t *args)
{
    if (fdr.configured)
        shutdown("fdcan_raw already configured");
    fdr.oid = args[0];
    fdr.enable_on = !args[4];
    fdr.enable = gpio_out_setup(args[3], !fdr.enable_on);
    fdr_hw_init(args[1], args[2]);
    gpio_out_write(fdr.enable, fdr.enable_on);
    fdr.configured = 1;
}
DECL_COMMAND(command_config_fdcan_raw,
             "config_fdcan_raw oid=%c rx_pin=%u tx_pin=%u"
             " enable_pin=%u enable_invert=%c");

void
command_fdcan_raw_tx(uint32_t *args)
{
    if (!fdr.configured || args[0] != fdr.oid)
        shutdown("fdcan_raw not configured");
    uint32_t id = args[1];
    uint8_t flags = args[2], len = args[3];
    uint8_t *data = command_decode_ptr(args[4]);
    if (fdr.stage_len + len > sizeof(fdr.stage)) {
        fdr.stage_len = 0;
        fdr.bad_cmd++;
        return;
    }
    memcpy(&fdr.stage[fdr.stage_len], data, len);
    fdr.stage_len += len;
    if (flags & 1)
        return;
    uint8_t flen = fdr.stage_len;
    fdr.stage_len = 0;
    if (len_to_dlc(flen) < 0) {
        fdr.bad_cmd++;
        return;
    }
    irqstatus_t flag = irq_save();
    if ((uint8_t)(fdr.tx_head - fdr.tx_tail) >= FDR_TX_RING) {
        fdr.tx_overflow++;
    } else {
        struct fdr_frame *f = &fdr.tx[fdr.tx_head % FDR_TX_RING];
        f->id = id;
        f->len = flen;
        memcpy(f->data, fdr.stage, flen);
        fdr.tx_head++;
        fdr_tx_kick();
    }
    irq_restore(flag);
}
DECL_COMMAND(command_fdcan_raw_tx,
             "fdcan_raw_tx oid=%c id=%u flags=%c data=%*s");

void
command_fdcan_raw_query(uint32_t *args)
{
    if (!fdr.configured || args[0] != fdr.oid)
        return;
    sendf("fdcan_raw_status oid=%c psr=%u ecr=%u rx_overflow=%u"
          " tx_overflow=%u bad_cmd=%u"
          , fdr.oid, FDCAN1->PSR, FDCAN1->ECR, fdr.rx_overflow, fdr.tx_overflow
          , fdr.bad_cmd);
}
DECL_COMMAND(command_fdcan_raw_query, "fdcan_raw_query oid=%c");

static uint_fast8_t
fdr_pace_event(struct timer *t)
{
    fdr.pace_active = 0;
    sched_wake_task(&fdr_wake);
    return SF_DONE;
}

static void
fdr_pace_arm(void)
{
    if (fdr.pace_active)
        return;
    fdr.pace_active = 1;
    fdr.pace_timer.func = fdr_pace_event;
    fdr.pace_timer.waketime = fdr.next_send;
    sched_add_timer(&fdr.pace_timer);
}

void
fdcan_raw_task(void)
{
    if (!sched_check_wake(&fdr_wake) || !fdr.configured)
        return;
    irqstatus_t flag = irq_save();
    fdr_tx_kick();
    irq_restore(flag);
    if (fdr.rx_tail == fdr.rx_head)
        return;
    uint32_t now = timer_read_time();
    if (timer_is_before(now, fdr.next_send)) {
        fdr_pace_arm();
        return;
    }
    struct fdr_frame *f = &fdr.rx[fdr.rx_tail % FDR_RX_RING];
    uint8_t off = 0, seq = fdr.rx_seq++;
    do {
        uint8_t n = f->len - off > FDR_CHUNK ? FDR_CHUNK : f->len - off;
        uint8_t more = off + n < f->len;
        sendf("fdcan_raw_rx oid=%c id=%u seq=%c flags=%c data=%*s"
              , fdr.oid, f->id, seq, more, n, &f->data[off]);
        off += n;
    } while (off < f->len);
    fdr.rx_tail++;
    fdr.next_send = now + timer_from_us(FDR_PACE_US);
    if (fdr.rx_tail != fdr.rx_head)
        fdr_pace_arm();
}
DECL_TASK(fdcan_raw_task);

void
fdcan_raw_shutdown(void)
{
    if (!fdr.configured)
        return;
    // Drop off the bus: no frame (heartbeat, bed target) can leave a
    // shut-down MCU.
    FDCAN1->CCCR |= FDCAN_CCCR_INIT;
    gpio_out_write(fdr.enable, !fdr.enable_on);
    fdr.tx_head = fdr.tx_tail = 0;
    fdr.stage_len = 0;
    fdr.pace_active = 0;
}
DECL_SHUTDOWN(fdcan_raw_shutdown);

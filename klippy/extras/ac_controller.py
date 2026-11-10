# Prusa CORE One L "AC Controller" (mains heated bed) over Cyphal/CAN FD
#
# On the CORE One L the bed is not wired to the xBuddy: it is mains-powered
# and regulated by a separate AC Controller board on a CAN FD bus owned by
# the xBuddy-extension (H503). The H503 runs src/stm32/fdcan_raw.c, a raw
# frame bridge; this module is the Cyphal bus master (node 0):
#
#   * answers plug-and-play node-ID allocation (uavcan.pnp 2.0)
#   * publishes the master heartbeat (1 Hz) while klippy is alive
#   * identifies the AC Controller (GetInfo), starts its app if the
#     bootloader stopped (ExecuteCommand start_app)
#   * sends prusa3d.ac_controller.Config (bed target, fans) and LED config
#   * decodes prusa3d.ac_controller.Status (temps, power, faults)
#   * exposes the bed as a regular Klipper heater ("heater_bed", gcode B)
#
# The AC Controller regulates the bed itself (it has its own thermal model
# and runaway protection), so the heater only forwards the target. Klipper
# adds: fault, status timeout and out-of-range temperature -> shutdown.
# On any shutdown the H503 drops off the bus, so heartbeat and targets stop.
# If the host itself dies (no shutdown is ever sent), the H503's watchdog
# takes the bus down after host_timeout seconds without a frame to send.
#
# Protocol reference: Prusa-Firmware-Buddy src/puppy/xbuddy_extension/
# (cyphal_*.cpp) and src/can/data_types/ (DSDL).
#
# Copyright (C) 2026
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math, threading
from . import cyphal_can as cy

MASTER_NODE_ID = 0

# Port IDs (uavcan fixed IDs and prusa3d.common.PortIds.0.1)
PORT_HEARTBEAT = 7509
PORT_PNP_V2 = 8165
PORT_RECORD = 8184
SRV_GET_INFO = 430
SRV_EXECUTE_COMMAND = 435
SRV_AC_SET_CONFIG = 21
MSG_AC_STATUS = 600
MSG_AC_LEDS = 601

AC_NODE_NAME = b"cz.prusa3d.honeybee.ac_controller"

HEALTH_NOMINAL, HEALTH_ADVISORY, HEALTH_CAUTION, HEALTH_WARNING = range(4)
MODE_OPERATIONAL, MODE_INITIALIZATION, MODE_MAINTENANCE, \
    MODE_SOFTWARE_UPDATE = range(4)
MODE_NAMES = {0: 'operational', 1: 'initialization', 2: 'maintenance',
              3: 'software_update'}

CMD_START_APP = 0           # prusa3d.common.CustomExecuteCommand

HEARTBEAT_PERIOD = 1.
HEARTBEAT_OFFLINE = 3.

# prusa3d.ac_controller.Status.1.0 FAULT_* and SharedFault bits
FAULT_NAMES = {
    0: 'RCD_TRIPPED', 1: 'POWERPANIC', 2: 'OVERHEAT', 3: 'PSU_FAN_NOK',
    4: 'PSU_NTC_DISCONNECT', 5: 'PSU_NTC_SHORT', 6: 'BED_NTC_DISCONNECT',
    7: 'BED_NTC_SHORT', 8: 'TRIAC_NTC_DISCONNECT', 9: 'TRIAC_NTC_SHORT',
    10: 'BED_FAN0_NOK', 11: 'BED_FAN1_NOK', 12: 'TRIAC_FAN_NOK',
    13: 'GRID_NOK', 14: 'BED_LOAD_NOK', 15: 'CHAMBER_LOAD_NOK',
    16: 'PSU_NOK', 17: 'BED_RUNAWAY', 26: 'CAN', 27: 'MCU_OVERHEAT',
    28: 'PCB_OVERHEAT', 29: 'DATA_TIMEOUT', 30: 'HEARTBEAT_MISSING',
    31: 'UNKNOWN',
}

def fault_names(mask):
    return [FAULT_NAMES.get(b, 'BIT%d' % (b,))
            for b in range(32) if mask & (1 << b)]

######################################################################
# DSDL codecs
######################################################################

def encode_heartbeat(uptime, health=HEALTH_NOMINAL, mode=MODE_OPERATIONAL,
                     vssc=0):
    w = cy.BitWriter()
    w.uint(uptime, 32)
    w.align().uint(health, 2)
    w.align().uint(mode, 3)
    w.align().uint(vssc, 8)
    return w.bytes()

def decode_heartbeat(data):
    r = cy.BitReader(data)
    uptime = r.uint(32)
    health = r.align().uint(2)
    mode = r.align().uint(3)
    vssc = r.align().uint(8)
    return {'uptime': uptime, 'health': health, 'mode': mode, 'vssc': vssc}

def encode_pnp(node_id, unique_id):
    w = cy.BitWriter()
    w.uint(node_id, 16)
    w.raw(unique_id)
    return w.bytes()

def decode_pnp(data):
    r = cy.BitReader(data)
    node_id = r.uint(16)
    return node_id, r.raw(16)

def decode_get_info_response(data):
    r = cy.BitReader(data)
    versions = [r.uint(8) for _ in range(6)]
    r.uint(64)
    unique_id = r.raw(16)
    name = r.raw(min(r.uint(8), 50))
    return {'protocol': versions[0:2], 'hardware': versions[2:4],
            'software': versions[4:6], 'unique_id': unique_id, 'name': name}

def encode_execute_command(command, parameter=b''):
    w = cy.BitWriter()
    w.uint(command, 16)
    w.uint(len(parameter), 8)
    w.raw(parameter)
    return w.bytes()

def decode_execute_command_response(data):
    r = cy.BitReader(data)
    return r.uint(8)

def decode_record(data):
    r = cy.BitReader(data)
    r.uint(56)                  # uavcan.time.SynchronizedTimestamp
    severity = r.align().uint(3)
    text = r.align().raw(r.uint(8))
    return severity, text.decode('utf-8', 'replace')

def encode_record(text, severity=2):
    """uavcan.diagnostic.Record.1.1 (timestamp unknown)"""
    text = text.encode('utf-8')[:255]
    w = cy.BitWriter().uint(0, 56).uint(severity, 8).uint(len(text), 8)
    return w.raw(text).bytes()

def _w_target_temp(w, temp):
    w.align()
    if temp is None:
        w.uint(0, 8)
    else:
        w.uint(1, 8).float32(temp)

def _r_target_temp(r):
    r.align()
    tag = r.uint(8)
    return r.float32() if tag == 1 else None

def _w_fan_config(w, pwm):
    # FanConfiguration union: 0 automatic, 1 pwm [0..1], 2 rpm
    w.align()
    if pwm is None:
        w.uint(0, 8)
    else:
        w.uint(1, 8).float32(pwm)

def _r_fan_state(r):
    r.align()
    return {'pwm': r.float32(), 'rpm': r.float32(), 'status': r.int(8)}

def _r_f32(r):
    r.align()
    return r.float32()

def encode_config(bed_target=None, bed_fan_pwm=None, psu_fan_pwm=None,
                  chamber_target=None):
    """prusa3d.ac_controller.Config.1.0 request (stock Buddy defaults for
    everything Klipper does not drive)"""
    w = cy.BitWriter()
    _w_target_temp(w, bed_target)
    _w_target_temp(w, chamber_target)
    _w_fan_config(w, None)              # chamber_fan_config: automatic
    w.align().float32(0.)               # servo_config (Buddy sends 0)
    w.align().float32(0.)               # intake_vent (Buddy sends 0)
    _w_fan_config(w, psu_fan_pwm)
    _w_fan_config(w, None)              # triac_fan_config: automatic
    _w_fan_config(w, bed_fan_pwm)       # external (bed) fans
    w.bool(False)                       # splitter_enabled
    return w.bytes()

def _r_led_config(r):
    r.align()
    tag = r.uint(8)
    out = {'mode': {0: 'off', 1: 'color', 2: 'progress', 3: 'kitt',
                    4: 'follow_head', 5: 'error'}.get(tag, 'unknown')}
    if tag:
        out['rgbw'] = [r.uint(8) for _ in range(4)]
        if tag == 2:
            out['progress'] = r.float32()
    return out

def decode_status(data):
    """prusa3d.ac_controller.Status.1.0"""
    r = cy.BitReader(data)
    s = {}
    s['bed_target'] = _r_target_temp(r)
    s['bed_temp'] = _r_f32(r)
    s['bed_pwm'] = _r_f32(r)
    s['chamber_target'] = _r_target_temp(r)
    s['chamber_pwm'] = _r_f32(r)
    s['psu_fan'] = _r_fan_state(r)
    s['psu_temp'] = _r_f32(r)
    s['triac_fan'] = _r_fan_state(r)
    s['triac_temp'] = _r_f32(r)
    s['bed_fan0'] = _r_fan_state(r)
    s['bed_fan1'] = _r_fan_state(r)
    s['board_temp'] = _r_f32(r)
    s['mcu_temp'] = _r_f32(r)
    s['supply_voltage'] = _r_f32(r)
    s['leds'] = _r_led_config(r)
    r.align()
    s['faults'] = r.uint(32)
    s['time_sync_precise'] = r.bool()
    s['powerpanic_pin'] = r.bool()
    s['chamber_can_heat'] = r.bool()
    s['ac_voltage'] = _r_f32(r)
    s['ac_frequency'] = _r_f32(r)
    s['bed_power'] = _r_f32(r)
    s['chamber_power'] = _r_f32(r)
    s['psu_power'] = _r_f32(r)
    return s

def encode_leds(mode, rgbw=(0, 0, 0, 0), progress=0.):
    """prusa3d.common.leds.Config.1.0: mode 'off'|'color'|'progress'"""
    w = cy.BitWriter()
    if mode == 'off':
        w.uint(0, 8)
    elif mode == 'color':
        w.uint(1, 8)
        for c in rgbw:
            w.uint(c, 8)
    else:
        w.uint(2, 8)
        for c in rgbw:
            w.uint(c, 8)
        w.float32(progress)
    return w.bytes()

######################################################################
# Remote heater (the bed)
######################################################################

class RemoteHeater:
    """Klipper heater whose regulation runs on the AC Controller"""
    def __init__(self, acc, config, name):
        self.acc = acc
        self.printer = config.get_printer()
        self.name = name
        self.short_name = name.split()[-1]
        self.min_temp = config.getfloat('min_temp', 0.)
        self.max_temp = config.getfloat('max_temp', 120., above=self.min_temp)
        self.tolerance = config.getfloat('target_tolerance', 1.5, above=0.)
        self.lock = threading.Lock()
        self.target_temp = 0.
        self.target_since = 0.
        self.temp = 0.
        self.power = 0.
        self.last_temp_time = 0.
    def get_name(self):
        return self.name
    def get_pwm_delay(self):
        return 0.
    def get_max_power(self):
        return 1.
    def get_smooth_time(self):
        return 1.
    def set_temp(self, degrees):
        if degrees and (degrees < self.min_temp or degrees > self.max_temp):
            raise self.printer.command_error(
                "Requested temperature (%.1f) out of range (%.1f:%.1f)"
                % (degrees, self.min_temp, self.max_temp))
        now = self.printer.get_reactor().monotonic()
        with self.lock:
            if degrees and now - self.last_temp_time > self.acc.status_timeout:
                raise self.printer.command_error(
                    "%s: no valid temperature from the AC Controller"
                    % (self.name,))
            if degrees and not self.target_temp:
                self.target_since = now
            self.target_temp = degrees
        self.acc.request_config_update()
    def alter_target(self, target_temp):
        if target_temp:
            target_temp = max(self.min_temp, min(self.max_temp, target_temp))
        self.set_temp(target_temp)
    def set_control(self, control):
        raise self.printer.command_error(
            "%s is regulated by the AC Controller" % (self.name,))
    def get_temp(self, eventtime):
        with self.lock:
            if eventtime - self.last_temp_time > self.acc.status_timeout:
                return 0., self.target_temp
            return self.temp, self.target_temp
    def check_busy(self, eventtime):
        with self.lock:
            if not self.target_temp:
                return False
            return abs(self.target_temp - self.temp) > self.tolerance
    def update(self, eventtime, temp, power):
        with self.lock:
            self.temp = temp
            self.power = power
            self.last_temp_time = eventtime
    def get_target(self):
        with self.lock:
            return self.target_temp
    def stats(self, eventtime):
        with self.lock:
            target, temp, power = self.target_temp, self.temp, self.power
        return (bool(target) or temp > 50.,
                '%s: target=%.0f temp=%.1f pwm=%.3f' % (
                    self.short_name, target, temp, power))
    def get_status(self, eventtime):
        with self.lock:
            return {'temperature': round(self.temp, 2),
                    'target': self.target_temp, 'power': self.power}
    cmd_SET_HEATER_TEMPERATURE_help = "Sets a heater temperature"
    def cmd_SET_HEATER_TEMPERATURE(self, gcmd):
        temp = gcmd.get_float('TARGET', 0.)
        pheaters = self.printer.lookup_object('heaters')
        pheaters.set_temperature(self, temp)

######################################################################
# Cyphal master / AC Controller node
######################################################################

class ACController:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name()
        ppins = self.printer.lookup_object('pins')
        rx = ppins.lookup_pin(config.get('rx_pin'))
        tx = ppins.lookup_pin(config.get('tx_pin'))
        en = ppins.lookup_pin(config.get('enable_pin'),
                              can_invert=True)
        if not (rx['chip'] is tx['chip'] is en['chip']):
            raise config.error("%s pins must be on one mcu" % (self.name,))
        self.mcu = rx['chip']
        self.pins = (rx['pin'], tx['pin'], en['pin'], en['invert'])
        self.oid = self.mcu.create_oid()
        self.cmd_queue = self.mcu.alloc_command_queue()
        self.tx_cmd = self.query_cmd = None
        self.mcu.register_config_callback(self._build_config)
        # Safety / behaviour
        self.status_timeout = config.getfloat('status_timeout', 5., above=1.)
        self.config_refresh = config.getfloat('config_refresh', 2., above=.2)
        # MCU-side: bus goes down if the host sends nothing for this long
        self.host_timeout = config.getfloat('host_timeout', 3.,
                                            minval=1.5, maxval=8.)
        ignore = config.getlist('ignore_faults', [])
        self.ignore_mask = 0
        for f in ignore:
            bits = [b for b, n in FAULT_NAMES.items() if n == f.upper()]
            if not bits:
                raise config.error("Unknown AC Controller fault '%s'" % (f,))
            self.ignore_mask |= 1 << bits[0]
        # Desired outputs (None = AC Controller decides)
        self.bed_fan_pwm = None
        self.psu_fan_pwm = None
        self.led = ('off', (0, 0, 0, 0), 0.)
        self.led_dirty = True
        self.config_dirty = True
        # Bus state
        self.reasm = cy.Reassembler(MASTER_NODE_ID)
        self.rx_chunks = {}
        self.tids = {}
        self.uptime_start = self.reactor.monotonic()
        self.alloc = {}             # unique_id -> node_id
        self.nodes = {}             # node_id -> state dict
        self.ac_node = None
        self.last_ac_node = None    # kept across heartbeat loss for faults
        self.status = {}
        self.status_raw = b''
        self.nan_logged = False
        self.last_status_time = 0.
        self.last_config_time = 0.
        self.bus_status = {}
        self.ever_online = False
        self.last_hb = 0.
        # Heater
        heater_name = config.get('heater', 'heater_bed')
        self.heater = RemoteHeater(self, config, heater_name)
        pheaters = self.printer.load_object(config, 'heaters')
        if heater_name in pheaters.heaters:
            raise config.error("Heater '%s' already defined" % (heater_name,))
        pheaters.heaters[heater_name] = self.heater
        pheaters.available_heaters.append(heater_name)
        self._register_heater_sensor(pheaters, heater_name)
        self.printer.add_object(heater_name, self.heater)
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command(
            "SET_HEATER_TEMPERATURE", "HEATER", self.heater.short_name,
            self.heater.cmd_SET_HEATER_TEMPERATURE,
            desc=self.heater.cmd_SET_HEATER_TEMPERATURE_help)
        if heater_name == 'heater_bed':
            gcode.register_command("M140", self.cmd_M140)
            gcode.register_command("M190", self.cmd_M190)
        gcode.register_command("AC_STATUS", self.cmd_AC_STATUS,
                               desc=self.cmd_AC_STATUS_help)
        gcode.register_command("SET_AC_FAN", self.cmd_SET_AC_FAN,
                               desc=self.cmd_SET_AC_FAN_help)
        gcode.register_command("SET_AC_LED", self.cmd_SET_AC_LED,
                               desc=self.cmd_SET_AC_LED_help)
        gcode.register_command("AC_TEST_HOST_LOSS",
                               self.cmd_AC_TEST_HOST_LOSS,
                               desc=self.cmd_AC_TEST_HOST_LOSS_help)
        self.loss_test = None
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.printer.register_event_handler("klippy:shutdown",
                                            self._handle_shutdown)
        self.timer = None
    def _register_heater_sensor(self, pheaters, heater_name):
        # PrinterHeaters.register_sensor() needs a config object; mirror it
        pheaters.available_sensors.append(heater_name)
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command('TEMPERATURE_WAIT', "SENSOR", heater_name,
                                   pheaters.cmd_TEMPERATURE_WAIT,
                                   desc=pheaters.cmd_TEMPERATURE_WAIT_help)
        if heater_name == 'heater_bed':
            if 'B' in pheaters.gcode_id_to_sensor:
                raise self.printer.config_error(
                    "G-Code sensor id B already registered")
            pheaters.gcode_id_to_sensor['B'] = self.heater
    # MCU setup
    def _build_config(self):
        rx, tx, en, inv = self.pins
        self.mcu.add_config_cmd(
            "config_fdcan_raw oid=%d rx_pin=%s tx_pin=%s enable_pin=%s"
            " enable_invert=%d watchdog_ms=%d"
            % (self.oid, rx, tx, en, inv, int(self.host_timeout * 1000.)))
        self.tx_cmd = self.mcu.lookup_command(
            "fdcan_raw_tx oid=%c id=%u flags=%c data=%*s", cq=self.cmd_queue)
        self.query_cmd = self.mcu.lookup_command(
            "fdcan_raw_query oid=%c", cq=self.cmd_queue)
        self.mcu.register_serial_response(
            self._handle_rx_chunk,
            "fdcan_raw_rx oid=%c id=%u seq=%c flags=%c data=%*s", self.oid)
        self.mcu.register_serial_response(
            self._handle_bus_status,
            "fdcan_raw_status oid=%c psr=%u ecr=%u rx_overflow=%u"
            " tx_overflow=%u bad_cmd=%u wd_trips=%u bus_off=%c", self.oid)
    def _handle_ready(self):
        self.timer = self.reactor.register_timer(self._tick, self.reactor.NOW)
    def _handle_shutdown(self):
        # The MCU leaves the bus on shutdown; also clear the target so a
        # recovered link never resumes heating by itself.
        with self.heater.lock:
            self.heater.target_temp = 0.
    # Raw frame I/O
    def _send_frame(self, can_id, data):
        if self.tx_cmd is None or self.printer.is_shutdown():
            return
        data = bytes(data)
        for off in range(0, len(data), 32):
            chunk = data[off:off + 32]
            more = 1 if off + 32 < len(data) else 0
            self.tx_cmd.send([self.oid, can_id, more, chunk])
    def _next_tid(self, key):
        tid = self.tids.get(key, 0)
        self.tids[key] = (tid + 1) % cy.TRANSFER_ID_MOD
        return tid
    def _send_transfer(self, kind, port_id, payload, dest=None, tid=None):
        if tid is None:
            tid = self._next_tid((kind, port_id, dest))
        can_id = cy.make_can_id(kind, port_id, MASTER_NODE_ID, dest)
        for frame in cy.make_frames(payload, tid):
            self._send_frame(can_id, frame)
    def _handle_rx_chunk(self, params):
        # Runs in the serial thread; hand over to the reactor
        self.reactor.register_async_callback(
            (lambda e, p=params: self._process_rx_chunk(e, p)))
    def _process_rx_chunk(self, eventtime, params):
        seq = params['seq']
        buf = self.rx_chunks.pop(seq, b'') + bytes(params['data'])
        if params['flags'] & 1:
            self.rx_chunks[seq] = buf
            if len(self.rx_chunks) > 8:
                self.rx_chunks.clear()
            return
        xfer = self.reasm.feed(params['id'], buf, eventtime)
        if xfer is not None:
            try:
                self._dispatch(eventtime, xfer)
            except Exception:
                logging.exception("ac_controller: bad transfer port %d",
                                  xfer.port_id)
    def _handle_bus_status(self, params):
        self.bus_status = dict(params)
    # Transfer dispatch
    def _dispatch(self, eventtime, x):
        if x.kind == cy.KIND_MESSAGE:
            if x.port_id == PORT_PNP_V2 and x.source is cy.NODE_ID_ANON:
                self._on_pnp(x)
            elif x.port_id == PORT_HEARTBEAT and x.source is not None:
                self._on_heartbeat(eventtime, x)
            elif (x.port_id == MSG_AC_STATUS and x.source is not None
                  and x.source in (self.ac_node, self.last_ac_node)):
                self._on_status(eventtime, x)
            elif x.port_id == PORT_RECORD:
                sev, text = decode_record(x.payload)
                logging.info("ac_controller: node %s log[%d]: %s",
                             x.source, sev, text)
        elif x.kind == cy.KIND_RESPONSE:
            node = self.nodes.get(x.source)
            if node is None:
                return
            if x.port_id == SRV_GET_INFO:
                info = decode_get_info_response(x.payload)
                node['name'] = info['name']
                node['info'] = info
                logging.info("ac_controller: node %d is %s sw %d.%d",
                             x.source, info['name'],
                             info['software'][0], info['software'][1])
                if info['name'] == AC_NODE_NAME:
                    self.ac_node = self.last_ac_node = x.source
                    self.config_dirty = self.led_dirty = True
            elif x.port_id == SRV_EXECUTE_COMMAND:
                status = decode_execute_command_response(x.payload)
                if status:
                    logging.warning("ac_controller: node %d command status"
                                    " %d", x.source, status)
    def _on_pnp(self, x):
        req_id, unique_id = decode_pnp(x.payload)
        node_id = self.alloc.get(unique_id)
        if node_id is None:
            used = set(self.alloc.values()) | set(self.nodes.keys())
            free = [n for n in range(1, 126) if n not in used]
            if not free:
                logging.error("ac_controller: node-ID table full")
                return
            node_id = free[0]
            self.alloc[unique_id] = node_id
        logging.info("ac_controller: allocating node %d to %s", node_id,
                     unique_id.hex())
        self._send_transfer(cy.KIND_MESSAGE, PORT_PNP_V2,
                            encode_pnp(node_id, unique_id))
    def _on_heartbeat(self, eventtime, x):
        hb = decode_heartbeat(x.payload)
        node = self.nodes.setdefault(x.source, {
            'name': None, 'info_sent': 0., 'start_sent': False})
        if node.get('hb') and hb['uptime'] < node['hb']['uptime']:
            # Node rebooted: forget what we knew about it
            node.update(name=None, info_sent=0., start_sent=False)
            if x.source == self.ac_node:
                self.ac_node = None
        node['hb'] = hb
        node['seen'] = eventtime
        if node['name'] is None and eventtime - node['info_sent'] > 1.:
            node['info_sent'] = eventtime
            self._send_transfer(cy.KIND_REQUEST, SRV_GET_INFO, b'',
                                dest=x.source)
        if node['name'] != AC_NODE_NAME:
            return
        if hb['mode'] == MODE_SOFTWARE_UPDATE:
            if hb['health'] == HEALTH_ADVISORY and not node['start_sent']:
                # Bootloader autoboot was cancelled: start the app
                node['start_sent'] = True
                logging.info("ac_controller: starting AC Controller app")
                self._send_transfer(cy.KIND_REQUEST, SRV_EXECUTE_COMMAND,
                                    encode_execute_command(CMD_START_APP),
                                    dest=x.source)
            elif hb['health'] == HEALTH_WARNING:
                self.printer.invoke_shutdown(
                    "AC Controller has no application firmware; restore it"
                    " with the stock Prusa firmware")
        elif hb['mode'] == MODE_OPERATIONAL and not self.ever_online:
            self.ever_online = True
            logging.info("ac_controller: AC Controller operational (node %d)",
                         x.source)
    def _on_status(self, eventtime, x):
        st = decode_status(x.payload)
        self.status = st
        self.status_raw = bytes(x.payload)
        self.last_status_time = eventtime
        faults = st['faults'] & ~self.ignore_mask
        if faults:
            self.printer.invoke_shutdown("AC Controller fault: %s" % (
                ", ".join(fault_names(faults)),))
            return
        temp = st['bed_temp']
        if not math.isfinite(temp):
            # No reading yet (or a bad sensor). Harmless while the bed is
            # off; heating is refused until a real temperature arrives.
            if not self.nan_logged:
                self.nan_logged = True
                logging.info("ac_controller: bed temperature not available;"
                             " status payload %s", self.status_raw.hex())
            if self.heater.get_target():
                self.printer.invoke_shutdown(
                    "AC Controller bed temperature lost while heating")
            return
        if not (self.heater.min_temp - 10. <= temp
                <= self.heater.max_temp + 10.):
            self.printer.invoke_shutdown(
                "AC Controller bed temperature %.1f out of range" % (temp,))
            return
        self.heater.update(eventtime, temp, st['bed_pwm'])
        test = self.loss_test
        if test is not None:
            node = self.nodes.get(x.source, {}).get('hb') or {}
            key = (st['bed_target'], st['bed_pwm'] > 0., node.get('mode'),
                   node.get('health'))
            if key != test['last']:
                test['last'] = key
                test['events'].append((eventtime - test['start'], st['bed_target'],
                                       st['bed_pwm'], temp, node.get('mode'),
                                       node.get('health')))
        want = self.heater.get_target() or None
        have = st['bed_target']
        if (want is None) != (have is None) or (
                want is not None and abs(want - have) > .05):
            self.config_dirty = True
    # Periodic work
    def request_config_update(self):
        self.config_dirty = True
        if self.timer is not None:
            self.reactor.update_timer(self.timer, self.reactor.NOW)
    def _send_config(self, eventtime):
        target = self.heater.get_target() or None
        self._send_transfer(cy.KIND_REQUEST, SRV_AC_SET_CONFIG,
                            encode_config(bed_target=target,
                                          bed_fan_pwm=self.bed_fan_pwm,
                                          psu_fan_pwm=self.psu_fan_pwm),
                            dest=self.ac_node)
        self.last_config_time = eventtime
        self.config_dirty = False
    def _tick(self, eventtime):
        if self.printer.is_shutdown():
            return self.reactor.NEVER
        test = self.loss_test
        if test is not None and eventtime >= test['end']:
            self._finish_loss_test(eventtime)
            test = None
        # Master heartbeat
        if eventtime - self.last_hb >= HEARTBEAT_PERIOD:
            self.last_hb = eventtime
            if test is None:
                uptime = int(eventtime - self.uptime_start)
                self._send_transfer(cy.KIND_MESSAGE, PORT_HEARTBEAT,
                                    encode_heartbeat(uptime))
            else:
                # Keep the bridge watchdog fed without a master heartbeat
                self._send_transfer(cy.KIND_MESSAGE, PORT_RECORD,
                                    encode_record("host-loss test"))
            self.query_cmd.send([self.oid])
        # Node liveness
        if self.ac_node is not None:
            node = self.nodes.get(self.ac_node, {})
            if eventtime - node.get('seen', 0.) > HEARTBEAT_OFFLINE:
                logging.warning("ac_controller: AC Controller heartbeat lost")
                self.nodes.pop(self.ac_node, None)
                self.ac_node = None
        # Safety: a requested bed temperature needs a live status stream
        if self.heater.get_target():
            last = max(self.last_status_time, self.heater.target_since)
            if eventtime - last > self.status_timeout:
                self.printer.invoke_shutdown(
                    "AC Controller status timeout (bed target %.0f)"
                    % (self.heater.get_target(),))
                return self.reactor.NEVER
        # Config / LEDs
        if (test is None and self.ac_node is not None
                and self._ac_app_running()):
            if self.config_dirty and (
                    eventtime - self.last_config_time >= .2):
                self._send_config(eventtime)
            elif eventtime - self.last_config_time >= self.config_refresh:
                self._send_config(eventtime)
            if self.led_dirty:
                mode, rgbw, progress = self.led
                self._send_transfer(cy.KIND_MESSAGE, MSG_AC_LEDS,
                                    encode_leds(mode, rgbw, progress))
                self.led_dirty = False
        return eventtime + .2
    def _ac_app_running(self):
        # Stock firmware configures the node in initialization mode too
        node = self.nodes.get(self.ac_node)
        return bool(node and node.get('hb') and node['hb']['mode'] in (
            MODE_OPERATIONAL, MODE_INITIALIZATION))
    def _ac_operational(self):
        node = self.nodes.get(self.ac_node)
        return bool(node and node.get('hb')
                    and node['hb']['mode'] == MODE_OPERATIONAL)
    # G-Code
    def cmd_M140(self, gcmd, wait=False):
        temp = gcmd.get_float('S', 0.)
        pheaters = self.printer.lookup_object('heaters')
        pheaters.set_temperature(self.heater, temp, wait)
    def cmd_M190(self, gcmd):
        self.cmd_M140(gcmd, wait=True)
    cmd_AC_STATUS_help = "Report AC Controller (mains bed) status"
    def cmd_AC_STATUS(self, gcmd):
        node = self.nodes.get(self.ac_node) if self.ac_node else None
        lines = ["AC Controller node: %s" % (
            self.ac_node if self.ac_node is not None else 'not found',)]
        if node and node.get('hb'):
            hb = node['hb']
            lines.append("mode=%s health=%d uptime=%ds" % (
                MODE_NAMES.get(hb['mode'], hb['mode']), hb['health'],
                hb['uptime']))
        st = self.status
        if st:
            lines.append("bed %.1fC target %s pwm %.2f power %.0fW" % (
                st['bed_temp'], st['bed_target'], st['bed_pwm'],
                st['bed_power']))
            lines.append("mains %.0fV %.1fHz psu %.1fC triac %.1fC" % (
                st['ac_voltage'], st['ac_frequency'], st['psu_temp'],
                st['triac_temp']))
            lines.append("fans: bed %.0f/%.0f rpm psu %.0f rpm" % (
                st['bed_fan0']['rpm'], st['bed_fan1']['rpm'],
                st['psu_fan']['rpm']))
            lines.append("board %.1fC mcu %.1fC supply %.1fV" % (
                st['board_temp'], st['mcu_temp'], st['supply_voltage']))
            lines.append("faults: %s" % (
                ", ".join(fault_names(st['faults'])) or 'none',))
            lines.append("raw: %s" % (self.status_raw.hex(),))
        if self.bus_status:
            b = self.bus_status
            lines.append("can psr=0x%x ecr=0x%x rx_ovf=%d tx_ovf=%d"
                         " rx_err=%d watchdog_trips=%d bus_off=%d" % (
                             b['psr'], b['ecr'], b['rx_overflow'],
                             b['tx_overflow'], self.reasm.errors,
                             b['wd_trips'], b['bus_off']))
        gcmd.respond_info("\n".join(lines))
    cmd_AC_TEST_HOST_LOSS_help = ("Stop the master heartbeat and config for a"
                                  " while and log how the AC Controller reacts")
    def cmd_AC_TEST_HOST_LOSS(self, gcmd):
        secs = gcmd.get_float('SECONDS', 20., minval=5., maxval=60.)
        if self.loss_test is not None:
            raise gcmd.error("AC_TEST_HOST_LOSS already running")
        if self.ac_node is None:
            raise gcmd.error("AC Controller not found")
        now = self.reactor.monotonic()
        self.loss_test = {'start': now, 'end': now + secs, 'events': [],
                          'last': None}
        gcmd.respond_info("Master heartbeat and config paused for %.0fs"
                          % (secs,))
    def _finish_loss_test(self, eventtime):
        test, self.loss_test = self.loss_test, None
        self.config_dirty = True
        lines = ["AC_TEST_HOST_LOSS done (%.0fs). Changes seen:"
                 % (test['end'] - test['start'],)]
        for t, target, pwm, temp, mode, health in test['events']:
            lines.append("  +%.1fs target=%s pwm=%.2f bed=%.1fC mode=%s"
                         " health=%s" % (t, target, pwm, temp,
                                         MODE_NAMES.get(mode, mode), health))
        if not test['events']:
            lines.append("  no status received")
        logging.info("ac_controller: %s", "\n".join(lines))
        self.printer.lookup_object('gcode').respond_info("\n".join(lines))
    cmd_SET_AC_FAN_help = "Set AC Controller fan: FAN=bed|psu SPEED=0..1|auto"
    def cmd_SET_AC_FAN(self, gcmd):
        fan = gcmd.get('FAN').lower()
        speed = gcmd.get('SPEED', 'auto').lower()
        value = None if speed == 'auto' else max(0., min(1., float(speed)))
        if fan == 'bed':
            self.bed_fan_pwm = value
        elif fan == 'psu':
            self.psu_fan_pwm = value
        else:
            raise gcmd.error("FAN must be 'bed' or 'psu'")
        self.request_config_update()
    cmd_SET_AC_LED_help = ("Set the bed LED strip: RED GREEN BLUE WHITE"
                           " (0..1) [PROGRESS=0..1] or OFF=1")
    def cmd_SET_AC_LED(self, gcmd):
        if gcmd.get_int('OFF', 0):
            self.led = ('off', (0, 0, 0, 0), 0.)
        else:
            rgbw = tuple(int(round(255 * gcmd.get_float(c, 0., minval=0.,
                                                         maxval=1.)))
                         for c in ('RED', 'GREEN', 'BLUE', 'WHITE'))
            progress = gcmd.get_float('PROGRESS', None, minval=0., maxval=1.)
            if progress is None:
                self.led = ('color', rgbw, 0.)
            else:
                self.led = ('progress', rgbw, progress)
        self.led_dirty = True
    def get_status(self, eventtime):
        # NaN is not valid JSON; report missing readings as None
        st = {k: (v if not isinstance(v, float) or math.isfinite(v) else None)
              for k, v in self.status.items()}
        return {
            'node_id': self.ac_node,
            'operational': self.ac_node is not None
                           and self._ac_operational(),
            'faults': fault_names(st.get('faults', 0)),
            'bed_power': st.get('bed_power', 0.),
            'ac_voltage': st.get('ac_voltage', 0.),
            'ac_frequency': st.get('ac_frequency', 0.),
            'psu_temp': st.get('psu_temp', 0.),
            'mcu_temp': st.get('mcu_temp', 0.),
            'bed_fan_rpm': [_finite(st.get('bed_fan0', {}).get('rpm', 0.)),
                            _finite(st.get('bed_fan1', {}).get('rpm', 0.))],
            'psu_fan_rpm': _finite(st.get('psu_fan', {}).get('rpm', 0.)),
        }

def _finite(v):
    return v if v is None or math.isfinite(v) else None

def load_config(config):
    return ACController(config)

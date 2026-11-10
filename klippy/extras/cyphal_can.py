# Minimal Cyphal/CAN (v1.0) transport and DSDL bit serialization
#
# Just enough Cyphal to be the bus master for the Core One L's AC Controller:
# message and service transfers over CAN FD (MTU 64), single- and multi-frame
# TX and RX, CRC-16/CCITT-FALSE, transfer-ID tracking, plus a little-endian
# bit-level reader/writer for DSDL payloads.
#
# Copyright (C) 2026
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import struct

PRIO_NOMINAL = 4
MTU = 64
TRANSFER_ID_MOD = 32
NODE_ID_ANON = None

KIND_MESSAGE = 'msg'
KIND_REQUEST = 'req'
KIND_RESPONSE = 'resp'

DLC_SIZES = (0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64)

def round_frame_size(n):
    for s in DLC_SIZES:
        if s >= n:
            return s
    raise ValueError("frame too long: %d" % (n,))

def crc16(data, crc=0xffff):
    for b in bytearray(data):
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xffff
            else:
                crc = (crc << 1) & 0xffff
    return crc

######################################################################
# CAN ID
######################################################################

def make_can_id(kind, port_id, source, dest=None, priority=PRIO_NOMINAL):
    cid = (priority & 7) << 26
    if kind == KIND_MESSAGE:
        if source is None:
            raise ValueError("anonymous transmit not supported")
        cid |= (3 << 21) | ((port_id & 0x1fff) << 8) | (source & 0x7f)
    else:
        cid |= (1 << 25) | ((port_id & 0x1ff) << 14)
        cid |= ((dest & 0x7f) << 7) | (source & 0x7f)
        if kind == KIND_REQUEST:
            cid |= 1 << 24
    return cid

def parse_can_id(cid):
    """Return (kind, port_id, source, dest, priority) or None if invalid"""
    priority = (cid >> 26) & 7
    source = cid & 0x7f
    if cid & (1 << 25):
        if cid & (1 << 23):
            return None
        kind = KIND_REQUEST if cid & (1 << 24) else KIND_RESPONSE
        return (kind, (cid >> 14) & 0x1ff, source, (cid >> 7) & 0x7f,
                priority)
    if cid & (1 << 23) or cid & (1 << 7):
        return None
    if cid & (1 << 24):
        source = NODE_ID_ANON
    return (KIND_MESSAGE, (cid >> 8) & 0x1fff, source, None, priority)

######################################################################
# Transfer -> frames
######################################################################

def make_frames(payload, transfer_id, mtu=MTU):
    """Split a transfer payload into CAN frame data blocks (with tails)"""
    payload = bytes(payload)
    tid = transfer_id % TRANSFER_ID_MOD
    cap = mtu - 1
    if len(payload) <= cap:
        size = round_frame_size(len(payload) + 1)
        pad = b'\x00' * (size - 1 - len(payload))
        return [payload + pad + bytes([0xe0 | tid])]
    # Multi-frame: payload + padding + CRC; padding lands in the last frame
    # ahead of the CRC and is covered by it.
    total = len(payload) + 2
    nframes = (total + cap - 1) // cap
    last = total - cap * (nframes - 1)
    pad = b'\x00' * (round_frame_size(last + 1) - 1 - last)
    body = payload + pad
    stream = body + struct.pack('>H', crc16(body))
    frames = []
    toggle = 1
    for i in range(nframes):
        chunk = stream[i * cap:(i + 1) * cap]
        tail = tid | (toggle << 5)
        if i == 0:
            tail |= 0x80
        if i == nframes - 1:
            tail |= 0x40
        frames.append(chunk + bytes([tail]))
        toggle ^= 1
    return frames

######################################################################
# Frames -> transfer
######################################################################

class RxTransfer:
    def __init__(self, kind, port_id, source, dest, priority, transfer_id,
                 payload, timestamp):
        self.kind = kind
        self.port_id = port_id
        self.source = source
        self.dest = dest
        self.priority = priority
        self.transfer_id = transfer_id
        self.payload = payload
        self.timestamp = timestamp

class Reassembler:
    """Per-session multi-frame reassembly (spec 4.2.2)"""
    def __init__(self, local_node_id, timeout=2.0):
        self.local_node_id = local_node_id
        self.timeout = timeout
        self.sessions = {}
        self.errors = 0
    def feed(self, can_id, data, timestamp):
        """Feed one frame; return an RxTransfer when one completes"""
        parsed = parse_can_id(can_id)
        if parsed is None or not data:
            self.errors += parsed is None
            return None
        kind, port_id, source, dest, priority = parsed
        if kind != KIND_MESSAGE and dest != self.local_node_id:
            return None
        if source == self.local_node_id and kind == KIND_MESSAGE:
            return None
        data = bytes(data)
        tail = data[-1]
        sot, eot, toggle = tail & 0x80, tail & 0x40, (tail >> 5) & 1
        tid = tail & 0x1f
        body = data[:-1]
        if sot and eot:
            if not toggle:
                self.errors += 1
                return None
            return RxTransfer(kind, port_id, source, dest, priority, tid,
                              body, timestamp)
        if source is NODE_ID_ANON:
            # Anonymous transfers are single-frame by definition
            self.errors += 1
            return None
        key = (kind, port_id, source)
        sess = self.sessions.get(key)
        if sot:
            if not toggle:
                self.errors += 1
                return None
            self.sessions[key] = sess = {
                'tid': tid, 'toggle': 1, 'buf': bytearray(), 'ts': timestamp}
        elif (sess is None or sess['tid'] != tid
              or timestamp - sess['ts'] > self.timeout):
            self.sessions.pop(key, None)
            return None
        if toggle != sess['toggle']:
            self.errors += 1
            del self.sessions[key]
            return None
        sess['buf'] += body
        sess['toggle'] ^= 1
        if not eot:
            return None
        del self.sessions[key]
        buf = bytes(sess['buf'])
        if len(buf) < 2 or crc16(buf) != 0:
            self.errors += 1
            return None
        return RxTransfer(kind, port_id, source, dest, priority, tid,
                          buf[:-2], sess['ts'])

######################################################################
# DSDL bit-level serialization (little-endian, LSB first)
######################################################################

class BitWriter:
    def __init__(self):
        self.bits = 0
        self.nbits = 0
    def uint(self, value, width):
        value &= (1 << width) - 1
        self.bits |= value << self.nbits
        self.nbits += width
        return self
    def int(self, value, width):
        return self.uint(value & ((1 << width) - 1), width)
    def bool(self, value):
        return self.uint(1 if value else 0, 1)
    def float32(self, value):
        return self.uint(struct.unpack('<I', struct.pack('<f', value))[0], 32)
    def align(self):
        self.nbits = (self.nbits + 7) & ~7
        return self
    def raw(self, data):
        self.align()
        for b in bytearray(data):
            self.uint(b, 8)
        return self
    def bytes(self):
        n = (self.nbits + 7) // 8
        return bytes((self.bits >> (8 * i)) & 0xff for i in range(n))

class BitReader:
    """Reads past the end yield zeros (DSDL implicit zero extension)"""
    def __init__(self, data):
        data = bytearray(data)
        self.bits = sum(b << (8 * i) for i, b in enumerate(data))
        self.size = len(data) * 8
        self.pos = 0
    def uint(self, width):
        value = (self.bits >> self.pos) & ((1 << width) - 1)
        self.pos += width
        return value
    def int(self, width):
        value = self.uint(width)
        if value & (1 << (width - 1)):
            value -= 1 << width
        return value
    def bool(self):
        return bool(self.uint(1))
    def float32(self):
        return struct.unpack('<f', struct.pack('<I', self.uint(32)))[0]
    def align(self):
        self.pos = (self.pos + 7) & ~7
        return self
    def raw(self, n):
        self.align()
        return bytes(self.uint(8) for _ in range(n))
    def remaining_bytes(self):
        return max(0, (self.size - self.pos) // 8)

"""serial_frame: the IR optical wire-frame format, shared by the flasher (encode)
and the receiver (decode). Mirrors watch-library/shared/utils/serial_frame.c.

Wire layout:

    +----------+---------+-------------+---------+---------+
    | preamble | id      | len + flags | payload | crc32   |
    | 4 bytes  | 2 bytes | 2 bytes     | N bytes | 4 bytes |
    +----------+---------+-------------+---------+---------+
                <------ CRC32 covers this ------>

  preamble:  AA 55 AA 55
  id:        uint16 LE, application-defined.
  len+flags: bits [9:0] = declared payload length [0, MAX_PAYLOAD];
             bits [15:10] = 6 application-defined flag bits.
  payload:   carried padded up to a 4-byte boundary (physical = ceil(len/4)*4);
             padding enters the CRC but is not delivered. A declared length of 0
             means no payload bytes on the wire.
  crc32:     standard CRC-32 (== zlib.crc32) over id || len+flags || padded
             payload. LE on the wire.
"""

import struct
import zlib
from collections import namedtuple

PREAMBLE      = b'\xAA\x55\xAA\x55'
MAX_PAYLOAD   = 512
PAYLOAD_ALIGN = 4
LENGTH_MASK   = 0x03FF   # low 10 bits
FLAGS_SHIFT   = 10
FLAGS_MASK    = 0x3F     # 6 bits


def build_frame(payload: bytes, frame_id: int, flags: int) -> bytes:
    """Wrap a payload in a serial frame. Mirrors serial_frame_encode()."""
    if not (0 <= len(payload) <= MAX_PAYLOAD):
        raise ValueError(f"payload length must be in [0, {MAX_PAYLOAD}], got {len(payload)}")
    if not (0 <= frame_id <= 0xFFFF):
        raise ValueError(f"id must fit in uint16, got {frame_id}")
    if flags & ~FLAGS_MASK:
        raise ValueError(f"flags must fit in 6 bits, got 0x{flags:X}")
    declared = len(payload)
    physical = (declared + PAYLOAD_ALIGN - 1) & ~(PAYLOAD_ALIGN - 1)
    padded   = payload + b'\x00' * (physical - declared)
    len_flags = ((flags & FLAGS_MASK) << FLAGS_SHIFT) | (declared & LENGTH_MASK)
    crc_region = struct.pack('<HH', frame_id, len_flags) + padded
    crc = zlib.crc32(crc_region) & 0xFFFFFFFF
    return PREAMBLE + crc_region + struct.pack('<I', crc)


# A validated frame delivered by FrameParser. `payload` is the first `length`
# bytes only (padding stripped); `flags` is the raw 6-bit field.
Frame = namedtuple('Frame', 'id length flags payload')

# Parser states, mirror the enum in serial_frame.c.
(_HUNT_AA0, _HUNT_55_0, _HUNT_AA_1, _HUNT_55_1,
 _ID_LO, _ID_HI, _LENFLAGS_LO, _LENFLAGS_HI, _PAYLOAD, _CRC_BYTES) = range(10)

_PREAMBLE_AA = 0xAA
_PREAMBLE_55 = 0x55


class FrameParser:
    """Byte-fed state machine that pulls validated frames out of a noisy stream,
    a faithful port of serial_frame_parser_feed() in serial_frame.c (same resync
    rules, same stats). feed() returns the list of valid frames in that slice;
    state persists across calls so partial frames continue."""

    def __init__(self):
        self._state = _HUNT_AA0
        self._crc_buf = bytearray(4 + MAX_PAYLOAD)  # id(2) + len+flags(2) + padded payload
        self._id = 0
        self._length = 0
        self._flags = 0
        self._physical = 0
        self._payload_idx = 0
        self._crc_received = 0
        self._crc_bytes_seen = 0
        # Stats since construction (mirror serial_frame_parser_t).
        self.bytes_total = 0
        self.frames_valid = 0
        self.frames_bad_crc = 0
        self.frames_bad_length = 0
        self.payload_bytes_valid = 0

    def reset(self):
        """Abandon any in-flight frame and return to hunting (e.g. after an
        optical dropout). Leaves stats and delivered frames alone."""
        self._state = _HUNT_AA0
        self._payload_idx = 0
        self._crc_received = 0
        self._crc_bytes_seen = 0

    def feed(self, data) -> list:
        frames = []
        for b in data:
            self.bytes_total += 1
            st = self._state

            if st == _HUNT_AA0:
                if b == _PREAMBLE_AA:
                    self._state = _HUNT_55_0
            elif st == _HUNT_55_0:
                if b == _PREAMBLE_55:
                    self._state = _HUNT_AA_1
                elif b == _PREAMBLE_AA:
                    pass                       # still matched one AA: stay
                else:
                    self._state = _HUNT_AA0
            elif st == _HUNT_AA_1:
                self._state = _HUNT_55_1 if b == _PREAMBLE_AA else _HUNT_AA0
            elif st == _HUNT_55_1:
                if b == _PREAMBLE_55:
                    self._crc_received = 0
                    self._crc_bytes_seen = 0
                    self._state = _ID_LO
                elif b == _PREAMBLE_AA:
                    self._state = _HUNT_55_0
                else:
                    self._state = _HUNT_AA0
            elif st == _ID_LO:
                self._crc_buf[0] = b
                self._state = _ID_HI
            elif st == _ID_HI:
                self._crc_buf[1] = b
                self._id = self._crc_buf[0] | (self._crc_buf[1] << 8)
                self._state = _LENFLAGS_LO
            elif st == _LENFLAGS_LO:
                self._crc_buf[2] = b
                self._state = _LENFLAGS_HI
            elif st == _LENFLAGS_HI:
                self._crc_buf[3] = b
                lf = self._crc_buf[2] | (self._crc_buf[3] << 8)
                self._length = lf & LENGTH_MASK
                self._flags = (lf >> FLAGS_SHIFT) & FLAGS_MASK
                if self._length > MAX_PAYLOAD:
                    # Bogus length: almost certainly noise that matched the
                    # preamble. Drop and resync.
                    self.frames_bad_length += 1
                    self._state = _HUNT_AA0
                else:
                    self._physical = (self._length + PAYLOAD_ALIGN - 1) & ~(PAYLOAD_ALIGN - 1)
                    self._payload_idx = 0
                    self._state = _CRC_BYTES if self._physical == 0 else _PAYLOAD
            elif st == _PAYLOAD:
                if self._payload_idx < MAX_PAYLOAD:
                    self._crc_buf[4 + self._payload_idx] = b
                self._payload_idx += 1
                if self._payload_idx >= self._physical:
                    self._state = _CRC_BYTES
            elif st == _CRC_BYTES:
                self._crc_received |= b << (8 * self._crc_bytes_seen)
                self._crc_bytes_seen += 1
                if self._crc_bytes_seen >= 4:
                    region = bytes(self._crc_buf[:4 + self._physical])
                    computed = zlib.crc32(region) & 0xFFFFFFFF
                    if self._crc_received == computed:
                        self.frames_valid += 1
                        self.payload_bytes_valid += self._length
                        payload = bytes(self._crc_buf[4:4 + self._length])
                        frames.append(Frame(self._id, self._length, self._flags, payload))
                    else:
                        self.frames_bad_crc += 1
                    self._state = _HUNT_AA0
            else:
                self._state = _HUNT_AA0
        return frames

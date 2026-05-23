"""ir_modem: shared host-side library for the IR modem firmware.

Talks the modem's USB command protocol (src/modem_*.cpp, UNO / XIAO SAMD21,
identical from the host's point of view). Used by bin/firmware_flasher.py and
bin/test_rx.py. The modem is a DUMB half-duplex byte pipe: it transmits a blob out
the IR LED and/or forwards every decoded optical byte back. All higher-level
protocol (frame assembly, ACK matching, UF2, retransmit) lives in the callers.

USB protocol (fixed 115200, decoupled from the IR baud):

    framing (both directions):  [SOF=0xA5][TYPE][LEN_LO][LEN_HI][payload]

    host -> modem:
        CMD_CONFIG 0x01   payload = tx_baud(4 LE) rx_baud(4 LE) enc(1)
        CMD_TX     0x02   payload = tx_flags(1) + frame bytes
                          tx_flags bit0 AUTO_RX: flip to RX after sending
                          (else fire-and-forget). The modem holds no ACK policy.
        CMD_RX     0x03   enter RX (listen); no payload -> MSG_OK, then MSG_RX
        CMD_STOP   0x04   stop RX / TX, go idle
        CMD_PING   0x05   -> MSG_PONG
    modem -> host:
        MSG_RX     0x10   payload = raw decoded optical bytes
        MSG_TX_DONE 0x11
        MSG_OK     0x12
        MSG_ERR    0x13   payload = code(1)
        MSG_PONG   0x15   payload = proto_version(1)
"""

import struct
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("error: pyserial not installed.  pipx install pyserial  (or pip install pyserial)")

# --- USB framing constants (must match src/modem_*.cpp) -----------------
USB_SOF      = 0xA5
CMD_CONFIG   = 0x01
CMD_TX       = 0x02
CMD_RX       = 0x03
CMD_STOP     = 0x04
CMD_PING     = 0x05
TX_FLAG_AUTO_RX = 0x01   # first byte of a CMD_TX payload
MSG_RX        = 0x10
MSG_TX_DONE   = 0x11
MSG_OK        = 0x12
MSG_ERR       = 0x13
MSG_RX_STATUS = 0x14   # DIAGNOSTIC (samd21 modem): ferr(2 LE) bufovf(2 LE)
MSG_PONG      = 0x15
MODEM_PROTO_VERSION = 1

# Encoding selector (matches the watch / firmware ENC_NRZ / ENC_IRDA).
ENC_NRZ  = 0
ENC_IRDA = 1

# Serial defaults. The UNO modem auto-resets when the port is opened; the
# bootloader takes ~2s before the sketch runs, so callers wait RESET_DELAY_S.
HANDSHAKE_BAUD = 115200
RESET_DELAY_S  = 2.0


def enc_byte(encoding: str) -> int:
    """'irda' -> ENC_IRDA, anything else -> ENC_NRZ."""
    return ENC_IRDA if encoding == 'irda' else ENC_NRZ


class Modem:
    def __init__(self, ser: serial.Serial):
        self.ser = ser
        self._buf = bytearray()

    def send(self, cmd: int, payload: bytes = b''):
        hdr = bytes((USB_SOF, cmd, len(payload) & 0xFF, (len(payload) >> 8) & 0xFF))
        self.ser.write(hdr + payload)
        self.ser.flush()

    def _read_into_buf(self, deadline: float):
        timeout = max(0.0, deadline - time.time())
        self.ser.timeout = min(0.1, timeout) if timeout else 0.0
        chunk = self.ser.read(256)
        if chunk:
            self._buf += chunk

    def read_message(self, deadline: float):
        """Return (type, payload) or None on deadline. Resyncs on SOF."""
        while True:
            # Try to parse a complete message out of the buffer.
            while self._buf and self._buf[0] != USB_SOF:
                del self._buf[0]                      # resync to SOF
            if len(self._buf) >= 4:
                mtype = self._buf[1]
                mlen = self._buf[2] | (self._buf[3] << 8)
                if len(self._buf) >= 4 + mlen:
                    payload = bytes(self._buf[4:4 + mlen])
                    del self._buf[:4 + mlen]
                    return mtype, payload
            if time.time() >= deadline:
                return None
            self._read_into_buf(deadline)


def handshake(modem: Modem, tx_baud: int, rx_baud: int, encoding: str):
    """PING/PONG to confirm the modem is alive + the right version, then CONFIG
    it with tx/rx baud and encoding. Exits on failure. Receive-only callers can
    pass any valid tx_baud (the modem only uses it on a CMD_TX)."""
    modem.send(CMD_PING)
    msg = modem.read_message(time.time() + 3.0)
    if not msg or msg[0] != MSG_PONG:
        sys.exit(f"error: no PONG from modem (got {msg!r}); is a modem firmware flashed?")
    ver = msg[1][0] if msg[1] else 0
    if ver != MODEM_PROTO_VERSION:
        sys.exit(f"error: modem proto version {ver}, expected {MODEM_PROTO_VERSION}")
    cfg = struct.pack('<IIB', tx_baud, rx_baud, enc_byte(encoding))
    modem.send(CMD_CONFIG, cfg)
    msg = modem.read_message(time.time() + 3.0)
    if not msg or msg[0] != MSG_OK:
        sys.exit(f"error: modem rejected CONFIG (got {msg!r})")

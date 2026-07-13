#!/usr/bin/env python3
"""totp_sender: push otpauth:// TOTP URIs to the watch over the IR link.

Host homologue of the watch's IR receive mode in `totp_lfs_face` (long-Alarm
press). Drives the same dumb modem firmware (src/modem_*.cpp, UNO / XIAO SAMD21)
as firmware_flasher.py / test_tx.py, using the shared ir_modem + serial_frame
libraries.

Protocol (a stripped-down stop-and-wait, matching the watch face):

  * Each URI is one serial frame: payload = the raw otpauth URI bytes, no flags.
    The frame id increments per URI (0, 1, 2, ...; wraps at 0xFFFF).
  * The watch validates the URI, appends new ones to totp_uris.txt, and ACKs by
    echoing the frame's bare 2-byte id (LE) — exactly like the flasher's ACK.
  * We retransmit the same frame (same id) until that id echoes back. If our ACK
    was lost and we resend, the watch recognises the repeated id, re-validates,
    does NOT add a second copy, and re-ACKs so we advance.
  * A URI the watch considers invalid is dropped SILENTLY (no ACK). So a frame
    that never ACKs means either the watch isn't in receive mode, the link is
    dead, or the URI is malformed. After --retries attempts we give up on it and
    report the failure rather than hang forever.

The IR parameters default to the watch face's hardcoded profile:
2400 baud data (host->watch), 300 baud ACK (watch->host), NRZ, no inversion.

Two modes:

    # interactive: paste a URI, hit enter, repeat (Ctrl-D / Ctrl-C to quit)
    totp_sender.py

    # batch: one otpauth URI per line, sent in order, waiting for each ACK
    totp_sender.py --file secrets.txt

Put the watch on the totp_lfs face and long-press Alarm to enter receive mode
before sending. A short Alarm press on the watch exits receive mode.
"""

import argparse
import atexit
import signal
import struct
import sys
import time

from ir_modem import (Modem, handshake, HANDSHAKE_BAUD, RESET_DELAY_S,
                      CMD_TX, CMD_STOP, TX_FLAG_AUTO_RX,
                      MSG_RX, MSG_TX_DONE, MSG_ERR,
                      RX_DIGITAL, RX_ANALOG)
from serial_frame import build_frame

try:
    import serial
except ImportError:
    sys.exit("error: pyserial not installed.  pipx install pyserial  (or pip install pyserial)")

# Defaults: the watch face's hardcoded IR profile.
DEFAULT_DATA_BAUD = 2400   # host -> watch (the watch's RX baud)
DEFAULT_ACK_BAUD  = 300    # watch -> host (the watch's TX baud)

# The watch face's receive buffer (TOTP_IR_MAX_URI) caps a URI at 255 bytes.
MAX_URI_LEN = 255

URI_PREFIX = "otpauth://totp/"

# Module-global so the atexit / signal handlers can park + close the port.
ser = None
modem = None


def _shutdown():
    """Park the modem idle (CMD_STOP), then close the port. Idempotent."""
    global ser, modem
    if ser is None:
        return
    try:
        if ser.is_open:
            if modem is not None:
                modem.send(CMD_STOP)
                time.sleep(0.05)
            ser.close()
    except Exception:
        pass
    ser = None


def _signal_handler(signum, _frame):
    _shutdown()
    sys.exit(130)


def looks_like_totp_uri(uri: str) -> bool:
    """Cheap client-side sanity check so obvious typos are caught here instead of
    silently dropped by the watch (which would just retransmit until --retries).
    Not a full parse — the watch is the authority — just enough to avoid sending
    junk: an otpauth TOTP prefix, a query string, and a secret= parameter."""
    if not uri.startswith(URI_PREFIX):
        return False
    q = uri.find('?')
    if q < 0:
        return False
    params = uri[q + 1:].split('&')
    return any(p.startswith('secret=') and len(p) > len('secret=') for p in params)


def send_uri_and_wait_ack(modem: Modem, uri: str, frame_id: int,
                          data_baud: int, ack_timeout: float) -> bool:
    """Send one URI frame, then wait for the 2-byte id echo. True on ACK.

    A faithful mirror of firmware_flasher.send_frame_and_wait_ack: AUTO_RX so the
    modem flips to RX after transmitting, then a rolling-window substring match on
    the decoded optical bytes for the expected id (tolerates surrounding noise /
    partial garbage on the weak return path)."""
    frame = build_frame(uri.encode('utf-8'), frame_id, 0)
    expected = struct.pack('<H', frame_id)
    modem.send(CMD_TX, bytes((TX_FLAG_AUTO_RX,)) + frame)

    # Overall ceiling: time to clock the frame out at data_baud, plus the ACK
    # window (which (re)starts tight once MSG_TX_DONE arrives).
    tx_estimate = len(frame) * 10 / data_baud
    deadline = time.time() + tx_estimate + ack_timeout + 1.0
    window = bytearray()
    while time.time() < deadline:
        msg = modem.read_message(deadline)
        if msg is None:
            break
        mtype, payload = msg
        if mtype == MSG_TX_DONE:
            deadline = time.time() + ack_timeout      # start the tight ACK window
        elif mtype == MSG_RX:
            window += payload
            if expected in window:
                return True
            del window[:-8]                           # keep only the last few bytes
        elif mtype == MSG_ERR:
            code = payload[0] if payload else 0
            sys.exit(f"error: modem reported error code {code}")
    return False


def send_uri(modem: Modem, uri: str, frame_id: int, args) -> bool:
    """Send a URI, retransmitting (same id) until ACKed or `args.retries`
    attempts are exhausted. Returns True on ACK, False on give-up."""
    for attempt in range(args.retries + 1):
        if attempt > 0:
            print(f"    no ACK, retransmit {attempt}/{args.retries}")
        if send_uri_and_wait_ack(modem, uri, frame_id, args.baud, args.timeout):
            return True
    return False


def run_batch(modem: Modem, path: str, args) -> int:
    """Send every URI (one per line) from `path`, waiting for each ACK before the
    next. Blank lines and lines starting with '#' are skipped. Returns a process
    exit code (0 = all sent)."""
    try:
        with open(path, 'r', encoding='utf-8') as f:
            lines = f.read().splitlines()
    except OSError as e:
        sys.exit(f"error: cannot read {path}: {e}")

    uris = [ln.strip() for ln in lines]
    uris = [u for u in uris if u and not u.startswith('#')]
    if not uris:
        sys.exit(f"error: no URIs found in {path}")

    print(f"sending {len(uris)} URI(s) from {path}...")
    frame_id = 0
    failures = 0
    for i, uri in enumerate(uris):
        label = f"[{i + 1}/{len(uris)}] id {frame_id}"
        if not _validate_len_and_shape(uri, label):
            failures += 1
            continue
        if i > 0:
            time.sleep(args.settle)   # let the watch reopen RX after the prev ACK
        print(f"  {label}: sending {uri[:48]}{'...' if len(uri) > 48 else ''}")
        if send_uri(modem, uri, frame_id, args):
            print(f"  {label}: ACKed")
            frame_id = (frame_id + 1) & 0xFFFF
        else:
            print(f"  {label}: FAILED after {args.retries} retries "
                  f"(watch in receive mode? URI valid?)")
            failures += 1
    print(f"done: {len(uris) - failures}/{len(uris)} sent"
          + (f", {failures} failed" if failures else ""))
    return 1 if failures else 0


def run_interactive(modem: Modem, args) -> int:
    """Prompt for a URI, send it, and prompt again after each ACK. The frame id
    advances only on a successful ACK, so a failed/re-typed URI reuses the id."""
    print("interactive mode: paste an otpauth:// URI and press Enter.")
    print("  (Ctrl-D or empty line + Ctrl-C to quit)\n")
    frame_id = 0
    while True:
        try:
            uri = input(f"URI (id {frame_id})> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not uri:
            continue
        if not _validate_len_and_shape(uri, "input"):
            continue
        if send_uri(modem, uri, frame_id, args):
            print("  ACKed\n")
            frame_id = (frame_id + 1) & 0xFFFF
        else:
            print(f"  FAILED after {args.retries} retries "
                  f"(watch in receive mode? URI valid?)\n")
    return 0


def _validate_len_and_shape(uri: str, label: str) -> bool:
    """Client-side length + shape guard; prints a warning and returns False if the
    URI can't be sent (too long) or clearly isn't a TOTP URI."""
    n = len(uri.encode('utf-8'))
    if n > MAX_URI_LEN:
        print(f"  {label}: SKIP — URI is {n} bytes, over the watch's "
              f"{MAX_URI_LEN}-byte limit")
        return False
    if not looks_like_totp_uri(uri):
        print(f"  {label}: WARNING — does not look like an "
              f"'{URI_PREFIX}...?secret=...' URI; the watch will likely drop it")
        # Not fatal: still let the user send it (the watch is the authority), but
        # only in interactive mode is a mistake cheap. Return True so it's sent.
    return True


def main():
    p = argparse.ArgumentParser(
        description="Send otpauth TOTP URIs to the watch over IR "
                    "(pairs with totp_lfs_face receive mode).")
    p.add_argument('--file', metavar='PATH',
                   help="Batch mode: a file with one otpauth URI per line. "
                        "Omit for interactive mode.")
    p.add_argument('--device', default='/dev/ttyACM0', help="Modem serial device.")
    p.add_argument('--baud', type=int, default=DEFAULT_DATA_BAUD,
                   help=f"Data baud, host->watch (default {DEFAULT_DATA_BAUD}; "
                        "must match the watch face's RX baud).")
    p.add_argument('--ack-baud', type=int, default=DEFAULT_ACK_BAUD,
                   help=f"ACK baud, watch->host (default {DEFAULT_ACK_BAUD}; "
                        "must match the watch face's TX baud).")
    p.add_argument('--encoding', choices=['irda', 'nrz'], default='nrz',
                   help="Optical encoding (default nrz; must match the watch face).")
    p.add_argument('--digital-rx', action='store_true',
                   help="Use the modem's digital edge RX instead of analog polled "
                        "RX (analog is XIAO SAMD21 only; the UNO ignores it).")
    p.add_argument('--timeout', type=float, default=0.5,
                   help="Per-attempt ACK wait, seconds (default 0.5).")
    p.add_argument('--retries', type=int, default=50,
                   help="Max retransmissions per URI before giving up (default 50).")
    p.add_argument('--settle', type=float, default=0.0,
                   help="Extra pause between URIs, seconds (default 0.0; the "
                        "watch's ACK settle usually suffices).")
    args = p.parse_args()

    if args.retries < 0:
        sys.exit("error: --retries must be >= 0")

    global ser, modem
    atexit.register(_shutdown)
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    try:
        ser = serial.Serial(args.device, baudrate=HANDSHAKE_BAUD, timeout=0.1)
    except serial.SerialException as e:
        sys.exit(f"error: cannot open {args.device}: {e}")
    print(f"opened {args.device}; waiting {RESET_DELAY_S:.1f}s for Arduino reset...")
    time.sleep(RESET_DELAY_S)
    ser.reset_input_buffer()
    modem = Modem(ser)
    handshake(modem, args.baud, args.ack_baud, args.encoding,
              RX_DIGITAL if args.digital_rx else RX_ANALOG)
    print("modem ready. Put the watch on totp_lfs and long-press Alarm to receive.\n")

    if args.file:
        rc = run_batch(modem, args.file, args)
    else:
        rc = run_interactive(modem, args)
    sys.exit(rc)


if __name__ == '__main__':
    main()

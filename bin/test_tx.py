#!/usr/bin/env python3
"""test_tx: transmit a few frames of random payload over the IR link, the host
homologue of the watch's ir_tx_face.

Companion to the modem firmware (src/modem_*.cpp); the SAME firmware the
flasher and test_rx use. Each frame is wrapped in a serial frame (the shared
serial_frame.build_frame, == the watch's serial_frame.c) with a running id and
sent fire-and-forget (no ACK expected): the modem transmits it and replies
MSG_TX_DONE, which we wait for before sending the next so its USB buffer can't
overflow mid-transmit.

Pair it with test_rx.py on a second modem (matching --baud / --encoding) to watch
the frames arrive and validate.

Flash any modem env, e.g.:  pio run -e modem_arduino_uno -t upload

Usage:
    test_tx.py                              # 8 frames, 256 B each, 3600 baud, NRZ
    test_tx.py --count 20 --payload-size 64
    test_tx.py --baud 3600 --encoding irda --gap 0.1 --seed 1
"""

import argparse
import atexit
import random
import signal
import sys
import time

from ir_modem import (Modem, handshake, HANDSHAKE_BAUD, RESET_DELAY_S,
                      CMD_TX, CMD_STOP, MSG_TX_DONE, MSG_ERR)
from serial_frame import build_frame, MAX_PAYLOAD

try:
    import serial
except ImportError:
    sys.exit("error: pyserial not installed.  pipx install pyserial  (or pip install pyserial)")

# Module-global so the atexit/signal handlers can find the open port + modem.
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
    sys.exit(0)


def main():
    global ser, modem

    p = argparse.ArgumentParser(
        description="Transmit frames of random payload over the IR link via the "
                    "modem firmware (host homologue of the watch's ir_tx_face).",
        epilog="Flash any modem env, e.g.: pio run -e modem_arduino_uno -t upload",
    )
    p.add_argument('--baud', type=int, default=3600,
                   help="TX baud rate, 50..9600. Default: 3600 (matches ir_rx_face)")
    p.add_argument('--encoding', choices=['irda', 'nrz'], default='nrz',
                   help="Line encoding. Default: nrz")
    p.add_argument('--count', type=int, default=8,
                   help="Number of frames to send. Default: 8")
    p.add_argument('--payload-size', type=int, default=256,
                   help=f"Random payload bytes per frame, 0..{MAX_PAYLOAD}. Default: 256")
    p.add_argument('--gap', type=float, default=0.0,
                   help="Seconds to wait between frames. Default: 0.0")
    p.add_argument('--seed', type=int, default=None,
                   help="Seed the payload RNG for reproducible frames. Default: random")
    p.add_argument('--device', default='/dev/ttyACM0',
                   help="Serial device. Default: /dev/ttyACM0")
    args = p.parse_args()

    if not (50 <= args.baud <= 9600):
        sys.exit(f"error: --baud must be in [50, 9600], got {args.baud}")
    if not (0 <= args.payload_size <= MAX_PAYLOAD):
        sys.exit(f"error: --payload-size must be in [0, {MAX_PAYLOAD}], got {args.payload_size}")
    if args.count < 1:
        sys.exit(f"error: --count must be >= 1, got {args.count}")

    rng = random.Random(args.seed)

    try:
        ser = serial.Serial(args.device, baudrate=HANDSHAKE_BAUD, timeout=0.1)
    except serial.SerialException as e:
        sys.exit(f"error: cannot open {args.device}: {e}")

    atexit.register(_shutdown)
    signal.signal(signal.SIGINT,  _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    print(f"opened {args.device}; waiting {RESET_DELAY_S:.1f}s for Arduino reset...",
          file=sys.stderr)
    time.sleep(RESET_DELAY_S)
    ser.reset_input_buffer()
    modem = Modem(ser)

    # We only transmit, so rx_baud is moot; pass the tx baud for both.
    handshake(modem, args.baud, args.baud, args.encoding)
    print(f"sending {args.count} frame(s), {args.payload_size} B each, "
          f"baud={args.baud} encoding={args.encoding}; Ctrl-C to stop.",
          file=sys.stderr)

    sent = 0
    for i in range(args.count):
        payload = bytes(rng.getrandbits(8) for _ in range(args.payload_size))
        frame = build_frame(payload, frame_id=i & 0xFFFF, flags=0)
        # tx_flags=0: fire-and-forget (no AUTO_RX). The modem goes idle after TX.
        modem.send(CMD_TX, b'\x00' + frame)

        # Wait for MSG_TX_DONE before queuing the next frame: TX is blocking on
        # the modem, so sending early could overflow its USB RX buffer.
        tx_estimate = len(frame) * 10 / args.baud
        deadline = time.time() + tx_estimate + 1.0
        done = False
        while time.time() < deadline:
            msg = modem.read_message(deadline)
            if msg is None:
                break
            mtype, mpayload = msg
            if mtype == MSG_TX_DONE:
                done = True
                break
            if mtype == MSG_ERR:
                code = mpayload[0] if mpayload else 0
                sys.exit(f"error: modem reported error code {code}")
        if not done:
            print(f"  frame {i}: no MSG_TX_DONE (modem stuck?)", file=sys.stderr)
            break

        sent += 1
        print(f"sent frame id={i} len={args.payload_size} "
              f"({sent}/{args.count}) {payload.hex(' ')}", flush=True)
        if args.gap:
            time.sleep(args.gap)

    print(f"done: {sent}/{args.count} frame(s) transmitted.", file=sys.stderr)


if __name__ == '__main__':
    main()

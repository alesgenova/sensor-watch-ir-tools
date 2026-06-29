#!/usr/bin/env python3
"""test_rx: listen on the IR link via the modem firmware and stream what the
watch sends, either as validated frames (+ stats) or as a raw byte dump.

Companion to the modem firmware (src/modem_*.cpp); the SAME firmware the
flasher uses. The modem is a dumb byte pipe: this tool puts it into RX with
CMD_RX and forwards every decoded optical byte back as MSG_RX. All decoding lives
here on the host: frame mode runs the shared serial_frame.FrameParser (a port of
the watch's serial_frame.c), so frame validation and the per-second stats that
the old dedicated receiver firmware produced on-device are computed here instead.

Flash any modem env, e.g.:  pio run -e modem_arduino_uno -t upload

Usage:
    test_rx.py                      # defaults: 300 baud, NRZ, frame mode
    test_rx.py --baud 3600 --encoding irda
    test_rx.py --mode raw --raw-output capture.bin

On Ctrl-C / kill, it sends CMD_STOP so the modem leaves RX and idles.
"""

import argparse
import atexit
import signal
import sys
import time

from ir_modem import (Modem, handshake, HANDSHAKE_BAUD, RESET_DELAY_S,
                     CMD_RX, CMD_STOP, MSG_OK, MSG_RX, MSG_ERR, MSG_RX_STATUS,
                     MSG_RX_TUNE, RX_DIGITAL, RX_ANALOG)
from serial_frame import FrameParser

try:
    import serial
except ImportError:
    sys.exit("error: pyserial not installed.  pipx install pyserial  (or pip install pyserial)")

# Module-global so the atexit/signal handlers can find the open port + modem.
ser = None
modem = None


def _shutdown():
    """Tell the modem to leave RX (CMD_STOP), then close the port. Idempotent."""
    global ser, modem
    if ser is None:
        return
    try:
        if ser.is_open:
            if modem is not None:
                modem.send(CMD_STOP)
                time.sleep(0.05)   # let the modem act on it before we close
            ser.close()
    except Exception:
        pass
    ser = None


def _signal_handler(signum, _frame):
    # Raising SystemExit triggers atexit -> _shutdown, which unwinds cleanly
    # even if we're parked in ser.read().
    sys.exit(0)


def main():
    global ser, modem

    p = argparse.ArgumentParser(
        description="Listen on the IR link via the modem firmware and stream "
                    "decoded frames / bytes.",
        epilog="Flash any modem env, e.g.: pio run -e modem_arduino_uno -t upload",
    )
    p.add_argument('--baud', type=int, default=300,
                   help="RX baud rate, 50..9600. Default: 300 (matches ir_tx_face)")
    p.add_argument('--encoding', choices=['irda', 'nrz'], default='nrz',
                   help="Line encoding. Default: nrz")
    p.add_argument('--digital-rx', action='store_true',
                   help="Use digital edge-detection RX instead of the default "
                        "analog polled RX (for strong, clean signals).")
    p.add_argument('--analog-rx-debug', action='store_true',
                   help="In analog RX mode (the default), print the [rx-tune] "
                        "data-slicer diagnostics. Off by default (quiet).")
    p.add_argument('--mode', choices=['frame', 'raw'], default='frame',
                   help="frame: parse + validate framed payloads (with stats). "
                        "raw: dump every decoded byte (to --raw-output, or "
                        "hex-dumped to stdout). Default: frame")
    p.add_argument('--raw-output', type=argparse.FileType('wb'),
                   help="In raw mode, append received bytes to this file as "
                        "binary. Without this, raw mode hex-dumps to stdout.")
    p.add_argument('--device', default='/dev/ttyACM0',
                   help="Serial device. Default: /dev/ttyACM0")
    p.add_argument('--quiet-stats', action='store_true',
                   help="In frame mode, suppress the per-second STATS line "
                        "(still prints frames).")
    args = p.parse_args()

    if not (50 <= args.baud <= 9600):
        sys.exit(f"error: --baud must be in [50, 9600], got {args.baud}")

    try:
        ser = serial.Serial(args.device, baudrate=HANDSHAKE_BAUD, timeout=0.1)
    except serial.SerialException as e:
        sys.exit(f"error: cannot open {args.device}: {e}")

    # Make sure cleanup runs on normal exit, Ctrl-C, or kill.
    atexit.register(_shutdown)
    signal.signal(signal.SIGINT,  _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    print(f"opened {args.device}; waiting {RESET_DELAY_S:.1f}s for Arduino reset...",
          file=sys.stderr)
    time.sleep(RESET_DELAY_S)
    ser.reset_input_buffer()
    modem = Modem(ser)

    # PING + CONFIG, then enter RX. We only ever receive, so tx_baud is moot;
    # pass the rx baud for both (the modem only uses tx_baud on a CMD_TX).
    handshake(modem, args.baud, args.baud, args.encoding,
              RX_DIGITAL if args.digital_rx else RX_ANALOG)
    modem.send(CMD_RX)
    msg = modem.read_message(time.time() + 3.0)
    if not msg or msg[0] != MSG_OK:
        sys.exit(f"error: modem did not enter RX (got {msg!r})")

    print(f"listening: baud={args.baud} encoding={args.encoding} "
          f"rx={'digital' if args.digital_rx else 'analog'} "
          f"mode={args.mode}; Ctrl-C to stop.", file=sys.stderr)

    if args.mode == 'raw':
        _stream_raw(args)
    else:
        _stream_frames(args)


def _print_rx_status(payload):
    """DIAGNOSTIC: surface the samd21 modem's per-session decode error counts."""
    if len(payload) >= 4:
        ferr   = payload[0] | (payload[1] << 8)
        bufovf = payload[2] | (payload[3] << 8)
        print(f"[rx-status] ferr={ferr} bufovf={bufovf}", file=sys.stderr, flush=True)


def _print_rx_tune(payload):
    """DIAGNOSTIC: surface the analog RX data-slicer state (envelopes/threshold)."""
    if len(payload) >= 12:
        dark  = payload[0] | (payload[1] << 8)   # high (dark) envelope
        light = payload[2] | (payload[3] << 8)   # low (light) envelope
        span  = payload[4] | (payload[5] << 8)   # live envelope span (dark - light)
        thr   = payload[6] | (payload[7] << 8)   # slice midpoint
        mc    = payload[8] | (payload[9] << 8)   # squelch floor
        ovs   = payload[10]
        locked = bool(payload[11] & 1)
        print(f"[rx-tune] dark={dark} light={light} span={span} thr={thr} "
              f"min_contrast={mc} oversample={ovs}x "
              f"{'LOCKED' if locked else 'squelched'}", file=sys.stderr, flush=True)


def _stream_raw(args):
    """Dump every decoded byte. MSG_RX payloads are pure binary (no ASCII
    status mixed in), so unlike the old receiver this is a clean stream."""
    out = args.raw_output
    while True:
        msg = modem.read_message(time.time() + 0.2)
        if msg is None:
            continue
        mtype, payload = msg
        if mtype == MSG_RX:
            if out is not None:
                out.write(payload)
                out.flush()
            else:
                sys.stdout.write(payload.hex(' ') + '\n')
                sys.stdout.flush()
        elif mtype == MSG_RX_STATUS:
            _print_rx_status(payload)
        elif mtype == MSG_RX_TUNE:
            if args.analog_rx_debug:
                _print_rx_tune(payload)
        elif mtype == MSG_ERR:
            code = payload[0] if payload else 0
            print(f"modem error code {code}", file=sys.stderr)


def _stream_frames(args):
    """Feed decoded bytes to the shared FrameParser; print valid frames and a
    per-second STATS line (unless --quiet-stats)."""
    parser = FrameParser()
    last_stats = time.time()
    while True:
        msg = modem.read_message(time.time() + 0.2)
        now = time.time()
        if msg is not None:
            mtype, payload = msg
            if mtype == MSG_RX:
                for f in parser.feed(payload):
                    print(f"F id=0x{f.id:04X} len={f.length} flags=0x{f.flags:02X} "
                          f"{f.payload.hex(' ')}", flush=True)
            elif mtype == MSG_RX_STATUS:
                _print_rx_status(payload)
            elif mtype == MSG_RX_TUNE:
                if args.analog_rx_debug:
                    _print_rx_tune(payload)
            elif mtype == MSG_ERR:
                code = payload[0] if payload else 0
                print(f"modem error code {code}", file=sys.stderr)
        if not args.quiet_stats and now - last_stats >= 1.0:
            print(f"STATS bytes={parser.bytes_total} valid={parser.frames_valid} "
                  f"bad_crc={parser.frames_bad_crc} bad_len={parser.frames_bad_length}",
                  file=sys.stderr, flush=True)
            last_stats = now


if __name__ == '__main__':
    main()

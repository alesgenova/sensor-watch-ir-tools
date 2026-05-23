#!/usr/bin/env python3
"""firmware_flasher: flash a UF2 image to the watch over IR, frame by frame.

Companion to the `modem_arduino_uno` firmware (src/modem_arduino_uno.cpp). The modem
is a dumb half-duplex byte pipe; ALL of the protocol lives here:

  * Parse the UF2 file into per-block (targetAddr, firmware data) payloads.
  * Wrap each payload in a serial frame (preamble + id + len/flags +
    padded payload + CRC32), matching watch-library/shared/utils/serial_frame.c.
  * Link test (--test-blocks): send the first M blocks with the TEST flag,
    mirroring the watch's flash TEST mode. Each is retried up to --retries times
    and the run aborts if one still can't be ACKed: a reliability gate before
    committing to a real flash. The watch only ACKs these; it does not commit them.
  * Stop-and-wait: send one frame, then wait for the watch's acknowledgement,
    which is the bare 2-byte frame id (little-endian) echoed back. If it
    doesn't arrive within the timeout, retransmit the same frame.
  * Print progress over stdout.

The modem USB command/message protocol is documented in bin/ir_modem.py and the
frame wire format in bin/serial_frame.py (both shared with bin/test_rx.py).

The watch side: the firmware_flasher_face receives a frame, validates its CRC,
and (in the real flash stage) commits it, then replies with the bare 2-byte id.
A corrupted reply simply fails to match the id we're waiting for, so we retry.
"""

import argparse
import io
import os
import struct
import sys
import time
import zlib

try:
    import serial
except ImportError:
    sys.exit("error: pyserial not installed.  pipx install pyserial  (or pip install pyserial)")

# Modem USB link (command/message protocol + the Modem class and handshake) and
# the frame wire format live in shared modules alongside this script.
from ir_modem import (
    Modem, handshake, HANDSHAKE_BAUD, RESET_DELAY_S,
    CMD_TX, CMD_STOP, TX_FLAG_AUTO_RX,
    MSG_RX, MSG_TX_DONE, MSG_ERR,
)
from serial_frame import build_frame, MAX_PAYLOAD

# Frame flag bits; MUST match watch firmware_flasher_ramfunc.h.
# (Bit 1 is unused: the old LAST_FRAME marker is gone. The watch stays in TEST
# until ENTER, and the EXIT frame terminates the data stream.)
FLAG_TEST        = 1 << 2   # link-test frame: watch ACKs but does NOT commit it
FLAG_ENTER       = 1 << 3   # enter the real flasher; EMPTY payload (full flash), or
                            # with FLAG_PATCH+FLAG_VERIFY = the 20-byte patch header
FLAG_VERIFY      = 1 << 4   # EXIT: final whole-image CRC check; payload = 12-byte descriptor
FLAG_PATCH       = 1 << 5   # delta (patch) flash, not a verbatim uf2-like one: on ENTER
                            # = patch session (+header); on a data frame = a patch chunk

# Real-flash protocol constants; MUST match the watch.
ROW_SIZE         = 256              # one NVMCTRL row = one data block's firmware bytes
APP_FLASH_END    = 0x3E000         # exclusive; the reserved EEPROM region begins here
FRAME_ID_ENTER   = 0xFFFF          # distinct ids for the control frames so they never
FRAME_ID_EXIT    = 0xFFFE          # collide with a data block's running counter (0..N-1)

# --- UF2 format -------------------------------------------------------------
UF2_BLOCK_SIZE          = 512
UF2_MAGIC0              = 0x0A324655
UF2_MAGIC1              = 0x9E5D5157
UF2_MAGIC_END          = 0x0AB16F30
UF2_FLAG_NOT_MAIN_FLASH = 0x00000001
BOOTLOADER_END          = 0x2000   # SAM L22 UF2 bootloader occupies [0, 0x2000)

# Defaults mirror the watch's firmware_flasher_face: rx 3600 (firmware in),
# tx 300 (ACK out), NRZ. Here "data baud" is what the modem transmits at
# (= the watch's rx baud) and "ack baud" is what the modem receives at
# (= the watch's tx baud).
DEFAULT_DATA_BAUD = 3600
DEFAULT_ACK_BAUD  = 300


def parse_uf2(data: bytes):
    """Return a list of (block_no, target_addr, payload) for main-flash blocks.

    payload = targetAddr(4 LE) || firmware data (payloadSize bytes).
    """
    if len(data) % UF2_BLOCK_SIZE != 0:
        sys.exit(f"error: not a UF2 file (size {len(data)} not a multiple of {UF2_BLOCK_SIZE})")
    blocks = []
    n = len(data) // UF2_BLOCK_SIZE
    for i in range(n):
        blk = data[i * UF2_BLOCK_SIZE:(i + 1) * UF2_BLOCK_SIZE]
        (magic0, magic1, flags, target_addr, payload_size,
         block_no, num_blocks, _famsize) = struct.unpack('<8I', blk[:32])
        magic_end = struct.unpack('<I', blk[508:512])[0]
        if magic0 != UF2_MAGIC0 or magic1 != UF2_MAGIC1 or magic_end != UF2_MAGIC_END:
            sys.exit(f"error: UF2 block {i} has bad magic")
        if flags & UF2_FLAG_NOT_MAIN_FLASH:
            continue   # skip metadata blocks (e.g. file containers)
        if payload_size > 476:
            sys.exit(f"error: UF2 block {i} payloadSize {payload_size} > 476")
        fw = blk[32:32 + payload_size]
        payload = struct.pack('<I', target_addr) + fw
        if len(payload) > MAX_PAYLOAD:
            sys.exit(f"error: block {i} frame payload {len(payload)} > {MAX_PAYLOAD}; "
                     f"reduce UF2 payloadSize")
        blocks.append((block_no, target_addr, payload))
    return blocks


DEBUG = False   # set from --debug; dumps raw optical bytes during the ACK wait


def send_frame_and_wait_ack(modem: Modem, frame: bytes, frame_id: int,
                            data_baud: int, ack_timeout: float) -> bool:
    """Send one frame, then wait for the 2-byte id echo. True on ACK."""
    expected = struct.pack('<H', frame_id)
    # Flasher always wants the reply, so AUTO_RX (flip to RX) on every frame.
    modem.send(CMD_TX, bytes((TX_FLAG_AUTO_RX,)) + frame)

    # Overall ceiling: time to clock the frame out at data_baud, plus the ack
    # window. The tight ack window (re)starts when MSG_TX_DONE arrives.
    tx_estimate = len(frame) * 10 / data_baud
    deadline = time.time() + tx_estimate + ack_timeout + 1.0
    rx_log = bytearray()      # everything the modem decoded this attempt (debug)
    window = bytearray()      # rolling tail of recently received optical bytes
    while time.time() < deadline:
        msg = modem.read_message(deadline)
        if msg is None:
            break
        mtype, payload = msg
        if mtype == MSG_TX_DONE:
            deadline = time.time() + ack_timeout      # start the tight ack window
        elif mtype == MSG_RX:
            if DEBUG:
                rx_log += payload
            window += payload
            if expected in window:
                if DEBUG:
                    print(f"    [debug] id {frame_id}: ACK matched; "
                          f"rx so far = {rx_log.hex(' ')}")
                return True
            del window[:-8]                           # keep only the last bytes
        elif mtype == MSG_ERR:
            code = payload[0] if payload else 0
            sys.exit(f"error: modem reported error code {code}")
    if DEBUG:
        print(f"    [debug] id {frame_id}: no ACK (want {expected.hex(' ')}); "
              f"modem decoded {len(rx_log)} byte(s): {rx_log.hex(' ') or '<none>'}")
    return False


def send_with_retries(modem: Modem, frame: bytes, frame_id: int, data_baud: int,
                      timeout: float, retries, label: str):
    """Send a frame and wait for its ACK, retransmitting until it ACKs.

    `retries` is the max number of retransmissions, or None for unlimited (used
    for data blocks once flashing has begun; see send_blocks). Returns the
    number of retransmissions used (0 = ACKed on the first try), or None if a
    finite `retries` was exhausted without an ACK.
    """
    attempt = 0
    while retries is None or attempt <= retries:
        if attempt > 0:
            cap = "∞" if retries is None else retries
            print(f"  {label} no ACK, retransmit {attempt}/{cap}")
        if send_frame_and_wait_ack(modem, frame, frame_id, data_baud, timeout):
            return attempt
        attempt += 1
    return None


def run_test_stage(modem: Modem, payloads, noun: str, args):
    """Link-reliability gate: send each payload as a TEST-flagged frame.

    These frames exercise the optical link end to end but are NOT committed by
    the watch. It's in its test phase and only ACKs them, staying there until
    the ENTER frame later moves it on. Each frame is retransmitted up to
    `args.retries` times, exactly like a real flash; the test fails only if one
    still can't be ACKed after that. Retransmit counts are reported so you can
    judge link quality before committing. `noun` ("block"/"frame") is cosmetic:
    a full flash tests with UF2 block payloads, a patch with crle chunks.
    """
    m = len(payloads)
    print(f"link test: {m} {noun}(s) with TEST flag (up to {args.retries} retries each)...")
    total_retx = 0
    for i, payload in enumerate(payloads):
        if i > 0:
            time.sleep(args.settle)   # let the watch reopen RX after the prev ACK
        frame_id = i & 0xFFFF
        frame = build_frame(payload, frame_id, FLAG_TEST)
        label = f"test {noun} {i + 1}/{m} (id {frame_id})"
        retx = send_with_retries(modem, frame, frame_id, args.baud, args.timeout,
                                 args.retries, label)
        if retx is None:
            sys.exit(f"error: link test FAILED: {label} no ACK after {args.retries} retries")
        total_retx += retx
        print(f"  {label} ACKed" + (f" (after {retx} retx)" if retx else ""))
    print(f"link test passed ({total_retx} retransmission(s) total).")


def send_enter(modem: Modem, enter_frame: bytes, args, label: str = "ENTER"):
    """Arm the watch: transmit the ENTER frame, retransmitting FOREVER until it
    ACKs (the watch re-ACKs repeats while it waits). The watch ACKs only once
    it's ready (and a *patch* ENTER only if its flash matches the reference),
    so a never-arriving ACK means it isn't responding, or the reference doesn't
    match: watch for the reply blink and Ctrl-C if it never comes."""
    send_with_retries(modem, enter_frame, FRAME_ID_ENTER, args.baud, args.timeout,
                      None, label)   # None = retry until ACKed


def stream_frames(modem: Modem, items, args, noun: str):
    """Stream data frames stop-and-wait, each retransmitted FOREVER until ACKed
    (once flashing has begun the watch stays in its flasher; Ctrl-C aborts a
    dead link). `items` is a list of (frame_id, frame_bytes, label); `noun`
    labels the progress line. Returns the total retransmission count."""
    total = len(items)
    t0 = time.time()
    last_report = t0
    total_retx = 0
    for i, (frame_id, frame, label) in enumerate(items):
        if i > 0:
            time.sleep(args.settle)            # let the watch reopen RX after the prev ACK
        total_retx += send_with_retries(modem, frame, frame_id, args.baud, args.timeout,
                                        None, label)   # None = retry until ACKed
        now = time.time()
        if now - last_report >= 1.0 or i == total - 1:
            pct = 100.0 * (i + 1) / total
            rate = (i + 1) / (now - t0) if now > t0 else 0
            eta = (total - i - 1) / rate if rate > 0 else 0
            print(f"  {i + 1}/{total} ({pct:5.1f}%)  {rate:.1f} {noun}/s  "
                  f"retx {total_retx}  ETA {eta:.0f}s")
            last_report = now
    print(f"  sent {total} {noun}(s) ({total_retx} retransmission(s)).")
    return total_retx


def send_exit_verify(modem: Modem, base: int, size: int, crc: int, args) -> bool:
    """Send the EXIT frame (the whole-image descriptor {base, size, crc} with
    FLAG_VERIFY) and wait for its ACK. The watch re-reads the flashed range and,
    only if its CRC matches, echoes this id and reboots into the new firmware.
    Returns True on ACK (verify OK). False means either the CRC did not match
    (the watch is still in its flasher) or it verified and already rebooted with
    the echo lost. The caller decides how to recover."""
    print("requesting EXIT / final verify...")
    exit_frame = build_frame(struct.pack('<III', base, size, crc),
                             FRAME_ID_EXIT, FLAG_VERIFY)
    if send_with_retries(modem, exit_frame, FRAME_ID_EXIT, args.baud, args.timeout,
                         args.retries, "EXIT") is not None:
        print("verify OK; watch is rebooting into the new firmware. done.")
        return True
    return False


# --- real flash stage -------------------------------------------------------

def assemble_image(blocks):
    """Turn the parsed UF2 blocks into the gap-free, row-aligned image the watch
    will verify. Returns (base, total_length, image_crc32, rows) where rows is a
    list of (target_addr, 256-byte row).

    Each block becomes exactly one NVMCTRL row (firmware bytes zero-padded to
    256). The rows must tile [base, base+N*256) with no gaps; the whole-image
    CRC the watch recomputes by reading the flash back covers that whole region,
    so a hole (an un-sent row left at its erased/old contents) would never match.
    """
    rows = []
    for _bn, addr, payload in blocks:
        fw = payload[4:]                       # payload = targetAddr(4 LE) + firmware
        if len(fw) > ROW_SIZE:
            sys.exit(f"error: block at 0x{addr:08X} carries {len(fw)} bytes > one "
                     f"{ROW_SIZE}-byte row; rebuild the UF2 with 256-byte blocks")
        if addr % ROW_SIZE != 0:
            sys.exit(f"error: block target 0x{addr:08X} is not {ROW_SIZE}-aligned")
        rows.append((addr, fw + b'\x00' * (ROW_SIZE - len(fw))))

    rows.sort(key=lambda r: r[0])
    base = rows[0][0]
    for i, (addr, _row) in enumerate(rows):
        expected = base + i * ROW_SIZE
        if addr != expected:
            sys.exit(f"error: image is not contiguous; expected a block at "
                     f"0x{expected:08X} but got 0x{addr:08X}. The whole-image CRC "
                     f"needs gap-free coverage; this UF2 has a hole.")

    total_length = len(rows) * ROW_SIZE
    if base < BOOTLOADER_END or base + total_length > APP_FLASH_END:
        sys.exit(f"error: image [0x{base:08X}, 0x{base+total_length:08X}) is outside "
                 f"writable app flash [0x{BOOTLOADER_END:08X}, 0x{APP_FLASH_END:08X})")

    image = b''.join(row for _addr, row in rows)
    image_crc = zlib.crc32(image) & 0xFFFFFFFF
    return base, total_length, image_crc, rows


# --- patch (delta) flash --------------------------------------------------- *
#
# When --reference is given we diff the new UF2 against the reference (the
# firmware currently on the watch) with detools (in-place, crle, the only
# decompressor the watch has), strip the detools header into the patch ENTER, and
# stream the compressed body as FLAG_PATCH frames. The watch verifies its flash
# matches the reference before applying, reconstructs NEW in place, then the EXIT
# CRC gates the reboot. See the watch-side firmware_flasher_ramfunc.c.

def _image_bytes(uf2_path):
    """Parse a UF2 -> (base, image_bytes, image_crc32) via the same gap-free,
    row-aligned assembly the watch's flash holds (so ref_crc matches)."""
    with open(uf2_path, 'rb') as f:
        blocks = parse_uf2(f.read())
    if not blocks:
        sys.exit(f"error: no main-flash blocks in {uf2_path}")
    low = [b for b in blocks if b[1] < BOOTLOADER_END]
    if low:
        sys.exit(f"error: {uf2_path} has block(s) in the bootloader region")
    base, _total, image_crc, rows = assemble_image(blocks)
    return base, b''.join(row for _addr, row in rows), image_crc


def _ceil_row(x):
    return ((x + ROW_SIZE - 1) // ROW_SIZE) * ROW_SIZE


def _build_patch(detools, ref_image, new_image, memory_size):
    fp = io.BytesIO()
    detools.create_patch(io.BytesIO(ref_image), io.BytesIO(new_image), fp,
                         compression='crle', patch_type='in-place',
                         memory_size=memory_size, segment_size=ROW_SIZE)
    return fp.getvalue()


def patch_geometry(base, ref_len, new_len):
    """Row-aligned sizes that bound the patch's shift_size:
       from_r/to_r  - REF/NEW lengths rounded up to a row
       growth_floor - to_r - from_r (the minimum shift: NEW must fit beside REF)
       min_shift    - max(growth_floor, 2 rows) (also detools' own floor)
       max_window   - REF shifted all the way to the writable-region end."""
    from_r = _ceil_row(ref_len)
    to_r   = _ceil_row(new_len)
    growth_floor = max(0, to_r - from_r)
    min_shift = max(growth_floor, 2 * ROW_SIZE)
    region = (APP_FLASH_END - base) - ((APP_FLASH_END - base) % ROW_SIZE)
    max_window = max(region - from_r, min_shift)
    return from_r, growth_floor, min_shift, max_window


def build_patch(detools, ref_image, new_image, from_r, window):
    """Build the crle in-place patch with the given aux window (= shift_size). The
    watch needs ~next_pow2(window) bytes of RAM to apply it (aux path), or (at the
    max window) must copy-shift the reference in flash. Returns
    (shift_size, from_size, to_size, body, total_patch_len)."""
    from detools.apply import read_header_in_place
    patch = _build_patch(detools, ref_image, new_image, from_r + window)
    f = io.BytesIO(patch)
    _comp, _mem, _seg, shift_size, from_size, to_size = read_header_in_place(f)
    return shift_size, from_size, to_size, f.read(), len(patch)


def send_blocks(modem: Modem, send_rows, args):
    """Send the selected rows as data blocks (targetAddr + 256), stop-and-wait.
    Each row is (abs_idx, addr, row); the frame id is the absolute index. The
    data stream is terminated by the EXIT frame, so no block needs a "last" flag."""
    total = len(send_rows)
    items = [(idx & 0xFFFF,
              build_frame(struct.pack('<I', addr) + row, idx & 0xFFFF, 0),
              f"block {idx} (id {idx & 0xFFFF})  [{i + 1}/{total}]")
             for i, (idx, addr, row) in enumerate(send_rows)]
    return stream_frames(modem, items, args, "block")


def flash_stage(modem: Modem, base, total_length, image_crc, send_rows, args,
                *, arm, restartable):
    """Drive the real flash: optional ENTER (arm) -> the selected blocks ->
    EXIT. The EXIT frame carries the whole-image descriptor and is always sent
    (it doubles as the end-to-end test in a dry run: the watch fakes a pass and
    reboots). On a failed final check the watch stays in its RAM flasher (it never
    reboots into a half-written image).

    `arm`: send the ENTER frame first; true when the watch is still in its TEST
    stage (a fresh full flash), false when it's already in the flasher (a resume,
    or the patch->full fallback). `restartable`: on a failed verify, may re-stream
    from block 0 (the watch's 'id 0 = restart' rule) and so offer a resend; true
    for any flash whose send_rows start at id 0, false for a resume (which starts
    mid-stream and can't cleanly restart)."""
    if arm:
        # ENTER (empty payload) drops the watch from its test stage into the RAM
        # flasher; only then is it safe to send the first block.
        send_enter(modem, build_frame(b'', FRAME_ID_ENTER, FLAG_ENTER), args)
        print("flasher armed.")

    while True:
        send_blocks(modem, send_rows, args)

        if send_exit_verify(modem, base, total_length, image_crc, args):
            return True

        print("\nNo EXIT ACK. Either the whole-image CRC did not match (the "
              "watch is still in its flasher, awaiting another attempt), or it "
              "verified OK and already rebooted into the new firmware, in which "
              "case it is fine; just check the watch.")
        if not restartable:
            # A resume (start>0): we can't cleanly re-stream from a mid-image id
            # (the watch only accepts last_id+1 or a restart at id 0). Recover by
            # running a fresh full flash from block 0.
            print("To retry, run a fresh full flash from block 0 (re-arm the watch).")
            return False
        try:
            ans = input("Resend the entire firmware from the start? [y/N] ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            ans = 'n'
        if ans != 'y':
            print("not resending. If the watch is unresponsive, use USB UF2 recovery.")
            return False
        print("resending firmware...")


def patch_flash(args, base, from_size, to_size, ref_crc, new_crc, shift_size, body,
                new_image):
    """Drive a delta-patch flash: handshake -> link test (patch frames) -> patch
    ENTER (the watch ACKs only if its flash matches the reference) -> stream the
    crle body -> EXIT (verify NEW + reboot). Mirrors the full-flash flow but with
    FLAG_PATCH frames and the patch ENTER.

    `new_image` is the full row-aligned new firmware (the same bytes the patch
    reconstructs). If the EXIT verify fails, the in-place patch has consumed the
    reference and can't be retried, but the watch is still in its flasher and we
    hold the whole image, so we offer to stream it as a full flash (block 0
    abandons the patch)."""
    chunks = [body[i:i + ROW_SIZE] for i in range(0, len(body), ROW_SIZE)] or [b'']
    num = len(chunks)
    if num > 0x10000:
        sys.exit("error: patch body too large for a 16-bit frame id")

    print(f"new:      {args.file}")
    print(f"ref:      {args.reference}")
    print(f"device:   {args.device}")
    print(f"patch:    {len(body)} body bytes -> {num} frame(s)  "
          f"(crle, in-place; vs {to_size} B full image)")
    print(f"window:   shift_size {shift_size} B (watch aux RAM, or in-flash shift "
          f"if it won't fit)   from {from_size} / to {to_size}")
    print(f"crc:      ref 0x{ref_crc:08X}  new 0x{new_crc:08X}")
    print(f"baud:     data {args.baud} / ack {args.ack_baud}  encoding {args.encoding}")

    try:
        ser = serial.Serial(args.device, baudrate=HANDSHAKE_BAUD, timeout=0.1)
    except serial.SerialException as e:
        sys.exit(f"error: cannot open {args.device}: {e}")
    print(f"opened; waiting {RESET_DELAY_S:.1f}s for Arduino reset...")
    time.sleep(RESET_DELAY_S)
    ser.reset_input_buffer()
    modem = Modem(ser)
    handshake(modem, args.baud, args.ack_baud, args.encoding)
    print("modem ready.")

    try:
        # Link-reliability test: first M body chunks flagged TEST (watch ACKs, does
        # not commit). Same gate as the full flash; needs the watch in TEST mode.
        m = min(args.test_blocks, num)
        if m:
            try:
                input("Put the watch in flash TEST mode (FLASH menu -> Alarm), then "
                      "press Enter to start the link test (Ctrl-C to abort)... ")
            except (EOFError, KeyboardInterrupt):
                print("\naborted"); modem.send(CMD_STOP); ser.close(); return
            run_test_stage(modem, chunks[:m], "frame", args)

        if args.test_only:
            print("test-only: done.")
            modem.send(CMD_STOP); ser.close(); return

        try:
            ans = input("POINT OF NO RETURN: the watch will be unresponsive until a "
                        "flash succeeds. Patch-flash now? [y/N] ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            ans = 'n'
        if ans != 'y':
            print("not flashing.")
            modem.send(CMD_STOP); ser.close(); return

        # Patch ENTER: {base, from_size, ref_crc, to_size, shift_size}. The watch
        # ACKs ONLY if its flash over [base,base+from_size) CRCs to ref_crc, so a
        # never-arriving ACK means the watch's firmware does not match --reference
        # (or the link is down). Watch the reply blink and Ctrl-C if it never comes.
        desc = struct.pack('<IIIII', base, from_size, ref_crc, to_size, shift_size)
        enter = build_frame(desc, FRAME_ID_ENTER, FLAG_ENTER | FLAG_PATCH | FLAG_VERIFY)
        print("sending patch ENTER (watch ACKs only if its flash matches the "
              "reference; Ctrl-C if it never ACKs: wrong --reference or dead link)...")
        send_enter(modem, enter, args, "ENTER(patch)")
        print("reference verified; flasher armed (point of no return on the watch "
              "once the first frame lands).")

        # Stream the crle body as FLAG_PATCH frames (ids 0..N-1).
        patch_items = [(i & 0xFFFF, build_frame(chunk, i & 0xFFFF, FLAG_PATCH),
                        f"frame {i} (id {i & 0xFFFF})  [{i + 1}/{num}]")
                       for i, chunk in enumerate(chunks)]
        stream_frames(modem, patch_items, args, "frame")

        # EXIT: {base, to_size, new_crc}. The watch CRCs the reconstructed image and,
        # only on a match, echoes this id and reboots into the new firmware.
        if not send_exit_verify(modem, base, to_size, new_crc, args):
            print("\nNo EXIT ACK. Either the reconstructed image's CRC did not match "
                  "(the watch is still in its flasher), or it verified and already "
                  "rebooted into the new firmware (echo lost), in which case it's fine.")
            # The patch is spent (applied in place, the reference is gone), so it
            # can't be retried, but the watch is still in its flasher and we hold
            # the whole new image. Offer to stream it as a full flash: block 0
            # makes the watch abandon the patch and start a fresh full reflash.
            try:
                ans = input("Fall back to a FULL flash of the new firmware now? [y/N] ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                ans = 'n'
            if ans == 'y':
                nfull = len(new_image) // ROW_SIZE
                fb_rows = [(i, base + i * ROW_SIZE, new_image[i * ROW_SIZE:(i + 1) * ROW_SIZE])
                           for i in range(nfull)]
                print(f"full flash: streaming {nfull} block(s) (block 0 abandons the patch)...")
                # arm=False: the watch is already in its flasher (no TEST stage, no
                # ENTER). restartable=True: these blocks start at id 0, so a failed
                # verify can re-stream from the top like any full flash.
                flash_stage(modem, base, len(new_image), new_crc, fb_rows, args,
                            arm=False, restartable=True)
            else:
                print("not falling back. If the watch is stuck, re-run with a full "
                      "flash (omit --reference).")
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        modem.send(CMD_STOP)
        ser.close()


def main():
    p = argparse.ArgumentParser(
        description="Flash a UF2 image to the watch over IR, frame by frame.")
    p.add_argument('file', help="Path to the UF2 file.")
    p.add_argument('--device', default='/dev/ttyACM0', help="Modem serial device.")
    p.add_argument('--baud', type=int, default=DEFAULT_DATA_BAUD,
                   help=f"Data baud: host->watch firmware bytes (the watch's RX "
                        f"baud). Default: {DEFAULT_DATA_BAUD}")
    p.add_argument('--ack-baud', type=int, default=DEFAULT_ACK_BAUD,
                   help=f"ACK baud: watch->host id echo (the watch's TX baud). "
                        f"Default: {DEFAULT_ACK_BAUD}")
    p.add_argument('--encoding', choices=['irda', 'nrz'], default='nrz',
                   help="IR encoding, both directions. Default: nrz")
    p.add_argument('--timeout', type=float, default=0.5,
                   help="Per-frame ACK timeout in seconds. Default: 0.5")
    p.add_argument('--retries', type=int, default=50,
                   help="Max retransmissions for a test/control frame before aborting "
                        "(data blocks are retried forever). Default: 50")
    p.add_argument('--test-blocks', type=int, default=4,
                   help="Send this many leading blocks as a TEST-flagged link "
                        "check before flashing; each is retried up to --retries times "
                        "and the run aborts if one still can't be ACKed. 0 disables. "
                        "Default: 4")
    p.add_argument('--test-only', action='store_true',
                   help="Stop after the link test; do not flash.")
    p.add_argument('--start-block', type=int, default=0, metavar='N',
                   help="Index of the first UF2 block to send (default 0). >0 means "
                        "RESUME: the watch must already be in its flasher from a prior "
                        "run with blocks 0..N-1 written; the ENTER (arm) frame and the "
                        "test stage are skipped.")
    p.add_argument('--count', type=int, default=None, metavar='N',
                   help="How many blocks to send, starting at --start-block (default: "
                        "all remaining). Use a small value to exercise the link quickly "
                        "without sending the whole image; the final VERIFY is still "
                        "requested (in a dry run the watch fakes a pass and reboots, "
                        "giving a full end-to-end test).")
    p.add_argument('--reference', metavar='REF.uf2',
                   help="The firmware CURRENTLY on the watch. If given, flash a delta "
                        "PATCH of <file> against it (much less to send) instead of the "
                        "whole image. The watch verifies its flash matches REF before "
                        "applying; falls back to a full flash if patching isn't viable.")
    p.add_argument('--patch-shift', type=int, default=None, metavar='BYTES',
                   help="Aux-window size (= detools shift_size, the watch RAM the patch "
                        "needs) for the interactive prompt's option 1. Clamped up to the "
                        "minimum the patch needs. Omit to default to 2048 B. Bigger = a "
                        "marginally smaller patch for more watch RAM.")
    p.add_argument('--debug', action='store_true',
                   help="Dump every raw byte the modem decodes during each ACK "
                        "wait (hex), and what id was expected. Use to see whether "
                        "the modem decodes nothing, wrong bytes, or the right id.")
    p.add_argument('--settle', type=float, default=0.0,
                   help="Delay (s) after an ACK before sending the next frame. "
                        "Default 0.0: not needed, because the watch-side ACK "
                        "settle handles the receiver-recovery timing. Raise it "
                        "only if you also lower the watch's ACK settle. Default: 0.0")
    args = p.parse_args()
    global DEBUG
    DEBUG = args.debug

    for b in (args.baud, args.ack_baud):
        if not (50 <= b <= 115200):
            sys.exit(f"error: baud must be in [50, 115200], got {b}")
    if args.test_blocks < 0:
        sys.exit("error: --test-blocks must be >= 0")
    if not os.path.isfile(args.file):
        sys.exit(f"error: file not found: {args.file}")

    # --- patch path (--reference): build the delta and, if it's worth it, flash it.
    if args.reference:
        if not os.path.isfile(args.reference):
            sys.exit(f"error: reference file not found: {args.reference}")
        if args.start_block:
            sys.exit("error: --start-block (resume) is not supported with --reference; "
                     "a patch is applied in place and cannot be restarted mid-stream.")
        try:
            import detools  # noqa: F401
        except ImportError:
            sys.exit("error: --reference patch flashing needs detools (pip install detools, "
                     "or use this repo's .venv/bin/python).")
        rbase, ref_img, ref_crc = _image_bytes(args.reference)
        nbase, new_img, new_crc = _image_bytes(args.file)
        if rbase != nbase:
            sys.exit(f"error: reference base 0x{rbase:08X} != new base 0x{nbase:08X}")
        from_r, _growth, min_shift, max_window = patch_geometry(
            nbase, len(ref_img), len(new_img))

        # Option 1 window: at least what the patch needs (min_shift), then 2048 B
        # by default or the user's --patch-shift, clamped to the writable region.
        floor = max(min_shift, 2048 if args.patch_shift is None
                    else _ceil_row(args.patch_shift))
        w1 = min(floor, max_window)

        # Build both candidate patches so we can show their sizes.
        s1, f1, t1, body1, plen1 = build_patch(detools, ref_img, new_img, from_r, w1)
        s2, f2, t2, body2, plen2 = build_patch(detools, ref_img, new_img, from_r, max_window)
        full_len = len(new_img)
        n1 = (len(body1) + ROW_SIZE - 1) // ROW_SIZE
        n2 = (len(body2) + ROW_SIZE - 1) // ROW_SIZE
        nfull = full_len // ROW_SIZE
        default = '1' if plen1 < full_len else '3'

        rows = [
            ('1', f"Patch w/ {s1} B shift", len(body1), n1),
            ('2', f"Patch w/ {s2} B shift", len(body2), n2),
            ('3', "Full",                   full_len,   nfull),
        ]
        lw = max(len(r[1]) for r in rows)                 # label column width
        sw = max(len(str(r[2])) for r in rows)            # size column width
        cw = max(len(str(r[3])) for r in rows)            # chunks column width
        print(f"\npatch {args.file} vs {args.reference}:")
        for key, label, size, chunks in rows:
            print(f"  [{key}] {label:<{lw}}  | Size: {size:>{sw}} B | Chunks: {chunks:>{cw}}")
        print("  [q] abort")
        try:
            choice = input(f"choose [1/2/3/q] (default {default}): ").strip().lower() or default
        except (EOFError, KeyboardInterrupt):
            choice = 'q'

        if choice == '1':
            patch_flash(args, nbase, f1, t1, ref_crc, new_crc, s1, body1, new_img)
            return
        elif choice == '2':
            patch_flash(args, nbase, f2, t2, ref_crc, new_crc, s2, body2, new_img)
            return
        elif choice == '3':
            pass   # fall through to the full-flash path below
        else:
            print("aborted.")
            return

    with open(args.file, 'rb') as f:
        data = f.read()
    blocks = parse_uf2(data)
    if not blocks:
        sys.exit("error: no main-flash blocks in UF2")

    # Bootloader-region guard: refuse to flash anything overlapping [0, 0x2000).
    low = [b for b in blocks if b[1] < BOOTLOADER_END]
    if low:
        sys.exit(f"error: {len(low)} block(s) target the bootloader region "
                 f"[0, 0x{BOOTLOADER_END:X}); refusing (first addr 0x{low[0][1]:08X})")

    num = len(blocks)
    # Frame id is the absolute block index (0..num-1); it both correlates the
    # ACK and tells the watch's stop-and-wait dedup where in the stream we are.
    if num > 0x10000:
        sys.exit("error: image has too many blocks for a 16-bit frame id")

    # Block selection (--start-block / --count). start>0 = resume into a watch
    # already in its flasher.
    start = args.start_block
    if start < 0 or start >= num:
        sys.exit(f"error: --start-block {start} out of range [0, {num})")
    count = (num - start) if args.count is None else args.count
    if count < 1:
        sys.exit("error: --count must be >= 1")
    if start + count > num:
        print(f"note: --count {count} exceeds the {num - start} remaining blocks; "
              f"clamping to {num - start}")
        count = num - start
    end = start + count                 # exclusive
    resume = (start > 0)
    arm    = not resume                 # arm only on a fresh flash from block 0

    # The descriptor (and the final CRC) always cover the WHOLE image, regardless
    # of which subset we send this run.
    base, total_length, image_crc, rows = assemble_image(blocks)

    print(f"file:     {args.file}")
    print(f"device:   {args.device}")
    print(f"image:    {num} blocks, 0x{base:08X}..0x{base + total_length:08X} "
          f"({total_length} bytes)  crc32 0x{image_crc:08X}")
    print(f"send:     blocks {start}..{end - 1} ({count} of {num})"
          f"{'  [RESUME: no arm/test]' if resume else ''}"
          f"{'  [partial]' if count < num else ''}")
    print(f"baud:     data {args.baud} / ack {args.ack_baud}  encoding {args.encoding}")
    print(f"retries:  {args.retries}  timeout {args.timeout}s")
    print(f"test:     {args.test_blocks} block(s)"
          f"{' (test-only)' if args.test_only else ''}"
          f"{' (skipped: resume)' if resume and args.test_blocks else ''}")

    try:
        ser = serial.Serial(args.device, baudrate=HANDSHAKE_BAUD, timeout=0.1)
    except serial.SerialException as e:
        sys.exit(f"error: cannot open {args.device}: {e}")

    print(f"opened; waiting {RESET_DELAY_S:.1f}s for Arduino reset...")
    time.sleep(RESET_DELAY_S)
    ser.reset_input_buffer()
    modem = Modem(ser)
    handshake(modem, args.baud, args.ack_baud, args.encoding)
    print("modem ready.")

    # Link-reliability test stage (mirrors the watch's flash TEST mode). Needs
    # the watch in its launcher face, so it is skipped when resuming.
    m = 0 if resume else min(args.test_blocks, num)
    if m and m < args.test_blocks:
        print(f"note: image has only {num} blocks; testing all {m}")
    if m:
        try:
            input("Put the watch in flash TEST mode (FLASH menu -> Alarm), then "
                  "press Enter to start the link test (Ctrl-C to abort)... ")
        except (EOFError, KeyboardInterrupt):
            print("\naborted")
            modem.send(CMD_STOP); ser.close(); return
        run_test_stage(modem, [b[2] for b in blocks[:m]], "block", args)

    if args.test_only:
        print("test-only: done.")
        modem.send(CMD_STOP)
        ser.close()
        return

    # Confirm before the real flash: explicit 'y', defaulting to No. This is the
    # point of no return; the watch goes unresponsive until a flash completes.
    try:
        ans = input("POINT OF NO RETURN: the watch will be unresponsive until a "
                    "flash succeeds. Flash now? [y/N] ").strip().lower()
    except (EOFError, KeyboardInterrupt):
        ans = 'n'
    if ans != 'y':
        print("not flashing.")
        modem.send(CMD_STOP)
        ser.close()
        return

    print("flashing...")
    send_rows = [(idx, rows[idx][0], rows[idx][1]) for idx in range(start, end)]
    try:
        flash_stage(modem, base, total_length, image_crc, send_rows, args,
                    arm=arm, restartable=arm)
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        modem.send(CMD_STOP)
        ser.close()


if __name__ == '__main__':
    main()

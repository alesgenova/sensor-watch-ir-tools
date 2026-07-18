#!/usr/bin/env python3
"""Generate a real detools crle in-place patch between two real Sensor Watch
images, mirroring bin/firmware_flasher.py's pipeline (row-padded images,
segment_size=256, memory_size=from_r+window), and emit the pieces the C
harness consumes: ref.bin, new.bin, body.bin, params.txt.

Usage: gen_patch.py [window] [ref_image new_image]

The default images are the Sensor Watch builds committed as UltraPatch's test
fixtures (sibling checkout); override with any two raw firmware binaries.
"""
import io
import os
import sys

import detools
from detools.apply import read_header_in_place

ROW = 256
HERE = os.path.dirname(os.path.abspath(__file__))
UP = os.environ.get("ULTRAPATCH_PATH",
                    os.path.join(HERE, "..", "..", "second-movement", "ultrapatch"))
FIX = os.path.join(UP, "test-bench", "fixtures")


def ceil_row(x):
    return (x + ROW - 1) // ROW * ROW


def pad(img):
    return img + b"\xff" * (ceil_row(len(img)) - len(img))


window = int(sys.argv[1]) if len(sys.argv) > 1 else 2048
ref_path = sys.argv[2] if len(sys.argv) > 3 else os.path.join(FIX, "v0_base", "watch.bin")
new_path = sys.argv[3] if len(sys.argv) > 3 else os.path.join(FIX, "v1_one_face", "watch.bin")

ref = pad(open(ref_path, "rb").read())
new = pad(open(new_path, "rb").read())

fp = io.BytesIO()
detools.create_patch(io.BytesIO(ref), io.BytesIO(new), fp,
                     compression='crle', patch_type='in-place',
                     memory_size=len(ref) + window, segment_size=ROW)
patch = fp.getvalue()
f = io.BytesIO(patch)
_comp, _mem, _seg, shift_size, from_size, to_size = read_header_in_place(f)
body = f.read()

open("ref.bin", "wb").write(ref)
open("new.bin", "wb").write(new)
open("body.bin", "wb").write(body)
open("params.txt", "w").write(f"{from_size} {to_size} {shift_size}\n")
print(f"ref={len(ref)} new={len(new)} patch={len(patch)} body={len(body)} "
      f"from={from_size} to={to_size} shift={shift_size} "
      f"frames={(len(body) + ROW - 1) // ROW}")

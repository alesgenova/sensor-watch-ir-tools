#!/usr/bin/env python3
"""Generate a real UltraPatch patch between two real Sensor Watch images via
the UltraPatch host CLI, mirroring the host flasher's pipeline (row-padded
images, whole blob = envelope + body streamed as the patch body), and emit the
pieces the C harness consumes: ref.bin, new.bin, body.bin, params.txt
(shift_size is always 0 in this format).

Usage: gen_upatch.py [ref_image new_image]

Needs an UltraPatch checkout (default: sibling of this repo; override with
ULTRAPATCH_PATH). Builds the CLI on demand and self-checks the patch with
--decode before emitting it.
"""
import os
import shutil
import subprocess
import sys

ROW = 256
HERE = os.path.dirname(os.path.abspath(__file__))
UP = os.environ.get("ULTRAPATCH_PATH",
                    os.path.join(HERE, "..", "..", "second-movement", "ultrapatch"))
FIX = os.path.join(UP, "test-bench", "fixtures")


def ceil_row(x):
    return (x + ROW - 1) // ROW * ROW


def pad(img):
    return img + b"\xff" * (ceil_row(len(img)) - len(img))


ref_path = sys.argv[1] if len(sys.argv) > 2 else os.path.join(FIX, "v0_base", "watch.bin")
new_path = sys.argv[2] if len(sys.argv) > 2 else os.path.join(FIX, "v1_one_face", "watch.bin")

subprocess.run(["make", "-C", UP, "-s"], check=True)
tool = subprocess.run(["make", "-C", UP, "-s", "host-tool-path"],
                      check=True, capture_output=True, text=True).stdout.strip()

ref = pad(open(ref_path, "rb").read())
new = pad(open(new_path, "rb").read())
open("ref.bin", "wb").write(ref)
open("new.bin", "wb").write(new)

subprocess.run([tool, "ref.bin", "new.bin", "up.patch"], check=True)

# Self-check: the production decoder must reproduce new.bin exactly.
shutil.copy("ref.bin", "up_check.bin")
subprocess.run([tool, "--decode", "up_check.bin", "up.patch"], check=True)
assert open("up_check.bin", "rb").read() == new, "CLI round-trip mismatch"
os.remove("up_check.bin")

body = open("up.patch", "rb").read()   # the WHOLE blob streams as the body
open("body.bin", "wb").write(body)
open("params.txt", "w").write(f"{len(ref)} {len(new)} 0\n")
print(f"ref={len(ref)} new={len(new)} patch={len(body)} "
      f"frames={(len(body) + ROW - 1) // ROW}")

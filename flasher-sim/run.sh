#!/bin/sh
# flasher-sim: compile the REAL watch flasher translation units for the host
# and drive complete simulated flash sessions through them, in BOTH backend
# modes. Compiling the TUs for the host is itself what selects their hosted
# mode (they key on __arm__), so no special define is needed and there is no
# extracted or duplicated watch code.
#
# DEVELOPMENT TOOL: requires a second-movement checkout (sibling by default;
# override with SECOND_MOVEMENT_PATH). Both the watch TUs and the UltraPatch
# headers/CLI come from that checkout (its `ultrapatch` submodule), so the
# encoder and decoder under test share one pin by construction. Needs gcc,
# python3, make, and network access the first time (pip install detools
# into a local venv).
set -e
cd "$(dirname "$0")"

SM=${SECOND_MOVEMENT_PATH:-../../second-movement}
UP=${ULTRAPATCH_PATH:-$SM/ultrapatch}
FLASHER="$SM/firmware-flasher"

[ -x venv/bin/python ] || {
    python3 -m venv venv
    ./venv/bin/pip install --quiet detools
}

CFLAGS="-O1 -Wall -Wextra -Wno-int-to-pointer-cast -I $FLASHER"

# --- detools backend (the default watch build) ---------------------------
gcc $CFLAGS -o harness \
    harness.c "$FLASHER/firmware_flasher_core.c" "$FLASHER/firmware_flasher_detools.c"

for w in 512 2048 132608; do
    echo "=== detools, window=$w ==="
    ./venv/bin/python gen_patch.py "$w"
    ./harness
done

# --- ultrapatch backend (FIRMWARE_FLASHER_ULTRAPATCH=1 watch build) ------
gcc $CFLAGS -DFIRMWARE_FLASHER_ULTRAPATCH -I "$UP/src" -o harness_up \
    harness.c "$FLASHER/firmware_flasher_core.c" "$FLASHER/firmware_flasher_ultrapatch.c"

echo "=== ultrapatch ==="
ULTRAPATCH_PATH="$UP" ./venv/bin/python gen_upatch.py
./harness_up

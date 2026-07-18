# flasher-sim

Development tool (rarely needed day to day): validates the watch's RAM-resident flasher
(`second-movement/firmware-flasher/firmware_flasher_core.c` + decoder
backend TUs) on the host, without hardware — by compiling the **real watch
translation units**: any non-ARM compile of those TUs is a hosted test build
(they key on the compiler's `__arm__`, no special define), which swaps only
the hardware layer (NVM, DSU, SERCOM, WFI, reboot) for hooks defined in
`harness.c`. Everything
else, including `flasher_run` itself, is the exact code the watch links; there
is no extracted or duplicated watch code.

- `gen_patch.py` builds a real detools crle in-place patch between two real
  Sensor Watch firmware images, using the exact pipeline of
  `bin/firmware_flasher.py` (row-padded images, `segment_size=256`,
  `memory_size=from+window`).
- `gen_upatch.py` builds a real UltraPatch blob with the UltraPatch host CLI
  (built on demand from a sibling checkout) and self-checks it with
  `--decode` before use.
- `harness.c` queues whole sessions into a fake RX stream (fake flash in a
  `MAP_32BIT` mmap so the watch's `uint32_t` absolute addresses dereference
  directly) and runs each through the real `flasher_run`:

  1. full (uf2-like) flash + EXIT verify → reboot
  2. clean patch apply + EXIT → reboot
  3. clean patch apply, in-flash shift mode (detools mode only)
  4. duplicated frames + re-sent final frame (lost-ACK simulation, dup re-ACK)
  5. mid-apply takeover by a full flash + EXIT
  6. corrupt patch body → park → full-flash recovery
  7. wrong base image → rejected before any flash write (ultrapatch mode only;
     detools guards this in the launcher's ENTER pre-flight instead)

Both decoder backends run: detools at three patch geometries (minimum shift,
default, max window) and UltraPatch (the `FIRMWARE_FLASHER_ULTRAPATCH=1`
watch build, linking the real `firmware_flasher_ultrapatch.c`).

Run everything:

```sh
sh run.sh
```

Expected output ends with `ALL PASS` for each geometry and backend. Requires
a `second-movement` checkout (sibling of this repo by default; override with
`SECOND_MOVEMENT_PATH`). Both the watch TUs and UltraPatch come from that
checkout (its `ultrapatch` submodule), so the encoder and decoder under test
share one pin by construction -- note this is deliberately NOT the
`ultrapatch` submodule at this repo's root, which serves the production
`firmware_flasher.py` encoder only.

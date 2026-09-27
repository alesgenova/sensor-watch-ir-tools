# Repository Guidelines

## Project Structure & Module Organization

- `src/` contains PlatformIO/Arduino firmware for the two supported IR modem boards: `modem_arduino_uno.cpp` and `modem_xiao_samd21.cpp`.
- `bin/` contains Python 3 host tools. `firmware_flasher.py` drives wireless firmware updates; `serial_frame.py` owns the shared wire-frame format; `ir_modem.py` speaks the USB modem protocol.
- `flasher-sim/` is a host-side integration harness for the real watch flasher sources in a sibling `second-movement` checkout.
- `img/` holds hardware photos and schematics. `platformio.ini` defines the board-specific build environments. The `ultrapatch` submodule supplies the production patch encoder.

## Build, Test, and Development Commands

Use Python 3 and PlatformIO. Create a virtual environment before installing host dependencies:

```sh
python3 -m venv .venv && source .venv/bin/activate
pip install pyserial detools
pio run -e modem_arduino_uno -t upload
pio run -e modem_xiao_samd21 -t upload
```

The repository's development tools are installed in `.venv`; invoke them by
activating it or by their explicit paths (for example, `.venv/bin/pio`) rather
than relying on a system-wide installation.

The last two commands build and upload the selected modem. Run `sh flasher-sim/run.sh` for the full hosted flasher integration suite; it requires GCC, a sibling `second-movement` checkout, and may create a local `flasher-sim/venv`. Success ends in `ALL PASS` for every backend.

## Coding Style & Naming Conventions

Match nearby code. Python uses four-space indentation, `snake_case` functions and modules, constants in `UPPER_SNAKE_CASE`, type hints where useful, and concise module/docstrings. Firmware uses four-space indentation, `snake_case` helpers, `UPPER_SNAKE_CASE` macros, and explicit fixed-width integer types for protocol and hardware values. Keep host and firmware protocol constants, framing, and byte order synchronized; document any wire-format change in `README.md`.

No formatter or linter is configured. Avoid unrelated formatting churn and retain the detailed hardware comments around timing, pins, and interrupt behavior.

## Testing Guidelines

There is no separate unit-test framework. Exercise changed Python framing or modem behavior with the relevant `bin/test_tx.py` or `bin/test_rx.py` workflow and hardware when available. For flashing or patch logic, run `sh flasher-sim/run.sh`; do not claim the change tested until its complete matrix passes.

## Commit & Pull Request Guidelines

Use short, imperative, lowercase subject lines describing the affected area, such as `firmware_flasher: validate UF2 face` or `add analog polled rx logic`. Keep commits focused. PRs should state the board/tooling affected, testing performed (including hardware conditions), linked issue if applicable, and photos or serial output when a hardware-facing change benefits from evidence. Never commit generated `.pio/`, virtual-environment, or editor files.

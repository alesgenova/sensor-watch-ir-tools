#!/usr/bin/env python3
"""Generate the IR flashing-rig probe schematic with schemdraw.

Writes a single SVG next to this script:

    flasher_schematics.svg

The probe is board-agnostic: it exposes four signals to whatever modem board
drives it (Arduino UNO, Seeed XIAO SAMD21, ...): +V, GND, an LED-drive pin,
and a phototransistor-signal pin. Only the pin *names* differ between boards;
the circuit is the same.

Two halves:

  * TX: the IR LED is switched by an NPN transistor (low-side switch) rather
    than driven straight off an IO pin, so the LED current isn't capped by the
    pin's drive strength. MCU pin HIGH -> transistor on -> LED on.

  * RX: the IR phototransistor is wired with a pull-up: +V -> pull-up -> node
    -> phototransistor collector, emitter to GND, and the node feeds the MCU.
    So the signal pin reads HIGH in the dark and is pulled LOW when the
    phototransistor is illuminated (verified on the bench).

Usage:
    python3 img/draw_flasher_schematics.py             # write flasher_schematics.svg
    python3 img/draw_flasher_schematics.py --show       # also pop up a window (needs matplotlib)
    python3 img/draw_flasher_schematics.py --format png # PNG instead of SVG (needs matplotlib)

Dependencies:
    pip install schemdraw                       # SVG backend needs nothing else
    pip install matplotlib                      # only for --show / PNG output
"""

import argparse
import os
import sys

import schemdraw
import schemdraw.elements as elm

HERE = os.path.dirname(os.path.abspath(__file__))


def build():
    """Return the schemdraw.Drawing for the probe wiring."""
    d = schemdraw.Drawing()
    d.config(fontsize=11, bgcolor="white", margin=0.5)

    # ================= TX: IR LED, NPN low-side switch ==================
    # +V -> series resistor -> IR LED -> Q1 collector ; Q1 emitter -> GND ;
    # MCU LED-drive pin -> base resistor -> Q1 base.
    d += elm.Vdd().label("VCC").at((0, 0))
    d += elm.Line().down().length(0.15)
    d += elm.Resistor().down().length(1.1).label("R1", loc="bottom")
    # Label on the visual left; for a down-pointing element that's loc="top"
    # (the LED's own emission arrows are on the right = loc="bottom").
    d += elm.LED().down().length(1.4).fill(True).label("IR LED\n(TX)", loc="top")
    # theta(0) keeps the collector-emitter channel vertical (otherwise it
    # inherits the downward chain direction and gets rotated 90 deg).
    d += (q1 := elm.BjtNpn().theta(0).anchor("collector").label("Q1", loc="right"))
    d += elm.Line().down().length(0.6).at(q1.emitter)
    d += elm.Ground()
    # Base drive from the MCU pin, through a base resistor.
    d += elm.Resistor().left().length(2.0).at(q1.base).label("R2")
    d += elm.Dot(open=True).label("LED drive\n(from MCU)", loc="left")

    # =============== RX: IR phototransistor, pull-up ====================
    # +V -> pull-up -> node -> Q2 collector ; Q2 emitter -> GND ;
    # node -> MCU PT-signal pin. Dark = HIGH, lit = LOW.
    rx_x = 7.5
    d += elm.Vdd().label("VCC").at((rx_x, 0))
    # The lead between +V and the collector node (0.15 + the TX side's 1.4 LED
    # + 0.15 resistor stub = 1.55) is split evenly above and below the resistor
    # so it sits centred, while the transistor/ground still line up with TX.
    d += elm.Line().down().length(0.775)
    d += elm.Resistor().down().length(1.1).label("R3", loc="bottom")
    d += elm.Line().down().length(0.775)
    d += (node := elm.Dot())
    # Built-in photo-transistor symbol (draws its own incident-light arrows).
    # theta(0) keeps the collector-emitter channel vertical.
    d += (q2 := elm.NpnPhoto().theta(0).anchor("collector")
          .label("Phototransistor\n(RX)", loc="left"))
    d += elm.Line().down().length(0.6).at(q2.emitter)
    d += elm.Ground()
    # Signal takeoff from the node to the MCU pin.
    d += elm.Line().right().length(1.8).at(node.center)
    d += elm.Dot(open=True).label("PT signal\n(to MCU)", loc="right")

    return d


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--show", action="store_true",
                   help="Also display the drawing in a window (needs matplotlib).")
    p.add_argument("--format", default="svg", choices=["svg", "png"],
                   help="Output image format (png needs matplotlib). Default: svg.")
    args = p.parse_args(argv)

    if args.format == "svg" and not args.show:
        # SVG backend is pure-python; no matplotlib needed.
        schemdraw.use("svg")

    d = build()
    out = os.path.join(HERE, f"flasher_schematics.{args.format}")
    # transparent=False so the configured white background is written out.
    d.save(out, transparent=False)
    print(f"wrote {out}")
    if args.show:
        d.draw()

    return 0


if __name__ == "__main__":
    sys.exit(main())

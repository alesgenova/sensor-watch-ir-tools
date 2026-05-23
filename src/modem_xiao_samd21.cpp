// Seeed XIAO SAMD21 half-duplex IR modem firmware for sensor-watch-ir-tools.
//
// *** SOFTWARE BIT-BANG version *** (replaces the earlier SERCOM/SIR port).
//
// Like the Arduino UNO modem (modem_arduino_uno.cpp), this drives the optical link in
// SOFTWARE: a hardware timer is the bit clock, and we encode/decode IrDA and NRZ
// ourselves. The whole point of doing it in software is POLARITY CONTROL: the
// SAM SERCOM's IrDA SIR decoder has a fixed pulse polarity that did not match
// this jig's idle-HIGH / active-LOW phototransistor (it decoded all-zeros), and
// the SAMD21 has no RXINV/CCL to flip the line. By decoding in software we choose
// the sense, so IrDA *and* NRZ work in *both* directions on the existing wiring
// (active-high LED + active-low/idle-high PT), no external inverter.
//
// Hardware mapping (UNO Timer1 -> SAMD21):
//   * Bit clock + RX bit-sampling : TC4 compare CC0 (the OCR1A/COMPA analogue)
//   * IR LED                       : PA04 GPIO, driven in the TX bit-clock ISR
//   * PT edge timestamps (RX)      : PA07 -> EIC EXTINT7 interrupt; the ISR reads
//                                    the free-running TC4 COUNT as the timestamp
//                                    (the ICP1/ICR1 input-capture analogue). The
//                                    SAMD21 TC's event-capture only does PPW/PWP
//                                    (pulse-width+period), NOT single-edge
//                                    timestamps, so we stamp in the EIC ISR (a
//                                    few us of jitter, negligible at <= a few kbaud).
// TC4 CC0 = compare (TX bit clock / RX bit sampler), serviced in TC4_Handler. PT
// edges come in on the EIC interrupt (pt_edge_isr, registered via the Arduino
// core's attachInterrupt so we don't fight it for the EIC_Handler symbol).
//
// Prescalers differ by direction (chosen on each enter_tx/enter_rx, just as the
// UNO rewrites TCCR1B): TX needs accurate per-bit periods, RX needs a whole
// 10-bit byte to fit in the 16-bit delta arithmetic across 50..9600 baud:
//   * TX: GCLK0/16  = 3 MHz   (a 50-baud bit = 60000 ticks < 65536)
//   * RX: GCLK0/256 = 187.5 kHz (a 50-baud BYTE = 37500 ticks < 65536)
//
// !!! BRING-UP NOTE !!!  This has NOT been run on hardware. The decode/encode
// state machines are ported verbatim from the proven UNO modem, but the SAMD21
// EIC-interrupt edge timing, the TC read/write synchronisation, and the IrDA TX
// pulse-width calibration all need scope verification.
//
// Turn cycle and USB framing are identical to modem_arduino_uno.cpp; see that file's
// header. In brief: CMD_TX(flags,frame) transmits; TX_FLAG_AUTO_RX flips to RX
// before reporting MSG_TX_DONE; CMD_RX enters RX; CMD_STOP idles.
//   USB framing (both directions): [SOF=0xA5][TYPE][LEN_LO][LEN_HI][payload]

#include <Arduino.h>
#include <string.h>

#if !defined(ARDUINO_ARCH_SAMD)
#error "modem_xiao_samd21.cpp targets the SAMD21 (XIAO). Build env:modem_xiao_samd21."
#endif

// ---------------------------------------------------------------------------
// Board / pin config (PORT group A)
// ---------------------------------------------------------------------------

#define LED_PIN   4u    // PA04  IR LED, active-high (pin HIGH = lit)
#define PT_PIN    7u    // PA07  phototransistor, active-low / idle-HIGH; -> EIC EXTINT7
#define PT_EXTINT 7u    // PA07 is on EXTINT[7]
#define PT_ARDUINO_PIN 8  // XIAO silk D8 / SCK == PA07 (for attachInterrupt)

#define LED_ON()   do { PORT->Group[0].OUTSET.reg = (1u << LED_PIN); } while (0)
#define LED_OFF()  do { PORT->Group[0].OUTCLR.reg = (1u << LED_PIN); } while (0)
#define PT_READ()  ((PORT->Group[0].IN.reg & (1u << PT_PIN)) != 0u)

// Receiver board polarity: idle-HIGH (dark -> pin HIGH, light -> pin LOW), same
// convention as the UNO jig (RX_IDLE_HIGH=1). Data bit == pin level; the optical
// start bit / IrDA pulse is a FALLING edge (light pulls the line low).
#define RX_LOGICAL(pin_high)  ((pin_high) ? 1u : 0u)

// Optional RX noise rejection. When 1, enables BOTH the EIC majority glitch
// filter on the PT edge (start bit / IrDA pulse) AND 3-sample majority voting on
// each NRZ data bit. Default OFF; turn on only if the bench shows noise issues.
#define RX_NOISE_FILTERING  0

// Optional NRZ clock-drift correction. When 1, the NRZ receiver re-anchors its
// bit phase on every 1->0 data edge (which the edge ISR otherwise ignores
// mid-byte), so accumulated baud mismatch between the watch's OSC16M and this
// modem can't walk the stop-bit sample out of its cell (the FERR-per-byte
// failure NRZ shows that IrDA's self-clocking hides). Default OFF; set to 1 to
// test. No effect on IrDA (which already re-syncs per pulse).
#define RX_NRZ_RESYNC  0

// TC tick rates (see header). RX must fit a whole byte in 16 bits.
#define TX_TICK_HZ  3000000UL    // GCLK0 (48 MHz) / 16
#define RX_TICK_HZ   187500UL    // GCLK0 (48 MHz) / 256

#define F_CPU_HZ    48000000UL

static inline uint16_t baud_to_rx_ticks(uint32_t baud) {
    return (uint16_t)((RX_TICK_HZ + (baud >> 1)) / baud);
}

// ---------------------------------------------------------------------------
// USB protocol constants (identical to modem_arduino_uno.cpp / bin/ir_modem.py)
// ---------------------------------------------------------------------------

#define USB_BAUD    115200
#define USB_SOF     0xA5
#define MODEM_PROTO_VERSION 1

// host -> modem
enum : uint8_t {
    CMD_CONFIG = 0x01,   // payload: tx_baud(4 LE) rx_baud(4 LE) enc(1)
    CMD_TX     = 0x02,   // payload: tx_flags(1) then the frame bytes to transmit
    CMD_RX     = 0x03,   // no payload: enter RX (listen) -> MSG_OK, then MSG_RX stream
    CMD_STOP   = 0x04,   // stop RX, go idle
    CMD_PING   = 0x05,   // -> MSG_PONG
};
enum : uint8_t {
    TX_FLAG_AUTO_RX = 0x01,  // after sending, flip to RX and forward bytes;
                             // if clear, go idle (fire-and-forget)
};
// modem -> host
enum : uint8_t {
    MSG_RX        = 0x10,  // payload: raw decoded optical bytes
    MSG_TX_DONE   = 0x11,  // payload: (none)
    MSG_OK        = 0x12,  // payload: (none)
    MSG_ERR       = 0x13,  // payload: error code(1)
    MSG_RX_STATUS = 0x14,  // DIAGNOSTIC: ferr(2 LE) bufovf(2 LE), this RX session
    MSG_PONG      = 0x15,  // payload: proto_version(1)
};
enum : uint8_t {
    ERR_BAD_LEN  = 1,
    ERR_BAD_CMD  = 2,
    ERR_NOT_CFG  = 3,
};

// Largest CMD_TX payload we accept = one wire frame (12-byte overhead + 512
// chunk = 524; round up for slack).
#define MAX_FRAME  540

// ---------------------------------------------------------------------------
// Encoding + runtime config
// ---------------------------------------------------------------------------

enum Encoding : uint8_t { ENC_NRZ = 0, ENC_IRDA = 1 };  // matches the watch / host

static uint32_t g_tx_baud  = 3600;       // main-context only (enter_tx)
static uint32_t g_rx_baud  = 150;        // main-context only (enter_rx)
static volatile Encoding g_encoding = ENC_IRDA;   // read in the TC4 ISR
static bool     g_configured = false;

enum Dir : uint8_t { DIR_IDLE = 0, DIR_TX = 1, DIR_RX = 2 };
static volatile Dir g_dir = DIR_IDLE;

// ---------------------------------------------------------------------------
// TX state (TC4 compare bit clock), adapted from modem_arduino_uno.cpp
// ---------------------------------------------------------------------------

static volatile uint16_t tx_bit_queue;   // 10-bit frame, LSB out first
static volatile uint8_t  tx_bit_count;   // bits remaining (0 = idle)
static volatile uint8_t  tx_is_irda;
static volatile uint32_t tx_pulse_loops; // IrDA '0' pulse width, in busy-wait loops

// Calibrated short busy-wait for the IrDA TX pulse (3/16 of a bit). Same idea as
// the UNO's busy_wait_loops; tuned to the M0+ at 48 MHz. VERIFY: the loop is ~3
// cycles/iter on Cortex-M0+, so tx_pulse_loops below assumes that; scope the
// pulse and adjust the divisor if it's off.
static inline void busy_wait_loops(uint32_t loops) {
    // Plain C loop; let the compiler emit valid Thumb-16. The empty volatile asm
    // is a side effect per iteration so the loop can't be optimized away. Cost is
    // a few cycles/iter (see the /3 estimate in enter_tx); calibrate on a scope.
    while (loops--) {
        __asm__ __volatile__("" ::: "memory");
    }
}

// ---------------------------------------------------------------------------
// RX state (TC4 capture), adapted from modem_arduino_uno.cpp
// ---------------------------------------------------------------------------

static volatile uint16_t g_rx_bit_ticks  = RX_TICK_HZ / 150;
enum RxState : uint8_t { ST_IDLE = 0, ST_RECEIVING = 1 };
static volatile RxState  g_rx_state        = ST_IDLE;
static volatile uint16_t g_byte_t0         = 0;
static volatile uint8_t  g_byte_data       = 0;
static volatile uint8_t  g_byte_nrz_index  = 0;
static volatile uint16_t g_irda_pulse_mask = 0;

#if RX_NOISE_FILTERING
// NRZ bit-sample majority vote: busy-wait loops between the 3 samples, set per
// session to ~1/16 of a bit (so the 3 samples straddle ~1/8 bit, biased a hair
// late, which also helps the sluggish dark-recovery edge). ~1e6/baud ≈ 1/16 bit
// at ~3 cycles/loop on the 48 MHz M0+; approximate, doesn't need to be exact.
static uint32_t g_rx_spread_loops = 1000000UL / 150;

// Read the PT at the bit center and twice more ~1/16 bit later; return the
// majority level. Rejects brief spikes (a glitch shorter than the spacing can
// only corrupt one of the three samples).
static inline bool pt_sample_majority(void) {
    uint8_t high = PT_READ() ? 1u : 0u;
    busy_wait_loops(g_rx_spread_loops);
    high += PT_READ() ? 1u : 0u;
    busy_wait_loops(g_rx_spread_loops);
    high += PT_READ() ? 1u : 0u;
    return high >= 2u;
}
#else
static inline bool pt_sample_majority(void) { return PT_READ(); }   // single sample
#endif

// ISR -> main-loop ring (power-of-2). Never touch Serial from the ISR.
#define RING_SIZE  256u
#define RING_MASK  (RING_SIZE - 1u)
static volatile uint8_t  g_ring[RING_SIZE];
static volatile uint16_t g_ring_head;
static volatile uint16_t g_ring_tail;

// DIAGNOSTIC counters (surfaced via MSG_RX_STATUS): framing errors (stop bit not
// mark) and ring overflows, this RX session.
static volatile uint16_t g_rx_ferr;
static volatile uint16_t g_rx_bufovf;
static uint16_t g_rx_ferr_reported;
static uint16_t g_rx_bufovf_reported;

static inline void byte_received(uint8_t b) {
    uint16_t next = (uint16_t)((g_ring_head + 1u) & RING_MASK);
    if (next == g_ring_tail) { g_rx_bufovf++; return; }   // overflow: drop
    g_ring[g_ring_head] = b;
    g_ring_head = next;
}

// ---------------------------------------------------------------------------
// TC4 low-level helpers (SAMD21 needs explicit read/write synchronisation)
// ---------------------------------------------------------------------------

static inline void tc4_sync(void) {
    while (TC4->COUNT16.STATUS.bit.SYNCBUSY) { /* spin */ }
}

// Read COUNT / CCx through the read-request mechanism. addr = register offset
// within COUNT16 (COUNT=0x10, CC0=0x18, CC1=0x1A).
static inline uint16_t tc4_read16(uint8_t addr) {
    TC4->COUNT16.READREQ.reg = TC_READREQ_RREQ | TC_READREQ_ADDR(addr);
    tc4_sync();
    if (addr == 0x10) return TC4->COUNT16.COUNT.reg;
    if (addr == 0x18) return TC4->COUNT16.CC[0].reg;
    return TC4->COUNT16.CC[1].reg;
}
#define TC4_READ_COUNT()  tc4_read16(0x10)

static inline void tc4_write_cc0(uint16_t v) {
    TC4->COUNT16.CC[0].reg = v;
    tc4_sync();
}
static inline void tc4_write_count(uint16_t v) {
    TC4->COUNT16.COUNT.reg = v;
    tc4_sync();
}

static void tc4_disable(void) {
    TC4->COUNT16.CTRLA.bit.ENABLE = 0;
    tc4_sync();
}

static void tc4_clock_once(void) {
    // One-time: APB + GCLK0 to TC4 (shared TC4/TC5 clock id).
    PM->APBCMASK.reg |= PM_APBCMASK_TC4;
    GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_CLKEN
                                 | GCLK_CLKCTRL_GEN_GCLK0
                                 | GCLK_CLKCTRL_ID_TC4_TC5);
    while (GCLK->STATUS.bit.SYNCBUSY) {}
}

// ---------------------------------------------------------------------------
// PT edge interrupt: timestamps each falling edge (light pulse / NRZ start bit)
// by reading the free-running TC4 COUNT. Registered via the Arduino core's
// attachInterrupt in enter_rx. This is the ICP1/CAPT analogue; the SAMD21 TC's
// event-capture can't do single-edge timestamps (PPW/PWP only), so we stamp here.
// Ported from modem_arduino_uno.cpp's TIMER1_CAPT_vect (CC1 read -> COUNT read).
// ---------------------------------------------------------------------------

static void pt_edge_isr(void) {
    if (g_dir != DIR_RX) return;
    uint16_t now = TC4_READ_COUNT();

    if (g_rx_state == ST_IDLE) {
        g_byte_t0         = now;
        g_byte_data       = 0;
        g_irda_pulse_mask = 1u;          // start bit is '0' -> has a pulse
        g_byte_nrz_index  = 1;
        g_rx_state        = ST_RECEIVING;
        if (g_encoding == ENC_IRDA) {
            tc4_write_cc0((uint16_t)(now + (uint16_t)(g_rx_bit_ticks * 10u)));
        } else {
            tc4_write_cc0((uint16_t)(now + (uint16_t)(g_rx_bit_ticks + (g_rx_bit_ticks >> 1))));
        }
        TC4->COUNT16.INTFLAG.reg  = TC_INTFLAG_MC0;       // clear stale
        TC4->COUNT16.INTENSET.reg = TC_INTENSET_MC0;
    } else if (g_encoding == ENC_IRDA) {
        uint16_t delta = now - g_byte_t0;
        uint16_t bit_n = (uint16_t)((delta + (g_rx_bit_ticks >> 1)) / g_rx_bit_ticks);
        if (bit_n >= 1u && bit_n <= 8u) {
            g_irda_pulse_mask |= (uint16_t)(1u << bit_n);
        } else if (bit_n == 9u) {
            g_irda_pulse_mask |= (uint16_t)(1u << 9);
        } else if (bit_n >= 10u) {
            // Start bit of the next back-to-back byte beat the end-of-byte
            // compare. Finalize the current byte and pivot to a fresh T0.
            uint16_t mask = g_irda_pulse_mask;
            bool framing_ok = ((mask & (uint16_t)(1u << 9)) == 0u);
            uint8_t data = 0;
            for (uint8_t i = 0; i < 8; i++) {
                if ((mask & (uint16_t)(1u << (i + 1u))) == 0u) data |= (uint8_t)(1u << i);
            }
            if (framing_ok) byte_received(data); else g_rx_ferr++;
            g_byte_t0         = now;
            g_irda_pulse_mask = 1u;
            tc4_write_cc0((uint16_t)(now + (uint16_t)(g_rx_bit_ticks * 10u)));
        }
    }
#if RX_NRZ_RESYNC
    else {
        // NRZ, receiving: a 1->0 data edge is a CRISP bit boundary (start of a
        // 0-bit we're about to sample). Re-anchor the bit phase to it so the
        // sample clock can't free-run far enough to push the stop bit out of its
        // cell, the weakness vs IrDA's self-clocking. Only during data bits;
        // the stop bit (index 9) carries no falling edge.
        if (g_byte_nrz_index <= 8u) {
            // Sample this bit at edge + half a bit, and re-anchor t0 so the rest
            // of the byte (and the stop-bit check) is timed from the fresh edge.
            tc4_write_cc0((uint16_t)(now + (g_rx_bit_ticks >> 1)));
            g_byte_t0 = (uint16_t)(now - (uint16_t)(g_rx_bit_ticks * g_byte_nrz_index));
        }
    }
#endif
    // (NRZ data bits are otherwise sampled by the TC4 compare, not by edges.)
}

// ---------------------------------------------------------------------------
// TC4 interrupt, mode-aware. Only one direction is armed at a time. MC0 only:
// TX bit clock, or RX bit-sample / end-of-byte. (RX edges come via pt_edge_isr.)
// ---------------------------------------------------------------------------

extern "C" void TC4_Handler(void) {
    uint8_t flags = TC4->COUNT16.INTFLAG.reg;

    // ---- TX bit clock (MFRQ: COUNT auto-resets to 0 at each CC0 match). ----
    if (g_dir == DIR_TX) {
        if (flags & TC_INTFLAG_MC0) {
            TC4->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0;
            uint8_t bit = tx_bit_queue & 1u;
            tx_bit_queue >>= 1;
            if (tx_is_irda) {
                if (bit == 0) { LED_ON(); busy_wait_loops(tx_pulse_loops); LED_OFF(); }
            } else {
                if (bit == 0) LED_ON(); else LED_OFF();
            }
            if (--tx_bit_count == 0) {
                TC4->COUNT16.INTENCLR.reg = TC_INTENCLR_MC0;  // frame done
            }
        }
        return;
    }

    if (g_dir != DIR_RX) { TC4->COUNT16.INTFLAG.reg = flags; return; }

    // ---- RX compare: NRZ bit-sample tick, or IrDA end-of-byte. Edges arrive
    //      via the PT edge ISR (pt_edge_isr), not here. ----
    if (flags & TC_INTFLAG_MC0) {
        TC4->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0;

        if (g_encoding == ENC_NRZ) {
            bool pin_high = pt_sample_majority();   // 3-sample majority at bit center
            uint8_t bit_n = g_byte_nrz_index;
            if (bit_n >= 1u && bit_n <= 8u) {
                if (RX_LOGICAL(pin_high)) g_byte_data |= (uint8_t)(1u << (bit_n - 1u));
            }
            if (bit_n < 9u) {
                g_byte_nrz_index = (uint8_t)(bit_n + 1u);
                tc4_write_cc0((uint16_t)(g_byte_t0
                              + (uint16_t)(g_rx_bit_ticks * ((uint16_t)bit_n + 1u)
                                           + (g_rx_bit_ticks >> 1))));
                return;
            }
            bool framing_ok = RX_LOGICAL(pin_high);  // stop bit is mark (1)
            if (framing_ok) byte_received(g_byte_data); else g_rx_ferr++;
            g_rx_state = ST_IDLE;
            TC4->COUNT16.INTENCLR.reg = TC_INTENCLR_MC0;   // wait for next start edge
        } else {
            // IrDA end-of-byte: assemble data bits from the pulse mask.
            uint16_t mask = g_irda_pulse_mask;
            bool framing_ok = ((mask & (uint16_t)(1u << 9)) == 0u);
            uint8_t data = 0;
            for (uint8_t i = 0; i < 8; i++) {
                if ((mask & (uint16_t)(1u << (i + 1u))) == 0u) data |= (uint8_t)(1u << i);
            }
            if (framing_ok) byte_received(data); else g_rx_ferr++;
            g_rx_state = ST_IDLE;
            g_irda_pulse_mask = 0;
            TC4->COUNT16.INTENCLR.reg = TC_INTENCLR_MC0;   // wait for next pulse edge
        }
    }
}

// ---------------------------------------------------------------------------
// Pin helpers
// ---------------------------------------------------------------------------

static inline void led_gpio_off(void) {
    PORT->Group[0].PINCFG[LED_PIN].reg = 0;            // plain push-pull GPIO
    PORT->Group[0].DIRSET.reg = (1u << LED_PIN);
    LED_OFF();                                         // active-high: off = LOW
}

static inline void pt_gate_off(void) {
    PORT->Group[0].PINCFG[PT_PIN].reg = 0;             // detach, no input buffer
    PORT->Group[0].DIRCLR.reg = (1u << PT_PIN);
}

// ---------------------------------------------------------------------------
// Direction control
// ---------------------------------------------------------------------------

static void enter_idle(void) {
    tc4_disable();
    TC4->COUNT16.INTENCLR.reg = TC_INTENCLR_MC0;
    detachInterrupt(digitalPinToInterrupt(PT_ARDUINO_PIN));
    g_dir = DIR_IDLE;
    led_gpio_off();
    pt_gate_off();
}

static void enter_tx(void) {
    detachInterrupt(digitalPinToInterrupt(PT_ARDUINO_PIN));
    pt_gate_off();
    led_gpio_off();                                    // LED is GPIO, parked off
    tc4_disable();
    TC4->COUNT16.CTRLA.bit.SWRST = 1;
    tc4_sync();

    g_dir       = DIR_TX;
    tx_is_irda  = (g_encoding == ENC_IRDA) ? 1u : 0u;
    tx_bit_count = 0;
    tx_bit_queue = 0;

    // MFRQ: CC0 is TOP, COUNT auto-resets and MC0 fires once per bit period.
    TC4->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16
                           | TC_CTRLA_WAVEGEN_MFRQ
                           | TC_CTRLA_PRESCALER_DIV16;
    uint16_t period = (uint16_t)((TX_TICK_HZ / g_tx_baud) - 1u);
    TC4->COUNT16.CC[0].reg = period;
    tc4_sync();

    // IrDA '0' pulse = 3/16 bit; busy-wait loop count (~3 cycles/iter on M0+).
    uint32_t pulse_cycles = (F_CPU_HZ * 3UL) / (16UL * g_tx_baud);
    uint32_t loops        = pulse_cycles / 3UL;
    if (loops < 1) loops = 1;
    tx_pulse_loops = loops;

    NVIC_EnableIRQ(TC4_IRQn);
    TC4->COUNT16.INTENCLR.reg = TC_INTENCLR_MC0;
    TC4->COUNT16.CTRLA.bit.ENABLE = 1;
    tc4_sync();
}

static void enter_rx(void) {
    led_gpio_off();
    g_rx_bit_ticks = baud_to_rx_ticks(g_rx_baud);
#if RX_NOISE_FILTERING
    g_rx_spread_loops = 1000000UL / g_rx_baud;   // ~1/16-bit sample spacing (approx)
#endif
    g_rx_state = ST_IDLE;
    g_irda_pulse_mask = 0;
    g_ring_head = g_ring_tail = 0;
    g_rx_ferr = g_rx_bufovf = 0;
    g_rx_ferr_reported = g_rx_bufovf_reported = 0;

    tc4_disable();
    TC4->COUNT16.CTRLA.bit.SWRST = 1;
    tc4_sync();

    g_dir = DIR_RX;

    // NFRQ free-running 16-bit counter at RX_TICK_HZ. CC0 = the rescheduled
    // sample / end-of-byte compare; PT edges are timestamped against COUNT in
    // pt_edge_isr (no TC capture channel).
    TC4->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16
                           | TC_CTRLA_WAVEGEN_NFRQ
                           | TC_CTRLA_PRESCALER_DIV256;
    tc4_sync();

    TC4->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0;
    NVIC_EnableIRQ(TC4_IRQn);                          // MC0 armed per byte by the edge ISR
    TC4->COUNT16.CTRLA.bit.ENABLE = 1;
    tc4_sync();

    // PT (D8 / PA07) falling-edge interrupt -> pt_edge_isr. INPUT first so the
    // input buffer is on and PT_READ()/PORT->IN works for NRZ bit-sampling; the
    // external pull-up holds the line idle-HIGH.
    pinMode(PT_ARDUINO_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(PT_ARDUINO_PIN), pt_edge_isr, FALLING);

#if RX_NOISE_FILTERING
    // Enable the EIC majority glitch filter on the PT line (samples the input
    // over 3 GCLK_EIC cycles and takes the majority, rejecting spikes on the
    // sluggish rising edge). The Arduino core's attachInterrupt doesn't set it.
    // EIC->CONFIG is enable-protected, so toggle ENABLE around the write.
    // PA07 = EXTINT7 -> CONFIG[0].FILTEN7.
    EIC->CTRL.bit.ENABLE = 0;
    while (EIC->STATUS.bit.SYNCBUSY) {}
    EIC->CONFIG[0].reg |= EIC_CONFIG_FILTEN7;
    EIC->CTRL.bit.ENABLE = 1;
    while (EIC->STATUS.bit.SYNCBUSY) {}
#endif
}

// Blocking transmit of a blob, byte by byte, reusing the TX bit-clock ISR.
static void send_blob(const uint8_t *buf, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        uint16_t frame = ((uint16_t)buf[i] << 1) | ((uint16_t)1u << 9);  // start=0, stop=1
        __disable_irq();
        tx_bit_queue = frame;
        tx_bit_count = 10;
        __enable_irq();
        tc4_write_count(0);                            // full first bit period
        TC4->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0;
        TC4->COUNT16.INTENSET.reg = TC_INTENSET_MC0;
        while (tx_bit_count > 0) { /* spin until this byte drains */ }
    }
    LED_OFF();
}

// ---------------------------------------------------------------------------
// USB framing (identical to modem_arduino_uno.cpp)
// ---------------------------------------------------------------------------

static void usb_send(uint8_t type, const uint8_t *payload, uint16_t len) {
    uint8_t hdr[4] = { USB_SOF, type, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8) };
    Serial.write(hdr, 4);
    if (len) Serial.write(payload, len);
}

static inline void usb_send_empty(uint8_t type) { usb_send(type, nullptr, 0); }
static inline void usb_send_err(uint8_t code)    { usb_send(MSG_ERR, &code, 1); }

enum RxCmdState : uint8_t { C_SOF, C_TYPE, C_LEN_LO, C_LEN_HI, C_PAYLOAD };
static RxCmdState g_cstate = C_SOF;
static uint8_t    g_ctype;
static uint16_t   g_clen;
static uint16_t   g_cidx;
static uint8_t    g_cbuf[MAX_FRAME];

static void handle_command(uint8_t type, const uint8_t *p, uint16_t len) {
    switch (type) {
        case CMD_CONFIG:
            if (len != 9) { usb_send_err(ERR_BAD_LEN); return; }
            g_tx_baud  = (uint32_t)p[0] | ((uint32_t)p[1] << 8)
                       | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
            g_rx_baud  = (uint32_t)p[4] | ((uint32_t)p[5] << 8)
                       | ((uint32_t)p[6] << 16) | ((uint32_t)p[7] << 24);
            g_encoding = (p[8] == ENC_IRDA) ? ENC_IRDA : ENC_NRZ;
            g_configured = true;
            enter_idle();
            usb_send_empty(MSG_OK);
            break;

        case CMD_TX: {
            if (!g_configured)   { usb_send_err(ERR_NOT_CFG); return; }
            if (len < 2)         { usb_send_err(ERR_BAD_LEN); return; }
            uint8_t tx_flags = p[0];
            enter_tx();
            send_blob(p + 1, len - 1);
            if (tx_flags & TX_FLAG_AUTO_RX) {
                enter_rx();         // listen before reporting done
            } else {
                enter_idle();
            }
            usb_send_empty(MSG_TX_DONE);
            break;
        }

        case CMD_RX:
            if (!g_configured) { usb_send_err(ERR_NOT_CFG); return; }
            enter_rx();
            usb_send_empty(MSG_OK);
            break;

        case CMD_STOP:
            enter_idle();
            usb_send_empty(MSG_OK);
            break;

        case CMD_PING: {
            uint8_t v = MODEM_PROTO_VERSION;
            usb_send(MSG_PONG, &v, 1);
            break;
        }

        default:
            usb_send_err(ERR_BAD_CMD);
            break;
    }
}

static void poll_usb_commands(void) {
    while (Serial.available()) {
        uint8_t b = (uint8_t)Serial.read();
        switch (g_cstate) {
            case C_SOF:    if (b == USB_SOF) g_cstate = C_TYPE; break;
            case C_TYPE:   g_ctype = b; g_cstate = C_LEN_LO; break;
            case C_LEN_LO: g_clen = b; g_cstate = C_LEN_HI; break;
            case C_LEN_HI:
                g_clen |= (uint16_t)b << 8;
                g_cidx = 0;
                if (g_clen > MAX_FRAME) {
                    usb_send_err(ERR_BAD_LEN);
                    g_cstate = C_SOF;
                } else {
                    g_cstate = (g_clen == 0) ? C_SOF : C_PAYLOAD;
                    if (g_clen == 0) handle_command(g_ctype, g_cbuf, 0);
                }
                break;
            case C_PAYLOAD:
                g_cbuf[g_cidx++] = b;
                if (g_cidx >= g_clen) {
                    handle_command(g_ctype, g_cbuf, g_clen);
                    g_cstate = C_SOF;
                }
                break;
        }
    }
}

// Drain decoded RX bytes from the ring to the host (one MSG_RX per loop pass),
// then surface decode-error counts when they change.
static void forward_rx_bytes(void) {
    if (g_dir != DIR_RX) return;
    uint8_t batch[64];
    uint8_t n = 0;
    while (n < sizeof(batch) && g_ring_tail != g_ring_head) {
        batch[n++] = g_ring[g_ring_tail];
        g_ring_tail = (uint16_t)((g_ring_tail + 1u) & RING_MASK);
    }
    if (n) usb_send(MSG_RX, batch, n);

    uint16_t ferr = g_rx_ferr, bufovf = g_rx_bufovf;   // snapshot volatile
    if (ferr != g_rx_ferr_reported || bufovf != g_rx_bufovf_reported) {
        g_rx_ferr_reported   = ferr;
        g_rx_bufovf_reported = bufovf;
        uint8_t pl[4] = { (uint8_t)ferr,   (uint8_t)(ferr >> 8),
                          (uint8_t)bufovf, (uint8_t)(bufovf >> 8) };
        usb_send(MSG_RX_STATUS, pl, 4);
    }
}

void setup() {
    Serial.begin(USB_BAUD);
    tc4_clock_once();
    enter_idle();
}

void loop() {
    poll_usb_commands();
    forward_rx_bytes();
}

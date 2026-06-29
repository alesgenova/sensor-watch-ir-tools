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
//
// DUAL-MODE RX. CMD_CONFIG's rx_mode byte selects the RX front-end:
//   * RX_DIGITAL (0, default): the edge detector described above (EIC falling-edge
//     ISR + TC bit sampling). Needs the light to swing the pin across a clean
//     digital threshold.
//   * RX_ANALOG  (1): a polled-ADC software UART. The ADC free-runs on PA07/AIN7
//     and TC4 ticks an auto-selected RX_OVERSAMPLE samples/bit; each sample is
//     thresholded against an ADAPTIVELY LEARNED dark/light pair (bootstrapped
//     from the guaranteed light start bit + dark stop bit of every frame) and fed
//     to an oversampling receiver. This recovers data when the received light is
//     too weak for a digital transition. It self-tunes (no user parameters) and
//     squelches noise via a measured min-contrast floor. Host flag: --analog-rx.
//     TX is shared and unchanged. See the ANALOG RX section below.

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
#define MODEM_PROTO_VERSION 2

// host -> modem
enum : uint8_t {
    CMD_CONFIG = 0x01,   // payload: tx_baud(4 LE) rx_baud(4 LE) enc(1) rx_mode(1)
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
    MSG_RX_TUNE   = 0x16,  // DIAGNOSTIC (analog RX): lvl_dark(2) lvl_light(2)
                           // noise(2) thr(2) min_contrast(2) oversample(1) flags(1)
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

// RX front-end selector (CMD_CONFIG rx_mode byte). Read in the TC4 ISR + enter_rx.
enum RxMode : uint8_t { RX_DIGITAL = 0, RX_ANALOG = 1 };
static volatile RxMode g_rx_mode = RX_DIGITAL;

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

// ===========================================================================
// ANALOG RX front-end (g_rx_mode == RX_ANALOG).
//
// The ADC free-runs on PA07/AIN7; TC4 ticks a_oversample samples/bit and its ISR
// feeds each sample to a_process_sample(). A DATA SLICER (a_track_env) keeps a
// peak (dark) and valley (light) envelope of the signal and slices at their
// midpoint; a_classify turns each sample into a logical bit and an oversampling
// UART receiver assembles frames. The envelopes attack fast / decay slow, so they
// self-calibrate amplitude and follow ambient drift but can't be dragged into the
// signal (the failure mode of the earlier EMA estimator). A fixed min-contrast
// (envelope span) floor squelches noise. Reliability guards: an idle-before-start
// arm gate, a saturation timeout, and per-frame contrast validation.
// ===========================================================================

// Tunables -- all self-scaling; no user parameters reach here.
#define A_OVERSAMPLE_MAX   32u
#define A_OVERSAMPLE_MIN    8u
#define A_ADC_CEIL_SPS  60000u   // ~ADC throughput with the long sample time below
#define A_ADC_SAMPLEN      31u   // ADC SAMPCTRL: long S/H for the 100k high-Z source

// Data slicer (peak/valley envelope detectors). The dark level is the HIGH
// envelope and the light level the LOW envelope of the signal; the slice
// threshold is their midpoint. Each envelope ATTACKS fast toward a new extreme
// and otherwise DECAYS toward the live sample (so the rate is proportional to how
// far it is from the signal -- a big DC/ambient shift, e.g. the pedestal that
// appears when a transmission starts, is closed quickly; small wobble is ignored).
// Decaying toward the SIGNAL (not toward the other envelope) is what makes onset
// re-acquisition fast; the constant is slower in-frame (to ride out a run of one
// symbol that leaves a rail untouched for ~10 bits) and faster while idle (to snap
// to a new DC and to squelch between frames). Envelopes can't be dragged into the
// signal by its own excursions (the bug that collapsed the old estimator). Kept in
// Q6 fixed point so the decay has sub-count resolution.
#define A_ENV_Q             6u   // envelope fixed-point fractional bits
#define A_ATK_SHIFT         2u   // attack toward a new extreme (~1/4 of the gap per sample)
#define A_DECAY_FRAME      11u   // in-frame decay toward the signal (~2048-sample TC)
#define A_DECAY_IDLE        7u   // idle decay (fast DC re-acquire / squelch, ~128-sample TC)
#define A_MIN_CONTRAST     60u   // squelch: need >= this envelope span to call it data
#define A_HYST_SHIFT        2u   // classify hysteresis = span >> 2 around the midpoint

// NRZ bit-decision sample point, as a fraction of the bit. Both optical edges are
// RC-limited and SYMMETRIC (measured: rise ~= fall), and each transition sits at
// the START of its cell, so the settled region is roughly [edge_frac, 1.0]. Sample
// past the transition but with margin before the next cell boundary: 2/3 centres
// us in that window for baud where the edge is < ~half a bit (i.e. run the link
// slow enough -- ~100-150 baud for a 100k pull-up -- that this holds).
#define A_NRZ_SAMPLE_NUM    2u
#define A_NRZ_SAMPLE_DEN    3u

#define A_ARM_BITS          1u   // bit-times of clean dark needed to (re)arm a start
#define A_SAT_BITS         15u   // light longer than this many bits => saturation

// Envelopes (Q6 ADC counts) and the levels/thresholds derived from them.
static volatile int32_t  a_env_hi_q     = 2048 << A_ENV_Q;  // dark / high envelope
static volatile int32_t  a_env_lo_q     = 2048 << A_ENV_Q;  // light / low envelope
static volatile uint16_t a_lvl_dark     = 2048;   // = a_env_hi_q >> Q (for diagnostics)
static volatile uint16_t a_lvl_light    = 2048;   // = a_env_lo_q >> Q
static volatile uint16_t a_min_contrast = A_MIN_CONTRAST;   // fixed squelch floor
static volatile uint16_t a_thr_lo       = 0;      // adc <  a_thr_lo -> light
static volatile uint16_t a_thr_hi       = 0;      // adc >  a_thr_hi -> dark (hysteresis)
static volatile uint16_t a_thr_dec      = 0;      // bit-decision threshold (= midpoint)
static volatile bool     a_line_light   = false;  // hysteretic logical line state

// Receiver FSM (reuses RxState ST_IDLE / ST_RECEIVING).
static volatile uint8_t  a_oversample   = A_OVERSAMPLE_MIN;
static volatile uint8_t  a_samp_in_bit  = 0;
static volatile uint8_t  a_bit_index    = 0;      // 0=start, 1..8=data, 9=stop
static volatile uint8_t  a_byte_data    = 0;
static volatile bool     a_armed        = false;
static volatile uint16_t a_idle_dark    = 0;      // consecutive dark idle samples
static volatile uint16_t a_light_run    = 0;      // consecutive light samples (sat. guard)
static volatile uint16_t a_bit_lvl      = 0;      // NRZ: ADC at the settled sample point
static volatile bool     a_bit_pulse    = false;  // IrDA: light pulse seen in 1st half
static volatile uint16_t a_frame_lmin   = 0;      // lightest ADC this frame
static volatile uint16_t a_frame_dmax   = 0;      // darkest ADC this frame
static volatile RxState  a_state        = ST_IDLE;

// --- ADC, free-running on PA07 / AIN7 ---
static inline void adc_sync(void) { while (ADC->STATUS.bit.SYNCBUSY) {} }

static void adc_start_freerun(void) {
    // Let the core load calibration, set the reference, route PA07's pinmux to
    // analog, and point INPUTCTRL.MUXPOS at AIN7 (D8 == ADC channel 7).
    analogReadResolution(12);
    (void)analogRead(PT_ARDUINO_PIN);

    ADC->CTRLA.bit.ENABLE = 0; adc_sync();
    // Long sample time for the high-impedance 100k source; FREERUN so RESULT
    // always holds the latest conversion (the tick ISR just reads it).
    ADC->SAMPCTRL.reg = A_ADC_SAMPLEN;
    ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV16
                   | ADC_CTRLB_RESSEL_12BIT
                   | ADC_CTRLB_FREERUN;
    adc_sync();
    ADC->CTRLA.bit.ENABLE = 1; adc_sync();
    ADC->SWTRIG.bit.START = 1;
}

static void adc_stop(void) {
    ADC->CTRLA.bit.ENABLE = 0; adc_sync();
    ADC->CTRLB.bit.FREERUN = 0; adc_sync();
}

// Advance the peak/valley envelopes with one sample, then re-derive the slice
// thresholds. Runs on EVERY sample (idle and in-frame) so the envelopes always
// reflect the live signal. Fast attack to each rail; slow symmetric decay toward
// the midpoint so they follow DC drift and recover from transients.
static inline void a_track_env(uint16_t adc) {
    int32_t s = (int32_t)adc << A_ENV_Q;
    // Slower decay inside a frame (protect symbol runs), faster while idle.
    uint8_t dk = (a_state == ST_RECEIVING) ? A_DECAY_FRAME : A_DECAY_IDLE;
    // Each envelope: attack toward a new extreme, else decay toward the sample.
    a_env_hi_q += (s - a_env_hi_q) >> ((s > a_env_hi_q) ? A_ATK_SHIFT : dk);
    a_env_lo_q += (s - a_env_lo_q) >> ((s < a_env_lo_q) ? A_ATK_SHIFT : dk);
    if (a_env_hi_q < a_env_lo_q) {                      // crossed: collapse to the DC
        int32_t mid = (a_env_hi_q + a_env_lo_q) >> 1;
        a_env_hi_q = a_env_lo_q = mid;
    }

    uint16_t hi = (uint16_t)(a_env_hi_q >> A_ENV_Q);
    uint16_t lo = (uint16_t)(a_env_lo_q >> A_ENV_Q);
    a_lvl_dark = hi; a_lvl_light = lo;
    uint16_t span = (hi > lo) ? (uint16_t)(hi - lo) : 0u;
    uint16_t mid  = (uint16_t)(((uint32_t)hi + lo) >> 1);
    uint16_t hyst = (uint16_t)(span >> A_HYST_SHIFT);
    a_thr_dec = mid;
    a_thr_lo  = (mid > hyst) ? (uint16_t)(mid - hyst) : 0u;   // below -> light
    a_thr_hi  = (uint16_t)(mid + hyst);                       // above -> dark
}

static inline bool a_classify(uint16_t adc) {
    if (a_line_light) { if (adc >= a_thr_hi) a_line_light = false; }
    else              { if (adc <  a_thr_lo) a_line_light = true;  }
    return a_line_light;
}

static inline void a_to_idle(bool armed) {
    a_state = ST_IDLE; a_armed = armed; a_idle_dark = 0; a_light_run = 0;
}

static void a_finalize_frame(bool framing_ok) {
    // Per-frame contrast validation squelches noise: a frame that "decodes" out
    // of a flat line (span below the floor) is rejected. The envelopes do the
    // level learning continuously, so nothing is nudged here.
    uint16_t contrast = (a_frame_dmax > a_frame_lmin)
                      ? (uint16_t)(a_frame_dmax - a_frame_lmin) : 0u;
    if (!framing_ok) {
        g_rx_ferr++;
        a_to_idle(/*armed=*/false);  // real garbage (bad stop): demand fresh idle
    } else if (contrast >= a_min_contrast) {
        byte_received(a_byte_data);
        a_to_idle(/*armed=*/true);   // a clean dark stop bit IS the inter-frame idle
    } else {
        // Framing OK but below the squelch floor: just noise. Drop the byte but
        // stay ARMED -- demanding re-idle here is what made us miss the first real
        // start bit after a noisy lull.
        a_to_idle(/*armed=*/true);
    }
}

// Decode the just-completed bit and advance. Mirrors the digital decode's byte
// assembly: NRZ uses the bit-centre level; IrDA uses "a pulse fell in this bit".
static void a_finalize_bit(void) {
    uint8_t idx = a_bit_index;
    if (g_encoding == ENC_NRZ) {
        bool dark = (a_bit_lvl >= a_thr_dec);             // dark == logical 1
        if (idx == 0)      { if (dark) { a_to_idle(false); return; } }  // false start
        else if (idx <= 8) { if (dark) a_byte_data |= (uint8_t)(1u << (idx - 1u)); }
        else               { a_finalize_frame(/*ok=*/dark); return; }   // stop = mark
    } else {                                              // IrDA: pulse == logical 0
        bool pulse = a_bit_pulse;
        if (idx == 0)      { if (!pulse) { a_to_idle(false); return; } }
        else if (idx <= 8) { if (!pulse) a_byte_data |= (uint8_t)(1u << (idx - 1u)); }
        else               { a_finalize_frame(/*ok=*/!pulse); return; } // stop = no pulse
    }
    a_bit_index = (uint8_t)(idx + 1u);
}

// Feed one ADC sample to the oversampling receiver (called from the TC4 ISR).
static void a_process_sample(uint16_t adc) {
    a_track_env(adc);                 // update envelopes + thresholds every sample
    bool light = a_classify(adc);

    if (a_state == ST_IDLE) {
        // Start-bit / leading-edge detection. The line idles HIGH (dark); a real
        // transmission opens with a bright pulse -- the level dropping a genuine
        // amount below the (possibly noisy/drifting) dark baseline. Triggering on
        // THIS absolute drop, not on the midpoint classifier, is what keeps a
        // near-zero-span squelched baseline from tripping fake starts on noise and
        // leaving us mid-fake-frame (misaligned) when the real burst arrives.
        bool real_pulse = ((int32_t)adc < (int32_t)a_lvl_dark - (int32_t)A_MIN_CONTRAST);
        if (!real_pulse) {
            a_light_run = 0;             // a non-pulse sample == idle/mark
            if (a_idle_dark < 0xFFFFu) a_idle_dark++;
            if (!a_armed && a_idle_dark >= (uint16_t)(a_oversample * A_ARM_BITS))
                a_armed = true;
            return;
        }
        if (!a_armed) { a_idle_dark = 0; return; }   // not settled (e.g. recovery ramp)
        // armed + a real bright pulse => leading edge of a start bit; anchor here.
        a_state       = ST_RECEIVING;
        a_bit_index   = 0;
        a_byte_data   = 0;
        a_samp_in_bit = 0;
        a_frame_lmin  = adc;
        a_frame_dmax  = adc;
        a_bit_lvl     = adc;
        a_bit_pulse   = true;        // the start-bit dip is itself a pulse
        a_light_run   = 0;
        // fall through to accumulate this sample (sample 0 of the start bit)
    }

    // ---- accumulate this sample into the current bit window ----
    if (adc < a_frame_lmin) a_frame_lmin = adc;
    if (adc > a_frame_dmax) a_frame_dmax = adc;

    if (g_encoding == ENC_NRZ) {
        // Sample once in the settled region (A_NRZ_SAMPLE_NUM/DEN of the bit). Both
        // edges are RC-limited and complete near the start of the cell, so a single
        // point past the transition reads the true level -- and, unlike a late-half
        // peak, it can't be fooled by an unfinished symmetric edge bleeding in.
        uint8_t pt = (uint8_t)(((uint16_t)a_oversample * A_NRZ_SAMPLE_NUM)
                               / A_NRZ_SAMPLE_DEN);
        if (a_samp_in_bit == pt) a_bit_lvl = adc;
    } else {
        // IrDA pulses sit in the first 3/16 of the cell; only look there, so a slow
        // recovery tail from the previous bit can't smear in as a false pulse.
        uint8_t half = (uint8_t)(a_oversample >> 1);
        if (a_samp_in_bit < half && light) a_bit_pulse = true;
    }

    if (light) { if (a_light_run < 0xFFFFu) a_light_run++; } else a_light_run = 0;
    if (a_light_run > (uint16_t)(a_oversample * A_SAT_BITS)) { a_to_idle(false); return; }

    if (++a_samp_in_bit < a_oversample) return;

    a_finalize_bit();
    a_samp_in_bit = 0;
    a_bit_pulse   = false;
}

// ---------------------------------------------------------------------------
// PT edge interrupt: timestamps each falling edge (light pulse / NRZ start bit)
// by reading the free-running TC4 COUNT. Registered via the Arduino core's
// attachInterrupt in enter_rx. This is the ICP1/CAPT analogue; the SAMD21 TC's
// event-capture can't do single-edge timestamps (PPW/PWP only), so we stamp here.
// Ported from modem_arduino_uno.cpp's TIMER1_CAPT_vect (CC1 read -> COUNT read).
// ---------------------------------------------------------------------------

static void pt_edge_isr(void) {
    if (g_dir != DIR_RX || g_rx_mode != RX_DIGITAL) return;   // analog RX uses the ADC
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

    // ---- Analog RX: every tick reads the latest free-running ADC conversion
    //      and feeds the oversampling receiver. ----
    if (g_rx_mode == RX_ANALOG) {
        if (flags & TC_INTFLAG_MC0) {
            TC4->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0;
            a_process_sample((uint16_t)ADC->RESULT.reg);
        }
        return;
    }

    // ---- Digital RX compare: NRZ bit-sample tick, or IrDA end-of-byte. Edges
    //      arrive via the PT edge ISR (pt_edge_isr), not here. ----
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
    detachInterrupt(digitalPinToInterrupt(PT_ARDUINO_PIN));  // no-op if not attached
    adc_stop();                                              // no-op if analog RX unused
    g_dir = DIR_IDLE;
    led_gpio_off();
    pt_gate_off();
}

static void enter_tx(void) {
    detachInterrupt(digitalPinToInterrupt(PT_ARDUINO_PIN));
    adc_stop();
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

static void enter_rx_digital(void) {
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

static void enter_rx_analog(void) {
    led_gpio_off();

    // Pick oversampling: as high as the ADC can sustain at this baud, capped.
    // (At high baud A_ADC_CEIL/baud < MIN; we clamp to MIN and it undersamples --
    // analog RX is the low-baud / weak-light path, so that's an accepted edge.)
    uint32_t ovs = A_ADC_CEIL_SPS / (g_rx_baud ? g_rx_baud : 1u);
    if (ovs > A_OVERSAMPLE_MAX) ovs = A_OVERSAMPLE_MAX;
    if (ovs < A_OVERSAMPLE_MIN) ovs = A_OVERSAMPLE_MIN;
    ovs &= ~1u;                                    // keep it even (clean bit centre)
    a_oversample = (uint8_t)ovs;

    // Reset the receiver FSM.
    a_state = ST_IDLE; a_armed = false;
    a_samp_in_bit = 0; a_bit_index = 0; a_byte_data = 0;
    a_bit_pulse = false; a_line_light = false;
    a_idle_dark = 0; a_light_run = 0;
    g_ring_head = g_ring_tail = 0;
    g_rx_ferr = g_rx_bufovf = 0;
    g_rx_ferr_reported = g_rx_bufovf_reported = 0;

    tc4_disable();
    TC4->COUNT16.CTRLA.bit.SWRST = 1;
    tc4_sync();

    g_dir = DIR_RX;

    // Start the ADC and seed the dark level from a handful of idle conversions
    // (the line is assumed quiescent/dark when RX is entered).
    adc_start_freerun();
    uint32_t acc = 0;
    for (uint8_t i = 0; i < 16; i++) {
        while (!ADC->INTFLAG.bit.RESRDY) { /* spin for a conversion */ }
        acc += ADC->RESULT.reg;
    }
    // Seed the slicer: high envelope at the (dark) idle level, low envelope one
    // min-contrast below it, so there's a usable threshold from the first frame.
    // If no real light ever arrives, the low envelope decays back up and squelches.
    uint16_t seed = (uint16_t)(acc >> 4);
    uint16_t seed_lo = (seed > A_MIN_CONTRAST) ? (uint16_t)(seed - A_MIN_CONTRAST) : 0u;
    a_env_hi_q = (int32_t)seed    << A_ENV_Q;
    a_env_lo_q = (int32_t)seed_lo << A_ENV_Q;
    a_track_env(seed);                                    // derive initial thresholds
    a_armed = true;                                       // seeded from idle
    a_idle_dark = (uint16_t)(a_oversample * A_ARM_BITS);

    // TC4 MFRQ tick at a_oversample * rx_baud; MC0 stays armed for every tick.
    uint32_t tick_hz = (uint32_t)a_oversample * (g_rx_baud ? g_rx_baud : 1u);
    uint16_t period  = (uint16_t)((TX_TICK_HZ / tick_hz) - 1u);
    TC4->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16
                           | TC_CTRLA_WAVEGEN_MFRQ
                           | TC_CTRLA_PRESCALER_DIV16;
    TC4->COUNT16.CC[0].reg = period;
    tc4_sync();

    TC4->COUNT16.INTFLAG.reg  = TC_INTFLAG_MC0;
    TC4->COUNT16.INTENSET.reg = TC_INTENSET_MC0;
    NVIC_EnableIRQ(TC4_IRQn);
    TC4->COUNT16.CTRLA.bit.ENABLE = 1;
    tc4_sync();
}

static void enter_rx(void) {
    if (g_rx_mode == RX_ANALOG) enter_rx_analog();
    else                        enter_rx_digital();
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
            if (len != 10) { usb_send_err(ERR_BAD_LEN); return; }
            g_tx_baud  = (uint32_t)p[0] | ((uint32_t)p[1] << 8)
                       | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
            g_rx_baud  = (uint32_t)p[4] | ((uint32_t)p[5] << 8)
                       | ((uint32_t)p[6] << 16) | ((uint32_t)p[7] << 24);
            g_encoding = (p[8] == ENC_IRDA) ? ENC_IRDA : ENC_NRZ;
            g_rx_mode  = (p[9] == RX_ANALOG) ? RX_ANALOG : RX_DIGITAL;
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

// Throttled diagnostic: surface the analog autotuner state so the bench can watch
// the levels/threshold converge. New message type; non-analog hosts ignore it.
static void emit_rx_tune(void) {
    if (g_dir != DIR_RX || g_rx_mode != RX_ANALOG) return;
    static uint32_t last_ms = 0;
    uint32_t now = millis();
    if ((uint32_t)(now - last_ms) < 250u) return;
    last_ms = now;

    uint16_t dark = a_lvl_dark, light = a_lvl_light;
    uint16_t thr = a_thr_dec, mc = a_min_contrast;
    uint16_t span = (dark > light) ? (uint16_t)(dark - light) : 0u;
    uint8_t locked = (span >= mc) ? 1u : 0u;
    uint8_t pl[12] = {
        (uint8_t)dark,  (uint8_t)(dark >> 8),
        (uint8_t)light, (uint8_t)(light >> 8),
        (uint8_t)span,  (uint8_t)(span >> 8),    // was "noise"; now the live envelope span
        (uint8_t)thr,   (uint8_t)(thr >> 8),
        (uint8_t)mc,    (uint8_t)(mc >> 8),
        a_oversample,   locked,
    };
    usb_send(MSG_RX_TUNE, pl, sizeof(pl));
}

void setup() {
    Serial.begin(USB_BAUD);
    tc4_clock_once();
    enter_idle();
}

void loop() {
    poll_usb_commands();
    forward_rx_bytes();
    emit_rx_tune();
}

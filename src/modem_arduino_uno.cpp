// Arduino UNO half-duplex IR modem firmware for sensor-watch-ir-tools.
//
// This is a DUMB modem: it knows nothing about frames, CRCs, ACKs, UF2, or
// the flashing protocol. All of that lives on the host (bin/firmware_flasher.py). The
// modem only does two hardware jobs, on command:
//
//   * TX: bit-bang a blob of bytes out the IR LED at the configured tx baud /
//         encoding (Timer1 CTC bit clock, adapted from platform/arduino_uno.cpp).
//   * RX: decode bytes off the phototransistor at the configured rx baud /
//         encoding (Timer1 input capture, adapted from main_receiver.cpp) and
//         forward every decoded byte to the host verbatim.
//
// Because TX and RX both need Timer1 (and would otherwise both define
// TIMER1_COMPA_vect), this firmware is self-contained rather than reusing the
// Transmitter/Receiver as separate translation units: it owns one mode-aware
// copy of each Timer1 ISR and fully reconfigures Timer1 on every direction
// flip. Only one direction is ever active at a time (the link is half-duplex).
//
// Pins (same as the TX/RX firmwares):
//   * IR LED          -> D9  (PB1)   driven during TX
//   * phototransistor -> D8  (PB0 / ICP1)  sampled during DIGITAL RX
//   * phototransistor -> A0  (PC0 / ADC0)  sampled during ANALOG RX (--analog-rx)
//
// DUAL-MODE RX (CMD_CONFIG rx_mode byte): RX_DIGITAL (default) uses the Timer1
// input-capture edge detector below; RX_ANALOG free-runs the ADC and slices the
// phototransistor in software (peak/valley data slicer), recovering data when the
// light is too weak to swing the digital pin. The ATmega's ADC can't read D8, so
// analog RX reads A0 -- wire the PT output to A0 too (it can share the D8 node).
// See modem_xiao_samd21.cpp for the slicer rationale; the logic is identical.
//
// Turn cycle, driven by the host:
//   host --CMD_TX(flags, frame)--> modem transmits the frame. If the frame's
//   TX_FLAG_AUTO_RX is set it IMMEDIATELY flips to RX (so it's listening
//   before the watch starts its reply) and streams back every decoded byte as
//   MSG_RX; if the flag is clear it just goes idle (fire-and-forget). Either
//   way it then sends MSG_TX_DONE. When listening, the host slides a window
//   over the MSG_RX bytes looking for the ACK it expects, runs its own
//   timeout, and either sends the next CMD_TX or re-sends the same one to
//   retransmit. The modem stays in RX until the next command preempts it.
//   Whether to listen is purely the host's call, per frame; the modem holds
//   no ACK policy of its own.
//
//   The host can also enter RX directly with CMD_RX (no preceding TX) to just
//   listen, and return to idle at any time with CMD_STOP. The TX_FLAG_AUTO_RX
//   path stays separate from CMD_RX because the modem must already be in RX
//   before it reports MSG_TX_DONE; a host-issued CMD_RX after the fact would
//   race the start of the watch's reply. Note CMD_STOP cannot abort a TX in
//   flight: send_blob() is blocking, so a TX always runs to completion before
//   the modem reads the next command.
//
// USB framing (both directions), fixed 115200 baud, decoupled from IR baud:
//   [SOF=0xA5] [TYPE] [LEN_LO] [LEN_HI] [payload (LEN bytes)]
// USB-CDC is reliable, so there's no CRC on this layer; the SOF only lets a
// reader resync after a reset.

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>

// ---------------------------------------------------------------------------
// Pin config
// ---------------------------------------------------------------------------

// TX: D9 = PB1. LED on = logical '0' / start bit (light = 0), matching the watch's
// optical convention. Independent of the host RX board polarity (see RX_IDLE_HIGH).
#define TX_BIT          1            // PB1 / D9
#define LED_ON()        do { PORTB |=  _BV(TX_BIT); } while (0)
#define LED_OFF()       do { PORTB &= ~_BV(TX_BIT); } while (0)

// RX: D8 = PB0 = ICP1.
#define RX_PIN_READ()   ((PINB & _BV(PB0)) != 0)

// ---- Receiver board polarity -------------------------------------------------
// Set to match how YOUR phototransistor front-end reads at the RX pin (measure
// with a multimeter: block vs. shine IR on the PT, read the pin at idle):
//   RX_IDLE_HIGH 1 : no light -> pin HIGH, light -> pin LOW  (active-low PT, idle-high)
//   RX_IDLE_HIGH 0 : no light -> pin LOW,  light -> pin HIGH  (active-high PT, idle-low)
#define RX_IDLE_HIGH    1
//
// Derived from RX_IDLE_HIGH -- don't edit these; flip RX_IDLE_HIGH above instead.
#if RX_IDLE_HIGH
#define RX_LOGICAL(pin_high)  ((pin_high) ? 1u : 0u)  // idle/mark = HIGH: data bit == pin level
#define RX_CAPTURE_EDGE       0                        // start bit / IrDA pulse = falling edge
#else
#define RX_LOGICAL(pin_high)  ((pin_high) ? 0u : 1u)  // idle/mark = LOW: data bit inverted
#define RX_CAPTURE_EDGE       _BV(ICES1)               // start bit / IrDA pulse = rising edge
#endif

// Timer1 /64 prescaler for RX capture -> 250 kHz tick (4 us). See main_receiver.cpp
// for why /64 is the only prescaler that covers the whole 50..9600 baud range.
#define RX_TICKS_PER_SEC 250000UL

static inline uint16_t baud_to_rx_ticks(uint32_t baud) {
    return (uint16_t)((RX_TICKS_PER_SEC + (baud >> 1)) / baud);
}

// ---------------------------------------------------------------------------
// USB protocol constants
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
// CMD_TX flag bits (the first byte of the CMD_TX payload). The host decides
// per frame whether the modem should listen for a reply afterwards; the modem
// holds no policy of its own.
enum : uint8_t {
    TX_FLAG_AUTO_RX = 0x01,  // after sending, flip to RX and forward bytes;
                             // if clear, go idle (fire-and-forget)
};
// modem -> host
enum : uint8_t {
    MSG_RX      = 0x10,  // payload: raw decoded optical bytes
    MSG_TX_DONE = 0x11,  // payload: (none)
    MSG_OK      = 0x12,  // payload: (none)
    MSG_ERR     = 0x13,  // payload: error code(1)
    MSG_PONG    = 0x15,  // payload: proto_version(1)
    MSG_RX_TUNE = 0x16,  // DIAGNOSTIC (analog RX): lvl_dark(2) lvl_light(2) span(2)
                         // thr(2) min_contrast(2) oversample(1) flags(1)
};
enum : uint8_t {
    ERR_BAD_LEN  = 1,
    ERR_BAD_CMD  = 2,
    ERR_NOT_CFG  = 3,
};

// Largest CMD_TX payload we accept = one wire frame. The host's biggest frame
// is frame overhead (12) + a 512-byte max chunk = 524; round up for slack.
#define MAX_FRAME  540

// ---------------------------------------------------------------------------
// Encoding + runtime config
// ---------------------------------------------------------------------------

enum Encoding : uint8_t { ENC_NRZ = 0, ENC_IRDA = 1 };  // matches the watch / host

static uint32_t g_tx_baud  = 3600;       // main-context only (enter_tx)
static uint32_t g_rx_baud  = 150;        // main-context only (enter_rx)
static volatile Encoding g_encoding = ENC_IRDA;   // read in the Timer1 ISRs
static bool     g_configured = false;

// RX front-end selector (CMD_CONFIG rx_mode byte). Read in the Timer1 ISRs + enter_rx.
enum RxMode : uint8_t { RX_DIGITAL = 0, RX_ANALOG = 1 };
static volatile RxMode g_rx_mode = RX_DIGITAL;

enum Dir : uint8_t { DIR_IDLE = 0, DIR_TX = 1, DIR_RX = 2 };
static volatile Dir g_dir = DIR_IDLE;

// ---------------------------------------------------------------------------
// TX state (Timer1 CTC bit clock), adapted from platform/arduino_uno.cpp
// ---------------------------------------------------------------------------

static volatile uint16_t tx_bit_queue;   // 10-bit frame, LSB out first
static volatile uint8_t  tx_bit_count;   // bits remaining (0 = idle)
static volatile uint8_t  tx_is_irda;
static volatile uint16_t tx_pulse_loops; // IrDA '0' pulse width, in 4-cycle loops

static inline void busy_wait_loops(uint16_t loops) {
    __asm__ __volatile__ ("1: sbiw %0,1 \n\t brne 1b \n\t" : "+w" (loops));
}

// ---------------------------------------------------------------------------
// RX state (Timer1 input capture), adapted from main_receiver.cpp
// ---------------------------------------------------------------------------

static volatile uint16_t g_rx_bit_ticks  = RX_TICKS_PER_SEC / 150;
enum RxState : uint8_t { ST_IDLE = 0, ST_RECEIVING = 1 };
static volatile RxState  g_rx_state        = ST_IDLE;
static volatile uint16_t g_byte_t0         = 0;
static volatile uint8_t  g_byte_data       = 0;
static volatile uint8_t  g_byte_nrz_index  = 0;
static volatile uint16_t g_irda_pulse_mask = 0;

// ISR -> main-loop ring buffer (power-of-2). Never touch Serial from an ISR.
#define RING_SIZE  128u
#define RING_MASK  (RING_SIZE - 1u)
static volatile uint8_t g_ring[RING_SIZE];
static volatile uint8_t g_ring_head;
static volatile uint8_t g_ring_tail;

static inline void byte_received(uint8_t b) {
    uint8_t next = (uint8_t)((g_ring_head + 1u) & RING_MASK);
    if (next == g_ring_tail) return;     // overflow: drop (host owns recovery)
    g_ring[g_ring_head] = b;
    g_ring_head = next;
}

// ===========================================================================
// ANALOG RX front-end (g_rx_mode == RX_ANALOG) -- ported from modem_xiao_samd21.cpp.
//
// The ADC free-runs on A0/ADC0; Timer1 (CTC) ticks a_oversample samples/bit and
// its COMPA ISR feeds each sample to a_process_sample(). A DATA SLICER keeps a
// peak (dark) and valley (light) envelope, slices at their midpoint, starts a
// frame on a real drop below the dark baseline, and reads each NRZ bit once in its
// settled region. Self-tuning, with arm/saturation/contrast guards. Identical to
// the SAMD21 modem (see its header), scaled to the ATmega's 10-bit ADC.
// ===========================================================================

#define RX_ADC_CHANNEL      0u    // A0 / PC0 / ADC0 (analog-RX phototransistor pin)
#define A_ADC_MAX        1023u    // 10-bit ADC full scale

#define A_OVERSAMPLE_MAX   32u
#define A_OVERSAMPLE_MIN    8u
#define A_ADC_CEIL_SPS   9000u    // ATmega free-run ADC @ 125 kHz clock ~ 9.2 kSPS
#define A_ENV_Q             6u    // envelope fixed-point fractional bits
#define A_ATK_SHIFT         2u    // attack toward a new extreme
#define A_DECAY_FRAME      11u    // in-frame decay toward the signal (protect runs)
#define A_DECAY_IDLE        7u    // idle decay (fast DC re-acquire / squelch)
#define A_MIN_CONTRAST     15u    // squelch span floor (~SAMD's 60, 12->10 bit)
#define A_HYST_SHIFT        2u    // classify hysteresis = span >> 2
#define A_NRZ_SAMPLE_NUM    2u    // NRZ bit-decision sample point = 2/3 of the cell
#define A_NRZ_SAMPLE_DEN    3u    // (past the RC edge, which sits at the cell start)
#define A_ARM_BITS          1u    // bit-times of clean dark to (re)arm a start
#define A_SAT_BITS         15u    // light longer than this many bits => saturation

static volatile int32_t  a_env_hi_q     = (int32_t)512 << A_ENV_Q;  // dark / high envelope
static volatile int32_t  a_env_lo_q     = (int32_t)512 << A_ENV_Q;  // light / low envelope
static volatile uint16_t a_lvl_dark     = 512;
static volatile uint16_t a_lvl_light    = 512;
static volatile uint16_t a_min_contrast = A_MIN_CONTRAST;
static volatile uint16_t a_thr_lo       = 0;
static volatile uint16_t a_thr_hi       = 0;
static volatile uint16_t a_thr_dec      = 0;
static volatile bool     a_line_light   = false;
static volatile uint8_t  a_oversample   = A_OVERSAMPLE_MIN;
static volatile uint8_t  a_samp_in_bit  = 0;
static volatile uint8_t  a_bit_index    = 0;     // 0=start, 1..8=data, 9=stop
static volatile uint8_t  a_byte_data    = 0;
static volatile bool     a_armed        = false;
static volatile uint16_t a_idle_dark    = 0;
static volatile uint16_t a_light_run    = 0;
static volatile uint16_t a_bit_lvl      = 0;     // NRZ: ADC at the settled sample point
static volatile bool     a_bit_pulse    = false; // IrDA: pulse seen in the 1st half
static volatile uint16_t a_frame_lmin   = 0;
static volatile uint16_t a_frame_dmax   = 0;
static volatile RxState  a_state        = ST_IDLE;

// --- ADC, free-running on the analog PT pin (A0 / ADC0) ---
static void adc_start_freerun(void) {
    DIDR0  |= _BV(RX_ADC_CHANNEL);                   // drop A0's digital input buffer
    ADMUX   = _BV(REFS0) | (RX_ADC_CHANNEL & 0x0F);  // AVcc ref, right-adjusted
    ADCSRB  = 0;                                     // free-running trigger source
    ADCSRA  = _BV(ADEN) | _BV(ADATE)                 // enable + auto-trigger (free-run)
            | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);  // /128 -> 125 kHz ADC clock
    ADCSRA |= _BV(ADSC);                             // kick the first conversion
}

static void adc_stop(void) { ADCSRA = 0; }

static inline uint16_t adc_read(void) {
    uint16_t raw = ADC;                              // latest free-run result
#if RX_IDLE_HIGH
    return raw;                                      // dark = HIGH already
#else
    return (uint16_t)(A_ADC_MAX - raw);              // invert so dark = HIGH
#endif
}

// Advance the peak/valley envelopes with one sample, then re-derive thresholds.
static inline void a_track_env(uint16_t adc) {
    int32_t s = (int32_t)adc << A_ENV_Q;
    uint8_t dk = (a_state == ST_RECEIVING) ? A_DECAY_FRAME : A_DECAY_IDLE;
    a_env_hi_q += (s - a_env_hi_q) >> ((s > a_env_hi_q) ? A_ATK_SHIFT : dk);
    a_env_lo_q += (s - a_env_lo_q) >> ((s < a_env_lo_q) ? A_ATK_SHIFT : dk);
    if (a_env_hi_q < a_env_lo_q) {                   // crossed: collapse to the DC
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
    a_thr_lo  = (mid > hyst) ? (uint16_t)(mid - hyst) : 0u;
    a_thr_hi  = (uint16_t)(mid + hyst);
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
    // Per-frame contrast validation squelches noise; the envelopes do the level
    // learning continuously. A clean stop pre-arms; garbage demands fresh idle; a
    // framed-but-too-weak frame drops the byte yet stays armed (don't miss the
    // first real start after a noisy lull).
    uint16_t contrast = (a_frame_dmax > a_frame_lmin)
                      ? (uint16_t)(a_frame_dmax - a_frame_lmin) : 0u;
    if (!framing_ok)                      a_to_idle(false);
    else if (contrast >= a_min_contrast) { byte_received(a_byte_data); a_to_idle(true); }
    else                                  a_to_idle(true);
}

static void a_finalize_bit(void) {
    uint8_t idx = a_bit_index;
    if (g_encoding == ENC_NRZ) {
        bool dark = (a_bit_lvl >= a_thr_dec);            // dark == logical 1
        if (idx == 0)      { if (dark) { a_to_idle(false); return; } }  // false start
        else if (idx <= 8) { if (dark) a_byte_data |= (uint8_t)(1u << (idx - 1u)); }
        else               { a_finalize_frame(/*ok=*/dark); return; }   // stop = mark
    } else {                                            // IrDA: pulse == logical 0
        bool pulse = a_bit_pulse;
        if (idx == 0)      { if (!pulse) { a_to_idle(false); return; } }
        else if (idx <= 8) { if (!pulse) a_byte_data |= (uint8_t)(1u << (idx - 1u)); }
        else               { a_finalize_frame(/*ok=*/!pulse); return; } // stop = no pulse
    }
    a_bit_index = (uint8_t)(idx + 1u);
}

// Feed one ADC sample to the oversampling receiver (called from the COMPA ISR).
static void a_process_sample(uint16_t adc) {
    a_track_env(adc);
    bool light = a_classify(adc);

    if (a_state == ST_IDLE) {
        // Start = a real bright pulse: the line dropping a genuine amount below the
        // dark baseline (not the jittery midpoint), so a near-zero-span squelched
        // baseline can't trip fake starts on noise and leave us misaligned.
        bool real_pulse = ((int32_t)adc < (int32_t)a_lvl_dark - (int32_t)A_MIN_CONTRAST);
        if (!real_pulse) {
            a_light_run = 0;
            if (a_idle_dark < 0xFFFFu) a_idle_dark++;
            if (!a_armed && a_idle_dark >= (uint16_t)(a_oversample * A_ARM_BITS))
                a_armed = true;
            return;
        }
        if (!a_armed) { a_idle_dark = 0; return; }
        a_state       = ST_RECEIVING;
        a_bit_index   = 0;
        a_byte_data   = 0;
        a_samp_in_bit = 0;
        a_frame_lmin  = adc;
        a_frame_dmax  = adc;
        a_bit_lvl     = adc;
        a_bit_pulse   = true;
        a_light_run   = 0;
    }

    if (adc < a_frame_lmin) a_frame_lmin = adc;
    if (adc > a_frame_dmax) a_frame_dmax = adc;

    if (g_encoding == ENC_NRZ) {
        // Single sample in the settled region (2/3); both RC edges sit at the cell
        // start, so this reads the true level without catching an unfinished edge.
        uint8_t pt = (uint8_t)(((uint16_t)a_oversample * A_NRZ_SAMPLE_NUM)
                               / A_NRZ_SAMPLE_DEN);
        if (a_samp_in_bit == pt) a_bit_lvl = adc;
    } else {
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
// Timer1 ISRs, mode-aware. Only one direction is armed at a time.
// ---------------------------------------------------------------------------

ISR(TIMER1_COMPA_vect) {
    if (g_dir == DIR_TX) {
        // --- TX bit clock: emit the next bit of the current 10-bit frame. ---
        uint8_t bit = tx_bit_queue & 1;
        tx_bit_queue >>= 1;
        if (tx_is_irda) {
            if (bit == 0) { LED_ON(); busy_wait_loops(tx_pulse_loops); LED_OFF(); }
        } else {
            if (bit == 0) LED_ON(); else LED_OFF();
        }
        if (--tx_bit_count == 0) {
            TIMSK1 &= ~_BV(OCIE1A);     // frame done; send_blob() re-arms per byte
        }
        return;
    }

    // --- Analog RX: every tick reads the latest free-run ADC and slices it. ---
    if (g_rx_mode == RX_ANALOG) {
        a_process_sample(adc_read());
        return;
    }

    // --- Digital RX: bit-sample tick (NRZ) or end-of-byte (IrDA). ---
    if (g_encoding == ENC_NRZ) {
        bool pin_high = RX_PIN_READ();
        uint8_t bit_n = g_byte_nrz_index;
        if (bit_n >= 1u && bit_n <= 8u) {
            uint8_t logical = RX_LOGICAL(pin_high);  // board polarity (RX_IDLE_HIGH)
            if (logical) g_byte_data |= (uint8_t)(1u << (bit_n - 1u));
        }
        if (bit_n < 9u) {
            g_byte_nrz_index = (uint8_t)(bit_n + 1u);
            OCR1A = (uint16_t)(g_byte_t0 + (uint16_t)(g_rx_bit_ticks * ((uint16_t)bit_n + 1u)
                                                      + (g_rx_bit_ticks >> 1)));
            return;
        }
        bool framing_ok = RX_LOGICAL(pin_high); // stop bit is logical '1' (mark)
        if (framing_ok) byte_received(g_byte_data);
        g_rx_state = ST_IDLE;
        TIMSK1 &= ~_BV(OCIE1A);
        TIFR1  =  _BV(ICF1);
        TIMSK1 |= _BV(ICIE1);
    } else {
        // IrDA end-of-byte: assemble data bits from the pulse mask.
        uint16_t mask = g_irda_pulse_mask;
        bool framing_ok = ((mask & (uint16_t)(1u << 9)) == 0u);
        uint8_t data = 0;
        for (uint8_t i = 0; i < 8; i++) {
            if ((mask & (uint16_t)(1u << (i + 1u))) == 0u) data |= (uint8_t)(1u << i);
        }
        if (framing_ok) byte_received(data);
        g_rx_state = ST_IDLE;
        g_irda_pulse_mask = 0;
        TIMSK1 &= ~_BV(OCIE1A);
        TIFR1  =  _BV(ICF1);
        TIMSK1 |= _BV(ICIE1);
    }
}

ISR(TIMER1_CAPT_vect) {
    if (g_dir != DIR_RX || g_rx_mode != RX_DIGITAL) return;  // analog RX uses the ADC
    uint16_t now = ICR1;

    if (g_rx_state == ST_IDLE) {
        g_byte_t0         = now;
        g_byte_data       = 0;
        g_irda_pulse_mask = 1u;          // start bit is '0' -> has a pulse
        g_byte_nrz_index  = 1;
        g_rx_state        = ST_RECEIVING;
        if (g_encoding == ENC_IRDA) {
            OCR1A = now + (uint16_t)(g_rx_bit_ticks * 10u);
        } else {
            OCR1A = now + (uint16_t)(g_rx_bit_ticks + (g_rx_bit_ticks >> 1));
        }
        TIFR1  |= _BV(OCF1A);
        TIMSK1 |= _BV(OCIE1A);
    } else if (g_encoding == ENC_IRDA) {
        uint16_t delta = now - g_byte_t0;
        uint16_t bit_n = (uint16_t)((delta + (g_rx_bit_ticks >> 1)) / g_rx_bit_ticks);
        if (bit_n >= 1u && bit_n <= 8u) {
            g_irda_pulse_mask |= (uint16_t)(1u << bit_n);
        } else if (bit_n == 9u) {
            g_irda_pulse_mask |= (uint16_t)(1u << 9);
        } else if (bit_n >= 10u) {
            // Start bit of the next back-to-back byte beat the end-of-byte
            // COMPA. Finalize the current byte and pivot to a fresh T0.
            uint16_t mask = g_irda_pulse_mask;
            bool framing_ok = ((mask & (uint16_t)(1u << 9)) == 0u);
            uint8_t data = 0;
            for (uint8_t i = 0; i < 8; i++) {
                if ((mask & (uint16_t)(1u << (i + 1u))) == 0u) data |= (uint8_t)(1u << i);
            }
            if (framing_ok) byte_received(data);
            g_byte_t0         = now;
            g_irda_pulse_mask = 1u;
            OCR1A = now + (uint16_t)(g_rx_bit_ticks * 10u);
            TIFR1 |= _BV(OCF1A);
        }
    }
}

// ---------------------------------------------------------------------------
// Direction control
// ---------------------------------------------------------------------------

static void enter_idle() {
    adc_stop();            // no-op if analog RX wasn't running
    cli();
    g_dir   = DIR_IDLE;
    TIMSK1  = 0;            // no Timer1 interrupts
    TCNT1   = 0;
    sei();
    LED_OFF();
}

static void enter_tx() {
    adc_stop();
    // Timer1 CTC bit clock at tx baud. (Adapted from arduino_uno.cpp begin().)
    DDRB |= _BV(TX_BIT);
    LED_OFF();

    uint32_t bit_cell_cycles = F_CPU / g_tx_baud;
    uint8_t  cs_bits;
    uint16_t ocr_value;
    if (bit_cell_cycles <= 65536UL) {
        cs_bits   = (1 << CS10);                  // prescaler 1
        ocr_value = (uint16_t)(bit_cell_cycles - 1);
    } else {
        cs_bits   = (1 << CS11);                  // prescaler 8
        ocr_value = (uint16_t)(bit_cell_cycles / 8 - 1);
    }

    cli();
    g_dir  = DIR_TX;
    TIMSK1 = 0;
    TCCR1A = 0;                                    // exit any PWM mode
    TCCR1B = (1 << WGM12) | cs_bits;              // CTC, TOP=OCR1A
    OCR1A  = ocr_value;
    TCNT1  = 0;
    TIFR1  = (1 << OCF1A);
    tx_is_irda = (g_encoding == ENC_IRDA) ? 1 : 0;
    uint32_t pulse_cycles = (F_CPU * 3UL) / (16UL * g_tx_baud);
    uint32_t loops        = pulse_cycles >> 2;
    if (loops < 1)        loops = 1;
    if (loops > 0xFFFFUL) loops = 0xFFFFUL;
    tx_pulse_loops = (uint16_t)loops;
    tx_bit_count = 0;
    tx_bit_queue = 0;
    sei();
}

static void enter_rx_digital() {
    adc_stop();            // in case we were in analog RX
    g_rx_bit_ticks = baud_to_rx_ticks(g_rx_baud);
    cli();
    g_dir   = DIR_RX;
    g_rx_state        = ST_IDLE;
    g_irda_pulse_mask = 0;
    g_ring_head = g_ring_tail = 0;
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;
    // Normal mode, /64 prescaler, 4-sample noise canceller. Capture edge follows
    // board polarity (RX_CAPTURE_EDGE): falling for idle-high, rising for idle-low.
    TCCR1B = RX_CAPTURE_EDGE | _BV(ICNC1) | _BV(CS11) | _BV(CS10);
    TIFR1  = _BV(ICF1) | _BV(OCF1A);
    TIMSK1 = _BV(ICIE1);
    sei();
}

static void enter_rx_analog() {
    // Pick oversampling: as high as the ADC can sustain at this baud, capped.
    uint16_t ovs = (uint16_t)(A_ADC_CEIL_SPS / (g_rx_baud ? g_rx_baud : 1u));
    if (ovs > A_OVERSAMPLE_MAX) ovs = A_OVERSAMPLE_MAX;
    if (ovs < A_OVERSAMPLE_MIN) ovs = A_OVERSAMPLE_MIN;
    ovs &= ~1u;
    a_oversample = (uint8_t)ovs;

    // Reset the receiver FSM.
    a_state = ST_IDLE; a_armed = false;
    a_samp_in_bit = 0; a_bit_index = 0; a_byte_data = 0;
    a_bit_pulse = false; a_line_light = false;
    a_idle_dark = 0; a_light_run = 0;
    g_ring_head = g_ring_tail = 0;

    TIMSK1 = 0;                                  // no Timer1 ISR while we set up

    // Start the ADC and seed the dark level from a handful of idle conversions.
    adc_start_freerun();
    uint32_t acc = 0;
    for (uint8_t i = 0; i < 16; i++) {
        while (!(ADCSRA & _BV(ADIF))) { /* wait conversion */ }
        ADCSRA |= _BV(ADIF);                     // clear (write-1) for the next
        acc += ADC;
    }
    uint16_t seed = (uint16_t)(acc >> 4);
#if !RX_IDLE_HIGH
    seed = (uint16_t)(A_ADC_MAX - seed);         // keep "dark = high" internally
#endif
    uint16_t seed_lo = (seed > A_MIN_CONTRAST) ? (uint16_t)(seed - A_MIN_CONTRAST) : 0u;
    a_env_hi_q = (int32_t)seed    << A_ENV_Q;
    a_env_lo_q = (int32_t)seed_lo << A_ENV_Q;
    a_track_env(seed);                           // derive initial thresholds
    a_armed = true;
    a_idle_dark = (uint16_t)(a_oversample * A_ARM_BITS);

    // Timer1 CTC tick at a_oversample * rx_baud (prescaler 1).
    uint32_t tick_hz = (uint32_t)a_oversample * (g_rx_baud ? g_rx_baud : 1u);
    uint16_t ocr = (uint16_t)((F_CPU / tick_hz) - 1u);
    cli();
    g_dir  = DIR_RX;
    TCCR1A = 0;
    TCCR1B = _BV(WGM12) | _BV(CS10);             // CTC, TOP=OCR1A, prescaler 1
    OCR1A  = ocr;
    TCNT1  = 0;
    TIFR1  = _BV(OCF1A);
    TIMSK1 = _BV(OCIE1A);
    sei();
}

static void enter_rx() {
    if (g_rx_mode == RX_ANALOG) enter_rx_analog();
    else                        enter_rx_digital();
}

// Blocking transmit of a blob, byte by byte, reusing the TX bit-clock ISR.
static void send_blob(const uint8_t *buf, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        uint16_t frame = ((uint16_t)buf[i] << 1) | ((uint16_t)1 << 9);  // start=0, stop=1
        cli();
        tx_bit_queue = frame;
        tx_bit_count = 10;
        TCNT1        = 0;
        TIFR1        = (1 << OCF1A);
        TIMSK1       = (1 << OCIE1A);
        sei();
        while (tx_bit_count > 0) { /* spin until this byte drains */ }
    }
    LED_OFF();
}

// ---------------------------------------------------------------------------
// USB framing
// ---------------------------------------------------------------------------

static void usb_send(uint8_t type, const uint8_t *payload, uint16_t len) {
    uint8_t hdr[4] = { USB_SOF, type, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8) };
    Serial.write(hdr, 4);
    if (len) Serial.write(payload, len);
}

static inline void usb_send_empty(uint8_t type) { usb_send(type, nullptr, 0); }
static inline void usb_send_err(uint8_t code)    { usb_send(MSG_ERR, &code, 1); }

// Incoming command parser state machine.
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
            if (len < 2)         { usb_send_err(ERR_BAD_LEN); return; }  // flags + >=1 byte
            uint8_t tx_flags = p[0];
            enter_tx();
            send_blob(p + 1, len - 1);
            if (tx_flags & TX_FLAG_AUTO_RX) {
                // Flip to RX FIRST so we're listening before the watch replies,
                // THEN tell the host transmission is done.
                enter_rx();
            } else {
                enter_idle();   // fire-and-forget: no reply expected
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

static void poll_usb_commands() {
    while (Serial.available()) {
        uint8_t b = (uint8_t)Serial.read();
        switch (g_cstate) {
            case C_SOF:    if (b == USB_SOF) g_cstate = C_TYPE; break;
            case C_TYPE:   g_ctype = b; g_cstate = C_LEN_LO; break;
            case C_LEN_LO: g_clen = b; g_cstate = C_LEN_HI; break;
            case C_LEN_HI:
                g_clen |= (uint16_t)b << 8;
                g_cidx = 0;
                if (g_clen > MAX_FRAME) {     // bogus / desync: drop and resync
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

// Drain decoded RX bytes from the ring and forward them to the host, batched
// into one MSG_RX per loop pass to amortize USB framing overhead.
static void forward_rx_bytes() {
    if (g_dir != DIR_RX) return;
    uint8_t batch[64];
    uint8_t n = 0;
    while (n < sizeof(batch) && g_ring_tail != g_ring_head) {
        batch[n++] = g_ring[g_ring_tail];
        g_ring_tail = (uint8_t)((g_ring_tail + 1u) & RING_MASK);
    }
    if (n) usb_send(MSG_RX, batch, n);
}

// Throttled diagnostic: surface the analog data-slicer state (levels/threshold).
static void emit_rx_tune() {
    if (g_dir != DIR_RX || g_rx_mode != RX_ANALOG) return;
    static uint32_t last_ms = 0;
    uint32_t now = millis();
    if ((uint32_t)(now - last_ms) < 250u) return;
    last_ms = now;

    uint16_t dark, light, thr, mc;
    uint8_t ovs;
    cli();                                       // 16-bit reads aren't atomic on AVR
    dark = a_lvl_dark; light = a_lvl_light;
    thr = a_thr_dec; mc = a_min_contrast; ovs = a_oversample;
    sei();
    uint16_t span = (dark > light) ? (uint16_t)(dark - light) : 0u;
    uint8_t locked = (span >= mc) ? 1u : 0u;
    uint8_t pl[12] = {
        (uint8_t)dark,  (uint8_t)(dark >> 8),
        (uint8_t)light, (uint8_t)(light >> 8),
        (uint8_t)span,  (uint8_t)(span >> 8),
        (uint8_t)thr,   (uint8_t)(thr >> 8),
        (uint8_t)mc,    (uint8_t)(mc >> 8),
        ovs, locked,
    };
    usb_send(MSG_RX_TUNE, pl, sizeof(pl));
}

void setup() {
    Serial.begin(USB_BAUD);
    DDRB  |= _BV(TX_BIT);    // LED output
    LED_OFF();
    DDRB  &= ~_BV(PB0);      // phototransistor input (D8/ICP1)
    enter_idle();
}

void loop() {
    poll_usb_commands();
    forward_rx_bytes();
    emit_rx_tune();
}

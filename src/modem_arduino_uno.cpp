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
//   * phototransistor -> D8  (PB0 / ICP1)  sampled during RX
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
#define MODEM_PROTO_VERSION 1

// host -> modem
enum : uint8_t {
    CMD_CONFIG = 0x01,   // payload: tx_baud(4 LE) rx_baud(4 LE) enc(1)
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

    // --- RX: bit-sample tick (NRZ) or end-of-byte (IrDA). ---
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
    if (g_dir != DIR_RX) return;
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
    cli();
    g_dir   = DIR_IDLE;
    TIMSK1  = 0;            // no Timer1 interrupts
    TCNT1   = 0;
    sei();
    LED_OFF();
}

static void enter_tx() {
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

static void enter_rx() {
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
}

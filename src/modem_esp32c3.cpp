// ESP32-C3-DevKitM-1 half-duplex IR modem for sensor-watch-ir-tools.
//
// This is the ESP32-C3 counterpart to modem_arduino_uno.cpp and
// modem_xiao_samd21.cpp: the host sees the identical framed USB protocol while
// this firmware only turns bytes into light and light back into bytes. GPIO7 is
// the active-high IR LED output; GPIO4 is the active-low, idle-high PT input.
// GPIO4 is ADC1_CH4, so the default analog receive mode and --digital-rx both
// use the same wire. GPIO7 is a regular exposed GPIO. Both pins are exposed by
// DevKitM-1; the pinout avoids GPIO2/GPIO8/GPIO9 (boot strapping),
// GPIO12-GPIO17 (flash signals), and GPIO18/GPIO19 (USB-JTAG).
//
// TX uses the C3's RMT peripheral as its bit clock, matching the hardware-timed
// approach of the UNO and XIAO modems. RX is a non-blocking,
// micros()-scheduled UART/IrDA state machine. The watch's return link is
// normally 300 baud, leaving ample polling margin while retaining the existing
// configurable baud and encoding behaviour.

#include <Arduino.h>
#include <esp32-hal-rmt.h>
#include <esp_adc/adc_continuous.h>
#include <esp_heap_caps.h>

#if !defined(ARDUINO_ARCH_ESP32)
#error "modem_esp32c3.cpp targets ESP32-C3. Build env:modem_esp32c3."
#endif

#define LED_PIN 7u
#define PT_PIN  4u
#define USB_BAUD 115200u
#define USB_SOF 0xA5u
#define MODEM_PROTO_VERSION 2u
#define MAX_FRAME 540u

enum : uint8_t { CMD_CONFIG = 0x01, CMD_TX = 0x02, CMD_RX = 0x03,
                 CMD_STOP = 0x04, CMD_PING = 0x05 };
enum : uint8_t { TX_FLAG_AUTO_RX = 0x01 };
enum : uint8_t { MSG_RX = 0x10, MSG_TX_DONE = 0x11, MSG_OK = 0x12,
                 MSG_ERR = 0x13, MSG_RX_STATUS = 0x14, MSG_PONG = 0x15,
                 MSG_RX_TUNE = 0x16 };
enum : uint8_t { ERR_BAD_LEN = 1, ERR_BAD_CMD = 2, ERR_NOT_CFG = 3 };
enum Encoding : uint8_t { ENC_NRZ = 0, ENC_IRDA = 1 };
enum RxMode : uint8_t { RX_DIGITAL = 0, RX_ANALOG = 1 };
enum Dir : uint8_t { DIR_IDLE = 0, DIR_TX = 1, DIR_RX = 2 };
enum RxState : uint8_t { ST_IDLE = 0, ST_RECEIVING = 1 };

static uint32_t g_tx_baud = 3600;
static uint32_t g_rx_baud = 150;
static Encoding g_encoding = ENC_IRDA;
static RxMode g_rx_mode = RX_ANALOG;
static Dir g_dir = DIR_IDLE;
static bool g_configured = false;

// ISR-free ring: all producers and consumers run in loop(), but keeping the
// same bounded buffer semantics as the other boards makes future timer ports
// safe and avoids a USB write in the decode path.
#define RING_SIZE 256u
#define RING_MASK (RING_SIZE - 1u)
static uint8_t g_ring[RING_SIZE];
static uint16_t g_ring_head, g_ring_tail;
static uint16_t g_rx_ferr, g_rx_bufovf;
static uint16_t g_rx_ferr_reported, g_rx_bufovf_reported;

// Worst case is IrDA: one RMT symbol per serial bit. The biggest permitted
// command has MAX_FRAME bytes, each encoded as start + 8 data + stop bits.
static rmt_data_t g_tx_symbols[MAX_FRAME * 10u];

static RxState g_rx_state = ST_IDLE;
static uint32_t g_rx_t0, g_rx_bit_us;
static uint8_t g_rx_index, g_rx_data;
static uint16_t g_irda_pulse_mask;
static bool g_was_light;

// The ADC's DMA stream is the analog RX clock. Samples are acquired at a fixed
// hardware rate and decoded in order later, so USB/loop latency cannot move a
// UART decision point. 76.8 kS/s gives 256 samples/bit at 300 baud and still
// provides eight samples/bit at the public 9600-baud limit.
#define ANALOG_ADC_HZ       76800u
#define ANALOG_DMA_BUF_SIZE 512u
static adc_continuous_handle_t g_adc_handle;
static uint8_t *g_adc_dma_buf;
static uint16_t g_analog_oversample;
static uint16_t g_analog_sample_in_bit;
static uint8_t g_analog_bit_index, g_analog_byte_data;
static bool g_analog_bit_pulse;

// Analog slicer state. The pull-up makes dark read high and reflected/red LED
// light read low. Keep peak/valley envelopes and slice at their midpoint: a
// fixed fraction of the dark level would need an implausibly large swing when
// the PT idles near 3.3 V (a 0.3 V optical signal is only about 9% of that).
static uint16_t g_analog_dark = 4095;
static uint16_t g_analog_light = 4095;
static uint16_t g_analog_threshold = 4047;
static uint16_t g_analog_last_sample;
static bool g_analog_line_light;
static uint8_t g_analog_seed_count;
static uint32_t g_analog_seed_total;
static uint32_t g_last_tune_ms;

// 48 raw ADC counts is roughly 40 mV over the C3 ADC's 3.3 V input range.
// This leaves ample noise margin while accepting the observed 0.3 Vpp signal.
#define ANALOG_MIN_CONTRAST 48u

static inline bool elapsed(uint32_t now, uint32_t target) {
    return (int32_t)(now - target) >= 0;
}

static inline void led_on()  { digitalWrite(LED_PIN, HIGH); }
static inline void led_off() { digitalWrite(LED_PIN, LOW); }

static void byte_received(uint8_t b) {
    uint16_t next = (uint16_t)((g_ring_head + 1u) & RING_MASK);
    if (next == g_ring_tail) { g_rx_bufovf++; return; }
    g_ring[g_ring_head] = b;
    g_ring_head = next;
}

static bool analog_sample_is_light(uint16_t sample) {
    g_analog_last_sample = sample;
    if (g_rx_state == ST_IDLE) {
        // Reacquire the dark DC level between bytes, but only on samples that
        // are not a plausible light pulse. This prevents a start bit from
        // dragging its own reference downward.
        if (sample + ANALOG_MIN_CONTRAST >= g_analog_dark) {
            g_analog_dark = (uint16_t)((31u * g_analog_dark + sample) >> 5);
            // With no live pulse, let the old valley slowly converge so the
            // next start is judged against the current ambient level.
            g_analog_light = (uint16_t)((127u * g_analog_light + sample) >> 7);
        } else {
            // A real start bit gives us the low envelope immediately. This is
            // critical for the first 1.5-bit NRZ sample after the start edge.
            g_analog_light = sample;
        }
    } else {
        // Inside a byte, follow new extrema promptly and let stale values
        // relax slowly. Long runs of either symbol therefore retain a usable
        // slicing threshold.
        if (sample > g_analog_dark)
            g_analog_dark = (uint16_t)((3u * g_analog_dark + sample) >> 2);
        if (sample < g_analog_light)
            g_analog_light = sample;
        else
            // Keep the low envelope for a whole character. At 300 baud the
            // 76.8 kS/s sampler sees 256 samples per bit, so the former
            // 1/128 decay nearly erased a real light valley in one dark bit.
            // That lifted the threshold toward the saturated idle level and
            // caused otherwise clean frames to fail their CRC.
            g_analog_light = (uint16_t)((2047u * g_analog_light + sample) >> 11);
    }

    uint16_t span = g_analog_dark > g_analog_light
                  ? (uint16_t)(g_analog_dark - g_analog_light) : 0u;
    uint16_t midpoint = (uint16_t)(((uint32_t)g_analog_dark + g_analog_light) >> 1);
    uint16_t hysteresis = span >> 3;
    if (hysteresis < 8u) hysteresis = 8u;
    g_analog_threshold = midpoint;

    // Before a pulse establishes a valley, use a small absolute drop from the
    // idle baseline. Thereafter hysteresis around the adaptive midpoint keeps
    // noisy samples from manufacturing extra edges.
    if (span < ANALOG_MIN_CONTRAST) {
        g_analog_line_light = sample + ANALOG_MIN_CONTRAST < g_analog_dark;
    } else if (g_analog_line_light) {
        if (sample >= midpoint + hysteresis) g_analog_line_light = false;
    } else if (sample + hysteresis < midpoint) {
        g_analog_line_light = true;
    }
    return g_analog_line_light;
}

static inline bool digital_pt_is_light() {
    return digitalRead(PT_PIN) == LOW;
}

static void finish_rx_byte(uint8_t data, bool stop_is_pulse) {
    if (stop_is_pulse) g_rx_ferr++;
    else byte_received(data);
    g_rx_state = ST_IDLE;
}

static void analog_stop(void) {
    if (g_adc_handle != nullptr) {
        adc_continuous_stop(g_adc_handle);
        adc_continuous_deinit(g_adc_handle);
        g_adc_handle = nullptr;
    }
    if (g_adc_dma_buf != nullptr) {
        heap_caps_free(g_adc_dma_buf);
        g_adc_dma_buf = nullptr;
    }
}

static bool analog_start(void) {
    analog_stop();
    adc_continuous_handle_cfg_t handle_cfg = {
        .max_store_buf_size = ANALOG_DMA_BUF_SIZE * 8u,
        .conv_frame_size = ANALOG_DMA_BUF_SIZE,
    };
    if (adc_continuous_new_handle(&handle_cfg, &g_adc_handle) != ESP_OK) return false;

    adc_digi_pattern_config_t pattern = {
        .atten = ADC_ATTEN_DB_12,
        .channel = ADC_CHANNEL_4,
        .unit = ADC_UNIT_1,
        .bit_width = ADC_BITWIDTH_12,
    };
    adc_continuous_config_t cfg = {
        .pattern_num = 1,
        .adc_pattern = &pattern,
        .sample_freq_hz = ANALOG_ADC_HZ,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
#if CONFIG_IDF_TARGET_ESP32
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
#else
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
#endif
    };
    if (adc_continuous_config(g_adc_handle, &cfg) != ESP_OK ||
        adc_continuous_start(g_adc_handle) != ESP_OK) {
        analog_stop();
        return false;
    }
    g_adc_dma_buf = (uint8_t *)heap_caps_malloc(ANALOG_DMA_BUF_SIZE,
                                                  MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (g_adc_dma_buf == nullptr) { analog_stop(); return false; }
    uint32_t ovs = ANALOG_ADC_HZ / (g_rx_baud ? g_rx_baud : 1u);
    g_analog_oversample = (uint16_t)(ovs < 8u ? 8u : ovs);
    g_analog_sample_in_bit = 0;
    g_analog_bit_index = g_analog_byte_data = 0;
    g_analog_bit_pulse = false;
    g_analog_seed_count = 0;
    g_analog_seed_total = 0;
    return true;
}

static void analog_process_sample(uint16_t sample) {
    if (g_analog_seed_count < 32u) {
        g_analog_seed_total += sample;
        if (++g_analog_seed_count == 32u) {
            g_analog_dark = g_analog_light = (uint16_t)(g_analog_seed_total >> 5);
            g_analog_threshold = g_analog_dark;
            g_analog_line_light = false;
        }
        return;
    }

    bool light = analog_sample_is_light(sample);
    if (g_rx_state == ST_IDLE) {
        if ((int32_t)sample >= (int32_t)g_analog_dark - (int32_t)ANALOG_MIN_CONTRAST)
            return;
        g_rx_state = ST_RECEIVING;
        g_analog_bit_index = 0;
        g_analog_byte_data = 0;
        g_analog_sample_in_bit = 0;
        g_analog_bit_pulse = light;
    }

    uint16_t decision_at = (uint16_t)(((uint32_t)g_analog_oversample * 2u) / 3u);
    if (g_encoding == ENC_NRZ) {
        if (g_analog_sample_in_bit == decision_at)
            g_analog_bit_pulse = sample < g_analog_threshold;
    } else if (g_analog_sample_in_bit < (g_analog_oversample >> 1) && light) {
        g_analog_bit_pulse = true;
    }

    if (++g_analog_sample_in_bit < g_analog_oversample) return;
    bool pulse = g_analog_bit_pulse;
    if (g_analog_bit_index == 0u) {
        if (!pulse) g_rx_state = ST_IDLE;
    } else if (g_analog_bit_index <= 8u) {
        if (!pulse)
            g_analog_byte_data |= (uint8_t)(1u << (g_analog_bit_index - 1u));
    } else {
        finish_rx_byte(g_analog_byte_data, pulse);
    }
    g_analog_bit_index++;
    g_analog_sample_in_bit = 0;
    g_analog_bit_pulse = false;
}

static void poll_analog_rx(void) {
    if (g_adc_handle == nullptr) return;
    uint32_t bytes_read = 0;
    while (adc_continuous_read(g_adc_handle, g_adc_dma_buf, ANALOG_DMA_BUF_SIZE,
                               &bytes_read, 0) == ESP_OK) {
        for (uint32_t i = 0; i < bytes_read; i += SOC_ADC_DIGI_RESULT_BYTES) {
            adc_digi_output_data_t *out = (adc_digi_output_data_t *)(g_adc_dma_buf + i);
            analog_process_sample((uint16_t)out->type2.data);
        }
    }
}

static void enter_idle() {
    analog_stop();
    g_dir = DIR_IDLE;
    g_rx_state = ST_IDLE;
    led_off();
    pinMode(PT_PIN, INPUT);
}

static void reset_rx() {
    g_rx_state = ST_IDLE;
    g_rx_index = g_rx_data = 0;
    g_irda_pulse_mask = 0;
    g_was_light = g_rx_mode == RX_DIGITAL ? digital_pt_is_light() : false;
    g_ring_head = g_ring_tail = 0;
    g_rx_ferr = g_rx_bufovf = 0;
    g_rx_ferr_reported = g_rx_bufovf_reported = 0;
}

static void enter_rx() {
    led_off();
    pinMode(PT_PIN, INPUT);
    g_rx_bit_us = 1000000UL / (g_rx_baud ? g_rx_baud : 1u);
    if (g_rx_bit_us == 0) g_rx_bit_us = 1;
    g_dir = DIR_RX;
    reset_rx();
    if (g_rx_mode == RX_ANALOG) (void)analog_start();
}

static void enter_tx() {
    analog_stop();
    g_dir = DIR_TX;
    led_off();
}

static void send_blob(const uint8_t *buf, uint16_t len) {
    uint32_t bit_us = 1000000UL / (g_tx_baud ? g_tx_baud : 1u);
    if (bit_us == 0) bit_us = 1;
    uint32_t pulse_us = (bit_us * 3u) / 16u;
    if (pulse_us == 0) pulse_us = 1;
    size_t n = 0;
    if (g_encoding == ENC_IRDA) {
        for (uint16_t i = 0; i < len; i++) {
            uint16_t frame = ((uint16_t)buf[i] << 1) | (uint16_t)(1u << 9);
            for (uint8_t bit = 0; bit < 10; bit++) {
                bool mark = (frame & (uint16_t)(1u << bit)) != 0;
                // A logical zero is a 3/16-bit light pulse. A mark stays dark
                // for the entire bit cell; split it only because RMT symbols
                // carry two level/duration pairs.
                g_tx_symbols[n].duration0 = mark ? 1u : pulse_us;
                g_tx_symbols[n].level0 = LOW;
                if (!mark) g_tx_symbols[n].level0 = HIGH;
                g_tx_symbols[n].duration1 = mark ? bit_us - 1u : bit_us - pulse_us;
                g_tx_symbols[n].level1 = LOW;
                n++;
            }
        }
    } else {
        // Each RMT symbol contains two full NRZ bit cells.
        for (uint16_t i = 0; i < len; i++) {
            uint16_t frame = ((uint16_t)buf[i] << 1) | (uint16_t)(1u << 9);
            for (uint8_t bit = 0; bit < 10; bit += 2) {
                bool mark0 = (frame & (uint16_t)(1u << bit)) != 0;
                bool mark1 = (frame & (uint16_t)(1u << (bit + 1u))) != 0;
                g_tx_symbols[n].duration0 = bit_us;
                g_tx_symbols[n].level0 = mark0 ? LOW : HIGH;
                g_tx_symbols[n].duration1 = bit_us;
                g_tx_symbols[n].level1 = mark1 ? LOW : HIGH;
                n++;
            }
        }
    }
    // RMT handles long arrays by streaming them through its encoder. Its
    // timeout includes a margin for scheduler overhead, not optical timing.
    uint32_t timeout_ms = (uint32_t)(((uint64_t)len * 10u * 1000u) / g_tx_baud) + 1000u;
    (void)rmtWrite(LED_PIN, g_tx_symbols, n, timeout_ms);
    led_off();
}

static void begin_digital_rx(uint32_t now) {
    g_rx_t0 = now;
    g_rx_state = ST_RECEIVING;
    g_rx_index = 0;
    g_rx_data = 0;
    g_irda_pulse_mask = 1u; // Start bit is necessarily a pulse/zero.
}

static void poll_digital_rx() {
    bool light = digital_pt_is_light();
    uint32_t now = micros();

    if (g_rx_state == ST_IDLE) {
        if (light && !g_was_light) begin_digital_rx(now);
        g_was_light = light;
        return;
    }

    if (g_encoding == ENC_NRZ) {
        // First decision is 1.5 bits after the start edge; remaining decisions
        // are exactly one bit apart, with g_rx_index 0..8 = data then stop.
        uint32_t sample_at = g_rx_t0 + g_rx_bit_us + (g_rx_bit_us >> 1)
                           + (uint32_t)g_rx_index * g_rx_bit_us;
        if (elapsed(now, sample_at)) {
            if (g_rx_index < 8u) {
                if (!light) g_rx_data |= (uint8_t)(1u << g_rx_index);
            } else {
                finish_rx_byte(g_rx_data, !light);
            }
            g_rx_index++;
        }
    } else {
        if (light && !g_was_light) {
            uint32_t since = now - g_rx_t0;
            uint8_t cell = (uint8_t)((since + (g_rx_bit_us >> 1)) / g_rx_bit_us);
            if (cell < 10u) g_irda_pulse_mask |= (uint16_t)(1u << cell);
        }
        if (elapsed(now, g_rx_t0 + 10u * g_rx_bit_us)) {
            bool framing_ok = (g_irda_pulse_mask & (uint16_t)(1u << 9)) == 0u;
            uint8_t data = 0;
            for (uint8_t i = 0; i < 8; i++) {
                if ((g_irda_pulse_mask & (uint16_t)(1u << (i + 1u))) == 0u)
                    data |= (uint8_t)(1u << i);
            }
            finish_rx_byte(data, !framing_ok);
        }
    }
    g_was_light = light;
}

// NRZ samples each bit at its centre. IrDA records each light-pulse edge in its
// bit cell and assembles once the stop-bit cell has elapsed.
static void poll_rx() {
    if (g_dir != DIR_RX) return;
    if (g_rx_mode == RX_ANALOG) poll_analog_rx();
    else poll_digital_rx();
}

static void usb_send(uint8_t type, const uint8_t *payload, uint16_t len) {
    uint8_t hdr[4] = { USB_SOF, type, (uint8_t)len, (uint8_t)(len >> 8) };
    Serial.write(hdr, sizeof(hdr));
    if (len) Serial.write(payload, len);
}
static inline void usb_send_empty(uint8_t type) { usb_send(type, nullptr, 0); }
static inline void usb_send_err(uint8_t code) { usb_send(MSG_ERR, &code, 1); }

enum RxCmdState : uint8_t { C_SOF, C_TYPE, C_LEN_LO, C_LEN_HI, C_PAYLOAD };
static RxCmdState g_cstate = C_SOF;
static uint8_t g_ctype, g_cbuf[MAX_FRAME];
static uint16_t g_clen, g_cidx;

static void handle_command(uint8_t type, const uint8_t *p, uint16_t len) {
    switch (type) {
        case CMD_CONFIG:
            if (len != 10) { usb_send_err(ERR_BAD_LEN); return; }
            g_tx_baud = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                        ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
            g_rx_baud = (uint32_t)p[4] | ((uint32_t)p[5] << 8) |
                        ((uint32_t)p[6] << 16) | ((uint32_t)p[7] << 24);
            g_encoding = p[8] == ENC_IRDA ? ENC_IRDA : ENC_NRZ;
            g_rx_mode = p[9] == RX_ANALOG ? RX_ANALOG : RX_DIGITAL;
            g_configured = true;
            enter_idle(); usb_send_empty(MSG_OK); break;
        case CMD_TX:
            if (!g_configured) { usb_send_err(ERR_NOT_CFG); return; }
            if (len < 2) { usb_send_err(ERR_BAD_LEN); return; }
            enter_tx(); send_blob(p + 1, len - 1);
            if (p[0] & TX_FLAG_AUTO_RX) enter_rx(); else enter_idle();
            usb_send_empty(MSG_TX_DONE); break;
        case CMD_RX:
            if (!g_configured) { usb_send_err(ERR_NOT_CFG); return; }
            enter_rx(); usb_send_empty(MSG_OK); break;
        case CMD_STOP: enter_idle(); usb_send_empty(MSG_OK); break;
        case CMD_PING: { uint8_t v = MODEM_PROTO_VERSION; usb_send(MSG_PONG, &v, 1); break; }
        default: usb_send_err(ERR_BAD_CMD); break;
    }
}

static void poll_usb_commands() {
    while (Serial.available()) {
        uint8_t b = (uint8_t)Serial.read();
        switch (g_cstate) {
            case C_SOF: if (b == USB_SOF) g_cstate = C_TYPE; break;
            case C_TYPE: g_ctype = b; g_cstate = C_LEN_LO; break;
            case C_LEN_LO: g_clen = b; g_cstate = C_LEN_HI; break;
            case C_LEN_HI:
                g_clen |= (uint16_t)b << 8; g_cidx = 0;
                if (g_clen > MAX_FRAME) { usb_send_err(ERR_BAD_LEN); g_cstate = C_SOF; }
                else if (g_clen == 0) { handle_command(g_ctype, g_cbuf, 0); g_cstate = C_SOF; }
                else g_cstate = C_PAYLOAD;
                break;
            case C_PAYLOAD:
                g_cbuf[g_cidx++] = b;
                if (g_cidx == g_clen) { handle_command(g_ctype, g_cbuf, g_clen); g_cstate = C_SOF; }
                break;
        }
    }
}

static void forward_rx_bytes() {
    if (g_dir != DIR_RX) return;
    uint8_t batch[64]; uint8_t n = 0;
    while (n < sizeof(batch) && g_ring_tail != g_ring_head) {
        batch[n++] = g_ring[g_ring_tail];
        g_ring_tail = (uint16_t)((g_ring_tail + 1u) & RING_MASK);
    }
    if (n) usb_send(MSG_RX, batch, n);
    if (g_rx_ferr != g_rx_ferr_reported || g_rx_bufovf != g_rx_bufovf_reported) {
        g_rx_ferr_reported = g_rx_ferr; g_rx_bufovf_reported = g_rx_bufovf;
        uint8_t status[4] = { (uint8_t)g_rx_ferr, (uint8_t)(g_rx_ferr >> 8),
                              (uint8_t)g_rx_bufovf, (uint8_t)(g_rx_bufovf >> 8) };
        usb_send(MSG_RX_STATUS, status, sizeof(status));
    }
}

static void emit_rx_tune() {
    if (g_dir != DIR_RX || g_rx_mode != RX_ANALOG || millis() - g_last_tune_ms < 250u) return;
    g_last_tune_ms = millis();
    uint16_t span = g_analog_dark > g_analog_light ? g_analog_dark - g_analog_light : 0;
    uint8_t tune[12] = { (uint8_t)g_analog_dark, (uint8_t)(g_analog_dark >> 8),
                         (uint8_t)g_analog_light, (uint8_t)(g_analog_light >> 8),
                         (uint8_t)span, (uint8_t)(span >> 8),
                         (uint8_t)g_analog_threshold, (uint8_t)(g_analog_threshold >> 8),
                         (uint8_t)ANALOG_MIN_CONTRAST, 0,
                         (uint8_t)(g_analog_oversample > 255u ? 255u : g_analog_oversample),
                         (uint8_t)(span >= ANALOG_MIN_CONTRAST) };
    usb_send(MSG_RX_TUNE, tune, sizeof(tune));
}

void setup() {
    Serial.begin(USB_BAUD);
    pinMode(LED_PIN, OUTPUT);
    pinMode(PT_PIN, INPUT);
    analogReadResolution(12);
    analogSetPinAttenuation(PT_PIN, ADC_11db); // full 0--3.3 V PT range
    rmtInit(LED_PIN, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_2, 1000000); // 1 tick = 1 us
    rmtSetEOT(LED_PIN, LOW); // active-high LED: dark between frames
    enter_idle();
}

void loop() {
    poll_usb_commands();
    poll_rx();
    forward_rx_bytes();
    emit_rx_tune();
}

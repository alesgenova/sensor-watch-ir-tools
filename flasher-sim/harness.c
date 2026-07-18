/* Host-side validation harness for the watch's RAM-resident flasher.
 *
 * Compiles the REAL watch translation units -- firmware_flasher_core.c plus
 * one decoder backend TU (firmware_flasher_detools.c, or
 * firmware_flasher_ultrapatch.c with -DFIRMWARE_FLASHER_ULTRAPATCH). Any
 * non-ARM compile of those TUs is a hosted test build (no define needed:
 * they key on the cross-compiler's __arm__), which swaps only the hardware
 * layer (NVM, DSU, SERCOM, WFI) for the hooks defined here. Everything
 * else, including flasher_run itself, is the exact code the watch runs.
 *
 * Each scenario queues a complete session's frames into the fake RX stream
 * and calls flasher_run(); the run ends either with a verified EXIT
 * ("reboot", via the host_reboot hook) or by draining the stream (the
 * wfi_standby hook longjmps out where the watch would sleep). Scenarios:
 *
 *   1. full flash, block 0 handed off + blocks + EXIT     -> reboot, image ==
 *   2. clean patch apply (+ EXIT)                         -> reboot, image ==
 *   3. clean patch apply, in-flash shift mode (detools)   -> reboot, image ==
 *   4. every body frame duplicated (lost-ACK sim)         -> reboot, dups re-ACKed
 *   5. re-sent final body frame after the apply           -> re-ACKed via dispatch
 *   6. mid-apply takeover by a full flash + EXIT          -> reboot, image ==
 *   7. corrupt patch body -> park -> full-flash recovery  -> reboot, image ==
 *   8. wrong base image (ultrapatch only)                 -> reject before any write
 *
 * The fake flash lives in a MAP_32BIT mmap so the watch's uint32_t absolute
 * addresses are directly dereferenceable, exactly as on the SAM L22.
 */
#define _GNU_SOURCE
#include <assert.h>
#include <setjmp.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>

#include "firmware_flasher_core.h"
#ifdef FIRMWARE_FLASHER_ULTRAPATCH
#include "firmware_flasher_ultrapatch.h"
#endif

/* ---- fake flash ------------------------------------------------------- */

#define REGION_SIZE 0x3C000u   /* same span as the SAM L22 app-flash window */

/* core.h's writable-region bounds resolve to these in hosted test builds. */
uint32_t flasher_hosted_bootloader_end, flasher_hosted_app_flash_end;
static uint32_t g_base;

static void flash_reset(const uint8_t *ref, uint32_t ref_len) {
    memset((void *)(uintptr_t)g_base, 0xFF, REGION_SIZE);
    memcpy((void *)(uintptr_t)g_base, ref, ref_len);
}

static uint32_t g_rows_written;
static bool g_writes_fail;   /* fault injection: every row write reports failure */

bool firmware_flasher_write_row(uint32_t addr, const uint8_t *data, uint8_t cooldown_ticks) {
    (void)cooldown_ticks;   /* no battery to rest on the host */
    if (g_writes_fail) return false;
    if (!firmware_flasher_range_writable(addr, FLASHER_ROW_SIZE)) return false;
    if ((addr & (FLASHER_ROW_SIZE - 1u)) != 0) return false;
    memcpy((void *)(uintptr_t)addr, data, FLASHER_ROW_SIZE);
    g_rows_written++;
    return true;
}

uint32_t firmware_flasher_crc32(const uint8_t *data, uint32_t len) {
    uint32_t c = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < len; i++) {
        c ^= data[i];
        for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(c & 1u)));
    }
    return c ^ 0xFFFFFFFFu;
}

#ifdef FIRMWARE_FLASHER_ULTRAPATCH
/* The UltraPatch decoder TU addresses the compile-time image window
 * [FLASHER_ULTRAPATCH_IMAGE_BASE, +CAPACITY); translate onto the fake flash,
 * mirroring the watch wiring (write page -> bounds-checked row write). */
uint8_t host_flash_read(uint32_t absolute_addr);
bool    host_flash_write_page(uint32_t absolute_page_addr, const uint8_t *page);
uint8_t host_flash_read(uint32_t absolute_addr) {
    return *(uint8_t *)(uintptr_t)(g_base + (absolute_addr - FLASHER_ULTRAPATCH_IMAGE_BASE));
}
bool host_flash_write_page(uint32_t absolute_page_addr, const uint8_t *page) {
    return firmware_flasher_write_row(
        g_base + (absolute_page_addr - FLASHER_ULTRAPATCH_IMAGE_BASE), page, 0);
}
#endif

/* ---- fake link + session exits (hooks the core TU calls) -------------- */

static uint8_t *g_rx;
static size_t   g_rx_len, g_rx_pos, g_rx_cap;

static uint16_t g_acks[65536];
static size_t   g_ack_n;

static jmp_buf g_session;               /* longjmp target around flasher_run */
enum { SESSION_STARVED = 1, SESSION_REBOOT = 2 };

static void rx_clear(void) { g_rx_len = g_rx_pos = 0; g_ack_n = 0; }

static void rx_queue(const uint8_t *d, size_t n) {
    if (g_rx_len + n > g_rx_cap) {
        g_rx_cap = (g_rx_len + n) * 2 + 4096;
        g_rx = realloc(g_rx, g_rx_cap);
        assert(g_rx);
    }
    memcpy(g_rx + g_rx_len, d, n);
    g_rx_len += n;
}

bool rx_read_byte(uint8_t *out) {
    if (g_rx_pos >= g_rx_len) return false;
    *out = g_rx[g_rx_pos++];
    return true;
}

/* The watch would sleep here awaiting more RX; a drained stream ends the
 * scenario instead. */
void wfi_standby(void) { longjmp(g_session, SESSION_STARVED); }

/* NVIC_SystemReset after a verified EXIT. */
void host_reboot(void) { longjmp(g_session, SESSION_REBOOT); }

void send_ack(uint16_t id) {
    assert(g_ack_n < sizeof g_acks / sizeof g_acks[0]);
    g_acks[g_ack_n++] = id;
}

/* Run one whole session through the REAL flasher_run; returns the SESSION_*
 * exit reason. */
static int run_session(const firmware_flasher_patch_t *patch,
                       uint16_t first_id, uint32_t first_addr,
                       bool first_is_patch, uint16_t first_len,
                       const uint8_t *first_block) {
    int why = setjmp(g_session);
    if (why == 0)
        flasher_run(patch, first_id, first_addr, first_is_patch, first_len, first_block);
    return why;
}

/* ---- frame building (mirrors bin/serial_frame.py build_frame) --------- */

static void queue_frame(uint16_t id, uint8_t flags, const uint8_t *payload, uint16_t len) {
    uint8_t buf[4 + 4 + IR_FLASHER_BLOCK_PAYLOAD + 4 + 4];
    assert(len <= IR_FLASHER_BLOCK_PAYLOAD);
    uint16_t phys = (uint16_t)((len + 3u) & ~3u);
    uint16_t lenflags = (uint16_t)(len | ((uint16_t)flags << 10));
    size_t n = 0;
    buf[n++] = 0xAA; buf[n++] = 0x55; buf[n++] = 0xAA; buf[n++] = 0x55;
    buf[n++] = (uint8_t)id;        buf[n++] = (uint8_t)(id >> 8);
    buf[n++] = (uint8_t)lenflags;  buf[n++] = (uint8_t)(lenflags >> 8);
    memcpy(&buf[n], payload, len); memset(&buf[n + len], 0, phys - len);
    n += phys;
    uint32_t crc = firmware_flasher_crc32(&buf[4], 4u + phys);
    buf[n++] = (uint8_t)crc; buf[n++] = (uint8_t)(crc >> 8);
    buf[n++] = (uint8_t)(crc >> 16); buf[n++] = (uint8_t)(crc >> 24);
    rx_queue(buf, n);
}

static void queue_full_block(uint16_t id, uint32_t addr, const uint8_t *row) {
    uint8_t p[IR_FLASHER_BLOCK_PAYLOAD];
    p[0] = (uint8_t)addr; p[1] = (uint8_t)(addr >> 8);
    p[2] = (uint8_t)(addr >> 16); p[3] = (uint8_t)(addr >> 24);
    memcpy(&p[4], row, FLASHER_ROW_SIZE);
    queue_frame(id, 0, p, IR_FLASHER_BLOCK_PAYLOAD);
}

static void queue_exit_frame(uint16_t id, uint32_t base, uint32_t len, uint32_t crc) {
    uint8_t p[IR_FLASHER_DESCRIPTOR_SIZE];
    p[0] = (uint8_t)base; p[1] = (uint8_t)(base >> 8); p[2] = (uint8_t)(base >> 16); p[3] = (uint8_t)(base >> 24);
    p[4] = (uint8_t)len;  p[5] = (uint8_t)(len >> 8);  p[6] = (uint8_t)(len >> 16);  p[7] = (uint8_t)(len >> 24);
    p[8] = (uint8_t)crc;  p[9] = (uint8_t)(crc >> 8);  p[10] = (uint8_t)(crc >> 16); p[11] = (uint8_t)(crc >> 24);
    queue_frame(id, IR_FLASHER_FLAG_VERIFY, p, IR_FLASHER_DESCRIPTOR_SIZE);
}

/* Queue the compressed body as FLAG_PATCH frames id 1..N-1 (frame 0 is handed
 * off via flasher_run's first-block argument), duplicating each `dups` extra
 * times. Returns the total frame count including frame 0. */
static size_t queue_body_frames(const uint8_t *body, size_t body_len, int dups) {
    size_t nframes = (body_len + FLASHER_ROW_SIZE - 1) / FLASHER_ROW_SIZE;
    if (nframes == 0) nframes = 1;
    for (size_t i = 1; i < nframes; i++) {
        size_t off = i * FLASHER_ROW_SIZE;
        uint16_t len = (uint16_t)(body_len - off < FLASHER_ROW_SIZE ? body_len - off
                                                                    : FLASHER_ROW_SIZE);
        for (int d = 0; d <= dups; d++)
            queue_frame((uint16_t)i, IR_FLASHER_FLAG_PATCH, body + off, len);
    }
    return nframes;
}

static void queue_full_flash(const uint8_t *img, uint32_t len, uint16_t exit_id) {
    for (uint32_t i = 0; i < len / FLASHER_ROW_SIZE; i++)
        queue_full_block((uint16_t)i, g_base + i * FLASHER_ROW_SIZE, img + i * FLASHER_ROW_SIZE);
    queue_exit_frame(exit_id, g_base, len, firmware_flasher_crc32(img, len));
}

static int check_image(const uint8_t *want, uint32_t len, const char *what) {
    for (uint32_t i = 0; i < len; i++)
        if (((uint8_t *)(uintptr_t)g_base)[i] != want[i]) {
            fprintf(stderr, "FAIL %s: first mismatch at offset 0x%x\n", what, i);
            return 1;
        }
    printf("PASS %s (image matches, %u rows written, %zu ACKs)\n",
           what, g_rows_written, g_ack_n);
    return 0;
}

/* ---- main ------------------------------------------------------------- */

static uint8_t *load_file(const char *path, size_t *n) {
    FILE *f = fopen(path, "rb");
    assert(f);
    fseek(f, 0, SEEK_END); long sz = ftell(f); fseek(f, 0, SEEK_SET);
    uint8_t *d = malloc((size_t)sz);
    assert(d && fread(d, 1, (size_t)sz, f) == (size_t)sz);
    fclose(f);
    *n = (size_t)sz;
    return d;
}

int main(void) {
    size_t ref_len, new_len, body_len;
    uint8_t *ref  = load_file("ref.bin", &ref_len);
    uint8_t *newi = load_file("new.bin", &new_len);
    uint8_t *body = load_file("body.bin", &body_len);
    uint32_t from, to, shift;
    { FILE *f = fopen("params.txt", "r");
      assert(f && fscanf(f, "%u %u %u", &from, &to, &shift) == 3);
      fclose(f); }
    assert(from == ref_len && to == new_len);

    void *mem = mmap(NULL, REGION_SIZE, PROT_READ | PROT_WRITE,
                     MAP_PRIVATE | MAP_ANONYMOUS | MAP_32BIT, -1, 0);
    assert(mem != MAP_FAILED);
    g_base = (uint32_t)(uintptr_t)mem;
    flasher_hosted_bootloader_end = g_base;
    flasher_hosted_app_flash_end  = g_base + REGION_SIZE;
    assert((g_base & (FLASHER_ROW_SIZE - 1u)) == 0);

#ifdef FIRMWARE_FLASHER_ULTRAPATCH
    uint32_t pow2 = 1;   /* mask arg is 0 and unused in this mode */
    uint8_t *aux = malloc(firmware_flasher_ultrapatch_state_size());
    assert(aux && shift == 0);
#else
    uint32_t pow2 = 1;
    while (pow2 < shift) pow2 <<= 1;
    uint8_t *aux = malloc(pow2);
    assert(aux);
#endif
    firmware_flasher_patch_t pd = {
        .aux = aux, .aux_mask = pow2 - 1,
        .base = 0, .from_size = from, .to_size = to, .shift_size = shift,
    };
    pd.base = g_base;
    uint16_t flen = (uint16_t)(body_len < FLASHER_ROW_SIZE ? body_len : FLASHER_ROW_SIZE);
    uint32_t newcrc = firmware_flasher_crc32(newi, to);

    int rc = 0;
    size_t nframes;

    /* 1. full (uf2-like) flash: block 0 handed off, 1..N-1 + EXIT queued */
    flash_reset(ref, (uint32_t)ref_len);
    rx_clear(); g_rows_written = 0;
    for (uint32_t i = 1; i < to / FLASHER_ROW_SIZE; i++)
        queue_full_block((uint16_t)i, g_base + i * FLASHER_ROW_SIZE, newi + i * FLASHER_ROW_SIZE);
    queue_exit_frame(0x7770, g_base, to, newcrc);
    if (run_session(NULL, 0, g_base, false, FLASHER_ROW_SIZE, newi) != SESSION_REBOOT)
        { fprintf(stderr, "FAIL full: no reboot\n"); return 1; }
    rc |= check_image(newi, to, "full flash");

    /* 2. clean patch apply + EXIT */
    flash_reset(ref, (uint32_t)ref_len);
    rx_clear(); g_rows_written = 0;
    nframes = queue_body_frames(body, body_len, 0);
    queue_exit_frame(0x7771, g_base, to, newcrc);
    if (run_session(&pd, 0, 0, true, flen, body) != SESSION_REBOOT)
        { fprintf(stderr, "FAIL clean: no reboot\n"); return 1; }
    rc |= check_image(newi, to, "patch apply (clean)");

#ifndef FIRMWARE_FLASHER_ULTRAPATCH
    /* 3. clean apply, in-flash shift mode (detools only) */
    { firmware_flasher_patch_t pds = pd; pds.aux = NULL; pds.aux_mask = 0;
      flash_reset(ref, (uint32_t)ref_len);
      rx_clear(); g_rows_written = 0;
      queue_body_frames(body, body_len, 0);
      queue_exit_frame(0x7772, g_base, to, newcrc);
      if (run_session(&pds, 0, 0, true, flen, body) != SESSION_REBOOT)
          { fprintf(stderr, "FAIL shift: no reboot\n"); return 1; }
      rc |= check_image(newi, to, "patch apply (in-flash shift)"); }
#endif

    /* 4. every body frame duplicated (host resends after losing our ACK).
     * The final frame's dup drains through the post-apply dispatch (re-ACK),
     * then the EXIT reboots: ACKs = nframes + (nframes-1) dups + 1 EXIT. */
    flash_reset(ref, (uint32_t)ref_len);
    rx_clear(); g_rows_written = 0;
    nframes = queue_body_frames(body, body_len, 1);
    { size_t off = (nframes - 1) * FLASHER_ROW_SIZE;   /* extra re-send of the final frame */
      uint16_t len = (uint16_t)(body_len - off);
      if (nframes > 1) queue_frame((uint16_t)(nframes - 1), IR_FLASHER_FLAG_PATCH, body + off, len); }
    queue_exit_frame(0x7773, g_base, to, newcrc);
    if (run_session(&pd, 0, 0, true, flen, body) != SESSION_REBOOT)
        { fprintf(stderr, "FAIL dup: no reboot\n"); return 1; }
    rc |= check_image(newi, to, "patch apply (dup + re-sent final frame)");
    /* nframes frame-ACKs + (nframes-2) in-pull dup re-ACKs (frames 1..N-2) +
     * 2 post-apply re-ACKs of the final frame (its queued dup AND the extra
     * resend, both through the dispatch) + 1 EXIT ACK = 2*nframes + 1. */
    { size_t want_acks = nframes > 1 ? 2 * nframes + 1 : 2;
      if (g_ack_n != want_acks)
          { fprintf(stderr, "FAIL dup: expected %zu ACKs, got %zu\n", want_acks, g_ack_n); rc = 1; } }

    /* 5. mid-apply takeover: a few patch frames, then a full flash + EXIT.
     * cut < nframes so the decoder is genuinely interrupted mid-apply. */
    flash_reset(ref, (uint32_t)ref_len);
    rx_clear(); g_rows_written = 0;
    { size_t cut = nframes > 4 ? 4 : (nframes > 1 ? nframes - 1 : 1);
      for (size_t i = 1; i < cut; i++) {
          size_t off = i * FLASHER_ROW_SIZE;
          uint16_t len = (uint16_t)(body_len - off < FLASHER_ROW_SIZE ? body_len - off
                                                                      : FLASHER_ROW_SIZE);
          queue_frame((uint16_t)i, IR_FLASHER_FLAG_PATCH, body + off, len);
      }
      queue_full_flash(newi, to, 0x7774);
      if (run_session(&pd, 0, 0, true, flen, body) != SESSION_REBOOT)
          { fprintf(stderr, "FAIL takeover: no reboot\n"); return 1; }
      rc |= check_image(newi, to, "mid-apply takeover -> full flash"); }

    /* 6. corrupt body -> park -> full-flash recovery + EXIT */
    flash_reset(ref, (uint32_t)ref_len);
    rx_clear(); g_rows_written = 0;
    { uint8_t *bad = malloc(body_len);
      memcpy(bad, body, body_len);
      bad[body_len / 2] ^= 0xA5;   /* corrupt a mid-stream byte */
      queue_body_frames(bad, body_len, 0);
      queue_full_flash(newi, to, 0x7775);
      if (run_session(&pd, 0, 0, true, flen, bad) != SESSION_REBOOT)
          { fprintf(stderr, "FAIL corrupt: no reboot\n"); return 1; }
      rc |= check_image(newi, to, "corrupt patch -> full-flash recovery");
      free(bad); }

    /* 7. wrong-dialect body frames (e.g. a --start-block resume with the wrong
     * --patch-format): the OTHER dialect's patch bit is not a patch frame for
     * this build, so they are IGNORED -- the session consumes only the
     * handed-off first frame, then starves silently instead of feeding the
     * wrong decoder. */
    flash_reset(ref, (uint32_t)ref_len);
    rx_clear(); g_rows_written = 0;
    { size_t nf = (body_len + FLASHER_ROW_SIZE - 1) / FLASHER_ROW_SIZE;
      for (size_t i = 1; i < nf; i++) {
          size_t off = i * FLASHER_ROW_SIZE;
          uint16_t len = (uint16_t)(body_len - off < FLASHER_ROW_SIZE ? body_len - off
                                                                      : FLASHER_ROW_SIZE);
          queue_frame((uint16_t)i,
                      IR_FLASHER_FLAG_PATCH == IR_FLASHER_FLAG_PATCH_ULTRA
                          ? IR_FLASHER_FLAG_PATCH_DETOOLS : IR_FLASHER_FLAG_PATCH_ULTRA,
                      body + off, len);
      }
      if (run_session(&pd, 0, 0, true, flen, body) != SESSION_STARVED)
          { fprintf(stderr, "FAIL wrongfmt: rebooted?!\n"); return 1; }
      if (g_ack_n > 1)
          { fprintf(stderr, "FAIL wrongfmt: %zu ACKs (frames were accepted)\n", g_ack_n); return 1; }
      printf("PASS wrong-dialect body frames ignored (parked after %zu ACKs)\n", g_ack_n); }

    /* 8. failing row writes (NVM wedge / battery sag model): every write
     * reports failure. The apply must park PROMPTLY -- ultrapatch latches the
     * first failure and its pull gate aborts the decoder within a page or two
     * (previously it would grind through the whole body with voided writes);
     * detools propagates the first failed row directly. Then a healthy full
     * flash must still work (the latch resets per backend run). */
    flash_reset(ref, (uint32_t)ref_len);
    rx_clear(); g_rows_written = 0;
    queue_body_frames(body, body_len, 0);
    g_writes_fail = true;
    if (run_session(&pd, 0, 0, true, flen, body) != SESSION_STARVED)
        { fprintf(stderr, "FAIL wfail: rebooted?!\n"); return 1; }
    g_writes_fail = false;
    if (g_rows_written != 0)
        { fprintf(stderr, "FAIL wfail: %u rows written\n", g_rows_written); return 1; }
    if (g_ack_n > 4)
        { fprintf(stderr, "FAIL wfail: not prompt (%zu ACKs before park)\n", g_ack_n); return 1; }
    { size_t parked_acks = g_ack_n;
      rx_clear(); g_rows_written = 0;
      for (uint32_t i = 1; i < to / FLASHER_ROW_SIZE; i++)
          queue_full_block((uint16_t)i, g_base + i * FLASHER_ROW_SIZE, newi + i * FLASHER_ROW_SIZE);
      queue_exit_frame(0x7776, g_base, to, newcrc);
      if (run_session(NULL, 0, g_base, false, FLASHER_ROW_SIZE, newi) != SESSION_REBOOT)
          { fprintf(stderr, "FAIL wfail-recovery: no reboot\n"); return 1; }
      if (check_image(newi, to, "failing writes -> prompt park -> healthy full flash"))
          return 1;
      printf("  (parked after only %zu ACKs)\n", parked_acks); }

#ifdef FIRMWARE_FLASHER_ULTRAPATCH
    /* 8. wrong base image (ultrapatch only): the decoder's revision-tagged
     * source-CRC gate must reject BEFORE the first flash write, leaving the
     * mismatched image completely untouched. (In the detools flow the same
     * protection lives in the launcher's ENTER pre-flight, which the harness
     * does not model.) No EXIT is queued: the session must starve, parked. */
    flash_reset(ref, (uint32_t)ref_len);
    ((uint8_t *)(uintptr_t)g_base)[100] ^= 0x5A;   /* not the image the patch expects */
    rx_clear(); g_rows_written = 0;
    queue_body_frames(body, body_len, 0);
    if (run_session(&pd, 0, 0, true, flen, body) != SESSION_STARVED)
        { fprintf(stderr, "FAIL wrong-base: applied?!\n"); return 1; }
    if (g_rows_written != 0)
        { fprintf(stderr, "FAIL wrong-base: %u rows written\n", g_rows_written); return 1; }
    if ((((uint8_t *)(uintptr_t)g_base)[100] ^ 0x5A) != ref[100] ||
        memcmp((uint8_t *)(uintptr_t)g_base + 256, ref + 256, ref_len - 256) != 0)
        { fprintf(stderr, "FAIL wrong-base: image disturbed\n"); return 1; }
    printf("PASS wrong base image rejected before any write\n");
#endif

    printf(rc == 0 ? "ALL PASS\n" : "FAILURES\n");
    return rc;
}

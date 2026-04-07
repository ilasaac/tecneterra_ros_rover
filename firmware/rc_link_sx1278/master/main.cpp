/**
 * rc_link_sx1278_master — RP2040 firmware
 *
 * Hardware map:
 *   PIO0 SM0  ← SBUS RX (inverted UART, 100kbaud, 8E2)  GP4
 *   PIO0 SM1  → PPM TX  (DMA-fed, 50 Hz, 20 ms frame)   GP15
 *   SPI0      ↔ SX1278 LoRa (433 MHz)                   GP16-21
 *   USB CDC   ↔ Jetson Nano (MAVLink serial protocol)
 *
 * PPM timing is 100% hardware (PIO + DMA):
 *   - DMA feeds 20-word buffer to PIO TX FIFO at PIO's own pace (DREQ-gated)
 *   - PIO generates exact 1 µs pulses via side-set
 *   - DMA IRQ fires at end of each frame → CPU updates buffer → restarts DMA
 *   - Zero CPU jitter on PPM output
 *
 * Jetson protocol (USB CDC, 50 Hz status + commands):
 *   TX: "CH:c0,...,c15 MODE:MANUAL\n"  every PPM frame
 *       "[SBUS_OK]" / "[SBUS_LOST]"    on state change
 *   RX: "<HB:N>"                       heartbeat (must arrive < 300 ms)
 *       "<J:c0,...,c7>"                8-channel autonomous override
 *
 * SX1278 TX: every other PPM frame (25 Hz) to respect LoRa duty cycle.
 *   Payload: 16 × uint16_t = 32 bytes (PPM channels in µs, little-endian)
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"

#include "sbus_rx.pio.h"
#include "ppm_tx.pio.h"
#include "sx1278.h"

// ── Pin assignments ───────────────────────────────────────────────────────────

#define SBUS_PIN    4    // SBUS input (inverted, from HM30 air unit)
#define PPM_PIN     15   // PPM output → motor controllers / ESC

// SX1278 pins defined in sx1278.h (GP16–21)

// ── PIO assignments ───────────────────────────────────────────────────────────

#define PIO_INST    pio0
#define SM_SBUS     0
#define SM_PPM      1

// ── Timing constants ──────────────────────────────────────────────────────────

#define CHANNELS        16   // total SBUS channels decoded; carried over LoRa and Jetson serial
#define PPM_CHANNELS     8   // channels on the PPM wire — 8 × 1500 µs = 12000 µs, fits in 20 ms
#define PPM_FRAME_US    20000U   // 20 ms frame period
#define PPM_HIGH_US     300U     // separator pulse width
#define PPM_HIGH_COUNT  (PPM_HIGH_US - 2U)  // subtract pull(1)+mov(1) overhead

// DMA buffer: [HIGH, LOW_CH0, ..., HIGH, LOW_SYNC] = PPM_CHANNELS*2+2 words = 20
#define PPM_BUF_LEN     (PPM_CHANNELS * 2 + 2)

#define SBUS_FRAME_LEN  25

// Safety timeouts (ms)
#define HB_TIMEOUT_MS   300
#define CMD_TIMEOUT_MS  500
#define SBUS_TIMEOUT_MS 200

// Channel limits
#define CH_EMERGENCY    4    // SWA: < 1700 → emergency
#define CH_AUTONOMOUS   5    // SWB: > 1700 → autonomous
#define CH_ROVER_SEL    8    // CH9: master relay window 1250–1750 (middle position)
#define EMERGENCY_THRESH  1700
#define AUTO_THRESH       1700
#define RELAY_LOW         1250
#define RELAY_HIGH        1750

// PPM speed limits (autonomous mode only; SBUS passthrough is unclamped).
// Full range: [1000, 2000] — Jetson sends real PPM values.
#define THR_LIMIT_MAX  2000U
#define THR_LIMIT_MIN  1000U
#define SPIN_LIMIT_MAX 2000U
#define SPIN_LIMIT_MIN 1000U

// SX1278 TX every 2nd PPM frame (25 Hz = 40 ms inter-packet gap)
// LoRa SF7/BW500 ToA ≈ 12.9 ms, so 40 ms gives comfortable margin.
#define SX_TX_DIVISOR   2

// ── PPM channel mapping (wFly RF209S layout from SIYI MK32 SBUS) ─────────────
//
// Hardware PPM output follows wFly channel order. USB serial always sends the
// raw 16 SBUS channels (sbus_ch[]) unchanged so the Jetson sees the original.
//
// PPM (1-indexed) = SBUS source (1-indexed)
//   CH1  = CH3           CH5  = CH11
//   CH2  = CH1 inverted  CH6  = CH12
//   CH3  = CH5            CH7  = CH7  inverted
//   CH4  = CH6            CH8  = CH8  inverted
//
// Inversion: 3000 − value  maps 1000↔2000, keeps 1500 at centre.

#define PPM_INV(v)  ((uint16_t)(3000u - (v)))

static void apply_ppm_map(const volatile uint16_t *src, volatile uint16_t *dst) {
    dst[0] = src[2];           // CH1 = SBUS CH3
    dst[1] = PPM_INV(src[0]);  // CH2 = SBUS CH1 inverted
    dst[2] = src[4];            // CH3 = SBUS CH5
    dst[3] = src[5];            // CH4 = SBUS CH6
    dst[4] = src[10];          // CH5 = SBUS CH11
    dst[5] = src[11];          // CH6 = SBUS CH12
    dst[6] = PPM_INV(src[6]);  // CH7 = SBUS CH7 inverted
    dst[7] = PPM_INV(src[7]);  // CH8 = SBUS CH8 inverted
}

// ── Mode ──────────────────────────────────────────────────────────────────────

typedef enum {
    MODE_MANUAL = 0,
    MODE_EMERGENCY,
    MODE_AUTONOMOUS,
    MODE_AUTO_NO_HB,
    MODE_AUTO_TIMEOUT,
    MODE_RELAY
} Mode;

static const char *const MODE_STR[] = {
    "MANUAL", "EMERGENCY", "AUTONOMOUS", "AUTO-NO-HB", "AUTO-TIMEOUT", "RELAY"
};

// ── State (shared between main loop and DMA IRQ) ──────────────────────────────

static volatile uint16_t sbus_ch[CHANNELS]  = {1500,1500,1500,1500,1500,1500,1500,1500,
                                               1500,1500,1500,1500,1500,1500,1500,1500};
static volatile uint16_t auto_ch[CHANNELS]  = {1500,1500,1500,1500,1500,1500,1500,1500,
                                               1500,1500,1500,1500,1500,1500,1500,1500};
static volatile uint16_t out_ch[CHANNELS]   = {1500,1500,1500,1500,1500,1500,1500,1500,
                                               1500,1500,1500,1500,1500,1500,1500,1500};

static volatile Mode mode     = MODE_EMERGENCY;
static volatile bool sbus_ok  = false;
static volatile bool hb_alive = false;
static volatile bool cmd_fresh = false;

static volatile uint64_t t_last_sbus = 0;
static volatile uint64_t t_last_hb   = 0;
static volatile uint64_t t_last_cmd  = 0;

// ── PPM DMA buffer ────────────────────────────────────────────────────────────

static uint32_t ppm_buf[PPM_BUF_LEN];
static int      dma_chan = -1;
static volatile int frame_count = 0;   // incremented each DMA IRQ

// Recompute ppm_buf from out_ch[]. Called only from DMA IRQ (no lock needed).
static void ppm_buf_update(void) {
    // Only the first PPM_CHANNELS channels go to the PPM wire.
    // All 16 decoded channels are available via out_ch[] for LoRa / Jetson serial.
    uint32_t sum = 0;
    for (int i = 0; i < PPM_CHANNELS; i++) {
        uint16_t ch = out_ch[i];
        if (ch < 1000) ch = 1000;
        if (ch > 2000) ch = 2000;
        // Speed limits: throttle (idx 0) ±20%, spin (idx 1) ±50%
        if (i == 0) {
            if (ch > THR_LIMIT_MAX)  ch = THR_LIMIT_MAX;
            if (ch < THR_LIMIT_MIN)  ch = THR_LIMIT_MIN;
        } else if (i == 1) {
            if (ch > SPIN_LIMIT_MAX) ch = SPIN_LIMIT_MAX;
            if (ch < SPIN_LIMIT_MIN) ch = SPIN_LIMIT_MIN;
        }
        ppm_buf[i * 2]     = PPM_HIGH_COUNT;
        ppm_buf[i * 2 + 1] = (uint32_t)(ch - 302);   // LOW = channel - HIGH(300) - overhead(2)
        sum += ch;
    }
    // Sync pulse: fills remainder of the 20 ms frame
    uint32_t sync_low = PPM_FRAME_US - sum - PPM_HIGH_US;
    ppm_buf[PPM_BUF_LEN - 2] = PPM_HIGH_COUNT;
    ppm_buf[PPM_BUF_LEN - 1] = (sync_low > 2) ? (sync_low - 2) : 0;
}

// DMA IRQ: fires when PIO has consumed all 20 words (end of frame).
// We update the buffer and restart DMA — PIO stalls on 'pull' for only
// the microseconds it takes to call dma_channel_start() (<1 µs).
static void __isr dma_irq_handler(void) {
    if (!dma_channel_get_irq0_status(dma_chan)) return;
    dma_channel_acknowledge_irq0(dma_chan);

    frame_count++;

    ppm_buf_update();

    // Restart DMA from the beginning of ppm_buf
    dma_channel_set_read_addr(dma_chan, ppm_buf, true /* trigger */);
}

// ── SBUS assembly & decode ────────────────────────────────────────────────────

static uint8_t sbus_buf[SBUS_FRAME_LEN];
static int     sbus_idx = 0;

// Decode a validated 25-byte SBUS frame into 11-bit channel values, scale to µs.
static bool sbus_decode(const uint8_t *f, volatile uint16_t *ch) {
    if (f[0] != 0x0F || f[24] != 0x00) return false;

    // Unpack all 16 × 11-bit channels from 22 bytes of data (f[1]..f[22]).
    // Each channel occupies 11 bits LSB-first; the pattern repeats every 8 channels
    // (every 11 bytes, same bit offsets within the shifted window).
    uint16_t raw[CHANNELS];
    raw[0]  = ((uint16_t)(f[1])       | (uint16_t)(f[2])  << 8)  & 0x7FF;
    raw[1]  = ((uint16_t)(f[2])  >> 3 | (uint16_t)(f[3])  << 5)  & 0x7FF;
    raw[2]  = ((uint16_t)(f[3])  >> 6 | (uint16_t)(f[4])  << 2  | (uint16_t)(f[5])  << 10) & 0x7FF;
    raw[3]  = ((uint16_t)(f[5])  >> 1 | (uint16_t)(f[6])  << 7)  & 0x7FF;
    raw[4]  = ((uint16_t)(f[6])  >> 4 | (uint16_t)(f[7])  << 4)  & 0x7FF;
    raw[5]  = ((uint16_t)(f[7])  >> 7 | (uint16_t)(f[8])  << 1  | (uint16_t)(f[9])  << 9)  & 0x7FF;
    raw[6]  = ((uint16_t)(f[9])  >> 2 | (uint16_t)(f[10]) << 6)  & 0x7FF;
    raw[7]  = ((uint16_t)(f[10]) >> 5 | (uint16_t)(f[11]) << 3)  & 0x7FF;
    raw[8]  = ((uint16_t)(f[12])      | (uint16_t)(f[13]) << 8)  & 0x7FF;
    raw[9]  = ((uint16_t)(f[13]) >> 3 | (uint16_t)(f[14]) << 5)  & 0x7FF;
    raw[10] = ((uint16_t)(f[14]) >> 6 | (uint16_t)(f[15]) << 2  | (uint16_t)(f[16]) << 10) & 0x7FF;
    raw[11] = ((uint16_t)(f[16]) >> 1 | (uint16_t)(f[17]) << 7)  & 0x7FF;
    raw[12] = ((uint16_t)(f[17]) >> 4 | (uint16_t)(f[18]) << 4)  & 0x7FF;
    raw[13] = ((uint16_t)(f[18]) >> 7 | (uint16_t)(f[19]) << 1  | (uint16_t)(f[20]) << 9)  & 0x7FF;
    raw[14] = ((uint16_t)(f[20]) >> 2 | (uint16_t)(f[21]) << 6)  & 0x7FF;
    raw[15] = ((uint16_t)(f[21]) >> 5 | (uint16_t)(f[22]) << 3)  & 0x7FF;

    // Scale 172–1811 (SBUS range) → 1000–2000 µs (PPM range)
    for (int i = 0; i < CHANNELS; i++)
        ch[i] = (uint16_t)(((uint32_t)(raw[i] - 172) * 1000) / 1639 + 1000);

    return true;
}

// Feed one byte from the PIO SBUS FIFO into the frame assembler.
static void sbus_feed_byte(uint8_t b) {
    static uint64_t t_prev = 0;
    uint64_t now = time_us_64();

    // Inter-frame gap detection: SBUS has a mandatory silence of >4 ms between frames.
    // After such a gap the very next byte on the wire is guaranteed to be the 0x0F header.
    // This is the only reliable way to find f[0] — 0x0F can also appear as a data byte
    // at positions f[8], f[14], f[22] etc. depending on channel values, so searching
    // for 0x0F alone is not sufficient.
    if ((now - t_prev) > 3500) sbus_idx = 0;   // gap > 3.5 ms → reset, expect header next
    t_prev = now;

    if (sbus_idx == 0 && b != 0x0F) return;    // still hunting for header byte

    sbus_buf[sbus_idx++] = b;

    if (sbus_idx == SBUS_FRAME_LEN) {
        if (sbus_decode(sbus_buf, sbus_ch)) {
            sbus_ok    = true;
            t_last_sbus = time_us_64();
        }
        sbus_idx = 0;
    }
}

// ── Mode logic ────────────────────────────────────────────────────────────────

static Mode compute_mode(void) {
    uint64_t now = time_us_64();

    // Check timeouts
    if (sbus_ok  && (now - t_last_sbus) > (uint64_t)SBUS_TIMEOUT_MS * 1000) sbus_ok   = false;
    if (hb_alive && (now - t_last_hb)   > (uint64_t)HB_TIMEOUT_MS  * 1000) hb_alive  = false;
    if (cmd_fresh && (now - t_last_cmd)  > (uint64_t)CMD_TIMEOUT_MS * 1000) cmd_fresh = false;

    if (!sbus_ok) return MODE_EMERGENCY;

    if (sbus_ch[CH_EMERGENCY] > EMERGENCY_THRESH) return MODE_EMERGENCY;

    // CH9 high: slave manually selected — master neutral, keep relaying over LoRa
    if (sbus_ch[CH_ROVER_SEL] > RELAY_HIGH) return MODE_RELAY;

    // CH9 middle: no rover manually selected — autonomous possible, else relay-neutral
    if (sbus_ch[CH_ROVER_SEL] >= RELAY_LOW && sbus_ch[CH_ROVER_SEL] <= RELAY_HIGH) {
        if (sbus_ch[CH_AUTONOMOUS] > AUTO_THRESH) {
            if (!hb_alive)  return MODE_AUTO_NO_HB;
            if (!cmd_fresh) return MODE_AUTO_TIMEOUT;
            return MODE_AUTONOMOUS;
        }
        return MODE_RELAY;
    }

    // CH9 low: master manually selected
    return MODE_MANUAL;
}

static void apply_mode(Mode m) {
    mode = m;
    switch (m) {
        case MODE_MANUAL:
            apply_ppm_map(sbus_ch, out_ch);
            break;
        case MODE_AUTONOMOUS:
            for (int i = 0; i < CHANNELS; i++) out_ch[i] = auto_ch[i];
            // CH2 (steering) and CH4 handled by navigator — no inversion.
            // CH7/CH8 hardware expects inverted values (valves/aux).
            out_ch[6] = PPM_INV(auto_ch[6]);  // CH7
            out_ch[7] = PPM_INV(auto_ch[7]);  // CH8
            break;
        case MODE_RELAY:
            // Local motors neutral; payload relayed to slave unchanged
            for (int i = 0; i < CHANNELS; i++) out_ch[i] = 1500;
            break;
        case MODE_EMERGENCY:
        case MODE_AUTO_NO_HB:
        case MODE_AUTO_TIMEOUT:
            for (int i = 0; i < CHANNELS; i++) out_ch[i] = 1500;
            break;
    }
}

// ── Jetson serial ─────────────────────────────────────────────────────────────

static char jetson_linebuf[128];  // 16 channels × 5 chars + " MODE:AUTONOMOUS" ≈ 98 chars
static int  jetson_lineidx = 0;

static void jetson_parse_line(const char *line) {
    int seq;
    if (sscanf(line, "<HB:%d>", &seq) == 1) {
        hb_alive  = true;
        t_last_hb = time_us_64();
        printf("<HB:%d>\n", seq + 1);
        return;
    }
    uint16_t tmp[8];
    if (sscanf(line, "<J:%hu,%hu,%hu,%hu,%hu,%hu,%hu,%hu>",
               &tmp[0], &tmp[1], &tmp[2], &tmp[3],
               &tmp[4], &tmp[5], &tmp[6], &tmp[7]) == 8) {
        for (int i = 0; i < 8; i++) auto_ch[i] = tmp[i];
        cmd_fresh  = true;
        t_last_cmd = time_us_64();
    }
}

// Non-blocking: read chars from USB CDC, assemble lines, parse commands.
static void jetson_poll_rx(void) {
    int c;
    while ((c = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
        if (c == '\n' || c == '\r') {
            jetson_linebuf[jetson_lineidx] = '\0';
            if (jetson_lineidx > 0) jetson_parse_line(jetson_linebuf);
            jetson_lineidx = 0;
        } else if (jetson_lineidx < (int)sizeof(jetson_linebuf) - 1) {
            jetson_linebuf[jetson_lineidx++] = (char)c;
        }
    }
}

static void jetson_send_status(void) {
    // Raw SBUS channels — Jetson always receives the original 16 channels
    // regardless of PPM remapping or current mode.
    printf("CH:%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d MODE:%s\n",
           sbus_ch[0],  sbus_ch[1],  sbus_ch[2],  sbus_ch[3],
           sbus_ch[4],  sbus_ch[5],  sbus_ch[6],  sbus_ch[7],
           sbus_ch[8],  sbus_ch[9],  sbus_ch[10], sbus_ch[11],
           sbus_ch[12], sbus_ch[13], sbus_ch[14], sbus_ch[15],
           MODE_STR[mode]);
}

// ── DMA init ──────────────────────────────────────────────────────────────────

static void dma_ppm_init(void) {
    dma_chan = dma_claim_unused_channel(true);

    dma_channel_config cfg = dma_channel_get_default_config(dma_chan);
    channel_config_set_dreq(&cfg, pio_get_dreq(PIO_INST, SM_PPM, true));
    channel_config_set_read_increment(&cfg, true);
    channel_config_set_write_increment(&cfg, false);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);

    dma_channel_configure(
        dma_chan, &cfg,
        &PIO_INST->txf[SM_PPM],   // write to PIO SM1 TX FIFO
        ppm_buf,                   // read from buffer
        PPM_BUF_LEN,               // 20 words per frame
        false                      // don't start yet
    );

    // IRQ on completion
    dma_channel_set_irq0_enabled(dma_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    // Prime the buffer with neutral channels and start
    ppm_buf_update();
    dma_channel_start(dma_chan);
}

// ── Main ──────────────────────────────────────────────────────────────────────

int main(void) {
    stdio_usb_init();   // USB CDC to Jetson — interrupt-driven in background
    sleep_ms(500);      // allow USB enumeration

    // PIO: SBUS RX on SM0
    sbus_rx_program_init(PIO_INST, SM_SBUS, SBUS_PIN);

    // PIO: PPM TX on SM1
    ppm_tx_program_init(PIO_INST, SM_PPM, PPM_PIN);

    // DMA: feed PPM PIO from ppm_buf
    dma_ppm_init();

    // SX1278: LoRa 433 MHz
    bool sx_ok = sx1278_init();
    if (!sx_ok) {
        printf("[SX1278_ERROR] chip not found — check wiring\n");
        // Continue without LoRa rather than halting (PPM and SBUS still work)
    }

    printf("[BOOT] RC link master ready\n");

    // ── State tracking for change-only prints ────────────────────────────────
    bool     prev_sbus_ok = false;
    int      last_frame   = 0;
    int      sx_frame_div = 0;

    // ── Main loop ─────────────────────────────────────────────────────────────
    while (true) {

        // 1. Drain PIO SBUS FIFO — non-blocking, assembles frame bytes
        while (!pio_sm_is_rx_fifo_empty(PIO_INST, SM_SBUS)) {
            uint32_t raw = pio_sm_get(PIO_INST, SM_SBUS);
            // Byte is in bits [31:24], inverted — XOR corrects SBUS inversion
            uint8_t byte = (uint8_t)((raw >> 24) ^ 0xFF);
#ifdef SBUS_DEBUG
            printf("RX:%02X\n", byte);   // remove after diagnosis
#endif
            sbus_feed_byte(byte);
        }

        // 2. Jetson serial RX
        jetson_poll_rx();

        // 3. Log SBUS state changes to Jetson
        if (sbus_ok != prev_sbus_ok) {
            printf(sbus_ok ? "[SBUS_OK]\n" : "[SBUS_LOST]\n");
            prev_sbus_ok = sbus_ok;
        }

        // 4. Per-frame work (triggered by DMA IRQ incrementing frame_count)
        int current_frame = frame_count;
        if (current_frame == last_frame) continue;   // no new frame yet
        last_frame = current_frame;

        // 4a. Compute mode and apply to out_ch
        //     (ppm_buf_update() in DMA IRQ reads out_ch, so we write before
        //      the next IRQ fires — we have ~20 ms which is plenty)
        apply_mode(compute_mode());

        // 4b. SX1278 TX every SX_TX_DIVISOR frames (25 Hz) — skip if not present
        if (sx_ok && ++sx_frame_div >= SX_TX_DIVISOR) {
            sx_frame_div = 0;
            // Payload: always raw SBUS channels — slave applies its own PPM map.
            // Do NOT use out_ch here: out_ch is already PPM-remapped (and neutral
            // in RELAY mode), so using it would double-map and break RELAY forwarding.
            uint8_t payload[CHANNELS * 2];
            for (int i = 0; i < CHANNELS; i++) {
                uint16_t ch = sbus_ch[i];
                payload[i * 2]     = (uint8_t)(ch & 0xFF);
                payload[i * 2 + 1] = (uint8_t)(ch >> 8);
            }
            // Fire-and-forget; returns false if previous TX not done yet
            sx1278_send(payload, sizeof(payload));
        }

        // 4c. Status to Jetson (50 Hz — every frame)
        jetson_send_status();
    }
}

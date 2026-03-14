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

#define CHANNELS        16
#define PPM_FRAME_US    20000U   // 20 ms frame period
#define PPM_HIGH_US     300U     // separator pulse width
#define PPM_HIGH_COUNT  (PPM_HIGH_US - 2U)  // subtract pull(1)+mov(1) overhead

// DMA buffer: [HIGH, LOW_CH0, HIGH, LOW_CH1, ..., HIGH, LOW_SYNC] = 34 words
// 16 channels × 2 words + 2 words sync = 34
#define PPM_BUF_LEN     34

#define SBUS_FRAME_LEN  25

// Safety timeouts (ms)
#define HB_TIMEOUT_MS   300
#define CMD_TIMEOUT_MS  500
#define SBUS_TIMEOUT_MS 200

// Channel limits
#define CH_EMERGENCY    4    // SWA: < 1700 → emergency
#define CH_AUTONOMOUS   5    // SWB: > 1700 → autonomous
#define CH_ROVER_SEL    8    // CH9: relay window 1250–1750
#define EMERGENCY_THRESH  1700
#define AUTO_THRESH       1700
#define RELAY_LOW         1250
#define RELAY_HIGH        1750

// SX1278 TX every 2nd PPM frame (25 Hz = 40 ms inter-packet gap)
// LoRa SF7/BW500 ToA ≈ 12.9 ms, so 40 ms gives comfortable margin.
#define SX_TX_DIVISOR   2

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
    uint32_t sum = 0;
    for (int i = 0; i < CHANNELS; i++) {
        uint16_t ch = out_ch[i];
        if (ch < 1000) ch = 1000;
        if (ch > 2000) ch = 2000;
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
    // Re-sync: if we see 0x0F, start a new frame regardless of position.
    if (b == 0x0F) sbus_idx = 0;

    if (sbus_idx < SBUS_FRAME_LEN)
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

    if (sbus_ch[CH_EMERGENCY] < EMERGENCY_THRESH) return MODE_EMERGENCY;

    if (sbus_ch[CH_ROVER_SEL] > RELAY_LOW && sbus_ch[CH_ROVER_SEL] < RELAY_HIGH)
        return MODE_RELAY;

    if (sbus_ch[CH_AUTONOMOUS] > AUTO_THRESH) {
        if (!hb_alive) return MODE_AUTO_NO_HB;
        if (!cmd_fresh) return MODE_AUTO_TIMEOUT;
        return MODE_AUTONOMOUS;
    }

    return MODE_MANUAL;
}

static void apply_mode(Mode m) {
    mode = m;
    switch (m) {
        case MODE_MANUAL:
            for (int i = 0; i < CHANNELS; i++) out_ch[i] = sbus_ch[i];
            break;
        case MODE_AUTONOMOUS:
            for (int i = 0; i < CHANNELS; i++) out_ch[i] = auto_ch[i];
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
    printf("CH:%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d MODE:%s\n",
           out_ch[0],  out_ch[1],  out_ch[2],  out_ch[3],
           out_ch[4],  out_ch[5],  out_ch[6],  out_ch[7],
           out_ch[8],  out_ch[9],  out_ch[10], out_ch[11],
           out_ch[12], out_ch[13], out_ch[14], out_ch[15],
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
    if (!sx1278_init()) {
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

        // 4b. SX1278 TX every SX_TX_DIVISOR frames (25 Hz)
        if (++sx_frame_div >= SX_TX_DIVISOR) {
            sx_frame_div = 0;
            // Payload: relay the same channels that go to local PPM
            uint8_t payload[CHANNELS * 2];
            for (int i = 0; i < CHANNELS; i++) {
                uint16_t ch = out_ch[i];
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

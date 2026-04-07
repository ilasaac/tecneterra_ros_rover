/**
 * rc_link_sx1278_slave — RP2040 firmware
 *
 * Hardware map:
 *   PIO0 SM0  → PPM TX  (DMA-fed, 50 Hz, 20 ms frame)   GP15
 *   SPI0      ↔ SX1278 LoRa RX (433 MHz, continuous)    GP16-21
 *   USB CDC   ↔ Jetson Nano
 *
 * SX1278 runs in continuous RX mode. Main loop polls sx1278_recv():
 *   - On packet received → decode 16 channels → apply mode → update PPM DMA buffer
 *   - On RF timeout (500 ms no packet) → MODE_EMERGENCY → PPM neutral
 *
 * Jetson protocol (USB CDC):
 *   TX: "CH:c0,...,c15 MODE:...\n" at 10 Hz
 *       "[RF_LINK_OK]" / "[RF_LINK_LOST]"  on state change
 *   RX: "<HB:N>"                   heartbeat → enables autonomous mode
 *       "<J:c0,...,c7>"            8-channel autonomous override (bypasses RF)
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"

#include "ppm_tx.pio.h"
#include "sx1278.h"

// ── Pin assignments ───────────────────────────────────────────────────────────

#define PPM_PIN     15   // PPM output → motor controllers / ESC
// SX1278 pins defined in sx1278.h

// ── PIO assignments ───────────────────────────────────────────────────────────

#define PIO_INST    pio0
#define SM_PPM      0    // only one SM needed on slave

// ── Timing ───────────────────────────────────────────────────────────────────

#define CHANNELS        16   // total channels decoded from LoRa packet
#define PPM_CHANNELS     8   // channels output on PPM wire
#define PPM_FRAME_US    20000U
#define PPM_HIGH_US     300U
#define PPM_HIGH_COUNT  (PPM_HIGH_US - 2U)
#define PPM_BUF_LEN     (PPM_CHANNELS * 2 + 2)   // = 20

#define RF_TIMEOUT_MS      500
#define HB_TIMEOUT_MS      300
#define CMD_TIMEOUT_MS     500
// How often to check SX1278 health while RF is lost (power-cycle recovery)
#define SX_WATCHDOG_MS    2000

// Channel gating thresholds (same as master)
#define CH_ROVER_SEL     8
#define CH_EMERGENCY     4     // same as master — emergency switch
#define CH_AUTONOMOUS    5     // same as master — auto switch
#define RELAY_LOW        1250  // below → master manually selected (slave neutral)
#define RELAY_HIGH       1750  // above → slave manually selected

// PPM speed limits (autonomous mode only; SBUS passthrough is unclamped).
// Full range: [1000, 2000] — Jetson sends real PPM values.
#define THR_LIMIT_MAX  2000U
#define THR_LIMIT_MIN  1000U
#define SPIN_LIMIT_MAX 2000U
#define SPIN_LIMIT_MIN 1000U
#define EMERGENCY_THRESH 1700
#define AUTO_THRESH      1700

// Jetson status rate: 10 Hz = 100 ms
#define STATUS_INTERVAL_MS  100

// ── Mode ──────────────────────────────────────────────────────────────────────

typedef enum {
    MODE_RF = 0,          // channels come from SX1278 (master rover RF link)
    MODE_AUTONOMOUS,      // channels from Jetson <J:...> command
    MODE_IDLE,            // neutral — other rover selected or middle with no AUTO (not an alarm)
    MODE_AUTO_TIMEOUT,    // CH5 AUTO + HB alive but no fresh <J:> cmd yet — neutral, bootstraps navigator
    MODE_EMERGENCY        // true emergency: RF lost or SWA (CH4) triggered
} Mode;

static const char *const MODE_STR[] = { "MANUAL", "AUTONOMOUS", "IDLE", "AUTO-TIMEOUT", "EMERGENCY" };

// ── State ─────────────────────────────────────────────────────────────────────

static volatile uint16_t rf_ch[CHANNELS]    = {1500,1500,1500,1500,1500,1500,1500,1500,
                                               1500,1500,1500,1500,1500,1500,1500,1500};
static volatile uint16_t auto_ch[CHANNELS]  = {1500,1500,1500,1500,1500,1500,1500,1500,
                                               1500,1500,1500,1500,1500,1500,1500,1500};
static volatile uint16_t out_ch[CHANNELS]   = {1500,1500,1500,1500,1500,1500,1500,1500,
                                               1500,1500,1500,1500,1500,1500,1500,1500};

static volatile Mode     mode       = MODE_EMERGENCY;
static volatile bool     rf_ok      = false;
static volatile bool     hb_alive   = false;
static volatile bool     cmd_fresh  = false;

static volatile uint64_t t_last_rf  = 0;
static volatile uint64_t t_last_hb  = 0;
static volatile uint64_t t_last_cmd = 0;

// ── PPM DMA (identical to master) ────────────────────────────────────────────

static uint32_t ppm_buf[PPM_BUF_LEN];
static int      dma_chan  = -1;
static volatile int frame_count = 0;

static void ppm_buf_update(void) {
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
        ppm_buf[i * 2 + 1] = (uint32_t)(ch - 302);
        sum += ch;
    }
    uint32_t sync_low = PPM_FRAME_US - sum - PPM_HIGH_US;
    ppm_buf[PPM_BUF_LEN - 2] = PPM_HIGH_COUNT;
    ppm_buf[PPM_BUF_LEN - 1] = (sync_low > 2) ? (sync_low - 2) : 0;
}

static void __isr dma_irq_handler(void) {
    if (!dma_channel_get_irq0_status(dma_chan)) return;
    dma_channel_acknowledge_irq0(dma_chan);
    frame_count++;
    ppm_buf_update();
    dma_channel_set_read_addr(dma_chan, ppm_buf, true);
}

static void dma_ppm_init(void) {
    dma_chan = dma_claim_unused_channel(true);
    dma_channel_config cfg = dma_channel_get_default_config(dma_chan);
    channel_config_set_dreq(&cfg, pio_get_dreq(PIO_INST, SM_PPM, true));
    channel_config_set_read_increment(&cfg, true);
    channel_config_set_write_increment(&cfg, false);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
    dma_channel_configure(dma_chan, &cfg, &PIO_INST->txf[SM_PPM], ppm_buf, PPM_BUF_LEN, false);
    dma_channel_set_irq0_enabled(dma_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);
    ppm_buf_update();
    dma_channel_start(dma_chan);
}

// ── PPM channel mapping (wFly RF209S layout from SIYI MK32 SBUS) ─────────────
//
// Hardware PPM output follows wFly channel order. USB serial always sends the
// raw 16 RF channels (rf_ch[]) unchanged so the Jetson sees the original.
//
// PPM (1-indexed) = RF/SBUS source (1-indexed)
//   CH1  = CH3           CH5  = CH11
//   CH2  = CH1 inverted  CH6  = CH12
//   CH3  = CH5            CH7  = CH7  inverted
//   CH4  = CH6            CH8  = CH8  inverted
//
// Inversion: 3000 − value  maps 1000↔2000, keeps 1500 at centre.

#define PPM_INV(v)  ((uint16_t)(3000u - (v)))

static void apply_ppm_map(const volatile uint16_t *src, volatile uint16_t *dst) {
    dst[0] = src[2];           // CH1 = RF CH3
    dst[1] = PPM_INV(src[0]);  // CH2 = RF CH1 inverted
    dst[2] = src[4];            // CH3 = RF CH5
    dst[3] = 1939;              // CH4 = fixed (VCU expects constant)
    dst[4] = src[10];          // CH5 = RF CH11
    dst[5] = src[11];          // CH6 = RF CH12
    dst[6] = PPM_INV(src[6]);  // CH7 = RF CH7 inverted
    dst[7] = PPM_INV(src[7]);  // CH8 = RF CH8 inverted
}

// ── Mode logic ────────────────────────────────────────────────────────────────

static Mode compute_mode(void) {
    uint64_t now = time_us_64();

    if (rf_ok    && (now - t_last_rf)   > (uint64_t)RF_TIMEOUT_MS  * 1000) rf_ok    = false;
    if (hb_alive && (now - t_last_hb)   > (uint64_t)HB_TIMEOUT_MS  * 1000) hb_alive = false;
    if (cmd_fresh && (now - t_last_cmd) > (uint64_t)CMD_TIMEOUT_MS * 1000) cmd_fresh = false;

    if (!rf_ok) return MODE_EMERGENCY;

    // Emergency switch (CH4) always overrides — read from received RF packet
    if (rf_ch[CH_EMERGENCY] < EMERGENCY_THRESH) return MODE_EMERGENCY;

    uint16_t ch9 = rf_ch[CH_ROVER_SEL];

    // CH9 low: master manually selected — slave neutral, not an alarm
    if (ch9 < RELAY_LOW) return MODE_IDLE;

    // CH9 middle: no rover manually selected — autonomous if all conditions met,
    //   otherwise idle (not emergency — SWA is the only emergency trigger)
    if (ch9 <= RELAY_HIGH) {
        if (rf_ch[CH_AUTONOMOUS] < AUTO_THRESH && hb_alive && cmd_fresh)
            return MODE_AUTONOMOUS;
        // CH5 in AUTO + HB alive but no fresh <J:> cmd yet.  Report AUTO-TIMEOUT
        // so rp2040_bridge treats it as AUTONOMOUS and the navigator starts publishing
        // cmd_override — the first <J:> received will flip cmd_fresh and enter true
        // AUTONOMOUS on the very next frame.  PPM output stays neutral (idle) here.
        if (rf_ch[CH_AUTONOMOUS] < AUTO_THRESH && hb_alive)
            return MODE_AUTO_TIMEOUT;
        return MODE_IDLE;
    }

    // CH9 high: slave manually selected — follow RC values from master RF relay
    return MODE_RF;
}

static void apply_mode(Mode m) {
    mode = m;
    switch (m) {
        case MODE_RF:
            apply_ppm_map(rf_ch, out_ch);
            break;
        case MODE_AUTONOMOUS:
            for (int i = 0; i < CHANNELS; i++) out_ch[i] = auto_ch[i];
            // CH7/CH8 hardware expects inverted values (valves/aux)
            out_ch[3] = 1939;                 // CH4 = fixed (VCU expects constant)
            out_ch[6] = PPM_INV(auto_ch[6]);  // CH7
            out_ch[7] = PPM_INV(auto_ch[7]);  // CH8
            break;
        case MODE_AUTO_TIMEOUT:
        case MODE_IDLE:
        case MODE_EMERGENCY:
            for (int i = 0; i < CHANNELS; i++) out_ch[i] = 1500;
            break;
    }
}

// ── Jetson serial ─────────────────────────────────────────────────────────────

static char jetson_linebuf[64];
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

// ── Main ──────────────────────────────────────────────────────────────────────

int main(void) {
    stdio_usb_init();
    sleep_ms(500);

    // PIO: PPM TX on SM0
    ppm_tx_program_init(PIO_INST, SM_PPM, PPM_PIN);

    // DMA for PPM
    dma_ppm_init();

    // SX1278: continuous RX
    bool sx_ok = sx1278_init();
    if (!sx_ok) {
        printf("[SX1278_ERROR] chip not found\n");
    } else {
        sx1278_start_rx();
    }

    printf("[BOOT] RC link slave ready\n");

    bool     prev_rf_ok      = false;
    int      last_frame      = 0;
    uint64_t t_last_status   = 0;
    uint64_t t_last_sx_check = 0;

    while (true) {

        // 1. SX1278 RX — poll DIO0 (HIGH when RxDone)
        //    sx1278_recv() also checks CRC error flag and discards bad packets
        if (gpio_get(SX_DIO0)) {
            uint8_t pkt[CHANNELS * 2];
            uint8_t pkt_len = 0;
            if (sx1278_recv(pkt, &pkt_len) && pkt_len == CHANNELS * 2) {
                for (int i = 0; i < CHANNELS; i++) {
                    rf_ch[i] = (uint16_t)(pkt[i * 2] | ((uint16_t)pkt[i * 2 + 1] << 8));
                    // Clamp to valid PPM range
                    if (rf_ch[i] < 1000) rf_ch[i] = 1000;
                    if (rf_ch[i] > 2000) rf_ch[i] = 2000;
                }
                rf_ok    = true;
                t_last_rf = time_us_64();
            }
        }

        // 2. Jetson serial RX
        jetson_poll_rx();

        // 3. RF link state change notification to Jetson
        if (rf_ok != prev_rf_ok) {
            printf(rf_ok ? "[RF_LINK_OK]\n" : "[RF_LINK_LOST]\n");
            prev_rf_ok = rf_ok;
        }

        // 4. Per-frame mode update (triggered by DMA IRQ)
        int current_frame = frame_count;
        if (current_frame != last_frame) {
            last_frame = current_frame;
            apply_mode(compute_mode());
        }

        // 5. SX1278 watchdog: while RF is lost, check if chip was power-cycled.
        //    A power cycle resets RegOpMode to 0x09 (FSK standby). If it is no
        //    longer 0x85 (LoRa RX continuous), re-initialise and restart RX.
        //    Only runs when rf_ok is false to avoid disturbing a working chip.
        {
            uint64_t now_wd = time_us_64();
            if (!rf_ok && (now_wd - t_last_sx_check) >= (uint64_t)SX_WATCHDOG_MS * 1000) {
                t_last_sx_check = now_wd;
                uint8_t opmode = sx_read_reg(SX_REG_OP_MODE);
                if (opmode != (SX_LORA_MODE | SX_MODE_RX_CONT)) {
                    printf("[SX1278_REINIT] opmode=0x%02x — reinitialising\n", opmode);
                    if (sx1278_init()) {
                        sx1278_start_rx();
                    } else {
                        printf("[SX1278_ERROR] reinit failed\n");
                    }
                }
            }
        }

        // 6. Periodic status to Jetson at 10 Hz
        uint64_t now = time_us_64();
        if ((now - t_last_status) >= (uint64_t)STATUS_INTERVAL_MS * 1000) {
            t_last_status = now;
            // Raw RF channels — Jetson always receives the original 16 channels
            // regardless of PPM remapping or current mode.
            printf("CH:%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d MODE:%s\n",
                   rf_ch[0],  rf_ch[1],  rf_ch[2],  rf_ch[3],
                   rf_ch[4],  rf_ch[5],  rf_ch[6],  rf_ch[7],
                   rf_ch[8],  rf_ch[9],  rf_ch[10], rf_ch[11],
                   rf_ch[12], rf_ch[13], rf_ch[14], rf_ch[15],
                   MODE_STR[mode]);
        }
    }
}

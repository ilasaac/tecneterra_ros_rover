/**
 * ppm_decoder — RP2040 firmware
 *
 * Reads two PPM inputs (one per rover), decodes 8 channels each, and streams
 * both rovers on a single line over USB CDC serial to the simulator Jetson Nano.
 *
 * ── Wiring ────────────────────────────────────────────────────────────────────
 *   GP0 ← PPM from Rover 1 RP2040 (GP15)   share GND
 *   GP1 ← PPM from Rover 2 RP2040 (GP15)   share GND
 *   USB → Simulator Jetson Nano (USB hub)
 *
 * ── Serial output (USB CDC, ~50 Hz) ──────────────────────────────────────────
 *   RV1:ch0,ch1,ch2,ch3,ch4,ch5,ch6,ch7 RV2:ch0,ch1,ch2,ch3,ch4,ch5,ch6,ch7\n
 *
 *   Both rovers are printed on the same line once both frames have been decoded.
 *   Channel values in µs, range 1000–2000, neutral = 1500.
 *
 * ── PPM signal format (wFly RF209S / ppm_tx.pio wFly style) ──────────────────
 *   Idle HIGH. Each channel:  LOW separator ~300 µs + HIGH gap = channel_µs − 300 µs
 *   Sync gap (frame end):     LOW separator + HIGH gap = 20000 − Σchannels − 300 µs
 *
 *   PIO measures each HIGH gap. CPU side:
 *     gap_µs + 300 = channel_µs       (channel gaps, gap < SYNC_GAP_US)
 *     idx >= PPM_CHANNELS → sync       (primary end-of-frame detector, polarity-safe)
 *     gap_µs > SYNC_GAP_US → sync      (secondary detector for startup sync)
 *
 * ── For Claude ────────────────────────────────────────────────────────────────
 *   PPM_CHANNELS = 8 (matches master/slave ppm_tx.pio PPM_CHANNELS).
 *   ppm_rx.pio measures HIGH gap duration using a 2-instruction count loop
 *   (jmp x-- / jmp pin) at clkdiv=62.5 → 1 µs per count.
 *   Output is held until BOTH rovers complete a frame so lines stay aligned.
 *   If one rover is offline, output stalls — acceptable for a wired simulator bench.
 */

#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

#include "ppm_rx.pio.h"

// ── Pin assignments ────────────────────────────────────────────────────────────

#define PPM_PIN_RV1     0       // GP0: PPM input from Rover 1 RP2040 GP15
#define PPM_PIN_RV2     1       // GP1: PPM input from Rover 2 RP2040 GP15

// ── PIO assignment ─────────────────────────────────────────────────────────────

#define PIO_INST        pio0
#define SM_RV1          0       // state machine for RV1
#define SM_RV2          1       // state machine for RV2

// ── PPM protocol constants ─────────────────────────────────────────────────────

#define PPM_CHANNELS    8       // channels per frame (must match master/slave PPM_CHANNELS)
#define PPM_SEP_US      300     // separator LOW pulse width in µs (constant)

// Sync gap threshold (HIGH gap duration, µs).
// sync_gap = 20000 − Σchannels − 300. Worst case (all ch=2000): sync = 1700 µs.
// idx >= PPM_CHANNELS handles that edge case; this threshold catches mid-frame
// startup where idx is still low but a large sync gap passes through.
#define SYNC_GAP_US     2500

// ── Per-rover frame assembler ──────────────────────────────────────────────────

typedef struct {
    uint16_t ch[PPM_CHANNELS];
    int      idx;       // next channel slot to fill (0 = waiting for first gap)
    bool     updated;   // set true when a complete 8-channel frame is decoded
} Rover;

static Rover rv1 = { .ch = {1500,1500,1500,1500,1500,1500,1500,1500}, .idx = 0, .updated = false };
static Rover rv2 = { .ch = {1500,1500,1500,1500,1500,1500,1500,1500}, .idx = 0, .updated = false };

// Feed one HIGH-gap measurement (µs) into the rover's frame assembler.
// Call this every time a value is popped from the PIO RX FIFO.
static void ppm_feed_gap(Rover *r, uint32_t gap_us) {

    // End-of-frame: large sync gap, OR all 8 channel slots filled.
    // The idx check handles the edge case where all channels are at 2000 µs
    // (making sync gap = 1700 µs, below SYNC_GAP_US threshold).
    if (gap_us > SYNC_GAP_US || r->idx >= PPM_CHANNELS) {
        // Require at least half the channels before declaring a valid frame
        // (prevents a spurious "complete" frame when joining mid-frame at boot).
        if (r->idx >= PPM_CHANNELS / 2) r->updated = true;
        r->idx = 0;
        return;
    }

    // Channel gap: channel_µs = gap_us + PPM_SEP_US, clamped to valid PPM range.
    uint16_t val = (uint16_t)(gap_us + PPM_SEP_US);
    if (val <  800) val =  800;
    if (val > 2200) val = 2200;
    r->ch[r->idx++] = val;
}

// ── Main ──────────────────────────────────────────────────────────────────────

int main(void) {
    stdio_usb_init();
    sleep_ms(500);      // allow USB enumeration

    // Load PPM RX program once; both SMs share the same instruction memory
    // but run independently on their own pins.
    uint offset = pio_add_program(PIO_INST, &ppm_rx_program);
    ppm_rx_program_init(PIO_INST, SM_RV1, offset, PPM_PIN_RV1);
    ppm_rx_program_init(PIO_INST, SM_RV2, offset, PPM_PIN_RV2);

    printf("[BOOT] PPM decoder ready  RV1=GP%d  RV2=GP%d\n",
           PPM_PIN_RV1, PPM_PIN_RV2);

    while (true) {

        // Drain both PIO FIFOs (non-blocking)
        while (!pio_sm_is_rx_fifo_empty(PIO_INST, SM_RV1))
            ppm_feed_gap(&rv1, pio_sm_get(PIO_INST, SM_RV1));

        while (!pio_sm_is_rx_fifo_empty(PIO_INST, SM_RV2))
            ppm_feed_gap(&rv2, pio_sm_get(PIO_INST, SM_RV2));

        // Print only when both rovers have a fresh decoded frame, so both
        // appear on the same line and are always time-aligned (~50 Hz).
        if (rv1.updated && rv2.updated) {
            rv1.updated = false;
            rv2.updated = false;
            printf("RV1:%d,%d,%d,%d,%d,%d,%d,%d RV2:%d,%d,%d,%d,%d,%d,%d,%d\n",
                   rv1.ch[0], rv1.ch[1], rv1.ch[2], rv1.ch[3],
                   rv1.ch[4], rv1.ch[5], rv1.ch[6], rv1.ch[7],
                   rv2.ch[0], rv2.ch[1], rv2.ch[2], rv2.ch[3],
                   rv2.ch[4], rv2.ch[5], rv2.ch[6], rv2.ch[7]);
        }
    }
}

# Firmware — CLAUDE.md

See root CLAUDE.md for system-wide context. This file covers firmware-specific details.

## Files

```
firmware/rc_link_sx1278/
├── master/
│   ├── sbus_rx.pio     PIO SBUS inverted UART, 8× oversampling, clkdiv=156.25
│   ├── ppm_tx.pio      PIO PPM generator, 1µs/cycle, side-set output, DMA-fed
│   ├── sx1278.h        SX1278 register map + init/send/recv inline functions
│   ├── main.cpp        Main: SBUS parse, mode logic, DMA IRQ, SX1278 TX, Jetson CDC
│   └── CMakeLists.txt  pico_generate_pio_header() for both .pio files
└── slave/
    ├── ppm_tx.pio      Same as master (copy)
    ├── sx1278.h        Same as master (copy) — slave uses sx1278_start_rx()
    ├── main.cpp        Main: SX1278 poll, mode logic, DMA IRQ, Jetson CDC
    └── CMakeLists.txt
firmware/ppm_decoder/               ← third RP2040, on simulator Jetson USB hub
    ├── ppm_rx.pio      PIO HIGH-gap decoder, clkdiv=62.5 (1µs/count), wFly polarity
    ├── main.cpp        Decodes 2 PPM inputs → single combined serial line at ~50 Hz
    └── CMakeLists.txt
```

## PIO timing derivations

### sbus_rx.pio (master only)
- SBUS: 100 kbaud, 8E2, inverted (idle LOW, start HIGH)
- 8× oversampling → clkdiv = 125 MHz / (8 × 100 kHz) = **156.25**
- 1 SM cycle = 1.25 µs = 1/8 bit period
- `wait 1 pin 0` → `set x,7 [10]` → loop `in [6] jmp`
  - Delay to first sample: wait(1) + set(1) + [10] = 12 SM cycles = 1.5 bit periods ✓
  - Per-bit loop: in(1) + [6] + jmp(1) = 8 SM cycles = 1 bit period ✓
- Read from FIFO: `byte = (pio_sm_get(pio0, SM_SBUS) >> 24) ^ 0xFF`
  - `>> 24`: right-shift ISR, byte is in bits [31:24]
  - `^ 0xFF`: invert bits (SBUS data is logically inverted)

### ppm_tx.pio (master SM1, slave SM0) — wFly polarity
- 1 µs per SM cycle → clkdiv = 125 MHz / 1 MHz = **125.0**
- **Idle HIGH** (wFly RF209S compatible). Separator pulse is LOW, channel gap is HIGH.
- `pull side 0` (1µs) + `mov x, osr side 0` (1µs) + `jmp x-- sep_loop side 0` (N µs) = **(N+2) µs LOW** separator
- `pull side 1` (1µs) + `mov x, osr side 1` (1µs) + `jmp x-- gap_loop side 1` (N µs) = **(N+2) µs HIGH** gap
- DMA buffer formula (unchanged from timing perspective — only polarity flipped in PIO):
  - `SEP_COUNT = 298` (= 300µs − 2 overhead, constant, drives separator LOW pulse)
  - `GAP_CHn   = channel_µs − 302` (range 698–1698, drives HIGH channel gap)
  - `GAP_SYNC  = 20000 − Σ(channels) − 302` (drives HIGH sync gap)
- Init sets GPIO HIGH before handing pin to PIO so line idles correctly from boot.
- **Only `ppm_buf_update()` writes `ppm_buf[]` — always called from DMA IRQ context.**
- `PPM_CHANNELS = 8` (hardware PPM wire); `CHANNELS = 16` (SBUS decode + serial).

## DMA setup (both master and slave)

```c
// DMA feeds PIO SM TX FIFO, paced by DREQ (PIO0_TX1 on master, PIO0_TX0 on slave)
channel_config_set_dreq(&cfg, pio_get_dreq(PIO_INST, SM_PPM, true));
// 18 words per frame (8 channels × 2 + 2), restarts in DMA_IRQ_0 handler
dma_channel_configure(chan, &cfg, &pio0->txf[SM_PPM], ppm_buf, PPM_BUF_LEN, false);
```

IRQ fires when PIO has consumed all 18 words (during sync HIGH gap, ~4–10 ms before
next frame needs data). Restart window is comfortable.

## ppm_decoder (simulator RP2040)

Third RP2040 sitting on the simulator Jetson's USB hub. Decodes the two hardware
PPM outputs (RV1 GP15 → GP0, RV2 GP15 → GP1) and forwards them over USB serial.

### ppm_rx.pio — HIGH-gap decoder (wFly polarity)
- `wait 0 pin 0` → `wait 1 pin 0`: detect rising edge after LOW separator
- Count-down loop: `jmp x-- check` + `jmp pin count` = **2 SM cycles = 1 µs** per count
- clkdiv = 62.5 → 2 MHz SM clock → 0.5 µs/cycle
- Push: `~x` = elapsed µs. CPU: `channel_µs = gap_µs + 300`
- Both SMs (SM0=RV1, SM1=RV2) share the same program offset, independent pins.

### Serial output format
```
RV1:ch0,ch1,ch2,ch3,ch4,ch5,ch6,ch7 RV2:ch0,ch1,ch2,ch3,ch4,ch5,ch6,ch7\n
```
Printed at ~50 Hz. Output is held until **both** rovers have a fresh decoded frame
so both always appear on the same line, time-aligned. If one rover is offline,
output stalls — acceptable for a wired simulator bench.

### End-of-frame detection (two-tier)
1. `gap_µs > 2500` → obvious sync (typical, sync gap usually >> 3000 µs)
2. `idx >= PPM_CHANNELS` → count-based fallback (handles worst case: all channels
   at 2000 µs makes sync gap = 1700 µs, below threshold)

## SX1278 LoRa configuration

| Register          | Value  | Meaning                              |
|-------------------|--------|--------------------------------------|
| RegOpMode (0x01)  | 0x81   | LoRa mode + Sleep (then Standby)     |
| RegFrMsb/Mid/Lsb  | 0x6C / 0x80 / 0x00 | 433.0 MHz              |
| RegPaConfig(0x09) | 0x8F   | PA_BOOST, +17 dBm                    |
| RegOcp (0x0B)     | 0x2B   | 120 mA overcurrent protection        |
| RegModemConfig1   | 0x92   | BW=500kHz, CR=4/5, explicit header   |
| RegModemConfig2   | 0x74   | SF=7, no TxCont, CRC on              |
| RegModemConfig3   | 0x04   | AGC auto on                          |
| RegSyncWord(0x39) | 0x12   | Private LoRa network (not LoRaWAN)   |
| RegDioMapping1    | 0x40   | DIO0 = TxDone (master TX mode)       |
| RegDioMapping1    | 0x00   | DIO0 = RxDone (slave RX mode)        |

**Time on air (32-byte payload, 16 × uint16_t):** ~12.9 ms → TX at 25 Hz (every 2nd PPM frame).

## Build (requires Pico SDK)

```bash
# Copy pico_sdk_import.cmake first (one-time per project)
cp $PICO_SDK_PATH/external/pico_sdk_import.cmake firmware/rc_link_sx1278/master/
cp $PICO_SDK_PATH/external/pico_sdk_import.cmake firmware/rc_link_sx1278/slave/
cp $PICO_SDK_PATH/external/pico_sdk_import.cmake firmware/ppm_decoder/

# Build master
cd firmware/rc_link_sx1278/master
cmake -B build -DPICO_SDK_PATH=$PICO_SDK_PATH
cmake --build build -j4
# Flash: copy build/rc_link_sx1278_master.uf2 to RP2040 USB drive

# Build slave / ppm_decoder (same steps, different directories)
```

## Pitfalls

- `ppm_buf[]` must only be written in `dma_irq_handler()`. Main loop writes `out_ch[]` (volatile uint16_t, M0+ aligned = safe), IRQ reads it.
- `sx1278_send()` returns `false` if previous TX still running — main loop skips that frame silently.
- SBUS re-syncs on every `0x0F` byte. This can cause a spurious frame if a data byte happens to be `0x0F` and the next 24 bytes validate. Unlikely but possible — the real guard is the end byte `0x00`.
- SX1278 `RegVersion` must read `0x12` on startup. If it doesn't, check SPI wiring and pull-ups on NSS.
- **PIO parity bit fix (applied)**: `push noblock [10]` delays the `wait 1 pin 0` instruction by 10 SM cycles (12.5 µs) past the last data bit sample. This lands the wait safely in stop bit 2 (always LOW on inverted SBUS), preventing false re-triggers on bytes with HIGH parity bits.
- **16 channels decoded, 8 on PPM wire**: `CHANNELS=16` (SBUS/LoRa/serial), `PPM_CHANNELS=8` (hardware output). `PPM_BUF_LEN=18` (8×2+2). SX1278 payload is 32 bytes (16×uint16_t). Serial format is `CH:c0,...,c15 MODE:...`.
- **PPM polarity is wFly style** (idle HIGH, LOW separator). Do not swap side-sets back — motor controllers expect this format.
- **ppm_decoder expects wFly polarity** (measures HIGH gap). If ppm_tx polarity is changed, ppm_rx.pio must change in lockstep.
- **PPM channel mapping** (hardware output only — USB serial is always raw SBUS/RF):

  | PPM CH | SBUS source | Inverted? |
  |--------|-------------|-----------|
  | CH1    | CH3         |           |
  | CH2    | CH1         | yes       |
  | CH3    | CH5         | yes       |
  | CH4    | CH6         | yes       |
  | CH5    | CH11        |           |
  | CH6    | CH12        |           |
  | CH7    | CH7         | yes       |
  | CH8    | CH8         | yes       |

  Inversion formula: `3000 − value` (maps 1000↔2000, 1500 stays centred).
  Implemented in `apply_ppm_map()` in both master and slave `main.cpp`.

- **LoRa payload must be `sbus_ch[]`, not `out_ch[]`:** `out_ch` is already PPM-remapped for local hardware output. Transmitting it causes slave to remap again (double-map = wrong channels). In RELAY mode `out_ch` is also neutralised to 1500, so transmitting it would prevent the slave from receiving real stick values. Fix applied in `master/main.cpp` — always pack `sbus_ch[]` into the LoRa payload.

- **`sx1278_recv()` stack overflow (fix applied):** `SX_REG_RX_NB_BYTES` can return a
  garbage value (e.g. `0xFF`) if the SPI bus glitches during RF reception. The caller
  allocates `pkt[CHANNELS*2]` = 32 bytes on the stack; `sx_read_fifo(pkt, 255)` would
  overflow it by 223 bytes, corrupting Cortex-M0+ return addresses → hard fault → PPM
  stops, USB CDC silent, requires manual reset. RF timeout (500 ms) is self-recovering;
  a crash is not.

  **Symptoms that confirm a crash (not RF dropout):**
  - `/rv2/mode` topic goes completely silent (RF dropout shows `EMERGENCY` first)
  - `rp2040_bridge` logs no serial error — `in_waiting` returns 0 silently; no `OSError`
    raised because TinyUSB stops but Linux keeps the CDC device "open" until timeout
  - PPM pin stops toggling entirely (DMA is single-shot; IRQ can't restart it when CPU is crashed)

  **Fix (applied in `sx1278.h`):** bounds-check `*len` before calling `sx_read_fifo`.
  `sx1278_start_rx` must be defined **before** `sx1278_recv` in the header (forward
  reference; moved in both master and slave copies):
  ```c
  if (*len == 0 || *len > 64) {
      sx1278_start_rx();   // re-arm receiver, discard corrupt packet
      return false;
  }
  ```

- **SX1278 power-cycle watchdog (slave only, applied in `slave/main.cpp`):**
  When the SX1278 module is power-cycled while the RP2040 is running (e.g. external 3.3 V rail glitch), `RegOpMode` resets to `0x09` (FSK standby). The slave firmware never detects this and stops receiving — `rf_ok` stays `false` forever and the RF link never recovers without a manual RP2040 reset.

  **Fix:** while `rf_ok == false`, every `SX_WATCHDOG_MS` (2000 ms) read `RegOpMode` via `sx_read_reg()`. If the value is not `SX_LORA_MODE | SX_MODE_RX_CONT` (0x85), call `sx1278_init()` then `sx1278_start_rx()`. Log `[SX1278_REINIT] opmode=0x%02x` on recovery. Watchdog only runs when RF is lost — it does not disturb a working chip.

  State variable: `uint64_t t_last_sx_check = 0` in main loop scope.

- **CH9 rover-select / mode logic (3-position switch):**

  | CH9 value      | Master              | Slave                    |
  |----------------|---------------------|--------------------------|
  | Low  < 1250    | MANUAL (moves)      | IDLE (neutral, no alarm) |
  | Mid  1250–1750 | RELAY or AUTO       | AUTONOMOUS or IDLE       |
  | High > 1750    | RELAY (neutral)     | MODE_RF (moves)          |

  - **RELAY** (master, CH9 mid, CH5 not set): master PPM neutral, raw `sbus_ch[]` forwarded over LoRa.
  - **AUTO** (either rover, CH9 mid + CH4 not triggered + CH5 set + HB alive + fresh `<J:>` cmd): Jetson drives PPM.
  - Slave checks CH4 and CH5 directly from the received `rf_ch[]` (raw SBUS values forwarded by master).

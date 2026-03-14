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

### ppm_tx.pio (master SM1, slave SM0)
- 1 µs per SM cycle → clkdiv = 125 MHz / 1 MHz = **125.0**
- `pull side 1` (1µs) + `mov x, osr side 1` (1µs) + `jmp x-- loop side 1` (N µs) = **(N+2) µs** HIGH
- Same structure for LOW with `side 0`
- DMA buffer formula:
  - `HIGH_COUNT = 298` (= 300µs − 2 overhead, constant)
  - `LOW_CHn   = channel_µs − 302` (range 698–1698 for 1000–2000µs channels)
  - `LOW_SYNC  = 20000 − Σ(channels) − 302`
- **Only `ppm_buf_update()` writes `ppm_buf[]` — always called from DMA IRQ context.**

## DMA setup (both master and slave)

```c
// DMA feeds PIO SM TX FIFO, paced by DREQ (PIO0_TX1 on master, PIO0_TX0 on slave)
channel_config_set_dreq(&cfg, pio_get_dreq(PIO_INST, SM_PPM, true));
// 20 words per frame, restarts in DMA_IRQ_0 handler
dma_channel_configure(chan, &cfg, &pio0->txf[SM_PPM], ppm_buf, 20, false);
```

IRQ fires when PIO has consumed all 20 words (during sync LOW phase, ~4–10 ms before
next frame needs data). Restart window is comfortable.

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

**Time on air (18-byte payload):** ~12.9 ms → TX at 25 Hz (every 2nd PPM frame).

## Build (requires Pico SDK)

```bash
# Copy pico_sdk_import.cmake first (one-time)
cp $PICO_SDK_PATH/external/pico_sdk_import.cmake firmware/rc_link_sx1278/master/
cp $PICO_SDK_PATH/external/pico_sdk_import.cmake firmware/rc_link_sx1278/slave/

# Build master
cd firmware/rc_link_sx1278/master
cmake -B build -DPICO_SDK_PATH=$PICO_SDK_PATH
cmake --build build -j4
# Flash: copy build/rc_link_sx1278_master.uf2 to RP2040 USB drive

# Build slave (same steps, different directory)
```

## Pitfalls

- `ppm_buf[]` must only be written in `dma_irq_handler()`. Main loop writes `out_ch[]` (volatile uint16_t, M0+ aligned = safe), IRQ reads it.
- `sx1278_send()` returns `false` if previous TX still running — main loop skips that frame silently.
- SBUS re-syncs on every `0x0F` byte. This can cause a spurious frame if a data byte happens to be `0x0F` and the next 24 bytes validate. Unlikely but possible — the real guard is the end byte `0x00`.
- SX1278 `RegVersion` must read `0x12` on startup. If it doesn't, check SPI wiring and pull-ups on NSS.

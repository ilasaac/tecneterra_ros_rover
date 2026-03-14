#pragma once
/**
 * sx1278.h — Minimal SX1278 LoRa driver (SPI register access + TX/RX)
 *
 * Configured for: 433 MHz, LoRa, SF7, BW500kHz, CR4/5, explicit header, CRC on.
 * Max output: +17 dBm via PA_BOOST.
 * Time on air (18-byte payload): ~12.9 ms → use at 25 Hz (every other PPM frame).
 *
 * NOTE on duty cycle: LoRa at 433 MHz is subject to 1–10% duty cycle limits
 * in many jurisdictions. On private agricultural land this is typically acceptable,
 * but switch to FSK mode (SX1278 supports it) for unrestricted operation.
 *
 * Usage:
 *   sx1278_init();
 *   sx1278_send(payload, 18);   // non-blocking, returns false if still TX
 *   sx1278_recv(buf, &len);     // slave only — check DIO0 interrupt
 */

#include <stdint.h>
#include <stdbool.h>
#include "hardware/spi.h"
#include "hardware/gpio.h"

// ── SPI instance and pins (match CMakeLists / main.cpp) ──────────────────────
#define SX_SPI      spi0
#define SX_MOSI     19
#define SX_MISO     16
#define SX_SCK      18
#define SX_NSS      17   // chip select (active LOW)
#define SX_RST      20
#define SX_DIO0     21   // TxDone / RxDone interrupt

// ── Register map ─────────────────────────────────────────────────────────────
#define SX_REG_FIFO              0x00
#define SX_REG_OP_MODE           0x01
#define SX_REG_FR_MSB            0x06
#define SX_REG_FR_MID            0x07
#define SX_REG_FR_LSB            0x08
#define SX_REG_PA_CONFIG         0x09
#define SX_REG_OCP               0x0B
#define SX_REG_LNA               0x0C
#define SX_REG_FIFO_ADDR_PTR     0x0D
#define SX_REG_FIFO_TX_BASE      0x0E
#define SX_REG_FIFO_RX_BASE      0x0F
#define SX_REG_FIFO_RX_CURRENT   0x10
#define SX_REG_IRQ_FLAGS_MASK    0x11
#define SX_REG_IRQ_FLAGS         0x12
#define SX_REG_RX_NB_BYTES       0x13
#define SX_REG_MODEM_CONFIG1     0x1D
#define SX_REG_MODEM_CONFIG2     0x1E
#define SX_REG_PREAMBLE_MSB      0x20
#define SX_REG_PREAMBLE_LSB      0x21
#define SX_REG_PAYLOAD_LENGTH    0x22
#define SX_REG_MODEM_CONFIG3     0x26
#define SX_REG_SYNC_WORD         0x39
#define SX_REG_DIO_MAPPING1      0x40
#define SX_REG_VERSION           0x42

// ── OpMode values ─────────────────────────────────────────────────────────────
#define SX_LORA_MODE    0x80   // bit 7: LoRa vs FSK
#define SX_MODE_SLEEP   0x00
#define SX_MODE_STANDBY 0x01
#define SX_MODE_TX      0x03
#define SX_MODE_RX_CONT 0x05

// ── IRQ flags (RegIrqFlags) ───────────────────────────────────────────────────
#define SX_IRQ_RX_DONE          0x40
#define SX_IRQ_TX_DONE          0x08
#define SX_IRQ_PAYLOAD_CRC_ERR  0x20
#define SX_IRQ_VALID_HEADER     0x10

// ── Config constants ──────────────────────────────────────────────────────────
// 433.0 MHz: Frf = 433e6 / (32e6 / 524288) = 7,094,272 = 0x6C8000
#define SX_FREQ_MSB  0x6C
#define SX_FREQ_MID  0x80
#define SX_FREQ_LSB  0x00

// ModemConfig1: BW=500kHz(0x9), CR=4/5(0x1), ExplicitHeader(0)
//   bits[7:4]=BW, bits[3:1]=CR, bit[0]=ImplicitHeader
#define SX_MC1  ((9 << 4) | (1 << 1) | 0)   // 0x92

// ModemConfig2: SF=7(0x7), TxContinuous=0, CRC=1
//   bits[7:4]=SF, bit[3]=TxCont, bit[2]=CRCon
#define SX_MC2  ((7 << 4) | (0 << 3) | (1 << 2))  // 0x74

// ModemConfig3: LowDataRateOpt=0, AGCAutoOn=1
#define SX_MC3  0x04

// PaConfig: PA_BOOST, MaxPower=7, OutputPower=15 → +17 dBm
#define SX_PA_CONFIG  0x8F

// DIO0 = TxDone (01) in LoRa TX mode
#define SX_DIO_TX_DONE  0x40

// ── Low-level SPI ─────────────────────────────────────────────────────────────

static inline void sx_cs_low(void)  { gpio_put(SX_NSS, 0); }
static inline void sx_cs_high(void) { gpio_put(SX_NSS, 1); }

static inline void sx_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { (uint8_t)(reg | 0x80), val };
    sx_cs_low();
    spi_write_blocking(SX_SPI, buf, 2);
    sx_cs_high();
}

static inline uint8_t sx_read_reg(uint8_t reg) {
    uint8_t tx[2] = { (uint8_t)(reg & 0x7F), 0x00 };
    uint8_t rx[2] = { 0, 0 };
    sx_cs_low();
    spi_write_read_blocking(SX_SPI, tx, rx, 2);
    sx_cs_high();
    return rx[1];
}

static inline void sx_write_fifo(const uint8_t *data, uint8_t len) {
    uint8_t addr = (uint8_t)(SX_REG_FIFO | 0x80);
    sx_cs_low();
    spi_write_blocking(SX_SPI, &addr, 1);
    spi_write_blocking(SX_SPI, data, len);
    sx_cs_high();
}

static inline void sx_read_fifo(uint8_t *data, uint8_t len) {
    uint8_t addr = SX_REG_FIFO & 0x7F;
    sx_cs_low();
    spi_write_blocking(SX_SPI, &addr, 1);
    spi_read_blocking(SX_SPI, 0x00, data, len);
    sx_cs_high();
}

// ── Public API ────────────────────────────────────────────────────────────────

static inline bool sx1278_init(void) {
    // SPI hardware
    spi_init(SX_SPI, 1000000);   // 1 MHz — safe for wiring on PCB
    gpio_set_function(SX_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(SX_MISO, GPIO_FUNC_SPI);
    gpio_set_function(SX_SCK,  GPIO_FUNC_SPI);

    // NSS / RST as GPIO outputs
    gpio_init(SX_NSS); gpio_set_dir(SX_NSS, GPIO_OUT); gpio_put(SX_NSS, 1);
    gpio_init(SX_RST); gpio_set_dir(SX_RST, GPIO_OUT);

    // DIO0 as input (TxDone / RxDone)
    gpio_init(SX_DIO0); gpio_set_dir(SX_DIO0, GPIO_IN);

    // Hardware reset: pull RST LOW 10 ms then HIGH
    gpio_put(SX_RST, 0); sleep_ms(10);
    gpio_put(SX_RST, 1); sleep_ms(10);

    // Verify chip: RegVersion should be 0x12
    uint8_t ver = sx_read_reg(SX_REG_VERSION);
    if (ver != 0x12) return false;   // SX1278 not found

    // Switch to LoRa sleep mode first (required before changing to LoRa)
    sx_write_reg(SX_REG_OP_MODE, SX_LORA_MODE | SX_MODE_SLEEP);
    sleep_ms(10);

    // Frequency: 433 MHz
    sx_write_reg(SX_REG_FR_MSB, SX_FREQ_MSB);
    sx_write_reg(SX_REG_FR_MID, SX_FREQ_MID);
    sx_write_reg(SX_REG_FR_LSB, SX_FREQ_LSB);

    // PA: +17 dBm via PA_BOOST, OCP 120 mA
    sx_write_reg(SX_REG_PA_CONFIG, SX_PA_CONFIG);
    sx_write_reg(SX_REG_OCP, 0x2B);   // OCP on, 120 mA

    // LNA: max gain, AGC on (overridden by ModemConfig3 AGC)
    sx_write_reg(SX_REG_LNA, 0x23);

    // Modem: SF7, BW500, CR4/5, CRC on, AGC on
    sx_write_reg(SX_REG_MODEM_CONFIG1, SX_MC1);
    sx_write_reg(SX_REG_MODEM_CONFIG2, SX_MC2);
    sx_write_reg(SX_REG_MODEM_CONFIG3, SX_MC3);

    // Preamble: 8 symbols
    sx_write_reg(SX_REG_PREAMBLE_MSB, 0x00);
    sx_write_reg(SX_REG_PREAMBLE_LSB, 0x08);

    // Sync word: 0x12 (LoRa private network, different from LoRaWAN 0x34)
    sx_write_reg(SX_REG_SYNC_WORD, 0x12);

    // TX FIFO base at 0x00, RX FIFO base at 0x00
    sx_write_reg(SX_REG_FIFO_TX_BASE, 0x00);
    sx_write_reg(SX_REG_FIFO_RX_BASE, 0x00);

    // DIO0 = TxDone (01) for TX mode
    sx_write_reg(SX_REG_DIO_MAPPING1, SX_DIO_TX_DONE);

    // Return to standby
    sx_write_reg(SX_REG_OP_MODE, SX_LORA_MODE | SX_MODE_STANDBY);

    return true;
}

// Send payload (non-blocking). Returns false if previous TX is still running.
static inline bool sx1278_send(const uint8_t *data, uint8_t len) {
    // Check TxDone — if DIO0 is HIGH, previous TX is complete
    // If DIO0 is LOW and we're in TX mode, previous TX is still running
    uint8_t irq = sx_read_reg(SX_REG_IRQ_FLAGS);
    if ((irq & SX_IRQ_TX_DONE) == 0) {
        uint8_t opmode = sx_read_reg(SX_REG_OP_MODE);
        if ((opmode & 0x07) == SX_MODE_TX) return false;   // still transmitting
    }

    // Clear all IRQ flags
    sx_write_reg(SX_REG_IRQ_FLAGS, 0xFF);

    // Standby before loading FIFO
    sx_write_reg(SX_REG_OP_MODE, SX_LORA_MODE | SX_MODE_STANDBY);

    // Reset FIFO pointer to TX base
    sx_write_reg(SX_REG_FIFO_ADDR_PTR, 0x00);

    // Write payload
    sx_write_fifo(data, len);
    sx_write_reg(SX_REG_PAYLOAD_LENGTH, len);

    // Start TX (returns immediately; TxDone fires on DIO0 when done)
    sx_write_reg(SX_REG_OP_MODE, SX_LORA_MODE | SX_MODE_TX);
    return true;
}

// Poll for received packet (slave). Call from main loop.
// Returns true if a valid packet is in buf[], length in *len.
static inline bool sx1278_recv(uint8_t *buf, uint8_t *len) {
    uint8_t irq = sx_read_reg(SX_REG_IRQ_FLAGS);

    if (!(irq & SX_IRQ_RX_DONE)) return false;           // nothing yet

    // Clear IRQ flags
    sx_write_reg(SX_REG_IRQ_FLAGS, 0xFF);

    if (irq & SX_IRQ_PAYLOAD_CRC_ERR) return false;      // discard CRC errors

    *len = sx_read_reg(SX_REG_RX_NB_BYTES);
    uint8_t rx_ptr = sx_read_reg(SX_REG_FIFO_RX_CURRENT);
    sx_write_reg(SX_REG_FIFO_ADDR_PTR, rx_ptr);
    sx_read_fifo(buf, *len);

    return true;
}

// Start continuous RX mode (slave). Call once after sx1278_init().
static inline void sx1278_start_rx(void) {
    // DIO0 = RxDone (00 in LoRa mode)
    sx_write_reg(SX_REG_DIO_MAPPING1, 0x00);
    sx_write_reg(SX_REG_FIFO_ADDR_PTR, 0x00);
    sx_write_reg(SX_REG_OP_MODE, SX_LORA_MODE | SX_MODE_RX_CONT);
}

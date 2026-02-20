/* ===================================================================
 * LatchPac Validator 3000 -- SWD Host (Bit-Bang) v2
 *
 * v2 adds:
 *   - Generic SWD transaction engine (swd_transfer)
 *   - DP/AP register read/write
 *   - Debug domain power-up
 *   - MEM-AP memory read/write/block
 *   - Signal integrity self-test
 *   - Raw IDCODE value readout
 *
 * All GPIO numbers come from fixture_pins.h.
 * Uses ESP-IDF ROM delay for precise SWD timing.
 *
 * Supports two wiring modes:
 *   DIRECT   -- single bidirectional GPIO for SWDIO (default)
 *   ISOLATED -- split SWDIO into two unidirectional GPIOs via
 *              6N137 optocouplers (CONFIG_LATCHPAC_SWD_ISOLATED)
 *
 * Expected IDCODE: 0x0BC11477 (Cortex-M0+ DP)
 * =================================================================== */

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"        /* esp_rom_delay_us */
#include "esp_timer.h"

#include "fixture_pins.h"
#include "swd_host.h"

/* ------------------------------------------------------------------ */
/*  Verbose debug logging (CONFIG_LATCHPAC_SWD_VERBOSE)                  */
/* ------------------------------------------------------------------ */
#ifdef CONFIG_LATCHPAC_SWD_VERBOSE
#define SWD_LOG(fmt, ...)  printf("SWD_DBG, " fmt "\n", ##__VA_ARGS__)
#else
#define SWD_LOG(fmt, ...)  do {} while(0)
#endif

/* ------------------------------------------------------------------ */
/*  Timing & Retry                                                      */
/* ------------------------------------------------------------------ */
#define SWD_DELAY_US    1       /* ~500 kHz SWD clock (1 us half-period) */
#define SWD_MAX_RETRIES 3       /* Attempts before declaring SWD failure */
#define SWD_RETRY_MS    50      /* Delay between retries                 */
#define SWD_IDLE_CYCLES 4       /* Extra idle clocks after transactions  */
#define SWD_WAIT_RETRIES 8      /* Max retries on WAIT ACK              */
#define SWD_WAIT_TIMEOUT_US 200000  /* 200ms wall-clock max for WAIT loop */

/* ABORT register bits -- write to DP addr 0x00 to clear sticky errors */
#define ABORT_ORUNERRCLR  (1u << 4)
#define ABORT_WDERRCLR    (1u << 3)
#define ABORT_STKERRCLR   (1u << 2)
#define ABORT_STKCMPCLR   (1u << 1)
#define ABORT_DAPABORT    (1u << 0)
#define ABORT_CLEAR_ALL   (ABORT_ORUNERRCLR | ABORT_WDERRCLR | \
                           ABORT_STKERRCLR | ABORT_STKCMPCLR | ABORT_DAPABORT)

/*
 * In isolated mode the optocoupler propagation delay (~100 ns for
 * 6N137) adds latency.  We use 2 us half-period to give the signal
 * time to settle through the opto-barrier.
 */
#ifdef SWD_ISOLATED
#define SWD_ISO_DELAY_US  2     /* Slower clock for opto path */
#define CLK_IDLE   1    /* GPIO level for CLK idle  (target sees LOW)  */
#define CLK_ACTIVE 0    /* GPIO level for CLK pulse (target sees HIGH) */
#define NRST_ASSERT   1 /* GPIO level to assert nRST  (target sees LOW)  */
#define NRST_DEASSERT 0 /* GPIO level to deassert nRST (target sees HIGH) */
#else
#define SWD_ISO_DELAY_US  SWD_DELAY_US
#define CLK_IDLE   0    /* Direct: CLK idles LOW   */
#define CLK_ACTIVE 1    /* Direct: CLK pulses HIGH */
#define NRST_ASSERT   0 /* Direct: nRST LOW = assert reset  */
#define NRST_DEASSERT 1 /* Direct: nRST HIGH = deassert     */
#endif

/* ------------------------------------------------------------------ */
/*  DP register addresses                                               */
/* ------------------------------------------------------------------ */
#define DP_ABORT    0x00    /* Write-only */
#define DP_DPIDR    0x00    /* Read-only  */
#define DP_CTRLSTAT 0x04
#define DP_SELECT   0x08
#define DP_RDBUFF   0x0C

/* ------------------------------------------------------------------ */
/*  AP register addresses                                               */
/* ------------------------------------------------------------------ */
#define AP_CSW      0x00
#define AP_TAR      0x04
#define AP_DRW      0x0C

/* ------------------------------------------------------------------ */
/*  CTRL/STAT bits                                                      */
/* ------------------------------------------------------------------ */
#define CDBGPWRUPREQ  (1u << 28)
#define CSYSPWRUPREQ  (1u << 30)
#define CDBGPWRUPACK  (1u << 29)
#define CSYSPWRUPACK  (1u << 31)

/* ------------------------------------------------------------------ */
/*  CSW bits                                                            */
/* ------------------------------------------------------------------ */
#define CSW_SIZE32      (2u << 0)   /* 32-bit access          */
#define CSW_ADDRINC_OFF (0u << 4)   /* No auto-increment      */
#define CSW_ADDRINC_SGL (1u << 4)   /* Single auto-increment  */
#define CSW_DBGSTAT     (1u << 6)   /* Enable debug transfers */

/* ------------------------------------------------------------------ */
/*  SWD Request Byte Builder                                            */
/*                                                                      */
/*  Bit layout: Start(1) APnDP RnW A[2] A[3] Parity Stop(0) Park(1)    */
/* ------------------------------------------------------------------ */
static uint8_t swd_request_byte(uint8_t APnDP, uint8_t RnW, uint8_t addr)
{
    uint8_t a2 = (addr >> 2) & 1;
    uint8_t a3 = (addr >> 3) & 1;
    uint8_t parity = APnDP ^ RnW ^ a2 ^ a3;
    uint8_t req = (1u << 0)         /* Start = 1    */
               |  (APnDP << 1)
               |  (RnW   << 2)
               |  (a2    << 3)
               |  (a3    << 4)
               |  (parity << 5)
               |  (0u    << 6)      /* Stop = 0     */
               |  (1u    << 7);     /* Park = 1     */
    return req;
}

/* ------------------------------------------------------------------ */
/*  Abstraction macros for SWDIO direction and I/O                      */
/* ------------------------------------------------------------------ */

#ifdef SWD_ISOLATED

static inline void swdio_set_output_mode(void)
{
    /* No-op: PIN_SWD_IO_OUT is always an output */
}

static inline void swdio_set_input_mode(void)
{
    gpio_set_level(PIN_SWD_IO_OUT, 0);
}

static inline void swdio_write(uint8_t bit)
{
    gpio_set_level(PIN_SWD_IO_OUT, !(bit & 1));
}

static inline uint8_t swdio_read(void)
{
    return gpio_get_level(PIN_SWD_IO_IN) ? 0 : 1;
}

#else  /* Direct-wire mode */

static inline void swdio_set_output_mode(void)
{
    gpio_set_direction(PIN_SWD_IO, GPIO_MODE_OUTPUT);
}

static inline void swdio_set_input_mode(void)
{
    gpio_set_direction(PIN_SWD_IO, GPIO_MODE_INPUT);
}

static inline void swdio_write(uint8_t bit)
{
    gpio_set_level(PIN_SWD_IO, bit & 1);
}

static inline uint8_t swdio_read(void)
{
    return gpio_get_level(PIN_SWD_IO) ? 1 : 0;
}

#endif /* SWD_ISOLATED */

/* ------------------------------------------------------------------ */
/*  Low-level bit helpers                                               */
/* ------------------------------------------------------------------ */

static inline void swd_write_bit(uint8_t bit)
{
    swdio_write(bit);
    esp_rom_delay_us(SWD_ISO_DELAY_US);
    gpio_set_level(PIN_SWD_CLK, CLK_ACTIVE);
    esp_rom_delay_us(SWD_ISO_DELAY_US);
    gpio_set_level(PIN_SWD_CLK, CLK_IDLE);
}

static inline uint8_t swd_read_bit(void)
{
    esp_rom_delay_us(SWD_ISO_DELAY_US);
    gpio_set_level(PIN_SWD_CLK, CLK_ACTIVE);
    esp_rom_delay_us(SWD_ISO_DELAY_US);
    uint8_t bit = swdio_read();
    gpio_set_level(PIN_SWD_CLK, CLK_IDLE);
    return bit;
}

static void swd_idle_cycles(int n)
{
    swdio_set_output_mode();
    swdio_write(0);
    for (int i = 0; i < n; i++) {
        esp_rom_delay_us(SWD_ISO_DELAY_US);
        gpio_set_level(PIN_SWD_CLK, CLK_ACTIVE);
        esp_rom_delay_us(SWD_ISO_DELAY_US);
        gpio_set_level(PIN_SWD_CLK, CLK_IDLE);
    }
}

/* ------------------------------------------------------------------ */
/*  Parity helper                                                       */
/* ------------------------------------------------------------------ */
static uint8_t parity32(uint32_t v)
{
    v ^= v >> 16;
    v ^= v >> 8;
    v ^= v >> 4;
    v ^= v >> 2;
    v ^= v >> 1;
    return (uint8_t)(v & 1);
}

/* ------------------------------------------------------------------ */
/*  SWD Line Reset (56 clocks with IO HIGH, >= 50 required)             */
/* ------------------------------------------------------------------ */
static void swd_line_reset(void)
{
    SWD_LOG("line_reset: 56 clocks with SWDIO=1");
    swdio_set_output_mode();
    swdio_write(1);
    for (int i = 0; i < 56; i++) {
        esp_rom_delay_us(SWD_ISO_DELAY_US);
        gpio_set_level(PIN_SWD_CLK, CLK_ACTIVE);
        esp_rom_delay_us(SWD_ISO_DELAY_US);
        gpio_set_level(PIN_SWD_CLK, CLK_IDLE);
    }
    SWD_LOG("line_reset: done");
}

/* ------------------------------------------------------------------ */
/*  JTAG-to-SWD switching sequence (16-bit, LSB first)                  */
/*  ARM IHI 0031F, section B4.3.3                                       */
/*  Wire value: 0xE79E transmitted LSB-first.                           */
/* ------------------------------------------------------------------ */
static void swd_jtag_to_swd(void)
{
    SWD_LOG("jtag_to_swd: sending 0xE79E (16 bits LSB-first)");
    const uint16_t switch_seq = 0xE79E;
    swdio_set_output_mode();
    for (int i = 0; i < 16; i++) {
        swd_write_bit((switch_seq >> i) & 1);
    }
    SWD_LOG("jtag_to_swd: done");
}

/* ================================================================== */
/*  GENERIC SWD TRANSACTION ENGINE (v2 core)                            */
/*                                                                      */
/*  Handles the full SWD packet protocol:                               */
/*    Host sends: 8-bit request (LSB first)                             */
/*    Turnaround                                                        */
/*    Target sends: 3-bit ACK                                           */
/*    For reads:  Target sends 32-bit data + parity, turnaround back    */
/*    For writes: Turnaround, host sends 32-bit data + parity           */
/*                                                                      */
/*  Handles WAIT retries internally (up to SWD_WAIT_RETRIES attempts).  */
/* ================================================================== */

static swd_status_t swd_transfer(uint8_t request, uint32_t *data)
{
    uint8_t RnW = (request >> 2) & 1;
    uint8_t APnDP = (request >> 1) & 1;
    uint8_t addr = ((request >> 3) & 0x01) << 2 | ((request >> 4) & 0x01) << 3;

    SWD_LOG("xfer: req=0x%02X %s %s addr=0x%02X",
            request, APnDP ? "AP" : "DP", RnW ? "RD" : "WR", addr);

    /* Wall-clock deadline prevents infinite hang on WAIT storms */
    int64_t deadline = esp_timer_get_time() + SWD_WAIT_TIMEOUT_US;

    for (int wait_retry = 0; wait_retry < SWD_WAIT_RETRIES; wait_retry++) {

        /* --- Send 8-bit request (LSB first) --- */
        swdio_set_output_mode();
        for (int i = 0; i < 8; i++) {
            swd_write_bit((request >> i) & 1);
        }

        /* --- Turnaround (1 clock, release IO to target) --- */
        swdio_set_input_mode();
        esp_rom_delay_us(SWD_ISO_DELAY_US);
        gpio_set_level(PIN_SWD_CLK, CLK_ACTIVE);
        esp_rom_delay_us(SWD_ISO_DELAY_US);
        gpio_set_level(PIN_SWD_CLK, CLK_IDLE);

        /* --- Read 3-bit ACK (LSB first, OK = 0b001) --- */
        uint8_t ack = 0;
        for (int i = 0; i < 3; i++) {
            ack |= (swd_read_bit() << i);
        }

        SWD_LOG("xfer: ACK=0b%d%d%d (0x%02X) %s",
                (ack>>2)&1, (ack>>1)&1, ack&1, ack,
                ack==0x01 ? "OK" : ack==0x02 ? "WAIT" : ack==0x04 ? "FAULT" :
                ack==0x00 ? "NO_RESPONSE" : ack==0x07 ? "ALL_ONES" : "PROTO_ERR");

        if (ack == 0x01) {
            /* ACK OK */
            if (RnW) {
                /* --- READ: target sends 32 data bits + parity --- */
                uint32_t val = 0;
                for (int i = 0; i < 32; i++) {
                    val |= ((uint32_t)swd_read_bit() << i);
                }
                uint8_t par = swd_read_bit();

                /* Turnaround back to host */
                swdio_set_output_mode();
                swdio_write(0);
                swd_idle_cycles(SWD_IDLE_CYCLES);

                if (par != parity32(val)) {
                    SWD_LOG("xfer: PARITY ERROR data=0x%08lX par=%d expected=%d",
                            (unsigned long)val, par, parity32(val));
                    return SWD_PARITY_ERROR;
                }
                if (data) *data = val;
                SWD_LOG("xfer: READ OK data=0x%08lX", (unsigned long)val);
                return SWD_OK;
            } else {
                /* --- WRITE: turnaround, then host sends 32 data bits + parity --- */
                swdio_set_output_mode();

                /* Turnaround clock */
                swdio_write(0);
                esp_rom_delay_us(SWD_ISO_DELAY_US);
                gpio_set_level(PIN_SWD_CLK, CLK_ACTIVE);
                esp_rom_delay_us(SWD_ISO_DELAY_US);
                gpio_set_level(PIN_SWD_CLK, CLK_IDLE);

                /* 32 data bits LSB first */
                uint32_t val = data ? *data : 0;
                for (int i = 0; i < 32; i++) {
                    swd_write_bit((val >> i) & 1);
                }
                /* Parity bit */
                swd_write_bit(parity32(val));

                swd_idle_cycles(SWD_IDLE_CYCLES);
                SWD_LOG("xfer: WRITE OK data=0x%08lX", (unsigned long)val);
                return SWD_OK;
            }
        } else if (ack == 0x02) {
            /* WAIT -- retry after restoring bus, but respect wall-clock */
            SWD_LOG("xfer: WAIT retry %d/%d", wait_retry+1, SWD_WAIT_RETRIES);
            swdio_set_output_mode();
            swd_idle_cycles(SWD_IDLE_CYCLES);
            esp_rom_delay_us(100);
            if (esp_timer_get_time() > deadline) {
                SWD_LOG("xfer: WAIT wall-clock timeout (%d us)", SWD_WAIT_TIMEOUT_US);
                return SWD_TIMEOUT;
            }
            continue;
        } else if (ack == 0x04) {
            /* FAULT -- clear sticky errors via ABORT before returning */
            SWD_LOG("xfer: FAULT -- clearing via ABORT");
            swdio_set_output_mode();
            swd_idle_cycles(SWD_IDLE_CYCLES);
            /* Inline ABORT write (can't call swd_abort_recovery here to avoid recursion) */
            {
                uint8_t abort_req = swd_request_byte(0, 0, DP_ABORT);
                uint32_t abort_val = ABORT_CLEAR_ALL;
                swdio_set_output_mode();
                for (int i = 0; i < 8; i++)
                    swd_write_bit((abort_req >> i) & 1);
                /* Turnaround */
                swdio_set_input_mode();
                esp_rom_delay_us(SWD_ISO_DELAY_US);
                gpio_set_level(PIN_SWD_CLK, CLK_ACTIVE);
                esp_rom_delay_us(SWD_ISO_DELAY_US);
                gpio_set_level(PIN_SWD_CLK, CLK_IDLE);
                /* Read ACK (discard -- best effort) */
                for (int i = 0; i < 3; i++) swd_read_bit();
                /* Write the ABORT value */
                swdio_set_output_mode();
                swdio_write(0);
                esp_rom_delay_us(SWD_ISO_DELAY_US);
                gpio_set_level(PIN_SWD_CLK, CLK_ACTIVE);
                esp_rom_delay_us(SWD_ISO_DELAY_US);
                gpio_set_level(PIN_SWD_CLK, CLK_IDLE);
                for (int i = 0; i < 32; i++)
                    swd_write_bit((abort_val >> i) & 1);
                swd_write_bit(parity32(abort_val));
                swd_idle_cycles(SWD_IDLE_CYCLES);
            }
            return SWD_ACK_FAULT;
        } else {
            /* Protocol error -- line reset to re-sync bus */
            SWD_LOG("xfer: PROTOCOL ERROR ack=0x%02X -- doing line reset", ack);
            swdio_set_output_mode();
            swd_idle_cycles(SWD_IDLE_CYCLES);
            swd_line_reset();
            return SWD_ERROR;
        }
    }

    SWD_LOG("xfer: WAIT exhausted after %d retries", SWD_WAIT_RETRIES);
    return SWD_ACK_WAIT;    /* Exhausted WAIT retries */
}

/* ================================================================== */
/*  DP Register Operations                                              */
/* ================================================================== */

swd_status_t swd_read_dp(uint8_t addr, uint32_t *value)
{
    uint8_t req = swd_request_byte(0, 1, addr);    /* APnDP=0, RnW=1 */
    return swd_transfer(req, value);
}

swd_status_t swd_write_dp(uint8_t addr, uint32_t value)
{
    uint8_t req = swd_request_byte(0, 0, addr);    /* APnDP=0, RnW=0 */
    return swd_transfer(req, &value);
}

/* ================================================================== */
/*  AP Register Operations                                              */
/* ================================================================== */

swd_status_t swd_write_ap(uint8_t addr, uint32_t value)
{
    uint8_t req = swd_request_byte(1, 0, addr);    /* APnDP=1, RnW=0 */
    return swd_transfer(req, &value);
}

swd_status_t swd_read_ap(uint8_t addr, uint32_t *value)
{
    /* AP reads are posted: first read kicks off the transfer,
     * actual data comes back via a RDBUFF read. */
    uint8_t req = swd_request_byte(1, 1, addr);    /* APnDP=1, RnW=1 */
    swd_status_t st = swd_transfer(req, value);     /* posted read */
    if (st != SWD_OK) return st;

    /* Read RDBUFF to get the actual value */
    return swd_read_dp(DP_RDBUFF, value);
}

/* ================================================================== */
/*  Debug Power-Up Sequence                                             */
/* ================================================================== */

swd_status_t swd_powerup_debug(void)
{
    swd_status_t st;

    /* Step 1: Request debug and system power up */
    st = swd_write_dp(DP_CTRLSTAT, CDBGPWRUPREQ | CSYSPWRUPREQ);
    if (st != SWD_OK) {
        printf("INFO, SWD debug powerup: CTRL/STAT write failed (%d)\n", st);
        return st;
    }

    /* Step 2: Poll for ACK bits (timeout 100ms) */
    int64_t deadline = esp_timer_get_time() + 100000;   /* 100ms */
    uint32_t ctrl_stat = 0;
    while (esp_timer_get_time() < deadline) {
        st = swd_read_dp(DP_CTRLSTAT, &ctrl_stat);
        if (st != SWD_OK) {
            printf("INFO, SWD debug powerup: CTRL/STAT read failed (%d)\n", st);
            return st;
        }
        if ((ctrl_stat & (CDBGPWRUPACK | CSYSPWRUPACK)) ==
            (CDBGPWRUPACK | CSYSPWRUPACK)) {
            break;
        }
        esp_rom_delay_us(100);
    }

    if ((ctrl_stat & (CDBGPWRUPACK | CSYSPWRUPACK)) !=
        (CDBGPWRUPACK | CSYSPWRUPACK)) {
        printf("INFO, SWD debug powerup: timeout waiting for ACK (CTRL/STAT=0x%08lX)\n",
               (unsigned long)ctrl_stat);
        return SWD_TIMEOUT;
    }

    /* Step 3: Select AP bank 0 */
    st = swd_write_dp(DP_SELECT, 0x00000000);
    if (st != SWD_OK) {
        printf("INFO, SWD debug powerup: SELECT write failed (%d)\n", st);
        return st;
    }

    /* Step 4: Read AP IDR (at address 0xFC, bank 0xF) to confirm MEM-AP */
    st = swd_write_dp(DP_SELECT, 0x000000F0);  /* Select AP bank 0xF */
    if (st != SWD_OK) return st;

    uint32_t ap_idr = 0;
    st = swd_read_ap(0x0C, &ap_idr);   /* IDR is at offset 0xFC = bank 0xF, reg 0x0C */
    if (st != SWD_OK) {
        printf("INFO, SWD debug powerup: AP IDR read failed (%d)\n", st);
        return st;
    }

    printf("INFO, SWD debug domain active -- AP IDR=0x%08lX\n",
           (unsigned long)ap_idr);

    /* Restore AP bank 0 for subsequent memory access */
    st = swd_write_dp(DP_SELECT, 0x00000000);
    return st;
}

/* ================================================================== */
/*  Memory Access via MEM-AP                                            */
/* ================================================================== */

swd_status_t swd_mem_read32(uint32_t addr, uint32_t *value)
{
    swd_status_t st;

    /* CSW: 32-bit, no auto-increment, debug enabled */
    st = swd_write_ap(AP_CSW, CSW_SIZE32 | CSW_ADDRINC_OFF | CSW_DBGSTAT);
    if (st != SWD_OK) return st;

    /* TAR: target address */
    st = swd_write_ap(AP_TAR, addr);
    if (st != SWD_OK) return st;

    /* DRW: read (posted) + RDBUFF for actual value */
    st = swd_read_ap(AP_DRW, value);
    return st;
}

swd_status_t swd_mem_write32(uint32_t addr, uint32_t value)
{
    swd_status_t st;

    st = swd_write_ap(AP_CSW, CSW_SIZE32 | CSW_ADDRINC_OFF | CSW_DBGSTAT);
    if (st != SWD_OK) return st;

    st = swd_write_ap(AP_TAR, addr);
    if (st != SWD_OK) return st;

    st = swd_write_ap(AP_DRW, value);
    return st;
}

swd_status_t swd_mem_read_block(uint32_t addr, uint32_t *buf, size_t word_count)
{
    swd_status_t st;

    if (word_count == 0) return SWD_OK;

    /* CSW: 32-bit, single auto-increment, debug enabled */
    st = swd_write_ap(AP_CSW, CSW_SIZE32 | CSW_ADDRINC_SGL | CSW_DBGSTAT);
    if (st != SWD_OK) return st;

    /* TAR: start address */
    st = swd_write_ap(AP_TAR, addr);
    if (st != SWD_OK) return st;

    /* Kick off first posted read */
    uint8_t req = swd_request_byte(1, 1, AP_DRW);
    uint32_t dummy;
    st = swd_transfer(req, &dummy);
    if (st != SWD_OK) return st;

    /* Read N-1 words via repeated DRW reads (each returns previous result) */
    for (size_t i = 0; i < word_count - 1; i++) {
        st = swd_transfer(req, &buf[i]);
        if (st != SWD_OK) return st;
    }

    /* Final word comes from RDBUFF */
    st = swd_read_dp(DP_RDBUFF, &buf[word_count - 1]);
    return st;
}

/* ================================================================== */
/*  Diagnostics                                                         */
/* ================================================================== */

swd_status_t swd_read_idcode_value(uint32_t *idcode)
{
    SWD_LOG("read_idcode: line_reset -> jtag_to_swd -> line_reset -> read DPIDR");
    swd_line_reset();
    swd_jtag_to_swd();
    swd_line_reset();
    swd_status_t st = swd_read_dp(DP_DPIDR, idcode);
    SWD_LOG("read_idcode: result status=%d idcode=0x%08lX",
            (int)st, (unsigned long)(idcode ? *idcode : 0));
    return st;
}

swd_status_t swd_integrity_test(int iterations, int *pass_count, int *fail_count)
{
    int passes = 0;
    int fails  = 0;

    /* Do one target reset before the integrity test loop */
    swd_reset_target();

    for (int i = 0; i < iterations; i++) {
        uint32_t idcode = 0;
        swd_status_t st = swd_read_idcode_value(&idcode);

        if (st == SWD_OK && idcode == SWD_IDCODE_STM32G030) {
            passes++;
        } else {
            fails++;
            /* Print diagnostic for first 3 failures */
            if (fails <= 3) {
                printf("INFO, SWD diag [%d]: status=%d idcode=0x%08lX\n",
                       i, (int)st, (unsigned long)idcode);
            }
        }
    }

    if (pass_count) *pass_count = passes;
    if (fail_count) *fail_count = fails;

    printf("INFO, SWD integrity test: %d/%d passed\n", passes, iterations);
    return (fails == 0) ? SWD_OK : SWD_ERROR;
}

/* ================================================================== */
/*  Public API -- v1 (unchanged interface, uses v2 internals)           */
/* ================================================================== */

void swd_init(void)
{
    SWD_LOG("swd_init: SWCLK=GPIO%d  SWDIO=GPIO%d  nRST=GPIO%d",
            PIN_SWD_CLK, PIN_SWD_IO, PIN_SWD_NRST);
    SWD_LOG("swd_init: CLK_IDLE=%d CLK_ACTIVE=%d NRST_ASSERT=%d NRST_DEASSERT=%d",
            CLK_IDLE, CLK_ACTIVE, NRST_ASSERT, NRST_DEASSERT);

    /* --- SWCLK: always push-pull output, start at idle level --- */
    gpio_reset_pin(PIN_SWD_CLK);
    gpio_set_direction(PIN_SWD_CLK, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_SWD_CLK, CLK_IDLE);

#ifdef SWD_ISOLATED
    gpio_reset_pin(PIN_SWD_IO_OUT);
    gpio_set_direction(PIN_SWD_IO_OUT, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_SWD_IO_OUT, 0);

    gpio_reset_pin(PIN_SWD_IO_IN);
    gpio_set_direction(PIN_SWD_IO_IN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_SWD_IO_IN, GPIO_PULLUP_ONLY);

    printf("INFO, SWD opto-isolated mode: OUT=GPIO%d, IN=GPIO%d\n",
           PIN_SWD_IO_OUT, PIN_SWD_IO_IN);
#else
    gpio_reset_pin(PIN_SWD_IO);
    gpio_set_direction(PIN_SWD_IO, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_pull_mode(PIN_SWD_IO, GPIO_PULLUP_ONLY);
    gpio_set_level(PIN_SWD_IO, 0);
    SWD_LOG("swd_init: SWDIO configured INPUT_OUTPUT with pullup");
#endif

    gpio_reset_pin(PIN_SWD_NRST);
    gpio_set_direction(PIN_SWD_NRST, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_SWD_NRST, NRST_DEASSERT);

    /* Read back GPIO levels to verify wiring */
    SWD_LOG("swd_init: post-config levels: SWCLK=%d SWDIO=%d nRST_out=%d",
            gpio_get_level(PIN_SWD_CLK),
            gpio_get_level(PIN_SWD_IO),
            gpio_get_level(PIN_SWD_NRST));
    SWD_LOG("swd_init: done");
}

bool swd_verify_target(void)
{
    for (int attempt = 1; attempt <= SWD_MAX_RETRIES; attempt++) {

        swd_reset_target();
        swd_line_reset();
        swd_jtag_to_swd();
        swd_line_reset();

        uint32_t idcode = 0;
        swd_status_t st = swd_read_dp(DP_DPIDR, &idcode);

        if (st == SWD_OK && idcode == SWD_IDCODE_STM32G030) {
            printf("INFO, SWD IDCODE OK: 0x%08lX (attempt %d/%d)\n",
                   (unsigned long)idcode, attempt, SWD_MAX_RETRIES);
            return true;
        }

        if (attempt < SWD_MAX_RETRIES) {
            printf("INFO, SWD attempt %d/%d failed (ID=0x%08lX status=%d), retrying...\n",
                   attempt, SWD_MAX_RETRIES,
                   (unsigned long)idcode, st);
            vTaskDelay(pdMS_TO_TICKS(SWD_RETRY_MS));
        }
    }

    printf("INFO, SWD IDCODE MISMATCH after %d attempts -- expected 0x%08lX\n",
           SWD_MAX_RETRIES, (unsigned long)SWD_IDCODE_STM32G030);
    return false;
}

void swd_reset_target(void)
{
    SWD_LOG("reset_target: asserting nRST (GPIO%d = %d)", PIN_SWD_NRST, NRST_ASSERT);
    gpio_set_level(PIN_SWD_NRST, NRST_ASSERT);
    vTaskDelay(pdMS_TO_TICKS(20));
    SWD_LOG("reset_target: deasserting nRST (GPIO%d = %d)", PIN_SWD_NRST, NRST_DEASSERT);
    gpio_set_level(PIN_SWD_NRST, NRST_DEASSERT);
    vTaskDelay(pdMS_TO_TICKS(10));
    SWD_LOG("reset_target: done, SWDIO level=%d", gpio_get_level(PIN_SWD_IO));
}

/* ================================================================== */
/*  v3 API: Production hardening                                        */
/* ================================================================== */

void swd_safe_state(void)
{
    SWD_LOG("safe_state: restoring all SWD GPIOs to idle");
    gpio_set_level(PIN_SWD_CLK, CLK_IDLE);
#ifdef SWD_ISOLATED
    gpio_set_level(PIN_SWD_IO_OUT, 0);
#else
    gpio_set_direction(PIN_SWD_IO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_SWD_IO, GPIO_PULLUP_ONLY);
#endif
    gpio_set_level(PIN_SWD_NRST, NRST_DEASSERT);
    SWD_LOG("safe_state: done");
}

swd_status_t swd_abort_recovery(void)
{
    SWD_LOG("abort_recovery: clearing sticky errors + line reset");
    swd_status_t st = swd_write_dp(DP_ABORT, ABORT_CLEAR_ALL);
    swd_line_reset();
    SWD_LOG("abort_recovery: done (ABORT write status=%d)", (int)st);
    return st;
}

swd_verify_result_t swd_verify_target_detailed(void)
{
    swd_verify_result_t result = {
        .status   = SWD_ERROR,
        .idcode   = 0,
        .attempts = 0
    };

    for (int attempt = 1; attempt <= SWD_MAX_RETRIES; attempt++) {
        result.attempts = attempt;

        swd_reset_target();
        swd_line_reset();
        swd_jtag_to_swd();
        swd_line_reset();

        uint32_t idcode = 0;
        swd_status_t st = swd_read_dp(DP_DPIDR, &idcode);
        result.idcode = idcode;
        result.status = st;

        if (st == SWD_OK && idcode == SWD_IDCODE_STM32G030) {
            printf("INFO, SWD IDCODE OK: 0x%08lX (attempt %d/%d)\n",
                   (unsigned long)idcode, attempt, SWD_MAX_RETRIES);
            return result;
        }

        if (attempt < SWD_MAX_RETRIES) {
            printf("INFO, SWD attempt %d/%d failed (ID=0x%08lX status=%d), retrying...\n",
                   attempt, SWD_MAX_RETRIES,
                   (unsigned long)idcode, st);
            /* Try bus recovery before next attempt */
            if (st == SWD_ACK_FAULT) {
                swd_abort_recovery();
            }
            vTaskDelay(pdMS_TO_TICKS(SWD_RETRY_MS));
        }
    }

    /* Classify the final failure reason */
    if (result.status == SWD_OK && result.idcode != SWD_IDCODE_STM32G030) {
        /* Got a valid SWD response but wrong chip ID */
        printf("INFO, SWD WRONG IDCODE: got 0x%08lX, expected 0x%08lX\n",
               (unsigned long)result.idcode, (unsigned long)SWD_IDCODE_STM32G030);
    } else {
        printf("INFO, SWD FAILED after %d attempts (status=%d idcode=0x%08lX)\n",
               SWD_MAX_RETRIES, (int)result.status, (unsigned long)result.idcode);
    }

    return result;
}

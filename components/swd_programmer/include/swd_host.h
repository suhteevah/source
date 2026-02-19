#ifndef SWD_HOST_H
#define SWD_HOST_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/* ===================================================================
 * LatchPac Validator 3000 -- SWD Host API (v2)
 *
 * v1 API: swd_init, swd_verify_target, swd_reset_target
 * v2 API: Full DP/AP register access, debug powerup, memory access,
 *         signal integrity test, IDCODE value readout
 * =================================================================== */

/* ------------------------------------------------------------------ */
/*  Return codes                                                        */
/* ------------------------------------------------------------------ */
typedef enum {
    SWD_OK = 0,
    SWD_ACK_WAIT,
    SWD_ACK_FAULT,
    SWD_PARITY_ERROR,
    SWD_TIMEOUT,
    SWD_ERROR
} swd_status_t;

/* ------------------------------------------------------------------ */
/*  v1 API (unchanged)                                                  */
/* ------------------------------------------------------------------ */

/**
 * @brief Initialise the SWD GPIO lines (CLK, IO, NRST).
 *        Sets safe default levels (CLK low, NRST high / de-asserted).
 */
void swd_init(void);

/**
 * @brief Read the IDCODE register of the STM32 target via SWD.
 *
 * Performs:  Line Reset -> JTAG-to-SWD switch -> Read DP IDCODE.
 *
 * @return true  if IDCODE matches SWD_IDCODE_STM32G030
 * @return false on mismatch (prints the read value over serial)
 */
bool swd_verify_target(void);

/**
 * @brief Hard-reset the target by pulsing NRST low for ~20 ms.
 */
void swd_reset_target(void);

/* ------------------------------------------------------------------ */
/*  v2 API: Register-level access                                       */
/* ------------------------------------------------------------------ */

/**
 * @brief Read a Debug Port register.
 * @param addr  Register address (0x00=DPIDR, 0x04=CTRL/STAT, 0x08=SELECT, 0x0C=RDBUFF)
 * @param value Pointer to receive the 32-bit register value.
 * @return SWD_OK on success, error code otherwise.
 */
swd_status_t swd_read_dp(uint8_t addr, uint32_t *value);

/**
 * @brief Write a Debug Port register.
 * @param addr  Register address (0x00=ABORT, 0x04=CTRL/STAT, 0x08=SELECT)
 * @param value 32-bit value to write.
 * @return SWD_OK on success, error code otherwise.
 */
swd_status_t swd_write_dp(uint8_t addr, uint32_t value);

/**
 * @brief Read an Access Port register (posted read -- actual value
 *        is obtained from the subsequent RDBUFF read internally).
 * @param addr  Register address (0x00=CSW, 0x04=TAR, 0x0C=DRW)
 * @param value Pointer to receive the 32-bit register value.
 * @return SWD_OK on success, error code otherwise.
 */
swd_status_t swd_read_ap(uint8_t addr, uint32_t *value);

/**
 * @brief Write an Access Port register.
 * @param addr  Register address (0x00=CSW, 0x04=TAR, 0x0C=DRW)
 * @param value 32-bit value to write.
 * @return SWD_OK on success, error code otherwise.
 */
swd_status_t swd_write_ap(uint8_t addr, uint32_t value);

/* ------------------------------------------------------------------ */
/*  v2 API: Debug domain                                                */
/* ------------------------------------------------------------------ */

/**
 * @brief Power up the debug domain.
 *
 * Sequence:
 *   1. Write CTRL/STAT with CDBGPWRUPREQ | CSYSPWRUPREQ
 *   2. Poll CTRL/STAT for CDBGPWRUPACK | CSYSPWRUPACK (timeout 100ms)
 *   3. Write SELECT to choose AP bank 0
 *   4. Read AP IDR to confirm MEM-AP
 *
 * @return SWD_OK on success, SWD_TIMEOUT if ACK bits never set.
 */
swd_status_t swd_powerup_debug(void);

/* ------------------------------------------------------------------ */
/*  v2 API: Memory access via MEM-AP                                    */
/* ------------------------------------------------------------------ */

/**
 * @brief Read a single 32-bit word from target memory.
 * @param addr  Target memory address (must be word-aligned).
 * @param value Pointer to receive the 32-bit value.
 * @return SWD_OK on success, error code otherwise.
 */
swd_status_t swd_mem_read32(uint32_t addr, uint32_t *value);

/**
 * @brief Write a single 32-bit word to target memory.
 * @param addr  Target memory address (must be word-aligned).
 * @param value 32-bit value to write.
 * @return SWD_OK on success, error code otherwise.
 */
swd_status_t swd_mem_write32(uint32_t addr, uint32_t value);

/**
 * @brief Read a block of 32-bit words from target memory.
 *        Uses auto-increment in CSW for efficient bulk reads.
 * @param addr       Target memory start address (must be word-aligned).
 * @param buf        Buffer to receive word_count 32-bit values.
 * @param word_count Number of 32-bit words to read.
 * @return SWD_OK on success, error code otherwise.
 */
swd_status_t swd_mem_read_block(uint32_t addr, uint32_t *buf, size_t word_count);

/* ------------------------------------------------------------------ */
/*  v2 API: Diagnostics                                                 */
/* ------------------------------------------------------------------ */

/**
 * @brief Read the raw IDCODE value without comparing against expected.
 * @param idcode Pointer to receive the 32-bit IDCODE.
 * @return SWD_OK on success, error code otherwise.
 */
swd_status_t swd_read_idcode_value(uint32_t *idcode);

/**
 * @brief SWD signal integrity self-test.
 *
 * Reads IDCODE @p iterations times and counts successes / failures.
 * Useful as a pre-flight check for fixture health diagnostics.
 *
 * @param iterations  Number of IDCODE reads to perform.
 * @param pass_count  Pointer to receive the number of successful reads.
 * @param fail_count  Pointer to receive the number of failed reads.
 * @return SWD_OK if all iterations passed, SWD_ERROR if any failed.
 */
swd_status_t swd_integrity_test(int iterations, int *pass_count, int *fail_count);

#endif /* SWD_HOST_H */

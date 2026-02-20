#ifndef MOCK_HARDWARE_H
#define MOCK_HARDWARE_H

#include "fixture_pins.h"

#ifdef MOCK_HARDWARE_MODE

#include <stdbool.h>
#include <stdint.h>

/* ===================================================================
 * LatchPac Validator 3000 -- Mock Hardware (v2/v3)
 *
 * v1: mock_update_simulation, mock_read_voltage, mock_swd_verify
 * v2: mock_swd_read_dp, mock_swd_write_dp, mock_swd_read_ap,
 *     mock_swd_write_ap, mock_swd_powerup_debug, mock_swd_mem_read32,
 *     mock_swd_integrity_test
 * v3: No new mock stubs needed -- swd_safe_state(), swd_abort_recovery(),
 *     and swd_verify_target_detailed() are provided by swd_host.c (always
 *     compiled) and guarded by #ifndef MOCK_HARDWARE_MODE at call sites.
 *
 * NOTE: swd_status_t is forward-declared here to avoid a circular
 * dependency between fixture_config and swd_programmer.  The full
 * enum definition lives in swd_host.h.
 * =================================================================== */

/* Forward-declare swd_status_t to avoid circular include dependency */
typedef enum {
    MOCK_SWD_OK = 0,
    MOCK_SWD_ACK_WAIT,
    MOCK_SWD_ACK_FAULT,
    MOCK_SWD_PARITY_ERROR,
    MOCK_SWD_TIMEOUT,
    MOCK_SWD_ERROR
} mock_swd_status_t;

/* ------------------------------------------------------------------ */
/*  v1 API                                                              */
/* ------------------------------------------------------------------ */

void  mock_update_simulation(void);
float mock_read_voltage(void);
bool  mock_swd_verify(void);

/* ------------------------------------------------------------------ */
/*  v2 API: Mock SWD register and memory stubs                         */
/*  Uses mock_swd_status_t (compatible with swd_status_t values)       */
/* ------------------------------------------------------------------ */

mock_swd_status_t mock_swd_read_dp(uint8_t addr, uint32_t *value);
mock_swd_status_t mock_swd_write_dp(uint8_t addr, uint32_t value);
mock_swd_status_t mock_swd_read_ap(uint8_t addr, uint32_t *value);
mock_swd_status_t mock_swd_write_ap(uint8_t addr, uint32_t value);
mock_swd_status_t mock_swd_powerup_debug(void);
mock_swd_status_t mock_swd_mem_read32(uint32_t addr, uint32_t *value);
mock_swd_status_t mock_swd_integrity_test(int iterations, int *pass_count, int *fail_count);

#endif /* MOCK_HARDWARE_MODE */

#endif /* MOCK_HARDWARE_H */

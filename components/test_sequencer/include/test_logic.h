#ifndef TEST_LOGIC_H
#define TEST_LOGIC_H

#include <stdint.h>
#include "swd_host.h"

/* ===================================================================
 * LatchPac Validator 3000 -- Test Sequencer (v3)
 *
 * v1 API: test_result_t, run_production_test, test_result_to_string
 * v2 API: test_report_t, run_production_test_v2
 * v3 API: Granular SWD failure codes, timeout, incomplete tracking
 * =================================================================== */

/* ------------------------------------------------------------------ */
/*  Result codes                                                        */
/* ------------------------------------------------------------------ */
typedef enum {
    TEST_PASS              = 0,
    FAIL_SAFETY_OPEN       = 1,   /* Lid interlock was open                  */
    FAIL_STUCK_ON          = 2,   /* Load was already energised pre-test     */
    FAIL_NO_LATCH          = 3,   /* Load did not turn on after latch cmd    */
    FAIL_STUCK_LATCHED     = 4,   /* Load did not turn off after unlatch     */
    FAIL_SWD_ERROR         = 5,   /* SWD IDCODE verify failed (generic)      */
    /* v3 granular codes */
    FAIL_TIMEOUT           = 6,   /* Overall test exceeded time limit        */
    FAIL_INCOMPLETE        = 7,   /* Previous test interrupted (power loss)  */
    FAIL_SWD_NO_TARGET     = 8,   /* SWD got no response (ALL_ONES / ERROR)  */
    FAIL_SWD_WRONG_ID      = 9,   /* SWD responded but IDCODE doesn't match */
    FAIL_SWD_BUS_ERROR     = 10   /* SWD bus fault, parity, or timeout       */
} test_result_t;

/**
 * @brief Convert a test_result_t to a human-readable status string.
 */
const char *test_result_to_string(test_result_t result);

/**
 * @brief Execute the full manufacturing test cycle (v1 interface).
 * @return test_result_t  TEST_PASS on success, specific FAIL_* on error.
 */
test_result_t run_production_test(void);

/* ------------------------------------------------------------------ */
/*  v2/v3 API: Enhanced test report with diagnostic data                */
/* ------------------------------------------------------------------ */

/** Overall test timeout in milliseconds */
#define TEST_TIMEOUT_MS  5000

/**
 * @brief Full test report with timing and SWD diagnostic info.
 */
typedef struct {
    test_result_t result;           /* PASS/FAIL code                    */
    uint32_t      swd_idcode;       /* Actual IDCODE value read          */
    int           swd_attempts;     /* How many SWD retries needed       */
    swd_status_t  swd_status;       /* Raw SWD status from last attempt  */
    uint32_t      duration_ms;      /* Total test execution time (ms)    */
} test_report_t;

/**
 * @brief Execute the full manufacturing test cycle with enhanced reporting.
 *
 * Same test sequence as v1 plus:
 *   - Wall-clock timeout (TEST_TIMEOUT_MS)
 *   - Granular SWD failure classification
 *   - After IDCODE check, attempts swd_powerup_debug() + memory read
 *     (informational only -- failure does NOT cause test failure)
 *   - Returns actual IDCODE, SWD status, retry count
 *
 * @return test_report_t  Full report struct.
 */
test_report_t run_production_test_v2(void);

#endif /* TEST_LOGIC_H */

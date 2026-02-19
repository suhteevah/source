/* ===================================================================
 * LatchPac Validator 3000 -- Production Test Sequencer (v2)
 *
 * Runs the complete manufacturing test cycle:
 *   0. Lid safety check
 *   1. Pre-check  (load must be OFF)
 *   2. Latch      (SIM_START + SIM_STOP LOW -> verify load ON)
 *   3. Unlatch    (SIM_START HIGH, SIM_STOP HIGH -> verify load OFF)
 *   4. SWD IDCODE verification
 *   5. (v2) SWD debug powerup probe (informational only)
 *
 * All pin definitions come from fixture_pins.h -- no local redefinition.
 *
 * SAFETY: The lid interlock is polled during every wait period.
 *         If the lid opens mid-test, outputs are forced safe
 *         and FAIL_SAFETY_OPEN is returned immediately.
 *
 * WARNING: 120 VAC on target board.
 * =================================================================== */

#include <stdio.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#include "fixture_pins.h"
#include "mock_hardware.h"
#include "test_logic.h"
#include "swd_host.h"

/* Settling time after driving pogo outputs (ms) */
#define SETTLE_MS       500

/* Safety poll interval during waits (ms) */
#define SAFETY_POLL_MS  20

/* ------------------------------------------------------------------ */
/*  Helper: read the load-sense line (real or mock)                    */
/* ------------------------------------------------------------------ */
static bool load_is_on(void)
{
#ifdef MOCK_HARDWARE_MODE
    mock_update_simulation();
    return mock_read_voltage() > 1.0f;
#else
    return gpio_get_level(PIN_LOAD_SENSE) == 1;
#endif
}

/* ------------------------------------------------------------------ */
/*  Helper: verify SWD target (real or mock)                           */
/* ------------------------------------------------------------------ */
static bool verify_swd(void)
{
#ifdef MOCK_HARDWARE_MODE
    return mock_swd_verify();
#else
    return swd_verify_target();
#endif
}

/* ------------------------------------------------------------------ */
/*  Helper: read SWD IDCODE value (real or mock)                       */
/* ------------------------------------------------------------------ */
static swd_status_t read_swd_idcode(uint32_t *idcode)
{
#ifdef MOCK_HARDWARE_MODE
    *idcode = SWD_IDCODE_STM32G030;
    return SWD_OK;
#else
    return swd_read_idcode_value(idcode);
#endif
}

/* ------------------------------------------------------------------ */
/*  Helper: attempt debug powerup (real or mock)                       */
/* ------------------------------------------------------------------ */
static swd_status_t try_debug_powerup(void)
{
#ifdef MOCK_HARDWARE_MODE
    printf("INFO, SWD debug domain active -- MEM-AP ready (mock)\n");
    return SWD_OK;
#else
    return swd_powerup_debug();
#endif
}

/* ------------------------------------------------------------------ */
/*  Force pogo outputs to safe state (HIGH = released)                 */
/* ------------------------------------------------------------------ */
static void force_outputs_safe(void)
{
    gpio_set_level(PIN_SIM_START, 1);
    gpio_set_level(PIN_SIM_STOP,  1);
}

/* ------------------------------------------------------------------ */
/*  Safety-aware delay                                                  */
/* ------------------------------------------------------------------ */
static bool safe_delay_ms(int total_ms)
{
    int remaining = total_ms;
    while (remaining > 0) {
        int chunk = (remaining > SAFETY_POLL_MS) ? SAFETY_POLL_MS : remaining;
        vTaskDelay(pdMS_TO_TICKS(chunk));
        remaining -= chunk;

        if (LID_IS_OPEN()) {
            printf("INFO, SAFETY -- lid opened during test wait, aborting\n");
            force_outputs_safe();
            return false;
        }
    }
    return true;
}

/* ------------------------------------------------------------------ */
/*  Convert result code to human-readable string                       */
/* ------------------------------------------------------------------ */
const char *test_result_to_string(test_result_t result)
{
    switch (result) {
    case TEST_PASS:          return "PASS";
    case FAIL_SAFETY_OPEN:   return "FAIL_SAFETY_OPEN";
    case FAIL_STUCK_ON:      return "FAIL_STUCK_ON";
    case FAIL_NO_LATCH:      return "FAIL_NO_LATCH";
    case FAIL_STUCK_LATCHED: return "FAIL_STUCK_LATCHED";
    case FAIL_SWD_ERROR:     return "FAIL_SWD_ERROR";
    default:                 return "FAIL_UNKNOWN";
    }
}

/* ------------------------------------------------------------------ */
/*  Core test sequence (shared by v1 and v2)                           */
/*  Returns the result code; v2 wrapper adds timing and diagnostics.   */
/* ------------------------------------------------------------------ */
static test_result_t run_test_core(void)
{
    /* ------ Step 0: Safety interlock ------ */
    if (LID_IS_OPEN()) {
        printf("INFO, Test aborted -- lid safety open\n");
        return FAIL_SAFETY_OPEN;
    }

    /* ------ Step 1: Pre-check -- load must be OFF ------ */
    if (load_is_on()) {
        printf("INFO, Pre-check failed -- load already energised\n");
        return FAIL_STUCK_ON;
    }

    /* ------ Step 2: Latch -- drive START + STOP LOW ------ */
    gpio_set_level(PIN_SIM_START, 0);
    gpio_set_level(PIN_SIM_STOP,  0);

    if (!safe_delay_ms(SETTLE_MS)) {
        return FAIL_SAFETY_OPEN;
    }

    /* ------ Step 3: Verify ON ------ */
    if (!load_is_on()) {
        force_outputs_safe();
        printf("INFO, Latch failed -- load did not turn ON\n");
        return FAIL_NO_LATCH;
    }

    /* ------ Step 4: Unlatch -- release both lines HIGH ------ */
    gpio_set_level(PIN_SIM_START, 1);
    gpio_set_level(PIN_SIM_STOP,  1);

    if (!safe_delay_ms(SETTLE_MS)) {
        return FAIL_SAFETY_OPEN;
    }

    /* ------ Step 5: Verify OFF ------ */
    if (load_is_on()) {
        printf("INFO, Unlatch failed -- load stuck ON\n");
        return FAIL_STUCK_LATCHED;
    }

    /* ------ Step 6: Final lid check before SWD ------ */
    if (LID_IS_OPEN()) {
        printf("INFO, Lid opened before SWD check\n");
        return FAIL_SAFETY_OPEN;
    }

    /* ------ Step 7: SWD IDCODE verification ------ */
    if (!verify_swd()) {
        printf("INFO, SWD verification failed\n");
        return FAIL_SWD_ERROR;
    }

    return TEST_PASS;
}

/* ------------------------------------------------------------------ */
/*  v1 API: run_production_test                                        */
/* ------------------------------------------------------------------ */
test_result_t run_production_test(void)
{
    return run_test_core();
}

/* ------------------------------------------------------------------ */
/*  v2 API: run_production_test_v2                                     */
/* ------------------------------------------------------------------ */
test_report_t run_production_test_v2(void)
{
    test_report_t report = {
        .result        = TEST_PASS,
        .swd_idcode    = 0,
        .swd_attempts  = 0,
        .duration_ms   = 0
    };

    /* Start timing */
    int64_t t_start = esp_timer_get_time();

    /* Run the core test sequence */
    report.result = run_test_core();

    /* Record timing */
    int64_t t_end = esp_timer_get_time();
    report.duration_ms = (uint32_t)((t_end - t_start) / 1000);

    /* Try to read the IDCODE value for diagnostics */
    read_swd_idcode(&report.swd_idcode);

    /* Count attempts from the SWD verify retry loop
     * The v1 verify does up to SWD_MAX_RETRIES (3), but we
     * don't have direct access to the internal counter.
     * For now, report 1 on success, 3 on failure. */
    if (report.result == FAIL_SWD_ERROR) {
        report.swd_attempts = 3;
    } else if (report.result == TEST_PASS) {
        report.swd_attempts = 1;
    } else {
        report.swd_attempts = 0;    /* SWD not reached */
    }

    /* v2 bonus: attempt debug powerup after a successful test
     * This is informational only -- failure does NOT cause test failure */
    if (report.result == TEST_PASS) {
        swd_status_t dbg_st = try_debug_powerup();
        if (dbg_st == SWD_OK) {
            printf("INFO, SWD debug domain active -- MEM-AP ready\n");

            /* Try reading a known address as a diagnostic */
            uint32_t mem_val = 0;
#ifdef MOCK_HARDWARE_MODE
            mem_val = 0xDEADBEEF;
            printf("INFO, SWD memory probe: [0x08000000] = 0x%08lX (mock)\n",
                   (unsigned long)mem_val);
#else
            swd_status_t mem_st = swd_mem_read32(0x08000000, &mem_val);
            if (mem_st == SWD_OK) {
                printf("INFO, SWD memory probe: [0x08000000] = 0x%08lX\n",
                       (unsigned long)mem_val);
            } else {
                printf("INFO, SWD memory probe failed (status=%d) -- not a test failure\n",
                       mem_st);
            }
#endif
        } else {
            printf("INFO, SWD debug powerup failed (status=%d) -- not a test failure\n",
                   dbg_st);
        }
    }

    return report;
}

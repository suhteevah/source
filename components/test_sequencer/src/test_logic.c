/* ===================================================================
 * LatchPac Validator 3000 -- Production Test Sequencer (v3)
 *
 * Runs the complete manufacturing test cycle:
 *   0. Lid safety check
 *   1. Pre-check  (load must be OFF)
 *   2. Latch      (SIM_START + SIM_STOP LOW -> verify load ON)
 *   3. Unlatch    (SIM_START HIGH, SIM_STOP HIGH -> verify load OFF)
 *   4. SWD IDCODE verification (granular failure codes)
 *   5. (v2+) SWD debug powerup probe (informational only)
 *
 * v3 production hardening:
 *   - Wall-clock timeout (TEST_TIMEOUT_MS) on entire test sequence
 *   - Task WDT feed at each major step boundary
 *   - Granular SWD failure classification (no-target, wrong-ID, bus-error)
 *   - swd_safe_state() called on every exit path
 *   - goto-based cleanup ensures outputs are ALWAYS forced safe
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
#include "esp_task_wdt.h"

#include "fixture_pins.h"
#include "mock_hardware.h"
#include "test_logic.h"
#include "swd_host.h"

/* Settling time after driving pogo outputs (ms) */
#define SETTLE_MS       500

/* Safety poll interval during waits (ms) */
#define SAFETY_POLL_MS  20

/* ------------------------------------------------------------------ */
/*  Helper: feed the task watchdog                                      */
/* ------------------------------------------------------------------ */
static void wdt_feed(void)
{
    esp_task_wdt_reset();
}

/* ------------------------------------------------------------------ */
/*  Helper: check if the overall test deadline has expired              */
/* ------------------------------------------------------------------ */
static bool deadline_expired(int64_t deadline_us)
{
    return esp_timer_get_time() >= deadline_us;
}

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
/*  Helper: detailed SWD verify (real or mock)                         */
/* ------------------------------------------------------------------ */
static swd_verify_result_t verify_swd_detailed(void)
{
#ifdef MOCK_HARDWARE_MODE
    /* Mock always succeeds with correct IDCODE */
    swd_verify_result_t r = {
        .status   = SWD_OK,
        .idcode   = SWD_IDCODE_STM32G030,
        .attempts = 1
    };
    return r;
#else
    return swd_verify_target_detailed();
#endif
}

/* ------------------------------------------------------------------ */
/*  Helper: legacy boolean SWD verify (for v1 API compatibility)       */
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
/*  Classify SWD verify result into granular test failure code          */
/* ------------------------------------------------------------------ */
static test_result_t classify_swd_failure(const swd_verify_result_t *r)
{
    /* Check for wrong IDCODE first (SWD communication worked but ID mismatch) */
    if (r->status == SWD_OK && r->idcode != SWD_IDCODE_STM32G030) {
        return FAIL_SWD_WRONG_ID;
    }

    /* Classify by SWD status */
    switch (r->status) {
    case SWD_OK:
        /* If status is OK but we're here, something unexpected */
        return FAIL_SWD_ERROR;

    case SWD_ERROR:
        /* ALL_ONES / no response -- target not connected or not powered */
        return FAIL_SWD_NO_TARGET;

    case SWD_ACK_FAULT:
    case SWD_PARITY_ERROR:
        /* Bus-level fault or parity error */
        return FAIL_SWD_BUS_ERROR;

    case SWD_ACK_WAIT:
    case SWD_TIMEOUT:
        /* Target stuck or wall-clock timeout */
        return FAIL_SWD_BUS_ERROR;

    default:
        return FAIL_SWD_ERROR;
    }
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
    case FAIL_TIMEOUT:       return "FAIL_TIMEOUT";
    case FAIL_INCOMPLETE:    return "FAIL_INCOMPLETE";
    case FAIL_SWD_NO_TARGET: return "FAIL_SWD_NO_TARGET";
    case FAIL_SWD_WRONG_ID:  return "FAIL_SWD_WRONG_ID";
    case FAIL_SWD_BUS_ERROR: return "FAIL_SWD_BUS_ERROR";
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
/*  v2/v3 API: run_production_test_v2                                  */
/*                                                                      */
/*  v3 enhancements over v2:                                            */
/*    - Wall-clock timeout (TEST_TIMEOUT_MS) at each step              */
/*    - Task WDT feed at every step boundary                           */
/*    - Granular SWD failure codes via swd_verify_target_detailed()    */
/*    - swd_safe_state() called on every exit (goto done pattern)      */
/*    - Real SWD attempt count and status in report                    */
/* ------------------------------------------------------------------ */
test_report_t run_production_test_v2(void)
{
    test_report_t report = {
        .result        = TEST_PASS,
        .swd_idcode    = 0,
        .swd_attempts  = 0,
        .swd_status    = SWD_ERROR,
        .duration_ms   = 0
    };

    /* Start timing */
    int64_t t_start = esp_timer_get_time();
    int64_t deadline_us = t_start + ((int64_t)TEST_TIMEOUT_MS * 1000);

    /* ====== Step 0: Safety interlock ====== */
    wdt_feed();
    if (LID_IS_OPEN()) {
        printf("INFO, Test aborted -- lid safety open\n");
        report.result = FAIL_SAFETY_OPEN;
        goto done;
    }

    /* ====== Step 1: Pre-check -- load must be OFF ====== */
    wdt_feed();
    if (deadline_expired(deadline_us)) {
        printf("INFO, TIMEOUT during pre-check\n");
        report.result = FAIL_TIMEOUT;
        goto done;
    }
    if (load_is_on()) {
        printf("INFO, Pre-check failed -- load already energised\n");
        report.result = FAIL_STUCK_ON;
        goto done;
    }

    /* ====== Step 2: Latch -- drive START + STOP LOW ====== */
    wdt_feed();
    if (deadline_expired(deadline_us)) {
        printf("INFO, TIMEOUT before latch\n");
        report.result = FAIL_TIMEOUT;
        goto done;
    }
    gpio_set_level(PIN_SIM_START, 0);
    gpio_set_level(PIN_SIM_STOP,  0);

    if (!safe_delay_ms(SETTLE_MS)) {
        report.result = FAIL_SAFETY_OPEN;
        goto done;
    }

    /* ====== Step 3: Verify ON ====== */
    wdt_feed();
    if (deadline_expired(deadline_us)) {
        printf("INFO, TIMEOUT during latch verify\n");
        report.result = FAIL_TIMEOUT;
        goto done;
    }
    if (!load_is_on()) {
        printf("INFO, Latch failed -- load did not turn ON\n");
        report.result = FAIL_NO_LATCH;
        goto done;
    }

    /* ====== Step 4: Unlatch -- release both lines HIGH ====== */
    wdt_feed();
    if (deadline_expired(deadline_us)) {
        printf("INFO, TIMEOUT before unlatch\n");
        report.result = FAIL_TIMEOUT;
        goto done;
    }
    gpio_set_level(PIN_SIM_START, 1);
    gpio_set_level(PIN_SIM_STOP,  1);

    if (!safe_delay_ms(SETTLE_MS)) {
        report.result = FAIL_SAFETY_OPEN;
        goto done;
    }

    /* ====== Step 5: Verify OFF ====== */
    wdt_feed();
    if (deadline_expired(deadline_us)) {
        printf("INFO, TIMEOUT during unlatch verify\n");
        report.result = FAIL_TIMEOUT;
        goto done;
    }
    if (load_is_on()) {
        printf("INFO, Unlatch failed -- load stuck ON\n");
        report.result = FAIL_STUCK_LATCHED;
        goto done;
    }

    /* ====== Step 6: Final lid check before SWD ====== */
    wdt_feed();
    if (LID_IS_OPEN()) {
        printf("INFO, Lid opened before SWD check\n");
        report.result = FAIL_SAFETY_OPEN;
        goto done;
    }

    /* ====== Step 7: SWD IDCODE verification (granular) ====== */
    wdt_feed();
    if (deadline_expired(deadline_us)) {
        printf("INFO, TIMEOUT before SWD verify\n");
        report.result = FAIL_TIMEOUT;
        goto done;
    }
    {
        swd_verify_result_t swd_r = verify_swd_detailed();
        report.swd_idcode   = swd_r.idcode;
        report.swd_attempts = swd_r.attempts;
        report.swd_status   = swd_r.status;

        if (swd_r.status == SWD_OK && swd_r.idcode == SWD_IDCODE_STM32G030) {
            /* SWD passed -- continue to optional debug probe */
        } else {
            report.result = classify_swd_failure(&swd_r);
            printf("INFO, SWD verify failed: %s (status=%d, idcode=0x%08lX, attempts=%d)\n",
                   test_result_to_string(report.result),
                   (int)swd_r.status,
                   (unsigned long)swd_r.idcode,
                   swd_r.attempts);
            goto done;
        }
    }

    /* ====== Step 8: SWD debug powerup probe (informational only) ====== */
    wdt_feed();
    {
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

    /* If we reach here, everything passed */
    report.result = TEST_PASS;

done:
    /* ====== Cleanup: ALWAYS force safe state ====== */
    force_outputs_safe();
#ifndef MOCK_HARDWARE_MODE
    swd_safe_state();
#endif

    /* Record timing */
    int64_t t_end = esp_timer_get_time();
    report.duration_ms = (uint32_t)((t_end - t_start) / 1000);

    wdt_feed();
    return report;
}

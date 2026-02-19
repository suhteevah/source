/* ===================================================================
 * LatchPac Validator 3000 -- Main Application (v2)
 *
 * State Machine:
 *   IDLE    -> (operator presses START & lid closed) -> TESTING
 *   TESTING -> (test completes)                      -> RESULT
 *   RESULT  -> (operator opens lid to remove DUT)    -> IDLE
 *
 * v2 additions:
 *   - NVS-persistent unit counter (survives power cycles)
 *   - Session counter (incremented on each boot)
 *   - Boot-time SWD integrity self-test
 *   - Enhanced CSV logging with IDCODE, attempts, duration, FW version
 *   - run_production_test_v2() with full diagnostic report
 *
 * WARNING: Target board carries 120 VAC.
 *          USB Galvanic Isolator MANDATORY in production mode.
 * =================================================================== */

#include <stdio.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_app_desc.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_timer.h"

#include "fixture_pins.h"
#include "mock_hardware.h"
#include "test_logic.h"
#include "csv_logger.h"
#include "swd_host.h"

/* ---------- Constants ---------- */
#define DEBOUNCE_MS         50
#define POLL_MS             20
#define FAIL_BLINK_COUNT    5
#define FAIL_BLINK_MS       300
#define BOOT_BLINK_COUNT    3
#define BOOT_BLINK_MS       200
#define INTEGRITY_ITERATIONS 10

/* ---------- NVS Keys ---------- */
#define NVS_NAMESPACE       "fixture"
#define NVS_KEY_UNIT_ID     "unit_id"
#define NVS_KEY_SESSION     "session_count"

/* ---------- State Machine ---------- */
typedef enum {
    STATE_IDLE,
    STATE_TESTING,
    STATE_RESULT
} fixture_state_t;

static int              unit_counter   = 0;
static int              session_count  = 0;
static fixture_state_t  state          = STATE_IDLE;
static const char      *fw_version_str = NULL;

/* ------------------------------------------------------------------ */
/*  NVS Persistent Storage                                              */
/* ------------------------------------------------------------------ */
static void nvs_init_counters(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        printf("INFO, NVS init failed (0x%x) -- counters will reset on reboot\n", err);
        return;
    }

    nvs_handle_t handle;
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        printf("INFO, NVS open failed (0x%x)\n", err);
        return;
    }

    /* Read and increment session counter */
    int32_t session = 0;
    nvs_get_i32(handle, NVS_KEY_SESSION, &session);
    session++;
    session_count = (int)session;
    nvs_set_i32(handle, NVS_KEY_SESSION, session);

    /* Read persistent unit counter */
    int32_t uid = 0;
    nvs_get_i32(handle, NVS_KEY_UNIT_ID, &uid);
    unit_counter = (int)uid;

    nvs_commit(handle);
    nvs_close(handle);

    printf("INFO, Session #%d, resuming at unit #%d\n", session_count, unit_counter);
}

static void nvs_save_unit_counter(void)
{
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK) {
        nvs_set_i32(handle, NVS_KEY_UNIT_ID, (int32_t)unit_counter);
        nvs_commit(handle);
        nvs_close(handle);
    }
}

/* ------------------------------------------------------------------ */
/*  GPIO Initialisation                                                */
/* ------------------------------------------------------------------ */
static void gpio_init_all(void)
{
    swd_init();

    gpio_reset_pin(PIN_START_BUTTON);
    gpio_set_direction(PIN_START_BUTTON, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_START_BUTTON, GPIO_PULLUP_ONLY);

    gpio_reset_pin(PIN_LID_SAFETY);
    gpio_set_direction(PIN_LID_SAFETY, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_LID_SAFETY, GPIO_PULLUP_ONLY);

    gpio_reset_pin(PIN_STATUS_LED_G);
    gpio_set_direction(PIN_STATUS_LED_G, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_STATUS_LED_G, 0);

    gpio_reset_pin(PIN_STATUS_LED_R);
    gpio_set_direction(PIN_STATUS_LED_R, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_STATUS_LED_R, 0);

    gpio_reset_pin(PIN_SIM_START);
    gpio_set_direction(PIN_SIM_START, GPIO_MODE_INPUT_OUTPUT);  /* INPUT_OUTPUT so mock can read back via gpio_get_level() */
    gpio_set_level(PIN_SIM_START, 1);

    gpio_reset_pin(PIN_SIM_STOP);
    gpio_set_direction(PIN_SIM_STOP, GPIO_MODE_INPUT_OUTPUT);   /* INPUT_OUTPUT so mock can read back via gpio_get_level() */
    gpio_set_level(PIN_SIM_STOP, 1);

    gpio_reset_pin(PIN_LOAD_SENSE);
    gpio_set_direction(PIN_LOAD_SENSE, GPIO_MODE_INPUT);
}

/* ------------------------------------------------------------------ */
/*  LED Helpers                                                        */
/* ------------------------------------------------------------------ */
static void leds_off(void)
{
    gpio_set_level(PIN_STATUS_LED_G, 0);
    gpio_set_level(PIN_STATUS_LED_R, 0);
}

static void blink_led(gpio_num_t pin, int count, int half_period_ms)
{
    for (int i = 0; i < count; i++) {
        gpio_set_level(pin, 1);
        vTaskDelay(pdMS_TO_TICKS(half_period_ms));
        gpio_set_level(pin, 0);
        vTaskDelay(pdMS_TO_TICKS(half_period_ms));
    }
}

/* ------------------------------------------------------------------ */
/*  Debounced button read                                               */
/* ------------------------------------------------------------------ */
static bool start_button_pressed(void)
{
    if (!BUTTON_PRESSED(PIN_START_BUTTON)) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS));
    return BUTTON_PRESSED(PIN_START_BUTTON);
}

/* ------------------------------------------------------------------ */
/*  Force all outputs to safe state                                    */
/* ------------------------------------------------------------------ */
static void force_safe_state(void)
{
    gpio_set_level(PIN_SIM_START, 1);
    gpio_set_level(PIN_SIM_STOP,  1);
    leds_off();
}

/* ------------------------------------------------------------------ */
/*  Boot-time SWD integrity self-test                                   */
/* ------------------------------------------------------------------ */
static void run_boot_integrity_test(void)
{
    printf("INFO, Running SWD integrity self-test (%d iterations)...\n",
           INTEGRITY_ITERATIONS);

    int pass = 0, fail = 0;
#ifdef MOCK_HARDWARE_MODE
    mock_swd_integrity_test(INTEGRITY_ITERATIONS, &pass, &fail);
#else
    swd_integrity_test(INTEGRITY_ITERATIONS, &pass, &fail);
#endif

    if (fail > 0) {
        printf("INFO, WARNING: SWD integrity %d/%d passed -- check pogo contact\n",
               pass, INTEGRITY_ITERATIONS);
    } else {
        printf("INFO, SWD integrity OK: %d/%d passed\n", pass, INTEGRITY_ITERATIONS);
    }
}

/* ------------------------------------------------------------------ */
/*  app_main -- ESP-IDF entry point                                     */
/* ------------------------------------------------------------------ */
void app_main(void)
{
    /* --- Boot banner with version info --- */
    const esp_app_desc_t *app = esp_app_get_description();
    fw_version_str = app->version;

    printf("INFO, LatchPac Validator 3000 v%s (%s %s) -- boot\n",
           app->version, app->date, app->time);

#ifdef MOCK_HARDWARE_MODE
    printf("INFO, *** WARNING: MOCK HARDWARE MODE ACTIVE ***\n");
    printf("INFO, *** No live 120VAC testing -- all results simulated ***\n");
    printf("INFO, *** Disable via: idf.py menuconfig -> LatchPac Fixture Config ***\n");
#else
    printf("INFO, PRODUCTION MODE -- USB isolator MANDATORY\n");
#ifdef SWD_ISOLATED
    printf("INFO, SWD opto-isolation ENABLED (6N137 optocouplers)\n");
#else
    printf("INFO, SWD direct-wire mode (ensure GND is isolated from mains)\n");
#endif
#endif

    /* --- NVS persistent counters --- */
    nvs_init_counters();

    /* --- GPIO init --- */
    gpio_init_all();

    /* Boot-up indication */
    blink_led(PIN_STATUS_LED_G, BOOT_BLINK_COUNT, BOOT_BLINK_MS);

    /* --- SWD integrity self-test at boot --- */
    run_boot_integrity_test();

    /* Print v2 CSV header */
    log_header_v2();

    printf("INFO, Fixture ready -- waiting for operator\n");

    /* ============================================================== */
    /*  Main loop -- never exits                                        */
    /* ============================================================== */
    while (1) {

        /* ---- Global safety gate ---- */
        if (LID_IS_OPEN() && state == STATE_TESTING) {
            printf("INFO, SAFETY -- lid opened during test, aborting\n");
            force_safe_state();
            blink_led(PIN_STATUS_LED_R, 3, 100);
            state = STATE_IDLE;
            continue;
        }

        /* ---- State machine ---- */
        switch (state) {

        case STATE_IDLE:
            leds_off();
            if (start_button_pressed() && LID_IS_CLOSED()) {
                unit_counter++;
                nvs_save_unit_counter();
                printf("INFO, Starting test #%d\n", unit_counter);
                state = STATE_TESTING;
            }
            break;

        case STATE_TESTING: {
            gpio_set_level(PIN_STATUS_LED_G, 1);
            gpio_set_level(PIN_STATUS_LED_R, 1);

            /* Run v2 test with full report */
            test_report_t report = run_production_test_v2();

            /* Ensure pogo pins are safe */
            gpio_set_level(PIN_SIM_START, 1);
            gpio_set_level(PIN_SIM_STOP,  1);
            leds_off();

            const char *status_str = test_result_to_string(report.result);

            /* Read load voltage for log */
            float voltage = gpio_get_level(PIN_LOAD_SENSE) ? 3.3f : 0.0f;
#ifdef MOCK_HARDWARE_MODE
            mock_update_simulation();
            voltage = mock_read_voltage();
#endif

            /* Log with v2 enhanced format */
            log_entry_t entry = {
                .unit_id        = unit_counter,
                .status         = status_str,
                .voltage        = voltage,
                .swd_idcode     = report.swd_idcode,
                .swd_attempts   = report.swd_attempts,
                .test_duration_ms = report.duration_ms,
                .fw_version     = fw_version_str
            };
            log_result_v2(&entry);

            if (report.result == TEST_PASS) {
                gpio_set_level(PIN_STATUS_LED_G, 1);
                printf("INFO, Unit %d PASSED (duration=%lums)\n",
                       unit_counter, (unsigned long)report.duration_ms);
            } else {
                blink_led(PIN_STATUS_LED_R, FAIL_BLINK_COUNT, FAIL_BLINK_MS);
                gpio_set_level(PIN_STATUS_LED_R, 1);
                printf("INFO, Unit %d FAILED -- %s (duration=%lums)\n",
                       unit_counter, status_str, (unsigned long)report.duration_ms);
            }

            state = STATE_RESULT;
            break;
        }

        case STATE_RESULT:
            if (LID_IS_OPEN()) {
                printf("INFO, Lid opened -- resetting to IDLE\n");
                leds_off();
                state = STATE_IDLE;
            }
            break;

        default:
            printf("INFO, ERROR -- invalid state %d, forcing safe reset\n", state);
            force_safe_state();
            state = STATE_IDLE;
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
    }
}

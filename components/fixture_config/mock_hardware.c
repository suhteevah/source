/* ===================================================================
 * LatchPac Validator 3000 -- Mock Hardware Simulation (v2)
 *
 * When MOCK_HARDWARE_MODE is defined, this module replaces the real
 * hardware interactions with software stubs so the fixture logic
 * can be validated on a bench without 120 VAC.
 *
 * v2 adds mock SWD register / memory stubs returning canned responses.
 *
 * Latch behaviour model (active-low pogo pins):
 *   SIM_START=0  + SIM_STOP=0  -> load turns ON   (both pressed)
 *   SIM_START=1  + SIM_STOP=1  -> load turns OFF   (both released)
 *   SIM_START=1  + SIM_STOP=0  -> load turns OFF   (unlatch via STOP)
 * =================================================================== */

#include "fixture_pins.h"

#ifdef MOCK_HARDWARE_MODE

#include <stdbool.h>
#include <stdio.h>
#include "driver/gpio.h"
#include "mock_hardware.h"

static float simulated_voltage = 0.0f;
static bool  latched           = false;

/* ------------------------------------------------------------------ */
/*  v1 API                                                              */
/* ------------------------------------------------------------------ */

void mock_update_simulation(void)
{
    int start_level = gpio_get_level(PIN_SIM_START);
    int stop_level  = gpio_get_level(PIN_SIM_STOP);

    if (start_level == 0 && stop_level == 0) {
        latched = true;
        simulated_voltage = 3.3f;
    } else if (start_level == 1 && stop_level == 1) {
        latched = false;
        simulated_voltage = 0.0f;
    } else if (stop_level == 0 && start_level == 1) {
        latched = false;
        simulated_voltage = 0.0f;
    }
}

float mock_read_voltage(void)
{
    return simulated_voltage;
}

bool mock_swd_verify(void)
{
    return true;
}

/* ------------------------------------------------------------------ */
/*  v2 API: Mock SWD register stubs                                    */
/* ------------------------------------------------------------------ */

mock_swd_status_t mock_swd_read_dp(uint8_t addr, uint32_t *value)
{
    switch (addr) {
    case 0x00:  /* DPIDR */
        *value = SWD_IDCODE_STM32G030;
        break;
    case 0x04:  /* CTRL/STAT -- report powerup ACK bits set */
        *value = (1u << 29) | (1u << 31);  /* CDBGPWRUPACK | CSYSPWRUPACK */
        break;
    case 0x0C:  /* RDBUFF */
        *value = 0xDEADBEEF;
        break;
    default:
        *value = 0;
        break;
    }
    return MOCK_SWD_OK;
}

mock_swd_status_t mock_swd_write_dp(uint8_t addr, uint32_t value)
{
    (void)addr;
    (void)value;
    return MOCK_SWD_OK;
}

mock_swd_status_t mock_swd_read_ap(uint8_t addr, uint32_t *value)
{
    switch (addr) {
    case 0x0C:  /* DRW -- mock memory read */
        *value = 0xDEADBEEF;
        break;
    default:
        *value = 0;
        break;
    }
    return MOCK_SWD_OK;
}

mock_swd_status_t mock_swd_write_ap(uint8_t addr, uint32_t value)
{
    (void)addr;
    (void)value;
    return MOCK_SWD_OK;
}

mock_swd_status_t mock_swd_powerup_debug(void)
{
    printf("INFO, SWD debug domain active -- AP IDR=0x04770031 (mock)\n");
    return MOCK_SWD_OK;
}

mock_swd_status_t mock_swd_mem_read32(uint32_t addr, uint32_t *value)
{
    (void)addr;
    *value = 0xDEADBEEF;
    return MOCK_SWD_OK;
}

mock_swd_status_t mock_swd_integrity_test(int iterations, int *pass_count, int *fail_count)
{
    if (pass_count) *pass_count = iterations;
    if (fail_count) *fail_count = 0;
    printf("INFO, SWD integrity test: %d/%d passed (mock)\n", iterations, iterations);
    return MOCK_SWD_OK;
}

#else /* !MOCK_HARDWARE_MODE -- production build */

typedef int _mock_hardware_production_stub;

#endif /* MOCK_HARDWARE_MODE */

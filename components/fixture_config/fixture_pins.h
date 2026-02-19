#ifndef FIXTURE_PINS_H
#define FIXTURE_PINS_H

#include "driver/gpio.h"

/* ===================================================================
 * LatchPac Validator 3000 -- Canonical Pin Mapping (ESP32-S3)
 *
 * WARNING: Target board carries 120 VAC Mains.
 *          ALL outputs default to SAFE / OFF at startup.
 *          USB Galvanic Isolator (ADuM4160) MANDATORY in production.
 *
 * Source of truth: resources/01_safety_and_hardware.md
 * =================================================================== */

/* ---------- SWD Interface (Tag-Connect TC2030) ---------- */
#define PIN_SWD_CLK     GPIO_NUM_15     /* Tag-Connect Pin 2 */
#define PIN_SWD_IO      GPIO_NUM_16     /* Tag-Connect Pin 4 (direct-wire SWDIO) */
#define PIN_SWD_NRST    GPIO_NUM_17     /* Tag-Connect Pin 5 */

/*
 * Opto-Isolated SWD Mode (6N137 optocouplers)
 *
 * When CONFIG_LATCHPAC_SWD_ISOLATED is set via menuconfig, the
 * bidirectional SWDIO line is split into two unidirectional GPIOs:
 *
 *   PIN_SWD_IO_OUT (GPIO 16) -- host-to-target via 6N137 #2
 *   PIN_SWD_IO_IN  (GPIO 18) -- target-to-host via 6N137 #3
 *
 * The firmware controls direction at the protocol level (it knows
 * when it's sending vs receiving), so no external direction control
 * is needed.  SWCLK and nRST also go through their own 6N137 units.
 */
#ifdef CONFIG_LATCHPAC_SWD_ISOLATED
#define PIN_SWD_IO_OUT  GPIO_NUM_16     /* Host-to-target (6N137 #2 LED drive)  */
#define PIN_SWD_IO_IN   GPIO_NUM_18     /* Target-to-host (6N137 #3 collector)  */
#define SWD_ISOLATED     1
#endif

/* ---------- Test Interface (Pogo Pins -- Simulated Buttons) ---------- */
#define PIN_SIM_START   GPIO_NUM_4      /* Pogo -> START pad   (active LOW) */
#define PIN_SIM_STOP    GPIO_NUM_5      /* Pogo -> STOP pad    (active LOW) */
#define PIN_LOAD_SENSE  GPIO_NUM_6      /* Opto-isolated load sense input   */

/* ---------- Fixture UI (Operator Panel) ---------- */
#define PIN_START_BUTTON GPIO_NUM_0     /* Boot button -- operator "GO"      */
#define PIN_STATUS_LED_G GPIO_NUM_10    /* Green LED -- PASS                 */
#define PIN_STATUS_LED_R GPIO_NUM_11    /* Red LED   -- FAIL                 */
#define PIN_LID_SAFETY   GPIO_NUM_12   /* Lid microswitch -- NC to GND      */

/* ---------- Build Mode ---------- */
/*
 * MOCK_HARDWARE_MODE is controlled via menuconfig / sdkconfig:
 *   idf.py menuconfig -> LatchPac Fixture Config -> Mock Hardware Mode
 *
 * CONFIG_LATCHPAC_MOCK_HARDWARE is set by Kconfig.
 * Do NOT hardcode a #define here.
 */
#ifdef CONFIG_LATCHPAC_MOCK_HARDWARE
#define MOCK_HARDWARE_MODE  1
#endif

/* ---------- Hardware Constants ---------- */
#define SWD_IDCODE_STM32G030  0x0BC11477u

/* Active-low button helpers (pogo pins and operator button) */
#define BUTTON_PRESSED(pin)   (gpio_get_level(pin) == 0)
#define BUTTON_RELEASED(pin)  (gpio_get_level(pin) == 1)

/* Safety: lid switch is Normally-Closed to GND when lid is shut */
#define LID_IS_CLOSED()       (gpio_get_level(PIN_LID_SAFETY) == 0)
#define LID_IS_OPEN()         (gpio_get_level(PIN_LID_SAFETY) == 1)

#endif /* FIXTURE_PINS_H */

#ifndef CSV_LOGGER_H
#define CSV_LOGGER_H

#include <stdint.h>

/* ===================================================================
 * LatchPac Validator 3000 -- CSV Result Logger (v2)
 *
 * v1 API: log_header, log_result (4-field format)
 * v2 API: log_header_v2, log_result_v2 (9-field format)
 * =================================================================== */

/* ------------------------------------------------------------------ */
/*  v1 API (kept for backward compatibility)                            */
/* ------------------------------------------------------------------ */

/**
 * @brief Print the CSV header line to serial.
 * Format:  LOG_HEADER, Timestamp_ms, Unit_ID, Status, Load_Voltage_V
 */
void log_header(void);

/**
 * @brief Print one CSV result line to serial.
 * Format:  LOG, <timestamp_ms>, <unit_id>, <status>, <voltage>
 */
void log_result(int unit_id, const char *status, float voltage);

/* ------------------------------------------------------------------ */
/*  v2 API (extended fields)                                            */
/* ------------------------------------------------------------------ */

/**
 * @brief Enhanced log entry with full diagnostic data.
 */
typedef struct {
    int          unit_id;           /* Sequential unit counter          */
    const char  *status;            /* Human-readable result string     */
    float        voltage;           /* Load-sense voltage at test time  */
    uint32_t     swd_idcode;        /* Actual IDCODE hex value read     */
    int          swd_attempts;      /* How many SWD retries needed      */
    uint32_t     test_duration_ms;  /* Total test execution time        */
    const char  *fw_version;        /* Firmware version string          */
} log_entry_t;

/**
 * @brief Print the v2 CSV header line to serial.
 *
 * Format: LOG_HEADER, Timestamp_ms, Unit_ID, Status, Load_Voltage_V,
 *         SWD_IDCODE, SWD_Attempts, Test_Duration_ms, FW_Version
 */
void log_header_v2(void);

/**
 * @brief Print one v2 CSV result line to serial.
 */
void log_result_v2(const log_entry_t *entry);

#endif /* CSV_LOGGER_H */

/* ===================================================================
 * LatchPac Validator 3000 -- CSV Result Logger (v2)
 *
 * v1 wire format:
 *   LOG_HEADER, Timestamp_ms, Unit_ID, Status, Load_Voltage_V
 *   LOG, 12345, 001, PASS, 3.30
 *
 * v2 wire format:
 *   LOG_HEADER, Timestamp_ms, Unit_ID, Status, Load_Voltage_V,
 *               SWD_IDCODE, SWD_Attempts, Test_Duration_ms, FW_Version
 *   LOG, 12345, 001, PASS, 3.30, 0x0BC11477, 1, 1523, 1.0.0
 * =================================================================== */

#include <stdio.h>
#include <inttypes.h>
#include "esp_timer.h"
#include "csv_logger.h"

/* ------------------------------------------------------------------ */
/*  v1 API                                                              */
/* ------------------------------------------------------------------ */

void log_header(void)
{
    printf("LOG_HEADER, Timestamp_ms, Unit_ID, Status, Load_Voltage_V\n");
}

void log_result(int unit_id, const char *status, float voltage)
{
    int64_t timestamp_ms = (int64_t)(esp_timer_get_time() / 1000);
    printf("LOG, %" PRId64 ", %03d, %s, %.2f\n",
           timestamp_ms, unit_id, status, voltage);
}

/* ------------------------------------------------------------------ */
/*  v2 API                                                              */
/* ------------------------------------------------------------------ */

void log_header_v2(void)
{
    printf("LOG_HEADER, Timestamp_ms, Unit_ID, Status, Load_Voltage_V, "
           "SWD_IDCODE, SWD_Attempts, Test_Duration_ms, FW_Version\n");
}

void log_result_v2(const log_entry_t *entry)
{
    int64_t timestamp_ms = (int64_t)(esp_timer_get_time() / 1000);

    printf("LOG, %" PRId64 ", %03d, %s, %.2f, 0x%08lX, %d, %lu, %s\n",
           timestamp_ms,
           entry->unit_id,
           entry->status,
           entry->voltage,
           (unsigned long)entry->swd_idcode,
           entry->swd_attempts,
           (unsigned long)entry->test_duration_ms,
           entry->fw_version ? entry->fw_version : "unknown");
}

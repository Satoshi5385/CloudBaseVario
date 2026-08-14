#pragma once

/**
 * @brief Run the complete CloudBaseVario startup and application lifecycle.
 * @note The function enters the active, fatal, or safe-stop lifecycle and does
 *       not return during normal operation.
 */
void app_startup_run(void);

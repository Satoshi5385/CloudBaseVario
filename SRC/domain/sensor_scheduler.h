#pragma once

#include <stdbool.h>
#include <stdint.h>

#define SENSOR_SCHEDULER_PERIOD_US INT64_C(10000)
#define SENSOR_SCHEDULER_WTM_TIMEOUT_US INT64_C(15000)

typedef enum {
    SENSOR_CADENCE_BMP_TIMER = 0,
    SENSOR_CADENCE_IMU_INIT,
    SENSOR_CADENCE_IMU_WTM,
} sensor_cadence_t;

typedef enum {
    SENSOR_TRIGGER_NONE = 0,
    SENSOR_TRIGGER_BMP_TIMER,
    SENSOR_TRIGGER_IMU_WTM,
    SENSOR_TRIGGER_WTM_TIMEOUT,
} sensor_trigger_t;

typedef struct {
    sensor_cadence_t cadence;
    int64_t next_bmp_us;
    int64_t wtm_timeout_us;
    int64_t last_cycle_us;
    uint32_t wtm_cycle_count;
    uint32_t bmp_timer_cycle_count;
    uint32_t wtm_timeout_count;
    uint32_t last_cycle_interval_us;
    uint32_t max_cycle_interval_us;
} sensor_scheduler_t;

/** Initialize independent BMP timing with an immediately due first cycle. */
void sensor_scheduler_init(sensor_scheduler_t *scheduler, int64_t now_us);
/** Enter staged IMU initialization while preserving the BMP deadline. */
void sensor_scheduler_start_imu_init(sensor_scheduler_t *scheduler);
/** Select BMP-only cadence and set its first absolute deadline. */
void sensor_scheduler_use_bmp_timer(sensor_scheduler_t *scheduler,
                                    int64_t first_deadline_us);
/** Enter WTM cadence and arm its independent timeout. */
void sensor_scheduler_imu_ready(sensor_scheduler_t *scheduler, int64_t now_us);
/** Select work that is due from notification and independent deadlines. */
sensor_trigger_t sensor_scheduler_trigger(const sensor_scheduler_t *scheduler,
                                          bool wtm_notified,
                                          int64_t now_us);
/** Return the next WTM timeout or BMP absolute deadline. */
int64_t sensor_scheduler_next_wake_us(const sensor_scheduler_t *scheduler);
/** Record a successful WTM cycle and rearm its timeout. */
void sensor_scheduler_complete_wtm(sensor_scheduler_t *scheduler,
                                   int64_t completed_us);
/** Advance the absolute BMP deadline and return skipped periods. */
uint32_t sensor_scheduler_complete_bmp_timer(sensor_scheduler_t *scheduler,
                                             int64_t completed_us);
/** Switch to BMP-only cadence after completing the fault cycle BMP read. */
void sensor_scheduler_fail_imu_after_bmp(sensor_scheduler_t *scheduler,
                                         int64_t completed_us);
/** Count a WTM timeout and make the same-cycle BMP read immediately due. */
void sensor_scheduler_expire_wtm(sensor_scheduler_t *scheduler,
                                 int64_t now_us);
/** Return the stable diagnostic name for a cadence. */
const char *sensor_scheduler_cadence_name(sensor_cadence_t cadence);

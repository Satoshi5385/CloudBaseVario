#include "domain/sensor_scheduler.h"

#include <limits.h>
#include <stddef.h>

static uint32_t saturating_increment(uint32_t value) {
    if (value == UINT32_MAX) {
        return UINT32_MAX;
    }
    return value + 1U;
}

static void record_cycle(sensor_scheduler_t *scheduler, int64_t completed_us) {
    int64_t interval_us;

    if (scheduler->last_cycle_us > 0 && completed_us > scheduler->last_cycle_us) {
        interval_us = completed_us - scheduler->last_cycle_us;
        if (interval_us > UINT32_MAX) {
            scheduler->last_cycle_interval_us = UINT32_MAX;
        } else {
            scheduler->last_cycle_interval_us = (uint32_t) interval_us;
        }
        if (scheduler->last_cycle_interval_us > scheduler->max_cycle_interval_us) {
            scheduler->max_cycle_interval_us = scheduler->last_cycle_interval_us;
        }
    }
    scheduler->last_cycle_us = completed_us;
}

void sensor_scheduler_init(sensor_scheduler_t *scheduler, int64_t now_us) {
    if (scheduler == NULL) {
        return;
    }
    *scheduler = (sensor_scheduler_t) {
        .cadence = SENSOR_CADENCE_BMP_TIMER,
        .next_bmp_us = now_us,
    };
}

void sensor_scheduler_start_imu_init(sensor_scheduler_t *scheduler) {
    if (scheduler != NULL && scheduler->cadence != SENSOR_CADENCE_IMU_WTM) {
        scheduler->cadence = SENSOR_CADENCE_IMU_INIT;
    }
}

void sensor_scheduler_use_bmp_timer(sensor_scheduler_t *scheduler,
                                    int64_t first_deadline_us) {
    if (scheduler == NULL) {
        return;
    }
    scheduler->cadence = SENSOR_CADENCE_BMP_TIMER;
    scheduler->next_bmp_us = first_deadline_us;
}

void sensor_scheduler_imu_ready(sensor_scheduler_t *scheduler, int64_t now_us) {
    if (scheduler == NULL) {
        return;
    }
    scheduler->cadence = SENSOR_CADENCE_IMU_WTM;
    scheduler->wtm_timeout_us = now_us + SENSOR_SCHEDULER_WTM_TIMEOUT_US;
}

sensor_trigger_t sensor_scheduler_trigger(const sensor_scheduler_t *scheduler,
                                          bool wtm_notified,
                                          int64_t now_us) {
    if (scheduler == NULL) {
        return SENSOR_TRIGGER_NONE;
    }
    if (scheduler->cadence == SENSOR_CADENCE_IMU_WTM) {
        if (wtm_notified) {
            return SENSOR_TRIGGER_IMU_WTM;
        }
        if (now_us >= scheduler->wtm_timeout_us) {
            return SENSOR_TRIGGER_WTM_TIMEOUT;
        }
        return SENSOR_TRIGGER_NONE;
    }
    if (now_us >= scheduler->next_bmp_us) {
        return SENSOR_TRIGGER_BMP_TIMER;
    }
    return SENSOR_TRIGGER_NONE;
}

int64_t sensor_scheduler_next_wake_us(const sensor_scheduler_t *scheduler) {
    if (scheduler == NULL) {
        return 0;
    }
    if (scheduler->cadence == SENSOR_CADENCE_IMU_WTM) {
        return scheduler->wtm_timeout_us;
    }
    return scheduler->next_bmp_us;
}

void sensor_scheduler_complete_wtm(sensor_scheduler_t *scheduler,
                                   int64_t completed_us) {
    if (scheduler == NULL) {
        return;
    }
    scheduler->wtm_cycle_count = saturating_increment(
        scheduler->wtm_cycle_count);
    scheduler->cadence = SENSOR_CADENCE_IMU_WTM;
    scheduler->wtm_timeout_us = completed_us +
        SENSOR_SCHEDULER_WTM_TIMEOUT_US;
    record_cycle(scheduler, completed_us);
}

uint32_t sensor_scheduler_complete_bmp_timer(sensor_scheduler_t *scheduler,
                                             int64_t completed_us) {
    int64_t periods_elapsed = 0;
    uint32_t overruns = 0U;

    if (scheduler == NULL) {
        return 0U;
    }
    if (completed_us > scheduler->next_bmp_us) {
        periods_elapsed = (completed_us - scheduler->next_bmp_us) /
                          SENSOR_SCHEDULER_PERIOD_US;
        if (periods_elapsed > UINT32_MAX) {
            overruns = UINT32_MAX;
        } else {
            overruns = (uint32_t) periods_elapsed;
        }
    }
    scheduler->next_bmp_us += (periods_elapsed + 1) *
                              SENSOR_SCHEDULER_PERIOD_US;
    scheduler->bmp_timer_cycle_count = saturating_increment(
        scheduler->bmp_timer_cycle_count);
    record_cycle(scheduler, completed_us);
    return overruns;
}

void sensor_scheduler_fail_imu_after_bmp(sensor_scheduler_t *scheduler,
                                         int64_t completed_us) {
    if (scheduler == NULL) {
        return;
    }
    sensor_scheduler_use_bmp_timer(
        scheduler, completed_us + SENSOR_SCHEDULER_PERIOD_US);
    record_cycle(scheduler, completed_us);
}

void sensor_scheduler_expire_wtm(sensor_scheduler_t *scheduler,
                                 int64_t now_us) {
    if (scheduler == NULL) {
        return;
    }
    scheduler->wtm_timeout_count = saturating_increment(
        scheduler->wtm_timeout_count);
    scheduler->cadence = SENSOR_CADENCE_BMP_TIMER;
    scheduler->next_bmp_us = now_us;
}

const char *sensor_scheduler_cadence_name(sensor_cadence_t cadence) {
    switch (cadence) {
        case SENSOR_CADENCE_IMU_INIT:
            return "IMU_INIT";
        case SENSOR_CADENCE_IMU_WTM:
            return "WTM";
        case SENSOR_CADENCE_BMP_TIMER:
        default:
            return "BMP_TIMER";
    }
}

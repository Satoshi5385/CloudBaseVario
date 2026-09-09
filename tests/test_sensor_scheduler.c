#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "domain/sensor_scheduler.h"

static void test_wtm_cadence(void) {
    sensor_scheduler_t scheduler;

    sensor_scheduler_init(&scheduler, INT64_C(1000000));
    assert(sensor_scheduler_trigger(&scheduler, false, INT64_C(1000000)) ==
           SENSOR_TRIGGER_BMP_TIMER);
    sensor_scheduler_start_imu_init(&scheduler);
    assert(strcmp(sensor_scheduler_cadence_name(scheduler.cadence),
                  "IMU_INIT") == 0);
    sensor_scheduler_imu_ready(&scheduler, INT64_C(1002000));
    assert(sensor_scheduler_trigger(&scheduler, false, INT64_C(1016999)) ==
           SENSOR_TRIGGER_NONE);
    assert(sensor_scheduler_trigger(&scheduler, true, INT64_C(1012000)) ==
           SENSOR_TRIGGER_IMU_WTM);
    sensor_scheduler_complete_wtm(&scheduler, INT64_C(1013500));
    assert(scheduler.wtm_cycle_count == 1U);
    assert(scheduler.wtm_timeout_us == INT64_C(1028500));
    assert(sensor_scheduler_trigger(&scheduler, false, INT64_C(1028500)) ==
           SENSOR_TRIGGER_WTM_TIMEOUT);
}

static void test_fault_switches_to_absolute_bmp_timer(void) {
    sensor_scheduler_t scheduler;

    sensor_scheduler_init(&scheduler, INT64_C(2000000));
    sensor_scheduler_imu_ready(&scheduler, INT64_C(2000000));
    sensor_scheduler_fail_imu_after_bmp(&scheduler, INT64_C(2005000));
    assert(scheduler.cadence == SENSOR_CADENCE_BMP_TIMER);
    assert(scheduler.next_bmp_us == INT64_C(2015000));
    assert(sensor_scheduler_trigger(&scheduler, false, INT64_C(2014999)) ==
           SENSOR_TRIGGER_NONE);
    assert(sensor_scheduler_trigger(&scheduler, false, INT64_C(2015000)) ==
           SENSOR_TRIGGER_BMP_TIMER);
    assert(sensor_scheduler_complete_bmp_timer(
               &scheduler, INT64_C(2016200)) == 0U);
    assert(scheduler.next_bmp_us == INT64_C(2025000));
    assert(sensor_scheduler_complete_bmp_timer(
               &scheduler, INT64_C(2056000)) == 3U);
    assert(scheduler.next_bmp_us == INT64_C(2065000));
}

static void test_timeout_is_counted_once(void) {
    sensor_scheduler_t scheduler;

    sensor_scheduler_init(&scheduler, INT64_C(3000000));
    sensor_scheduler_imu_ready(&scheduler, INT64_C(3000000));
    sensor_scheduler_expire_wtm(&scheduler, INT64_C(3015000));
    assert(scheduler.wtm_timeout_count == 1U);
    assert(scheduler.next_bmp_us == INT64_C(3015000));
    assert(sensor_scheduler_trigger(&scheduler, false, INT64_C(3015000)) ==
           SENSOR_TRIGGER_BMP_TIMER);
}

static void test_imu_init_preserves_bmp_absolute_deadline(void) {
    sensor_scheduler_t scheduler;

    sensor_scheduler_init(&scheduler, INT64_C(4000000));
    sensor_scheduler_start_imu_init(&scheduler);
    assert(scheduler.cadence == SENSOR_CADENCE_IMU_INIT);
    assert(sensor_scheduler_trigger(&scheduler, false, INT64_C(4000000)) ==
           SENSOR_TRIGGER_BMP_TIMER);
    assert(sensor_scheduler_complete_bmp_timer(
               &scheduler, INT64_C(4000800)) == 0U);
    assert(scheduler.cadence == SENSOR_CADENCE_IMU_INIT);
    assert(scheduler.next_bmp_us == INT64_C(4010000));
    assert(sensor_scheduler_complete_bmp_timer(
               &scheduler, INT64_C(4041500)) == 3U);
    assert(scheduler.next_bmp_us == INT64_C(4050000));
}

static void test_imu_unused_remains_on_bmp_timer(void) {
    sensor_scheduler_t scheduler;

    sensor_scheduler_init(&scheduler, INT64_C(5000000));
    assert(scheduler.cadence == SENSOR_CADENCE_BMP_TIMER);
    assert(sensor_scheduler_trigger(&scheduler, true, INT64_C(5000000)) ==
           SENSOR_TRIGGER_BMP_TIMER);
    sensor_scheduler_complete_bmp_timer(&scheduler, INT64_C(5000500));
    assert(scheduler.next_bmp_us == INT64_C(5010000));
}

int main(void) {
    test_wtm_cadence();
    test_fault_switches_to_absolute_bmp_timer();
    test_timeout_is_counted_once();
    test_imu_init_preserves_bmp_absolute_deadline();
    test_imu_unused_remains_on_bmp_timer();
    puts("sensor scheduler tests passed");
    return 0;
}

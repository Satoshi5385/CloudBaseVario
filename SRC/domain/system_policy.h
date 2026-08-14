#pragma once

#include <stdbool.h>
#include <stdint.h>

#define SYSTEM_POLICY_SAMPLE_PERIOD_MS UINT32_C(10)
#define SYSTEM_POLICY_SWITCH_DEBOUNCE_MS UINT32_C(30)
#define SYSTEM_POLICY_POWER_OFF_HOLD_MS UINT32_C(2000)
#define SYSTEM_POLICY_IMU_SKIP_HOLD_MS UINT32_C(3000)
#define SYSTEM_POLICY_SHUTDOWN_DEADLINE_MS UINT32_C(15000)
#define SYSTEM_POLICY_LOW_BATTERY_THRESHOLD_V 3.3f

typedef struct {
    bool candidate_pressed;
    bool stable_pressed;
    bool stable_valid;
    uint32_t stable_time_ms;
} system_policy_button_t;

typedef struct {
    system_policy_button_t sw1;
    system_policy_button_t sw2;
    system_policy_button_t sw3;
    uint32_t sw1_hold_ms;
    uint32_t sw3_hold_ms;
    bool sw1_was_released;
    bool previous_sw1_pressed;
    bool previous_sw2_pressed;
    bool previous_sw3_pressed;
    bool sw1_short_press_pending;
    bool sw1_power_off_issued;
    bool sw3_short_press_pending;
    bool sw3_skip_request_issued;
} system_policy_state_t;

typedef struct {
    bool sw1_pressed;
    bool sw2_pressed;
    bool sw3_pressed;
    bool imu_calibration_required;
} system_policy_input_t;

typedef struct {
    bool sw1_pressed;
    bool sw2_pressed;
    bool sw3_pressed;
    uint32_t sw1_hold_ms;
    uint32_t sw3_hold_ms;
    bool advance_volume;
    bool toggle_sink;
    bool select_next_profile;
    bool request_calibration_skip;
    bool request_power_off;
} system_policy_actions_t;

typedef struct {
    uint32_t elapsed_ms;
    uint32_t sw1_hold_ms;
    float battery_voltage_v;
    bool fatal;
    bool fatal_bmp581;
    bool bmp581_startup_complete;
    bool vario_available;
    bool pressure_valid;
    bool climb_rate_valid;
    bool estimator_warming_up;
    bool bmp581_recovering;
    bool imu_calibrating;
    bool imu_degraded;
    bool storage_mode_active;
    bool external_power_present;
    bool battery_valid;
    bool ble_notify_active;
} system_led_policy_input_t;

typedef struct {
    uint32_t green_brightness_percent;
    bool yellow_on;
} system_led_policy_output_t;

/** Initialize switch policy with the raw levels sampled before the first delay. */
void system_policy_init(system_policy_state_t *state,
                        const system_policy_input_t *input);

/** Evaluate one 10 ms switch sample and return edge-triggered actions. */
void system_policy_step(system_policy_state_t *state,
                        const system_policy_input_t *input,
                        system_policy_actions_t *actions);

/** Debounce one raw switch sample using the shared 30 ms policy. */
bool system_policy_debounce(system_policy_button_t *button, bool pressed);

/** Add one sample period without wrapping a millisecond counter. */
uint32_t system_policy_increment_ms(uint32_t elapsed_ms);

/** Select status LED output using the documented lifecycle priority. */
void system_policy_select_leds(const system_led_policy_input_t *input,
                               system_led_policy_output_t *output);

/** Power-on and power-off hold progress rendered as green LED brightness. */
uint32_t system_policy_power_on_brightness(uint32_t hold_ms,
                                           uint32_t required_hold_ms);
/** Render power-off hold progress as green LED brightness. */
uint32_t system_policy_power_off_brightness(uint32_t hold_ms);

/** Saturating time remaining before the hard shutdown deadline. */
uint32_t system_policy_shutdown_remaining_ms(int64_t deadline_us,
                                             int64_t now_us);

/** A shutdown sound starts only when workers quiesced with enough time left. */
bool system_policy_can_start_shutdown_sound(bool workers_quiesced,
                                            uint32_t remaining_ms,
                                            uint32_t sound_duration_ms);

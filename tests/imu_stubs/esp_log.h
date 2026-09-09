#pragma once
void test_imu_log(const char *tag, const char *format, ...);
#define ESP_LOGE(...) test_imu_log(__VA_ARGS__)
#define ESP_LOGW(...) test_imu_log(__VA_ARGS__)
#define ESP_LOGI(...) test_imu_log(__VA_ARGS__)

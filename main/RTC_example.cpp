#include <stdio.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "DS3231_RTC.h"

static const char *TAG = "RTC_example";

/*
 * Function to convert decimal to BCD
 */
static uint8_t dec_to_bcd(uint8_t dec) {
    return ((dec / 10 * 16) + (dec % 10));
}

extern "C" void app_main(void)
{
    struct tm timeinfo;

    // set time manually
    timeinfo.tm_sec = 45;
    timeinfo.tm_min = 30;
    timeinfo.tm_hour = 12;
    timeinfo.tm_mday = 22;
    timeinfo.tm_mon = 8; // 0=Jan, Sept=8
    timeinfo.tm_year = 2024 - 1900; // years since 1900
    timeinfo.tm_wday = 0; // sun = 0

    // init rtc
    DS3231_RTC rtc;
    rtc.init();

    // set the time on the RTC
    rtc.seconds = dec_to_bcd(timeinfo.tm_sec);
    rtc.minutes = dec_to_bcd(timeinfo.tm_min);
    rtc.hours = dec_to_bcd(timeinfo.tm_hour);
    rtc.day = dec_to_bcd(timeinfo.tm_mday);
    rtc.month = dec_to_bcd(timeinfo.tm_mon);
    rtc.year = dec_to_bcd(timeinfo.tm_year);
    rtc.weekday = dec_to_bcd(timeinfo.tm_wday);

    ESP_LOGI(TAG, "%x:%x:%x", rtc.hours, rtc.minutes, rtc.seconds);

    rtc.setTime();
    
    vTaskDelay(10000 / portTICK_PERIOD_MS);

    rtc.getTime();

    ESP_LOGI(TAG, "%x:%x:%x", rtc.hours, rtc.minutes, rtc.seconds);

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    rtc.getTime();
    ESP_LOGI(TAG, "%x:%x:%x", rtc.hours, rtc.minutes, rtc.seconds);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    rtc.getTime();
    ESP_LOGI(TAG, "%x:%x:%x", rtc.hours, rtc.minutes, rtc.seconds);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    rtc.getTime();
    ESP_LOGI(TAG, "%x:%x:%x", rtc.hours, rtc.minutes, rtc.seconds);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    rtc.getTime();
    ESP_LOGI(TAG, "%x:%x:%x", rtc.hours, rtc.minutes, rtc.seconds);

}

#include "sdkconfig.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_mac.h"
#include "sdkconfig.h"
#include "esp_random.h"

#define SAMPLE_RATE 20 // Hz
#define QUEUE_SIZE 100
#define SYNC_RETRY_DELAY 500 // ms
#define MAX_SYNC_RETRIES 100

static const char *TAG = "ADC_ESP_NOW";
static QueueHandle_t adc_data_queue;
// COM8
static uint8_t unicast_address[ESP_NOW_ETH_ALEN] = {0xf4, 0x12, 0xfa, 0x88, 0x01, 0x9c}; // Replace with your target MAC address
static bool sync_complete = false;

/**
 * @brief Function to initialize Wi-Fi in station mode (required for ESP-NOW).
 */
static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_efuse_mac_get_default(mac));

    // Set the Wi-Fi station interface to use the default MAC address
    ESP_ERROR_CHECK(esp_wifi_set_mac(ESP_IF_WIFI_STA, mac));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));

    ESP_LOGI(TAG, "Default MAC address: " MACSTR, MAC2STR(mac));

    ESP_ERROR_CHECK(esp_wifi_get_mac(ESP_IF_WIFI_STA, mac));
    ESP_LOGI(TAG, "Wi-Fi Station MAC address: " MACSTR, MAC2STR(mac));

    ESP_ERROR_CHECK(esp_wifi_get_mac(ESP_IF_WIFI_AP, mac));
    ESP_LOGI(TAG, "Wi-Fi SoftAP MAC address: " MACSTR, MAC2STR(mac));
    uint8_t primary;
    wifi_second_chan_t second;
    ESP_ERROR_CHECK(esp_wifi_get_channel(&primary, &second));
    ESP_LOGI(TAG, "Current Wi-Fi channel: %d", primary);
}
/**
 * @brief Callback function for ESP-NOW send status.
 */
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if (mac_addr == NULL)
    {
        ESP_LOGE(TAG, "Send callback error: NULL MAC address");
        return;
    }
    ESP_LOGI(TAG, "Send status to " MACSTR ": %s", MAC2STR(mac_addr), status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
}

/**
 * @brief Callback function for ESP-NOW receive events.
 */
static void espnow_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len)
{
    if (mac_addr == NULL || data == NULL || len <= 0)
    {
        ESP_LOGE(TAG, "Receive callback error: invalid arguments");
        return;
    }

    if (len == 4 && memcmp(data, "SYNC", len) == 0)
    {
        ESP_LOGI(TAG, "Sync response received from " MACSTR, MAC2STR(mac_addr));
        sync_complete = true;
    }
}

/**
 * @brief Function to initialize ESP-NOW and register callbacks.
 */
static void espnow_init(void)
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    esp_now_peer_info_t peer_info = {
        .channel = 0,
        .ifidx = ESP_IF_WIFI_STA,
        .encrypt = false,
    };
    memcpy(peer_info.peer_addr, unicast_address, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));
}

/**
 * @brief ADC sampling task. Samples data at 20Hz and pushes it to the queue.
 */
static void adc_sampling_task(void *arg)
{
    float sample = 0.0f;

    // Wait for synchronization to complete before starting ADC sampling
    while (!sync_complete)
    {
        vTaskDelay(pdMS_TO_TICKS(100)); // Check every 100ms
    }

    while (1)
    {
        sample = (float)esp_random() / UINT32_MAX; // Simulate ADC float32 data
        if (xQueueSend(adc_data_queue, &sample, pdMS_TO_TICKS(10)) != pdTRUE)
        {
            ESP_LOGW(TAG, "ADC queue full, sample dropped");
        }
        vTaskDelay(pdMS_TO_TICKS(1000 / SAMPLE_RATE)); // Sample at the correct rate
    }
}

/**
 * @brief ESP-NOW data sending task. Sends ADC samples to the unicast address.
 */
static void espnow_sending_task(void *arg)
{
    float sample;
    char sync_msg[] = "SYNC";

    // Ensure synchronization
    int sync_attempts = 0;
    while (!sync_complete && sync_attempts < MAX_SYNC_RETRIES)
    {
        ESP_LOGI(TAG, "Sending sync message");
        ESP_ERROR_CHECK(esp_now_send(unicast_address, (uint8_t *)sync_msg, strlen(sync_msg)));
        vTaskDelay(pdMS_TO_TICKS(SYNC_RETRY_DELAY));
        sync_attempts++;
    }

    if (!sync_complete)
    {
        ESP_LOGE(TAG, "Failed to synchronize after %d attempts", MAX_SYNC_RETRIES);
        vTaskDelete(NULL);
    }

    ESP_LOGI(TAG, "Synchronization complete, starting data transmission");

    while (1)
    {
        if (xQueueReceive(adc_data_queue, &sample, portMAX_DELAY) == pdTRUE)
        {
            ESP_ERROR_CHECK(esp_now_send(unicast_address, (uint8_t *)&sample, sizeof(sample)));
            ESP_LOGI(TAG, "Sent ADC sample: %f", sample);
        }
    }
}

/**
 * @brief Main application entry point.
 */

void app_main(void)
{
    // Initialize NVS (required for Wi-Fi)
    ESP_ERROR_CHECK(nvs_flash_init());

    // Initialize Wi-Fi and ESP-NOW
    wifi_init();
    espnow_init();

    // Create the ADC data queue
    adc_data_queue = xQueueCreate(QUEUE_SIZE, sizeof(float));
    if (adc_data_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create ADC data queue");
        return;
    }

    // Create tasks
    xTaskCreatePinnedToCore(adc_sampling_task, "adc_sampling_task", 2048, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(espnow_sending_task, "espnow_sending_task", 2048, NULL, 5, NULL, 0);
}
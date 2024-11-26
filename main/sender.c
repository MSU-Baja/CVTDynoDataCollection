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
#include "esp_adc/adc_oneshot.h"
#include "hal/adc_types.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define SAMPLE_RATE 20 // Hz
#define QUEUE_SIZE 100
#define SYNC_RETRY_DELAY 500 // ms
#define MAX_SYNC_RETRIES 1000

// ADC Configuration
#define ADC_UNIT ADC_UNIT_1
#define ADC_CHANNEL ADC_CHANNEL_0
#define ADC_ATTEN ADC_ATTEN_DB_0
#define ADC_BITWIDTH ADC_BITWIDTH_DEFAULT

static const char *TAG = "ADC_ESP_NOW";
static QueueHandle_t adc_data_queue;
static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t adc_cali_handle;

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

static void adc_init(void)
{
    // Initialize ADC unit
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));

    // Configure ADC channel
    adc_oneshot_chan_cfg_t channel_config = {
        .bitwidth = ADC_BITWIDTH,
        .atten = ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL, &channel_config));

    ESP_LOGI(TAG, "ADC initialized on Unit: %d, Channel: %d", ADC_UNIT, ADC_CHANNEL);
}

static void adc_cali_init()
{
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT,
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH,
    };

    // Check if curve fitting calibration is supported
    esp_err_t ret = adc_cali_create_scheme_curve_fitting(&cali_config, &adc_cali_handle);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "ADC calibration initialized (Curve Fitting)");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to initialize ADC calibration: %s", esp_err_to_name(ret));
        adc_cali_handle = NULL;
    }
}

static void adc_cali_deinit()
{
    if (adc_cali_handle)
    {
        ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(adc_cali_handle));
        ESP_LOGI(TAG, "ADC calibration deinitialized");
    }
}
/**
 * @brief ADC sampling task. Samples data at 20Hz and pushes it to the queue.
 */
static void adc_sampling_task(void *arg)
{
    int raw_data = 0;
    int voltage = 0.0;
    while (!sync_complete)
    {
        vTaskDelay(pdMS_TO_TICKS(50)); // Check every 100ms
    }
    while (1)
    {
        // Read raw ADC data
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL, &raw_data));
        if (adc_cali_handle)
        {
            ESP_GOTO_ON_ERROR(adc_cali_raw_to_voltage(adc_cali_handle, raw_data, &voltage), err, TAG, "adc_cali_raw_to_voltage failed");
            ESP_LOGI(TAG, "ADC Calibrated Voltage: %d mV", voltage);
        }
        else
        {
            ESP_LOGW(TAG, "ADC Calibration not available, raw data: %d", raw_data);
        }
        // Convert raw data to voltage

        // voltage = raw_data * 1.1 / (1 << 12); // Assuming 12-bit ADC resolution
        // ESP_LOGI(TAG, "ADC Voltage: %.3f V", voltage);
        float voltage_float = (float)voltage / 1000.0f;
        // Push data to queue
        if (xQueueSend(adc_data_queue, &voltage_float, pdMS_TO_TICKS(10)) != pdTRUE)
        {
            ESP_LOGW(TAG, "ADC queue full, sample dropped");
        }
        // Delay to maintain 20 Hz sample rate
        vTaskDelay(pdMS_TO_TICKS(1000 / SAMPLE_RATE));
    }

err:
    adc_cali_deinit();
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
static void espnow_recv_cb(const esp_now_recv_info_t *messageInfo, const uint8_t *data, int len)
{
    const uint8_t *mac_addr = messageInfo->src_addr;
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
    adc_init();
    adc_cali_init();
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
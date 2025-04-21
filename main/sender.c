#include "sdkconfig.h"
#include <stdio.h>
#include <string.h>
#include <math.h> // Added for floating point conversion
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

// #include "sdkconfig.h"
// #include "esp_random.h"

// Add I2C and ADS111x includes
#include "driver/i2c.h" // Needed for GPIO definitions
#include "i2cdev.h"
#include "ads111x.h"

// ADC/I2C Configuration
#define I2C_MASTER_SCL_IO           GPIO_NUM_48      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           GPIO_NUM_47      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0        /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          100000           /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                /*!< I2C master doesn't need buffer */

#define ADS111X_ADDR                ADS111X_ADDR_GND /*!< Slave address of the ADS1115 device (ADDR pin connected to GND) */
#define ADS111X_GAIN                ADS111X_GAIN_2V048 /*!< PGA gain, e.g., +/-2.048V */
#define ADS101X_DATA_RATE           ADS101X_DATA_RATE_1600 /*!< ADS1015 data rate */

// ESP-NOW Configuration (Unchanged)
#define SAMPLE_RATE 20 // Hz
#define QUEUE_SIZE 100
#define SYNC_RETRY_DELAY 500 // ms
#define MAX_SYNC_RETRIES 100

// Globals (ADC device descriptor added)
static const char *TAG = "ADC_ESP_NOW";
static QueueHandle_t adc_data_queue;
static i2c_dev_t adc_dev; // ADC device descriptor
static const float gain_val = 2.048F; // Corresponds to ADS111X_GAIN_2V048
// COM8 (Keep your target MAC)
static uint8_t unicast_address[ESP_NOW_ETH_ALEN] = {0xf4, 0x12, 0xfa, 0x88, 0x01, 0x9c}; // Replace with your target MAC address
static bool sync_complete = false;

// Data Structure for Queue
typedef struct {
    float voltage;
    // uint32_t timestamp_ms; // Optional: Add timestamp if needed
} adc_sample_t;

/**
 * @brief Initialize I2C master bus using i2cdev component
 */
static esp_err_t i2c_master_init(void)
{
    ESP_LOGI(TAG, "Initializing I2C master...");
    ESP_ERROR_CHECK(i2cdev_init()); // Initialize I2Cdev library
    ESP_LOGI(TAG, "I2C master initialized successfully.");
    return ESP_OK;
}

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
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    if (recv_info == NULL || data == NULL || len <= 0)
    {
        ESP_LOGE(TAG, "Receive callback error: invalid arguments");
        return;
    }

    const uint8_t *mac_addr = recv_info->src_addr; // Extract MAC address

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
 * @brief ADC sampling task. Initializes ADS1015, samples data, and pushes to queue.
 */
static void adc_sampling_task(void *arg)
{
    adc_sample_t sample_data;
    int16_t raw_value;

    // --- Initialize ADS1015 Device ---
    ESP_LOGI(TAG, "Initializing ADS1015 device in task...");
    // Initialize descriptor
    ESP_ERROR_CHECK_WITHOUT_ABORT(ads111x_init_desc(&adc_dev, ADS111X_ADDR, I2C_MASTER_NUM, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO));
    ESP_LOGI(TAG, "ADS1015 descriptor initialized.");

    // Configure settings
    ESP_ERROR_CHECK_WITHOUT_ABORT(ads111x_set_mode(&adc_dev, ADS111X_MODE_CONTINUOUS));
    ESP_LOGI(TAG, "Set mode to continuous.");
    ESP_ERROR_CHECK_WITHOUT_ABORT(ads111x_set_gain(&adc_dev, ADS111X_GAIN));
    ESP_LOGI(TAG, "Set gain to +/- %.3fV.", gain_val);
    // Note: Cast needed for ADS1015 specific rate enum value
    ESP_ERROR_CHECK_WITHOUT_ABORT(ads111x_set_data_rate(&adc_dev, (ads111x_data_rate_t)ADS101X_DATA_RATE));
    ESP_LOGI(TAG, "Set data rate.");
    ESP_ERROR_CHECK_WITHOUT_ABORT(ads111x_set_input_mux(&adc_dev, ADS111X_MUX_0_GND));
    ESP_LOGI(TAG, "Set MUX to AIN0 vs GND.");
    ESP_LOGI(TAG, "ADS1015 device configured.");
    // --- End ADS1015 Initialization ---


    // Wait for synchronization to complete before starting ADC sampling
    ESP_LOGI(TAG, "ADC task waiting for sync...");
    while (!sync_complete)
    {
        vTaskDelay(pdMS_TO_TICKS(100)); // Check every 100ms
    }
    ESP_LOGI(TAG, "ADC task starting sampling loop.");


    while (1)
    {
        // Read raw value from ADS1015
        esp_err_t read_status = ads111x_get_value(&adc_dev, &raw_value);

        if (read_status == ESP_OK)
        {
            // Convert raw value (12-bit effective resolution for ADS1015) to voltage
            // Value is sign-extended to 16 bits, max positive is 2047.
            sample_data.voltage = (float)raw_value * gain_val / 2047.0f;
            // sample_data.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS; // Optional timestamp

            // Send data to queue
            if (xQueueSend(adc_data_queue, &sample_data, pdMS_TO_TICKS(10)) != pdTRUE)
            {
                ESP_LOGW(TAG, "ADC queue full, sample dropped");
            } else {
                 ESP_LOGD(TAG, "ADC Sample: %.4f V (Raw: %d)", sample_data.voltage, raw_value); // Verbose log
            }
        }
        else
        {
            ESP_LOGE(TAG, "Failed to read ADC value: %s (%d)", esp_err_to_name(read_status), read_status);
             // Optional: try re-initializing I2C or ADC on persistent errors?
             vTaskDelay(pdMS_TO_TICKS(100)); // Delay before retrying after error
        }

        // Delay for SAMPLE_RATE
        vTaskDelay(pdMS_TO_TICKS(1000 / SAMPLE_RATE));
    }
}

/**
 * @brief ESP-NOW data sending task. Sends ADC voltage to the unicast address.
 */
static void espnow_sending_task(void *arg)
{
    adc_sample_t received_sample; // Use the struct type
    char sync_msg[] = "SYNC";

    // Ensure synchronization
    int sync_attempts = 0;
    while (!sync_complete && sync_attempts < MAX_SYNC_RETRIES)
    {
        ESP_LOGI(TAG, "Sending sync message (attempt %d/%d)", sync_attempts + 1, MAX_SYNC_RETRIES);
        esp_err_t send_result = esp_now_send(unicast_address, (uint8_t *)sync_msg, strlen(sync_msg));
        if (send_result != ESP_OK) {
             ESP_LOGE(TAG, "Sync send failed: %s", esp_err_to_name(send_result));
        }
        vTaskDelay(pdMS_TO_TICKS(SYNC_RETRY_DELAY));
        sync_attempts++;
    }

    if (!sync_complete)
    {
        ESP_LOGE(TAG, "Failed to synchronize after %d attempts. Stopping send task.", MAX_SYNC_RETRIES);
        // Consider notifying ADC task or other error handling
        vTaskDelete(NULL); // Stop this task
    }

    ESP_LOGI(TAG, "Synchronization complete, starting data transmission");

    while (1)
    {
        // Receive sample from queue (wait indefinitely)
        if (xQueueReceive(adc_data_queue, &received_sample, portMAX_DELAY) == pdTRUE)
        {
            // Send only the voltage float
            esp_err_t send_result = esp_now_send(unicast_address, (uint8_t *)&received_sample.voltage, sizeof(received_sample.voltage));
             if (send_result == ESP_OK) {
                ESP_LOGD(TAG, "Sent ADC voltage: %.4f V", received_sample.voltage); // Verbose log
            } else {
                ESP_LOGW(TAG, "ESP-NOW send failed: %s", esp_err_to_name(send_result));
            }
        }
        // If queue receive fails (shouldn't with portMAX_DELAY unless queue deleted), loop continues
    }
}

/**
 * @brief Main application entry point.
 */

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Application...");
    // Initialize NVS (required for Wi-Fi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS Initialized.");

    // Initialize I2C Master
    ESP_ERROR_CHECK(i2c_master_init());

    // Initialize Wi-Fi and ESP-NOW
    wifi_init();
    espnow_init();
    ESP_LOGI(TAG, "Wi-Fi and ESP-NOW Initialized.");

    // Create the ADC data queue
    adc_data_queue = xQueueCreate(QUEUE_SIZE, sizeof(adc_sample_t));
    if (adc_data_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create ADC data queue");
        // Consider restart or fatal error
        return;
    }
    ESP_LOGI(TAG, "ADC Data Queue Created.");

    // Create tasks
    // Stack size might need tuning depending on logging levels and complexity
    xTaskCreatePinnedToCore(adc_sampling_task, "adc_sampling_task", 3584, NULL, 5, NULL, 1); // Core 1 for ADC
    xTaskCreatePinnedToCore(espnow_sending_task, "espnow_sending_task", 3584, NULL, 5, NULL, 0); // Core 0 for Wi-Fi/ESP-NOW
    ESP_LOGI(TAG, "Tasks Created.");

    ESP_LOGI(TAG, "Application Initialization Complete. Device running.");
    // The tasks now run indefinitely
}
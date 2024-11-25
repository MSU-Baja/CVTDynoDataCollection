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

#define QUEUE_SIZE 100

static const char *TAG = "Receiver";
static QueueHandle_t received_data_queue;

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
        ESP_LOGI(TAG, "Sync message received from " MACSTR, MAC2STR(mac_addr));

        // Dynamically add the sender as a peer
        esp_now_peer_info_t peer_info = {
            .channel = 0, // Use the same Wi-Fi channel as the receiver
            .ifidx = ESP_IF_WIFI_STA,
            .encrypt = false,
        };
        memcpy(peer_info.peer_addr, mac_addr, ESP_NOW_ETH_ALEN);

        // Check if the peer already exists
        if (!esp_now_is_peer_exist(mac_addr))
        {
            if (esp_now_add_peer(&peer_info) != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to add sender as a peer " MACSTR, MAC2STR(mac_addr));
                return; // Exit if peer addition fails
            }
            else
            {
                ESP_LOGI(TAG, "Sender added as a peer " MACSTR, MAC2STR(mac_addr));
            }
        }
        else
        {
            ESP_LOGI(TAG, "Sender already exists as a peer " MACSTR, MAC2STR(mac_addr));
        }

        // Send a sync acknowledgment
        char sync_ack[] = "SYNC";
        if (esp_now_send(mac_addr, (uint8_t *)sync_ack, strlen(sync_ack)) != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to send sync acknowledgment to " MACSTR, MAC2STR(mac_addr));
        }
        else
        {
            ESP_LOGI(TAG, "Sync acknowledgment sent to " MACSTR, MAC2STR(mac_addr));
        }
    }
    else if (len == sizeof(float))
    {
        // Receive ADC data (float32)
        float received_sample;
        memcpy(&received_sample, data, sizeof(received_sample));
        ESP_LOGI(TAG, "Received ADC sample: %f from " MACSTR, received_sample, MAC2STR(mac_addr));

        // Add data to the queue for further processing
        if (xQueueSend(received_data_queue, &received_sample, pdMS_TO_TICKS(10)) != pdTRUE)
        {
            ESP_LOGW(TAG, "Data queue full, sample dropped");
        }
    }
    else
    {
        ESP_LOGW(TAG, "Received unknown data from " MACSTR ", length: %d", MAC2STR(mac_addr), len);
    }
}

/**
 * @brief Function to initialize ESP-NOW and register callbacks.
 */
static void espnow_init(void)
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
}

/**
 * @brief Task to process received ADC samples from the queue.
 */
static void process_received_data_task(void *arg)
{
    float received_sample;
    while (1)
    {
        if (xQueueReceive(received_data_queue, &received_sample, portMAX_DELAY) == pdTRUE)
        {
            // Process the received ADC sample (for demonstration, just log it)
            ESP_LOGI(TAG, "Processing ADC sample: %f", received_sample);
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

    // Create the data queue
    received_data_queue = xQueueCreate(QUEUE_SIZE, sizeof(float));
    if (received_data_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create data queue");
        return;
    }

    // Create task to process received ADC samples
    xTaskCreatePinnedToCore(process_received_data_task, "process_received_data_task", 2048, NULL, 5, NULL, 1);

    ESP_LOGI(TAG, "Receiver is ready and waiting for packets...");
}

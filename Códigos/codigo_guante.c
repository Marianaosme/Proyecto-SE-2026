#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nvs_flash.h"

#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"

#include "esp_wifi.h"
#include "esp_now.h"

#include "driver/adc.h"
#include "driver/gpio.h"

static const char *TAG = "ESP_GLOVE";

/* ========================================================= */
/* GPIO CONFIG                                               */
/* ========================================================= */

#define LED_PIN            GPIO_NUM_2

#define BUTTON_CALIB_PIN   GPIO_NUM_18
#define BUTTON_EXTRA_PIN   GPIO_NUM_5

#define VIBRATION_PIN      GPIO_NUM_19

/* ========================================================= */
/* ADC CHANNELS                                              */
/* ========================================================= */

#define POT_THUMB   ADC1_CHANNEL_4   // GPIO32
#define POT_INDEX   ADC1_CHANNEL_0   // GPIO36
#define POT_MIDDLE  ADC1_CHANNEL_3   // GPIO39
#define POT_RING    ADC1_CHANNEL_6   // GPIO34
#define POT_PINKY   ADC1_CHANNEL_7   // GPIO35

/* ========================================================= */
/* FILTER SETTINGS                                           */
/* ========================================================= */

#define EMA_DIVIDER     8
#define DEADZONE        2

/* ========================================================= */
/* MANUAL CALIBRATION VALUES                                 */
/* ========================================================= */

static uint16_t finger_min[5] = {
    1200,
    1200,
    1200,
    1200,
    1200
};

static uint16_t finger_max[5] = {
    3200,
    3200,
    3200,
    3200,
    3200
};

/* ========================================================= */
/* ESP-NOW PEER                                              */
/* ========================================================= */

static const uint8_t peer_receiver[ESP_NOW_ETH_ALEN] = {
    0xE0, 0x8C, 0xFE, 0x57, 0x9C, 0xB8
};

/* ========================================================= */
/* PACKET STRUCTURE                                          */
/* ========================================================= */

typedef struct{
    uint8_t finger[5];
    int16_t wrist_pitch;
    int16_t wrist_roll;
    uint8_t flags;
} glove_packet_t;

/* ========================================================= */
/* FILTER VARIABLES                                          */
/* ========================================================= */

static uint16_t filtered_adc[5] = {0};
static uint8_t stable_output[5] = {0};

/* ========================================================= */
/* GPIO INIT                                                 */
/* ========================================================= */

static void gpio_init_custom(void){

    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN, 0);

    gpio_reset_pin(VIBRATION_PIN);
    gpio_set_direction(VIBRATION_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(VIBRATION_PIN, 0);

    gpio_config_t button_config = {
        .pin_bit_mask =
            (1ULL << BUTTON_CALIB_PIN) |
            (1ULL << BUTTON_EXTRA_PIN),

        .mode = GPIO_MODE_INPUT,

        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,

        .intr_type = GPIO_INTR_DISABLE
    };

    gpio_config(&button_config);
}

/* ========================================================= */
/* LED BLINK                                                 */
/* ========================================================= */

static void led_blink(void){

    gpio_set_level(LED_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(LED_PIN, 0);
}

/* ========================================================= */
/* WIFI INIT                                                 */
/* ========================================================= */

static void wifi_init(void){

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi inicializado");
}

/* ========================================================= */
/* ESPNOW CALLBACK                                           */
/* ========================================================= */

static void on_data_sent(const uint8_t *mac_addr, esp_now_send_status_t status){

    if(status == ESP_NOW_SEND_SUCCESS){
        led_blink();
    }

    ESP_LOGI(TAG,
            "ESP-NOW -> %s",
            status == ESP_NOW_SEND_SUCCESS ? "OK" : "FALLO");
}

/* ========================================================= */
/* ESPNOW INIT                                               */
/* ========================================================= */

static void espnow_init(void){

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(on_data_sent));

    esp_now_peer_info_t peer = {0};

    memcpy(peer.peer_addr, peer_receiver, ESP_NOW_ETH_ALEN);

    peer.channel = 0;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;

    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    ESP_LOGI(TAG, "ESP-NOW inicializado");
}

/* ========================================================= */
/* ADC INIT                                                  */
/* ========================================================= */

static void adc_init_custom(void){

    adc1_config_width(ADC_WIDTH_BIT_12);

    adc1_config_channel_atten(POT_THUMB,  ADC_ATTEN_DB_11);
    adc1_config_channel_atten(POT_INDEX,  ADC_ATTEN_DB_11);
    adc1_config_channel_atten(POT_MIDDLE, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(POT_RING,   ADC_ATTEN_DB_11);
    adc1_config_channel_atten(POT_PINKY,  ADC_ATTEN_DB_11);
}

/* ========================================================= */
/* NORMALIZE VALUE                                           */
/* ========================================================= */

static uint8_t normalize_value(uint16_t raw, uint16_t min, uint16_t max){

    if(raw <= min){
        return 0;
    }

    if(raw >= max){
        return 100;
    }

    return (uint8_t)(((raw - min) * 100) / (max - min));
}

/* ========================================================= */
/* PROCESS FINGER                                            */
/* ========================================================= */

static uint8_t process_finger(uint8_t index,
                              adc1_channel_t channel){

    uint16_t raw = adc1_get_raw(channel);

    if(filtered_adc[index] == 0){
        filtered_adc[index] = raw;
    }

    filtered_adc[index] =
        ((filtered_adc[index] * (EMA_DIVIDER - 1)) + raw)
        / EMA_DIVIDER;

    uint8_t normalized =
        normalize_value(
            filtered_adc[index],
            finger_min[index],
            finger_max[index]
        );

    if(abs(normalized - stable_output[index]) >= DEADZONE){
        stable_output[index] = normalized;
    }

    return stable_output[index];
}

/* ========================================================= */
/* MAIN                                                      */
/* ========================================================= */

void app_main(void){

    esp_err_t ret = nvs_flash_init();

    if(ret == ESP_ERR_NVS_NO_FREE_PAGES ||
       ret == ESP_ERR_NVS_NEW_VERSION_FOUND){

        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);

    gpio_init_custom();

    wifi_init();
    espnow_init();

    adc_init_custom();

    ESP_LOGI(TAG, "Sistema iniciado");

    while(1){

        glove_packet_t packet;

        packet.finger[0] =
            process_finger(0, POT_THUMB);

        packet.finger[1] =
            process_finger(1, POT_INDEX);

        packet.finger[2] =
            process_finger(2, POT_MIDDLE);

        packet.finger[3] =
            process_finger(3, POT_RING);

        packet.finger[4] =
            process_finger(4, POT_PINKY);

        packet.wrist_pitch = 0;
        packet.wrist_roll  = 0;

        packet.flags = 0;

        esp_err_t result =
            esp_now_send(
                peer_receiver,
                (uint8_t *)&packet,
                sizeof(packet)
            );

        if(result == ESP_OK){

            ESP_LOGI(TAG,
                    "T:%3u I:%3u M:%3u R:%3u P:%3u",
                    packet.finger[0],
                    packet.finger[1],
                    packet.finger[2],
                    packet.finger[3],
                    packet.finger[4]);
        }
        else{

            ESP_LOGW(TAG,
                    "Error ESP-NOW: %s",
                    esp_err_to_name(result));
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

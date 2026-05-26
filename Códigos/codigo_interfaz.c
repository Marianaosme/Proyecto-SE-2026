#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nvs_flash.h"
#include "esp_log.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_spp_api.h"

#include "driver/ledc.h"

#define SERVO_FREQ_HZ        50
#define SERVO_MIN_US         500    
#define SERVO_MAX_US         2500   
#define SERVO_PERIODO_US     20000  
#define LEDC_MAX_DUTY        65535  

#define PIN_PULGAR    13
#define PIN_INDICE    26
#define PIN_MEDIO     27
#define PIN_ANULAR    25
#define PIN_MENIQUE   33
#define PIN_MUNECA    12   

#define CH_PULGAR    LEDC_CHANNEL_0
#define CH_INDICE    LEDC_CHANNEL_1
#define CH_MEDIO     LEDC_CHANNEL_2
#define CH_ANULAR    LEDC_CHANNEL_3
#define CH_MENIQUE   LEDC_CHANNEL_4
#define CH_MUNECA    LEDC_CHANNEL_5

static const char *TAG = "ROBOT_HAND";

static int ang_pulgar  = 90;
static int ang_indice  = 90;
static int ang_medio   = 90;
static int ang_anular  = 90;
static int ang_menique = 90;
static int ang_muneca  = 90;

static uint32_t spp_handle  = 0;
static bool     bt_conectado = false;

static char dedo_actual      = 0;
static bool esperando_angulo = false;

static uint32_t angulo_a_duty(int angulo)
{
    if (angulo < 0)   angulo = 0;
    if (angulo > 180) angulo = 180;

    uint32_t pulso_us = (uint32_t)SERVO_MIN_US +
        ((uint32_t)(SERVO_MAX_US - SERVO_MIN_US) * (uint32_t)angulo) / 180U;

    return (pulso_us * (uint32_t)LEDC_MAX_DUTY) / (uint32_t)SERVO_PERIODO_US;
}

static void servo_mover(ledc_channel_t canal, int *ang_actual, int nuevo_ang)
{
    if (nuevo_ang < 0)   nuevo_ang = 0;
    if (nuevo_ang > 180) nuevo_ang = 180;

    *ang_actual = nuevo_ang;

    ledc_set_duty(LEDC_HIGH_SPEED_MODE, canal, angulo_a_duty(nuevo_ang));
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, canal);
}

static void servos_init(void)
{

    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_HIGH_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_16_BIT,
        .freq_hz         = SERVO_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_cfg);

    const int pines[6]            = { PIN_PULGAR, PIN_INDICE, PIN_MEDIO,
                                      PIN_ANULAR,  PIN_MENIQUE, PIN_MUNECA };
    const ledc_channel_t canales[6] = { CH_PULGAR, CH_INDICE, CH_MEDIO,
                                        CH_ANULAR,  CH_MENIQUE, CH_MUNECA };

    ledc_channel_config_t ch_cfg = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_sel  = LEDC_TIMER_0,
        .intr_type  = LEDC_INTR_DISABLE,
        .duty       = angulo_a_duty(90),  
        .hpoint     = 0
    };

    for (int i = 0; i < 6; i++) {
        ch_cfg.gpio_num = pines[i];
        ch_cfg.channel  = (ledc_channel_t)canales[i];
        ledc_channel_config(&ch_cfg);
    }

    ESP_LOGI(TAG, "Servos inicializados — posicion central (90 grados)");
}

static void procesar_byte(uint8_t byte_rx)
{
    if (!esperando_angulo) {

        if (byte_rx == 'p' || byte_rx == 'i' || byte_rx == 'm' ||
            byte_rx == 'a' || byte_rx == 'n' || byte_rx == 'w') {
            dedo_actual      = (char)byte_rx;
            esperando_angulo = true;
        }

    } else {

        int angulo       = (int)byte_rx;   
        esperando_angulo = false;

        switch (dedo_actual) {

            case 'p':   
                servo_mover(CH_PULGAR, &ang_pulgar, angulo);

                if (bt_conectado) {
                    char respuesta[16];
                    int  n = snprintf(respuesta, sizeof(respuesta),
                                      "%d:\n", ang_pulgar);
                    esp_spp_write(spp_handle, (int16_t)n,
                                  (uint8_t *)respuesta);
                }
                ESP_LOGI(TAG, "Pulgar  → %d grados", ang_pulgar);
                break;

            case 'i':   
                servo_mover(CH_INDICE, &ang_indice, angulo);
                ESP_LOGI(TAG, "Indice  → %d grados", ang_indice);
                break;

            case 'm':   
                servo_mover(CH_MEDIO, &ang_medio, angulo);
                ESP_LOGI(TAG, "Medio   → %d grados", ang_medio);
                break;

            case 'a':   
                servo_mover(CH_ANULAR, &ang_anular, angulo);
                ESP_LOGI(TAG, "Anular  → %d grados", ang_anular);
                break;

            case 'n':   
                servo_mover(CH_MENIQUE, &ang_menique, angulo);
                ESP_LOGI(TAG, "Menique → %d grados", ang_menique);
                break;

            case 'w':   
                servo_mover(CH_MUNECA, &ang_muneca, angulo);
                ESP_LOGI(TAG, "Muneca  → %d grados", ang_muneca);
                break;

            default:
                break;
        }
    }
}

static void spp_callback(esp_spp_cb_event_t evento,
                          esp_spp_cb_param_t *param)
{
    switch (evento) {

        case ESP_SPP_INIT_EVT:

            esp_bt_dev_set_device_name("ROBOT_HAND");
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE,
                                     ESP_BT_GENERAL_DISCOVERABLE);
            esp_spp_start_srv(ESP_SPP_SEC_NONE,
                              ESP_SPP_ROLE_SLAVE, 0, "SPP_SERVER");
            ESP_LOGI(TAG, "Bluetooth listo — buscame como ROBOT_HAND");
            break;

        case ESP_SPP_SRV_OPEN_EVT:

            spp_handle   = param->srv_open.handle;
            bt_conectado = true;
            ESP_LOGI(TAG, "App conectada");
            break;

        case ESP_SPP_CLOSE_EVT:

            bt_conectado = false;
            ESP_LOGI(TAG, "App desconectada");
            break;

        case ESP_SPP_DATA_IND_EVT: {

            uint8_t  *datos = param->data_ind.data;
            uint16_t  largo = param->data_ind.len;

            for (uint16_t i = 0; i < largo; i++) {
                procesar_byte(datos[i]);
            }
            break;
        }

        default:
            break;
    }
}

void app_main(void)
{

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);

    esp_bluedroid_init();
    esp_bluedroid_enable();

    esp_spp_register_callback(spp_callback);
    esp_spp_init(ESP_SPP_MODE_CB);

    servos_init();

    ESP_LOGI(TAG, "=== Mano Robot lista ===");

}

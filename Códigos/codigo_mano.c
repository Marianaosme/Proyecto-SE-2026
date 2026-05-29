// =============================================================================
//  MANO ROBÓTICA — Código v3.3 (5 dedos + muñeca) — Receptor ESP-NOW
//
//  CAMBIOS v3.3 respecto a v3.2:
//  ─────────────────────────────────────────────────────────────────────────────
//  [ESPNOW-1] Se eliminan lecturas ADC de potenciómetros y acelerómetro local.
//             La fuente de datos es ahora el paquete recibido por ESP-NOW.
//  [ESPNOW-2] Se agrega callback on_data_recv() que actualiza los porcentajes
//             objetivo (pct1–5, pctA) desde el paquete del guante.
//  [ESPNOW-3] Variables de porcentaje objetivo son volatile para acceso seguro
//             desde la ISR de ESP-NOW.
//  [ESPNOW-4] Se mantiene todo el pipeline: smooth_step → pct_to_us → PWM.
//  [ESPNOW-5] Se agrega watchdog de comunicación: si no llegan datos en
//             COMM_TIMEOUT_MS ms, se loguea advertencia.
//  [ESPNOW-6] Se conserva el sistema de logging y códigos de error v3.2.
//  [ESPNOW-7] Se eliminan adc_init(), adc1_read_safe(), adc2_read_safe()
//             ya que no se usa ADC en la mano receptora.
//
//  PINOUT MANO v3.3:
//  ─────────────────────────────────────────────────────────────────────────────
//  GPIO 23 → Servo 1 (Pulgar)
//  GPIO 21 → Servo 2 (Índice)
//  GPIO 18 → Servo 3 (Medio)
//  GPIO 16 → Servo 4 (Anular)
//  GPIO 25 → Servo 5 (Meñique)
//  GPIO 26 → Servo A (Muñeca)
//  GPIO  2 → LED indicador de recepción
//
//  ESTRUCTURA DEL PAQUETE (debe coincidir con el guante):
//    typedef struct { uint8_t fingers[5]; uint8_t wrist; } glove_packet_t;
// =============================================================================

// =============================================================================
//  1. LIBRERÍAS
// =============================================================================
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "driver/gpio.h"
extern void ets_delay_us(uint32_t us);

// =============================================================================
//  2. PINES GPIO
// =============================================================================
/* ---- Servos (GPIO salida) ---- */
#define SERVO1_GPIO     GPIO_NUM_23         /* Pulgar                     */
#define SERVO2_GPIO     GPIO_NUM_21         /* Índice                     */
#define SERVO3_GPIO     GPIO_NUM_18         /* Medio                      */
#define SERVO4_GPIO     GPIO_NUM_16         /* Anular                     */
#define SERVO5_GPIO     GPIO_NUM_25         /* Meñique                    */
#define SERVOA_GPIO     GPIO_NUM_26         /* Muñeca                     */
/* ---- LED indicador ---- */
#define LED_PIN         GPIO_NUM_2

// =============================================================================
//  3. CONSTANTES Y CONFIGURACIÓN
// =============================================================================
static const char *TAG = "MANO";

/* ---- Códigos de error estructurados ---- */
#define ERR_NONE                 0
#define ERR_SERVO_INIT_FAIL     -4

/* ---- Watchdog de comunicación ---- */
#define COMM_TIMEOUT_MS         500     /* [ESPNOW-5] Alarma si no hay datos en 500 ms */

/* ---- Servo PWM por software ---- */
#define SERVO_PERIOD_US         20000u
#define STD_MIN_US              544u
#define STD_MAX_US              2400u
#define ACCEL_MIN_US            500u
#define ACCEL_MAX_US            2500u
#define MY_PI                   3.14159265f
#define NUM_SERVOS              6

/* ---- [PCT-3] Velocidad de movimiento suave (porcentaje por ciclo) ----
 *
 *  Porcentaje máximo que un servo puede moverse por ciclo (cada 20 ms).
 *
 *  Ejemplo con SERVO_STEP_PCT = 2:
 *    De 0% a 100% tarda: 100/2 = 50 ciclos × 20 ms = 1 segundo
 */
#define SERVO_STEP_PCT      5       /* [PCT-3] Dedos: 5% por ciclo  */
#define SERVOA_STEP_PCT     3       /* [PCT-3] Muñeca: 3% por ciclo */

/* ---- Rangos de ángulo de cada servo ----
 *  0% siempre corresponde a ANG_MIN
 *  100% siempre corresponde a ANG_MAX
 */
#define SERVO1_ANG_MIN      2
#define SERVO1_ANG_MAX      30
#define SERVO2_ANG_MIN      70
#define SERVO2_ANG_MAX      0
#define SERVO3_ANG_MIN      85
#define SERVO3_ANG_MAX      160
#define SERVO4_ANG_MIN      100
#define SERVO4_ANG_MAX      170
#define SERVO5_ANG_MIN      100
#define SERVO5_ANG_MAX      210

// =============================================================================
//  4. ESTRUCTURA DE PAQUETE (igual que en el GUANTE)
// =============================================================================
typedef struct {
    uint8_t fingers[5];    /* Porcentaje 0–100 de cada dedo (pulgar→meñique) */
    uint8_t wrist;         /* Porcentaje 0–100 de la muñeca                  */
} glove_packet_t;

// =============================================================================
//  5. VARIABLES GLOBALES
// =============================================================================

/* ---- [ESPNOW-3] Porcentaje OBJETIVO recibido del guante (volatile) ---- */
static volatile int pct1 = 0;
static volatile int pct2 = 0;
static volatile int pct3 = 0;
static volatile int pct4 = 0;
static volatile int pct5 = 0;
static volatile int pctA = 50;

/* ---- [PCT-2] Porcentaje ACTUAL de cada dedo (posición real, 0–100) ---- */
static int cur_pct1 = 0;
static int cur_pct2 = 0;
static int cur_pct3 = 0;
static int cur_pct4 = 0;
static int cur_pct5 = 0;
static int cur_pctA = 50;       /* Muñeca arranca en 50% (centro) */

/* ---- Anchos de pulso (compartidos con tarea PWM) ---- */
static volatile uint32_t servo1_pw = 0;
static volatile uint32_t servo2_pw = 0;
static volatile uint32_t servo3_pw = 0;
static volatile uint32_t servo4_pw = 0;
static volatile uint32_t servo5_pw = 0;
static volatile uint32_t servoA_pw = 0;

/* ---- [ESPNOW-5] Timestamp del último paquete recibido ---- */
static volatile int64_t last_recv_us = 0;

/* ---- Contadores de estado ---- */
static uint32_t packets_received = 0;
static uint32_t loop_count       = 0;

// =============================================================================
//  6. FUNCIONES AUXILIARES
// =============================================================================

/* ───────────────── Logging con timestamp ───────────────── */
static uint32_t get_timestamp_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static void log_system_status(void)
{
    ESP_LOGI(TAG, "[%lu ms] === ESTADO DEL SISTEMA ===", get_timestamp_ms());
    ESP_LOGI(TAG, "  Ciclos ejecutados   : %lu", loop_count);
    ESP_LOGI(TAG, "  Paquetes recibidos  : %lu", packets_received);
    ESP_LOGI(TAG, "  Posición actual     : S1:%d%% S2:%d%% S3:%d%% S4:%d%% S5:%d%% SA:%d%%",
             cur_pct1, cur_pct2, cur_pct3, cur_pct4, cur_pct5, cur_pctA);
    ESP_LOGI(TAG, "  ========================");
}

/* ───────────────── Utilidades matemáticas ───────────────── */
static int map_val(int x, int in_min, int in_max, int out_min, int out_max)
{
    return (int)((long)(x - in_min) * (out_max - out_min)
                 / (in_max - in_min) + out_min);
}

static int constrain_i(int x, int lo, int hi)
{
    return (x < lo) ? lo : (x > hi) ? hi : x;
}

static uint32_t angle_to_us(int angle, uint32_t min_us, uint32_t max_us)
{
    uint32_t pulse = min_us +
           (uint32_t)((float)angle / 180.0f * (float)(max_us - min_us));
    if (pulse < min_us) pulse = min_us;
    if (pulse > max_us) pulse = max_us;
    return pulse;
}

/* ───────────────── [PCT-4] Conversión porcentaje → pulso ───────────────── */
/**
 * Convierte un porcentaje (0–100) directamente a pulso PWM en µs.
 *
 *  Flujo:  pct (0–100) → ángulo (ang_min – ang_max) → pulso (us_min – us_max)
 */
static uint32_t pct_to_us(int pct, int ang_min, int ang_max,
                           uint32_t us_min, uint32_t us_max)
{
    int angle = map_val(pct, 0, 100, ang_min, ang_max);
    return angle_to_us(angle, us_min, us_max);
}

/* ───────────────── Movimiento suave ───────────────── */
/**
 * Mueve 'current' hacia 'target' un máximo de 'step' por llamada.
 * Opera sobre porcentajes (0–100).
 */
static int smooth_step(int current, int target, int step)
{
    int diff = target - current;
    if (diff > step)
        return current + step;
    else if (diff < -step)
        return current - step;
    else
        return target;
}

/* ───────────────── LED ───────────────── */
static void led_init(void)
{
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN, 0);
}


/* ───────────────── Inicialización GPIO servos ───────────────── */
static int gpio_init_servos(void)
{
    ESP_LOGI(TAG, "[%lu ms] INFO: Configurando GPIOs de servo...", get_timestamp_ms());
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << SERVO1_GPIO) | (1ULL << SERVO2_GPIO) |
                        (1ULL << SERVO3_GPIO) | (1ULL << SERVO4_GPIO) |
                        (1ULL << SERVO5_GPIO) | (1ULL << SERVOA_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    esp_err_t ret = gpio_config(&io);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%lu ms] ERR_SERVO_INIT_FAIL: gpio_config falló (ret=%d)",
                 get_timestamp_ms(), ret);
        return ERR_SERVO_INIT_FAIL;
    }
    ESP_LOGI(TAG, "[%lu ms] INFO: GPIOs servo OK", get_timestamp_ms());
    return ERR_NONE;
}

// =============================================================================
//  7. ESP-NOW
// =============================================================================

/* ---- [ESPNOW-2] Callback de recepción ---- */
/**
 * Se ejecuta en contexto de tarea WiFi (no en ISR).
 * Actualiza las variables volatile de porcentaje objetivo
 * y registra el timestamp para el watchdog.
 */
static void on_data_recv(const esp_now_recv_info_t *recv_info,
                          const uint8_t *data,
                          int data_len)
{
    const uint8_t *mac_addr = recv_info->src_addr;  /* ← extraer MAC de aquí */
    if (data_len != sizeof(glove_packet_t)) {
        ESP_LOGW(TAG, "[%lu ms] WARN: Paquete de tamaño inesperado (%d bytes, esperado %d)",
                 get_timestamp_ms(), data_len, (int)sizeof(glove_packet_t));
        return;
    }

    glove_packet_t pkt;
    memcpy(&pkt, data, sizeof(glove_packet_t));

    /* [ESPNOW-3] Actualizar porcentajes objetivo de forma atómica */
    pct1 = constrain_i(pkt.fingers[0], 0, 100);
    pct2 = constrain_i(pkt.fingers[1], 0, 100);
    pct3 = constrain_i(pkt.fingers[2], 0, 100);
    pct4 = constrain_i(pkt.fingers[3], 0, 100);
    pct5 = constrain_i(pkt.fingers[4], 0, 100);
    pctA = constrain_i(pkt.wrist,       0, 100);

    /* [ESPNOW-5] Actualizar watchdog */
    last_recv_us = esp_timer_get_time();
    packets_received++;

    /* Parpadeo LED para confirmar recepción */
    gpio_set_level(LED_PIN, 1);   /* Se apagará en el loop principal */

    ESP_LOGD(TAG,
             "[%lu ms] RX T:%3u I:%3u M:%3u R:%3u P:%3u | W:%3u  (MAC "
             "%02X:%02X:%02X:%02X:%02X:%02X)",
             get_timestamp_ms(),
             pkt.fingers[0], pkt.fingers[1], pkt.fingers[2],
             pkt.fingers[3], pkt.fingers[4], pkt.wrist,
             mac_addr[0], mac_addr[1], mac_addr[2],
             mac_addr[3], mac_addr[4], mac_addr[5]);
}

/* ---- Inicialización WiFi (necesaria para ESP-NOW) ---- */
static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "[%lu ms] INFO: WiFi inicializado (modo STA)", get_timestamp_ms());
}

/* ---- Inicialización ESP-NOW ---- */
static void espnow_init(void)
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv));
    ESP_LOGI(TAG, "[%lu ms] INFO: ESP-NOW inicializado — esperando paquetes del guante",
             get_timestamp_ms());
}

// =============================================================================
//  8. TAREA PWM SERVO (core 1)
// =============================================================================
typedef struct {
    gpio_num_t gpio;
    uint32_t   pulse_us;
} servo_info_t;

static void sort_servos(servo_info_t *s, int n)
{
    for (int i = 0; i < n - 1; i++) {
        for (int j = i + 1; j < n; j++) {
            if (s[j].pulse_us < s[i].pulse_us) {
                servo_info_t tmp = s[i];
                s[i] = s[j];
                s[j] = tmp;
            }
        }
    }
}

static void servo_pwm_task(void *arg)
{
    ESP_LOGI(TAG, "[%lu ms] INFO: Tarea PWM servo iniciada en core 1 (%d servos)",
             get_timestamp_ms(), NUM_SERVOS);

    while (1) {
        int64_t t_frame_start = esp_timer_get_time();

        servo_info_t servos[NUM_SERVOS] = {
            { SERVO1_GPIO, servo1_pw },
            { SERVO2_GPIO, servo2_pw },
            { SERVO3_GPIO, servo3_pw },
            { SERVO4_GPIO, servo4_pw },
            { SERVO5_GPIO, servo5_pw },
            { SERVOA_GPIO, servoA_pw }
        };

        sort_servos(servos, NUM_SERVOS);

        for (int i = 0; i < NUM_SERVOS; i++) {
            if (servos[i].pulse_us < STD_MIN_US)
                servos[i].pulse_us = STD_MIN_US;
            if (servos[i].pulse_us > ACCEL_MAX_US)
                servos[i].pulse_us = ACCEL_MAX_US;
        }

        portDISABLE_INTERRUPTS();
        for (int i = 0; i < NUM_SERVOS; i++)
            gpio_set_level(servos[i].gpio, 1);

        uint32_t elapsed = 0;
        for (int i = 0; i < NUM_SERVOS; i++) {
            uint32_t wait = servos[i].pulse_us - elapsed;
            if (wait > 0)
                ets_delay_us(wait);
            gpio_set_level(servos[i].gpio, 0);
            elapsed = servos[i].pulse_us;
        }
        portENABLE_INTERRUPTS();

        int64_t t_active     = esp_timer_get_time() - t_frame_start;
        int64_t remaining_us = (int64_t)SERVO_PERIOD_US - t_active;

        if (remaining_us > 1500) {
            uint32_t sleep_ms = (uint32_t)((remaining_us - 500) / 1000);
            if (sleep_ms > 0)
                vTaskDelay(pdMS_TO_TICKS(sleep_ms));
            while ((esp_timer_get_time() - t_frame_start) < (int64_t)SERVO_PERIOD_US)
                ;
        } else if (remaining_us > 0) {
            ets_delay_us((uint32_t)remaining_us);
        }  
    }
}

// =============================================================================
//  9. APP_MAIN
// =============================================================================
void app_main(void)
{
    /* ─── Inicialización NVS ─── */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "=============================================");
    ESP_LOGI(TAG, "  MANO ROBÓTICA — Sistema de Control v3.3");
    ESP_LOGI(TAG, "  5 dedos + muñeca | Receptor ESP-NOW");
    ESP_LOGI(TAG, "  Paso por ciclo: %d%% (dedos) %d%% (muñeca)",
             SERVO_STEP_PCT, SERVOA_STEP_PCT);
    ESP_LOGI(TAG, "  Inicio: timestamp %lu ms", get_timestamp_ms());
    ESP_LOGI(TAG, "=============================================");

    /* ─── GPIO ─── */
    led_init();

    if (gpio_init_servos() != ERR_NONE) {
        ESP_LOGE(TAG, "[%lu ms] ERROR CRÍTICO: Fallo en GPIO. Sistema detenido.",
                 get_timestamp_ms());
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    /* ─── WiFi + ESP-NOW ─── */
    wifi_init();
    espnow_init();

    /* ─── [PCT-1] Posiciones iniciales: dedos 0%, muñeca 50% ─── */
    cur_pct1 = 0;  cur_pct2 = 0;  cur_pct3 = 0;
    cur_pct4 = 0;  cur_pct5 = 0;  cur_pctA = 50;

    servo1_pw = pct_to_us(cur_pct1, SERVO1_ANG_MIN, SERVO1_ANG_MAX, STD_MIN_US,   STD_MAX_US);
    servo2_pw = pct_to_us(cur_pct2, SERVO2_ANG_MIN, SERVO2_ANG_MAX, STD_MIN_US,   STD_MAX_US);
    servo3_pw = pct_to_us(cur_pct3, SERVO3_ANG_MIN, SERVO3_ANG_MAX, STD_MIN_US,   STD_MAX_US);
    servo4_pw = pct_to_us(cur_pct4, SERVO4_ANG_MIN, SERVO4_ANG_MAX, STD_MIN_US,   STD_MAX_US);
    servo5_pw = pct_to_us(cur_pct5, SERVO5_ANG_MIN, SERVO5_ANG_MAX, STD_MIN_US,   STD_MAX_US);
    servoA_pw = pct_to_us(cur_pctA, 0,              180,             ACCEL_MIN_US, ACCEL_MAX_US);

    ESP_LOGI(TAG, "[%lu ms] INFO: Posiciones iniciales → S1:0%% S2:0%% S3:0%% S4:0%% S5:0%% SA:50%%",
             get_timestamp_ms());

    /* Inicializar watchdog con el tiempo actual */
    last_recv_us = esp_timer_get_time();

    /* ─── Tarea PWM en core 1 ─── */
    xTaskCreatePinnedToCore(servo_pwm_task, "servo_pwm",
                            4096, NULL, 5, NULL, 1);

    ESP_LOGI(TAG, "[%lu ms] INFO: Sistema listo. Esperando datos del guante.",
             get_timestamp_ms());

    /* =========================================================
     *              BUCLE PRINCIPAL — LÓGICA
     * =========================================================
     *
     *  [ESPNOW-4] Flujo de control (equivalente al v3.2 pero
     *  la fuente de pct1–5 y pctA es el callback ESP-NOW,
     *  no el ADC local):
     *
     *    Guante ESP-NOW → pct OBJETIVO (volatile, 0–100)
     *                          ↓ smooth_step
     *                     cur_pct ACTUAL (0–100)
     *                          ↓ pct_to_us()
     *                     ángulo → pulso µs → servo
     * ========================================================= */
    while (1) {
        loop_count++;

        /* ──── Leer porcentajes objetivo (snapshot atómico) ──── */
        int target1 = pct1;
        int target2 = pct2;
        int target3 = pct3;
        int target4 = pct4;
        int target5 = pct5;
        int targetA = pctA;

        /* ──── [ESPNOW-5] Watchdog de comunicación ──── */
        int64_t now_us       = esp_timer_get_time();
        int64_t elapsed_ms   = (now_us - last_recv_us) / 1000LL;
        if (elapsed_ms > COMM_TIMEOUT_MS) {
            ESP_LOGW(TAG,
                     "[%lu ms] WARN: Sin datos del guante desde hace %lld ms",
                     get_timestamp_ms(), elapsed_ms);
        }

        /* ──── [PCT-2] Mover porcentaje ACTUAL hacia OBJETIVO ──── */
        cur_pct1 = smooth_step(cur_pct1, target1, SERVO_STEP_PCT);
        cur_pct2 = smooth_step(cur_pct2, target2, SERVO_STEP_PCT);
        cur_pct3 = smooth_step(cur_pct3, target3, SERVO_STEP_PCT);
        cur_pct4 = smooth_step(cur_pct4, target4, SERVO_STEP_PCT);
        cur_pct5 = smooth_step(cur_pct5, target5, SERVO_STEP_PCT);
        cur_pctA = smooth_step(cur_pctA, targetA, SERVOA_STEP_PCT);

        /* ──── [PCT-4] Porcentaje ACTUAL → Pulso PWM ──── */
        servo1_pw = pct_to_us(cur_pct1, SERVO1_ANG_MIN, SERVO1_ANG_MAX, STD_MIN_US,   STD_MAX_US);
        servo2_pw = pct_to_us(cur_pct2, SERVO2_ANG_MIN, SERVO2_ANG_MAX, STD_MIN_US,   STD_MAX_US);
        servo3_pw = pct_to_us(cur_pct3, SERVO3_ANG_MIN, SERVO3_ANG_MAX, STD_MIN_US,   STD_MAX_US);
        servo4_pw = pct_to_us(cur_pct4, SERVO4_ANG_MIN, SERVO4_ANG_MAX, STD_MIN_US,   STD_MAX_US);
        servo5_pw = pct_to_us(cur_pct5, SERVO5_ANG_MIN, SERVO5_ANG_MAX, STD_MIN_US,   STD_MAX_US);
        servoA_pw = pct_to_us(cur_pctA, 0,              180,             ACCEL_MIN_US, ACCEL_MAX_US);

        /* Apagar LED (fue encendido en on_data_recv) */
        gpio_set_level(LED_PIN, 0);

        /* ──── [PCT-6] Log: porcentaje objetivo y actual ──── */
        ESP_LOGI(TAG,
                 "[%lu ms] OBJ→ %d%% %d%% %d%% %d%% %d%% W:%d%% | "
                 "ACT→ S1:%d%% S2:%d%% S3:%d%% S4:%d%% S5:%d%% SA:%d%%",
                 get_timestamp_ms(),
                 target1, target2, target3, target4, target5, targetA,
                 cur_pct1, cur_pct2, cur_pct3, cur_pct4, cur_pct5, cur_pctA);

        /* ──── Reporte periódico cada 500 ciclos ──── */
        if (loop_count % 500 == 0) {
            log_system_status();
        }

        /* ──── Delay de ciclo (20 ms = 50 Hz) ──── */
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

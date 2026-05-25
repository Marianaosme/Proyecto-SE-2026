// =============================================================================
//  MANO ROBÓTICA — Código v3 (5 dedos + muñeca)
//
//  CAMBIOS v3:
//  ─────────────────────────────────────────────────────────────────────────────
//  [ADD-1] Servo 4 (anular):  POT → GPIO 39 (ADC1_CH3), Servo → GPIO 16
//  [ADD-2] Servo 5 (meñique): POT → GPIO 14 (ADC2_CH6), Servo → GPIO 17
//  [ADD-3] servo_pwm_task actualizada para manejar 6 servos
//  [ADD-4] Reincorporada adc2_read_safe() para POT5
//
//  FIXES PREVIOS:
//  [FIX-1] SERVO3_GPIO: GPIO 12 → GPIO 18 (evita JTAG MTCK)
//  [FIX-2] SERVO2_GPIO: GPIO 27 → GPIO 19 (pin seguro)
//  [FIX-3] servo_pwm_task: timing con esp_timer_get_time (tiempo real)
//  [FIX-4] Guardia de pulso mínimo en servo_pwm_task
//  [FIX-5] constrain_i hardcodeado → usa defines con CONSTRAIN_AUTO
// =============================================================================

// =============================================================================
//  1. LIBRERÍAS
// =============================================================================
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/adc.h"
extern void ets_delay_us(uint32_t us);

// =============================================================================
//  2. PINES GPIO
// =============================================================================

/* ---- Potenciómetros (ADC) ---- */
#define POT1_CH         ADC1_CHANNEL_4      /* GPIO 32 — pulgar           */
#define POT2_CH         ADC1_CHANNEL_5      /* GPIO 33 — índice           */
#define POT3_CH         ADC1_CHANNEL_0      /* GPIO 36 — medio            */
#define POT4_CH         ADC1_CHANNEL_3      /* GPIO 39 — anular   [ADD-1] */
#define POT5_CH         ADC2_CHANNEL_6      /* GPIO 14 — meñique  [ADD-2] ⚠ ADC2 */

/* ---- Acelerómetro (ADC) ---- */
#define ACCEL_X_CH      ADC1_CHANNEL_6      /* GPIO 34                    */
#define ACCEL_Y_CH      ADC1_CHANNEL_7      /* GPIO 35                    */

/* ---- Servos (GPIO salida) ---- */
#define SERVO1_GPIO     GPIO_NUM_26         /* Pulgar                     */
#define SERVO2_GPIO     GPIO_NUM_19         /* Índice   [FIX-2]           */
#define SERVO3_GPIO     GPIO_NUM_18         /* Medio    [FIX-1]           */
#define SERVO4_GPIO     GPIO_NUM_16         /* Anular   [ADD-1]           */
#define SERVO5_GPIO     GPIO_NUM_17         /* Meñique  [ADD-2]           */
#define SERVOA_GPIO     GPIO_NUM_25         /* Muñeca (acelerómetro)      */

// =============================================================================
//  3. VARIABLES
// =============================================================================
static const char *TAG = "MANO";

/* ---- Códigos de error estructurados ---- */
#define ERR_NONE                 0
#define ERR_SENSOR_TIMEOUT      -1
#define ERR_ADC2_READ_FAIL      -2
#define ERR_ADC_OUT_OF_RANGE    -3
#define ERR_SERVO_INIT_FAIL     -4
#define ERR_ADC_INIT_FAIL       -5

/* ---- Umbrales de diagnóstico ---- */
#define ADC_MIN_VALID           10
#define ADC_MAX_VALID           4085
#define ADC2_MAX_RETRIES        3
#define SENSOR_WARN_THRESHOLD   50

/* ---- Servo PWM por software ---- */
#define SERVO_PERIOD_US     20000u
#define STD_MIN_US          544u
#define STD_MAX_US          2400u
#define ACCEL_MIN_US        500u
#define ACCEL_MAX_US        2500u
#define MY_PI               3.14159265f

/* ---- Cantidad de servos para la tarea PWM ---- */
#define NUM_SERVOS          6           /* [ADD-3] antes era 4 */

/* ---- Rangos de ángulo de cada servo ---- */
/* ✅ Solo cambia estos defines y TODO se actualiza automáticamente */
#define SERVO1_ANG_MIN      2       /* Pulgar:  0% →  2°           */
#define SERVO1_ANG_MAX      30      /* Pulgar:  100% → 30°         */
#define SERVO2_ANG_MIN      70      /* Índice:  0% → 70°           */
#define SERVO2_ANG_MAX      0       /* Índice:  100% → 0° (invert) */
#define SERVO3_ANG_MIN      85      /* Medio:   0% → 85°           */
#define SERVO3_ANG_MAX      160     /* Medio:   100% → 160°        */
#define SERVO4_ANG_MIN      90      /* Anular:  0% → 90°   [ADD-1] */
#define SERVO4_ANG_MAX      150     /* Anular:  100% → 150° [ADD-1] */
#define SERVO5_ANG_MIN      90      /* Meñique: 0% → 90°   [ADD-2] */
#define SERVO5_ANG_MAX      150     /* Meñique: 100% → 150° [ADD-2] */

/* ---- [FIX-5] Constrain automático: maneja MIN > MAX (servos invertidos) ---- */
#define CONSTRAIN_AUTO(val, a, b) \
    constrain_i((val), ((a) < (b) ? (a) : (b)), ((a) > (b) ? (a) : (b)))

/* ---- Parámetros del acelerómetro ---- */
static const float ANG_OFFSET =   0.0f;
static const float ANG_MIN    = -60.0f;
static const float ANG_MAX    =  60.0f;

/* ---- Variables de lectura ADC ---- */
static int valPot1 = 0;
static int valPot2 = 0;
static int valPot3 = 0;
static int valPot4 = 0;    /* [ADD-1] */
static int valPot5 = 0;    /* [ADD-2] */

/* ---- Porcentaje de cada potenciómetro (0–100) ---- */
static int pct1 = 0;
static int pct2 = 0;
static int pct3 = 0;
static int pct4 = 0;       /* [ADD-1] */
static int pct5 = 0;       /* [ADD-2] */

/* ---- Ángulo resultante de cada servo ---- */
static int ang1 = 0;
static int ang2 = 0;
static int ang3 = 0;
static int ang4 = 0;       /* [ADD-1] */
static int ang5 = 0;       /* [ADD-2] */

/* ---- Anchos de pulso (compartidos con tarea PWM) ---- */
static volatile uint32_t servo1_pw = 0;
static volatile uint32_t servo2_pw = 0;
static volatile uint32_t servo3_pw = 0;
static volatile uint32_t servo4_pw = 0;     /* [ADD-1] */
static volatile uint32_t servo5_pw = 0;     /* [ADD-2] */
static volatile uint32_t servoA_pw = 0;

/* ---- Contadores de errores ---- */
static uint32_t err_adc2_count    = 0;      /* [ADD-4] */
static uint32_t err_timeout_count = 0;
static uint32_t warn_satur_count  = 0;
static uint32_t loop_count        = 0;

// =============================================================================
//  4. MÉTODOS (FUNCIONES AUXILIARES)
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
    ESP_LOGI(TAG, "  Errores ADC2        : %lu (ERR_ADC2_READ_FAIL)", err_adc2_count);
    ESP_LOGI(TAG, "  Errores timeout     : %lu (ERR_SENSOR_TIMEOUT)", err_timeout_count);
    ESP_LOGI(TAG, "  Advertencias satur. : %lu", warn_satur_count);
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

static float constrain_f(float x, float lo, float hi)
{
    return (x < lo) ? lo : (x > hi) ? hi : x;
}

static float fast_atan2(float y, float x)
{
    if (x == 0.0f && y == 0.0f) return 0.0f;
    float ax = (x < 0) ? -x : x;
    float ay = (y < 0) ? -y : y;
    float a  = (ax < ay) ? (ax / ay) : (ay / ax);
    float s  = a * a;
    float r  = ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;
    if (ay > ax) r = 1.57079637f - r;
    if (x  < 0)  r = MY_PI - r;
    if (y  < 0)  r = -r;
    return r;
}

static uint32_t angle_to_us(int angle, uint32_t min_us, uint32_t max_us)
{
    return min_us +
           (uint32_t)((float)angle / 180.0f * (float)(max_us - min_us));
}

/* ───────────────── Lectura ADC con validación ───────────────── */
static int adc1_read_safe(adc1_channel_t ch, int *out_val, const char *name)
{
    int raw = adc1_get_raw(ch);
    if (raw < 0) {
        ESP_LOGE(TAG, "[%lu ms] ERR_SENSOR_TIMEOUT: Fallo lectura ADC1 '%s'",
                 get_timestamp_ms(), name);
        err_timeout_count++;
        *out_val = 0;
        return ERR_SENSOR_TIMEOUT;
    }
    if (raw < ADC_MIN_VALID || raw > ADC_MAX_VALID) {
        ESP_LOGW(TAG, "[%lu ms] ADC_OUT_OF_RANGE: '%s' = %d (posible saturación)",
                 get_timestamp_ms(), name, raw);
        warn_satur_count++;
    }
    if (raw < SENSOR_WARN_THRESHOLD || raw > (4095 - SENSOR_WARN_THRESHOLD)) {
        ESP_LOGW(TAG, "[%lu ms] WARN: Sensor '%s' cerca del límite (raw=%d)",
                 get_timestamp_ms(), name, raw);
    }
    *out_val = raw;
    return ERR_NONE;
}

/* [ADD-4] Lectura ADC2 con reintentos (necesaria para POT5/meñique) */
static int adc2_read_safe(adc2_channel_t ch, int *out_val, const char *name)
{
    int raw = 0;
    esp_err_t ret;

    for (int i = 0; i < ADC2_MAX_RETRIES; i++) {
        ret = adc2_get_raw(ch, ADC_WIDTH_BIT_12, &raw);
        if (ret == ESP_OK) {
            if (raw < ADC_MIN_VALID || raw > ADC_MAX_VALID) {
                ESP_LOGW(TAG, "[%lu ms] ADC_OUT_OF_RANGE: '%s' = %d",
                         get_timestamp_ms(), name, raw);
                warn_satur_count++;
            }
            *out_val = raw;
            return ERR_NONE;
        }
        ets_delay_us(100);
    }

    ESP_LOGE(TAG, "[%lu ms] ERR_ADC2_READ_FAIL: '%s' falló tras %d intentos (ret=%d)",
             get_timestamp_ms(), name, ADC2_MAX_RETRIES, ret);
    err_adc2_count++;
    *out_val = 0;
    return ERR_ADC2_READ_FAIL;
}

/* ───────────────── Inicialización ADC ───────────────── */
static int adc_init(void)
{
    ESP_LOGI(TAG, "[%lu ms] INFO: Iniciando configuración ADC...", get_timestamp_ms());

    /* ADC1: POT1, POT2, POT3, POT4, ACCEL_X, ACCEL_Y */
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(POT1_CH,    ADC_ATTEN_DB_11);
    adc1_config_channel_atten(POT2_CH,    ADC_ATTEN_DB_11);
    adc1_config_channel_atten(POT3_CH,    ADC_ATTEN_DB_11);
    adc1_config_channel_atten(POT4_CH,    ADC_ATTEN_DB_11);   /* [ADD-1] */
    adc1_config_channel_atten(ACCEL_X_CH, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ACCEL_Y_CH, ADC_ATTEN_DB_11);
    ESP_LOGI(TAG, "[%lu ms] INFO: ADC1 configurado (12 bits, 11dB) — CH0, CH3, CH4, CH5, CH6, CH7",
             get_timestamp_ms());

    /* [ADD-4] ADC2: POT5 (meñique) */
    esp_err_t ret = adc2_config_channel_atten(POT5_CH, ADC_ATTEN_DB_11);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%lu ms] ERR_ADC_INIT_FAIL: ADC2_CH6 falló (ret=%d)",
                 get_timestamp_ms(), ret);
        return ERR_ADC_INIT_FAIL;
    }
    ESP_LOGI(TAG, "[%lu ms] INFO: ADC2 configurado (12 bits, 11dB) — CH6 (meñique)",
             get_timestamp_ms());

    return ERR_NONE;
}

/* ───────────────── Inicialización GPIO servos ───────────────── */
static int gpio_init_servos(void)
{
    ESP_LOGI(TAG, "[%lu ms] INFO: Configurando GPIOs de servo...", get_timestamp_ms());
    gpio_config_t io = {
        /* [ADD-3] Ahora incluye SERVO4 y SERVO5 */
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
    ESP_LOGI(TAG, "[%lu ms] INFO: GPIOs servo OK — 26(S1) 19(S2) 18(S3) 16(S4) 17(S5) 25(SA)",
             get_timestamp_ms());
    return ERR_NONE;
}

/* ───────────────── Servo PWM por software ───────────────── */
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
        /* [FIX-3] Marca el inicio real del frame */
        int64_t t_frame_start = esp_timer_get_time();

        /* [ADD-3] Captura instantánea — ahora 6 servos */
        servo_info_t servos[NUM_SERVOS] = {
            { SERVO1_GPIO, servo1_pw },
            { SERVO2_GPIO, servo2_pw },
            { SERVO3_GPIO, servo3_pw },
            { SERVO4_GPIO, servo4_pw },     /* [ADD-1] anular   */
            { SERVO5_GPIO, servo5_pw },     /* [ADD-2] meñique  */
            { SERVOA_GPIO, servoA_pw }
        };
        sort_servos(servos, NUM_SERVOS);

        /* [FIX-4] Guardia de pulso mínimo */
        for (int i = 0; i < NUM_SERVOS; i++) {
            if (servos[i].pulse_us < STD_MIN_US) {
                servos[i].pulse_us = STD_MIN_US;
            }
        }

        /* ── Fase activa: generar pulsos ── */
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

        /* ── Fase de espera: completar período de 20 ms ── */
        int64_t t_active = esp_timer_get_time() - t_frame_start;
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
//  5. APP_MAIN
// =============================================================================
void app_main(void)
{
    /* =============================================================
     *                    CONFIGURACIONES
     * ============================================================= */
    ESP_LOGI(TAG, "=============================================");
    ESP_LOGI(TAG, "  MANO ROBÓTICA — Sistema de Control v3");
    ESP_LOGI(TAG, "  5 dedos + muñeca (acelerómetro)");
    ESP_LOGI(TAG, "  Inicio: timestamp %lu ms", get_timestamp_ms());
    ESP_LOGI(TAG, "  Servos: S1=G26 S2=G19 S3=G18 S4=G16 S5=G17 SA=G25");
    ESP_LOGI(TAG, "  Pots:   P1=G32 P2=G33 P3=G36 P4=G39 P5=G14");
    ESP_LOGI(TAG, "=============================================");

    if (adc_init() != ERR_NONE) {
        ESP_LOGE(TAG, "[%lu ms] ERROR CRÍTICO: Fallo en ADC. Sistema detenido.",
                 get_timestamp_ms());
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    if (gpio_init_servos() != ERR_NONE) {
        ESP_LOGE(TAG, "[%lu ms] ERROR CRÍTICO: Fallo en GPIO. Sistema detenido.",
                 get_timestamp_ms());
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    /* Posiciones iniciales */
    servo1_pw = angle_to_us(SERVO1_ANG_MIN, STD_MIN_US,  STD_MAX_US);
    servo2_pw = angle_to_us(SERVO2_ANG_MIN, STD_MIN_US,  STD_MAX_US);
    servo3_pw = angle_to_us(SERVO3_ANG_MIN, STD_MIN_US,  STD_MAX_US);
    servo4_pw = angle_to_us(SERVO4_ANG_MIN, STD_MIN_US,  STD_MAX_US);   /* [ADD-1] */
    servo5_pw = angle_to_us(SERVO5_ANG_MIN, STD_MIN_US,  STD_MAX_US);   /* [ADD-2] */
    servoA_pw = angle_to_us(90,             ACCEL_MIN_US, ACCEL_MAX_US);

    ESP_LOGI(TAG, "[%lu ms] INFO: Posiciones iniciales → S1:%d° S2:%d° S3:%d° S4:%d° S5:%d° SA:90°",
             get_timestamp_ms(),
             SERVO1_ANG_MIN, SERVO2_ANG_MIN, SERVO3_ANG_MIN,
             SERVO4_ANG_MIN, SERVO5_ANG_MIN);

    xTaskCreatePinnedToCore(servo_pwm_task, "servo_pwm",
                            4096, NULL, 5, NULL, 1);  /* Stack ampliado a 4096 */

    ESP_LOGI(TAG, "[%lu ms] INFO: Sistema listo. Entrando al bucle principal.",
             get_timestamp_ms());

    /* =============================================================
     *              BUCLE PRINCIPAL — LÓGICA
     * ============================================================= */
    while (1) {
        loop_count++;

        /* ================= POTENCIÓMETROS ================= */
        /*
         *  Flujo:  ADC (0–4095) → Porcentaje (0–100) → Ángulo servo
         *
         *  Pot1 (pulgar):  0% →  2°,  100% →  30°
         *  Pot2 (índice):  0% →  70°, 100% →   0°  (invertido)
         *  Pot3 (medio):   0% →  92°, 100% → 160°
         *  Pot4 (anular):  0% →  90°, 100% → 160°  [ADD-1]
         *  Pot5 (meñique): 0% →  90°, 100% → 160°  [ADD-2]
         */

        /* Lecturas ADC */
        adc1_read_safe(POT1_CH, &valPot1, "POT_PULGAR");
        adc1_read_safe(POT2_CH, &valPot2, "POT_INDICE");
        adc1_read_safe(POT3_CH, &valPot3, "POT_MEDIO");
        adc1_read_safe(POT4_CH, &valPot4, "POT_ANULAR");       /* [ADD-1] ADC1 */
        adc2_read_safe(POT5_CH, &valPot5, "POT_MENIQUE");      /* [ADD-2] ADC2 */

        /* Paso 1: ADC (0–4095) → Porcentaje (0–100) */
        pct1 = constrain_i(map_val(valPot1, 0, 4095, 0, 100), 0, 100);
        pct2 = constrain_i(map_val(valPot2, 0, 4095, 0, 100), 0, 100);
        pct3 = constrain_i(map_val(valPot3, 0, 4095, 0, 100), 0, 100);
        pct4 = constrain_i(map_val(valPot4, 0, 4095, 0, 100), 0, 100);  /* [ADD-1] */
        pct5 = constrain_i(map_val(valPot5, 0, 4095, 0, 100), 0, 100);  /* [ADD-2] */

        /* Paso 2: Porcentaje (0–100) → Ángulo del servo */
        ang1 = map_val(pct1, 0, 100, SERVO1_ANG_MIN, SERVO1_ANG_MAX);
        ang2 = map_val(pct2, 0, 100, SERVO2_ANG_MIN, SERVO2_ANG_MAX);
        ang3 = map_val(pct3, 0, 100, SERVO3_ANG_MIN, SERVO3_ANG_MAX);
        ang4 = map_val(pct4, 0, 100, SERVO4_ANG_MIN, SERVO4_ANG_MAX);   /* [ADD-1] */
        ang5 = map_val(pct5, 0, 100, SERVO5_ANG_MIN, SERVO5_ANG_MAX);   /* [ADD-2] */

        /* [FIX-5] Constrain usando defines con CONSTRAIN_AUTO */
        ang1 = CONSTRAIN_AUTO(ang1, SERVO1_ANG_MIN, SERVO1_ANG_MAX);
        ang2 = CONSTRAIN_AUTO(ang2, SERVO2_ANG_MIN, SERVO2_ANG_MAX);
        ang3 = CONSTRAIN_AUTO(ang3, SERVO3_ANG_MIN, SERVO3_ANG_MAX);
        ang4 = CONSTRAIN_AUTO(ang4, SERVO4_ANG_MIN, SERVO4_ANG_MAX);    /* [ADD-1] */
        ang5 = CONSTRAIN_AUTO(ang5, SERVO5_ANG_MIN, SERVO5_ANG_MAX);    /* [ADD-2] */

        /* Paso 3: Ángulo → Pulso PWM */
        servo1_pw = angle_to_us(ang1, STD_MIN_US, STD_MAX_US);
        servo2_pw = angle_to_us(ang2, STD_MIN_US, STD_MAX_US);
        servo3_pw = angle_to_us(ang3, STD_MIN_US, STD_MAX_US);
        servo4_pw = angle_to_us(ang4, STD_MIN_US, STD_MAX_US);          /* [ADD-1] */
        servo5_pw = angle_to_us(ang5, STD_MIN_US, STD_MAX_US);          /* [ADD-2] */

        /* ================= ACELERÓMETRO =================== */
        int rawX = 0, rawY = 0;
        adc1_read_safe(ACCEL_X_CH, &rawX, "ACCEL_X");
        adc1_read_safe(ACCEL_Y_CH, &rawY, "ACCEL_Y");

        float vX = ((float)rawX / 4095.0f) * 3.3f;
        float vY = ((float)rawY / 4095.0f) * 3.3f;
        float angRad = fast_atan2(vY - 1.65f, vX - 1.65f);
        float angDeg = angRad * 180.0f / MY_PI;
        angDeg -= ANG_OFFSET;
        angDeg  = constrain_f(angDeg, ANG_MIN, ANG_MAX);

        int angServo = constrain_i(
            map_val((int)angDeg, (int)ANG_MIN, (int)ANG_MAX, 0, 180),
            0, 180);
        servoA_pw = angle_to_us(angServo, ACCEL_MIN_US, ACCEL_MAX_US);

        /* ================= SERIAL MONITOR ================= */
        ESP_LOGI(TAG,
                 "[%lu ms] POT→ P1:%d%% P2:%d%% P3:%d%% P4:%d%% P5:%d%% | "
                 "ANG→ S1:%d° S2:%d° S3:%d° S4:%d° S5:%d° | "
                 "Accel:%.2f° SA:%d°",
                 get_timestamp_ms(),
                 pct1, pct2, pct3, pct4, pct5,
                 ang1, ang2, ang3, ang4, ang5,
                 angDeg, angServo);

        /* ================= REPORTE PERIÓDICO ============== */
        if (loop_count % 500 == 0) {
            log_system_status();
        }

        /* ================= DELAY ========================== */
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

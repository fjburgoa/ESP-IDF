#include <math.h>

#include "esp_check.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "BNO055.h"
#include "BNO055_driver.h"
#include "config.h"
#include "settings.h"

#define RAD_TO_DEG 57.29577951308232f
#define BNO055_MODE_BUTTON_GPIO GPIO_NUM_0
#define BNO055_BUTTON_DEBOUNCE_MS 60U

static const char *TAG = "BNO055";
static TaskHandle_t s_task;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static bno055_data_t s_data;
static int s_button_raw_level = 1;
static int s_button_stable_level = 1;
static TickType_t s_button_change_tick;

static void bno055_check_mode_button(void)
{
    const TickType_t now = xTaskGetTickCount();
    const int level = gpio_get_level(BNO055_MODE_BUTTON_GPIO);

    if (level != s_button_raw_level)
    {
        s_button_raw_level = level;
        s_button_change_tick = now;
    }

    if ((level != s_button_stable_level) &&
        ((now - s_button_change_tick) >= pdMS_TO_TICKS(BNO055_BUTTON_DEBOUNCE_MS)))
    {
        s_button_stable_level = level;

        /* BOOT es activo a nivel bajo. Solo actuamos en el flanco de pulsación. */
        if (level == 0)
        {
            const esp_err_t err = bno055_driver_cycle_test_mode();
            if (err != ESP_OK)
                ESP_LOGW(TAG, "No se pudo cambiar el modo BNO055: %s", esp_err_to_name(err));
        }
    }
}

/*------------------------------------------------------------*/
/*------------------------------------------------------------*/

static void bno055_compute_accel_attitude(
    float ax, float ay, float az, float *roll_deg, float *pitch_deg)
{
    *pitch_deg = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
    *roll_deg = atan2f(ay, -az) * RAD_TO_DEG;
}

/*
 * Único lugar donde se ajustan manualmente los ejes del BNO055.
 * El driver mantiene siempre el mismo remapeo base de registros.
 */
static void bno055_to_aircraft_axes(const bno055_vector3f_t *raw_accel, const bno055_vector3f_t *raw_gyro,
                                    bno055_vector3f_t *accel, bno055_vector3f_t *gyro)
{
    if (s_mount_mode == MOUNT_VERTICAL)
    {
        accel->x = -raw_accel->y;
        accel->y = -raw_accel->x;
        accel->z = -raw_accel->z;
        /*
        Los giros positivos se determinan mediante la regla de la mano derecha. Así, con estos ejes:
            p>0: baja el ala derecha.
            q>0: sube el morro.
            r>0: el morro gira hacia la derecha.
        */
        gyro->x = raw_gyro->y;  /* p: roll rate  */
        gyro->y = raw_gyro->x;  /* q: pitch rate */
        gyro->z = -raw_gyro->z; /* r: yaw rate   */
    }
    else
    {
        accel->x = -raw_accel->x;
        accel->y = raw_accel->y;
        accel->z = -raw_accel->z;
        /*
        Los giros positivos se determinan mediante la regla de la mano derecha. Así, con estos ejes:
            p>0: baja el ala derecha.
            q>0: sube el morro.
            r>0: el morro gira hacia la derecha.
        */
        gyro->x = raw_gyro->x;
        gyro->y = -raw_gyro->y;
        gyro->z = -raw_gyro->z;
    }
}

static void bno055_vector_to_aircraft_axes(const bno055_vector3f_t *raw,
                                           bno055_vector3f_t *aircraft)
{
    if (s_mount_mode == MOUNT_VERTICAL)
    {
        aircraft->x = -raw->y;
        aircraft->y = -raw->x;
        aircraft->z = -raw->z;
    }
    else
    {
        aircraft->x = -raw->x;
        aircraft->y = raw->y;
        aircraft->z = -raw->z;
    }
}

/*------------------------------------------------------------*/
/*------------------------------------------------------------*/

bno055_data_t BNO055_get_data(void)
{
    bno055_data_t copy;
    portENTER_CRITICAL(&s_mux);
    copy = s_data;
    portEXIT_CRITICAL(&s_mux);
    return copy;
}

/*------------------------------------------------------------*/
/*------------------------------------------------------------*/
static void bno_task(void *arg)
{
    (void)arg;

    TickType_t wake = xTaskGetTickCount();

    while (1)
    {
        bno055_check_mode_button();

        bno055_vector3f_t raw_accel = {0};
        bno055_vector3f_t raw_gyro = {0};
        bno055_vector3f_t raw_linear_accel = {0};
        bno055_vector3f_t raw_gravity = {0};
        bno055_data_t sample = {0};

        esp_err_t err1 = bno055_driver_read_acceleration(&raw_accel);
        esp_err_t err2 = bno055_driver_read_gyro(&raw_gyro);

        bno055_to_aircraft_axes(&raw_accel, &raw_gyro, &sample.acceleration_ms2, &sample.gyro_dps);

        sample.operation_mode = bno055_driver_get_operation_mode();

        if ((sample.operation_mode == BNO055_OPERATION_MODE_IMUPLUS) ||
            (sample.operation_mode == BNO055_OPERATION_MODE_NDOF_FMC_OFF) ||
            (sample.operation_mode == BNO055_OPERATION_MODE_NDOF))
        {
            const esp_err_t linear_err = bno055_driver_read_linear_acceleration(&raw_linear_accel);
            if (linear_err == ESP_OK)
            {
                bno055_vector_to_aircraft_axes(&raw_linear_accel, &sample.linear_accel);
                sample.linear_accel_valid = true;
            }

            const esp_err_t gravity_err = bno055_driver_read_gravity(&raw_gravity);
            if (gravity_err == ESP_OK)
            {
                bno055_vector_to_aircraft_axes(&raw_gravity, &sample.gravity);
                sample.gravity_valid = true;
            }

            bno055_compute_accel_attitude(
                sample.gravity.x,
                sample.gravity.y,
                sample.gravity.z,
                &sample.roll_deg, &sample.pitch_deg);

            // printf(
            //     "sample: acc=(%.2f, %.2f, %.2f) "
            //     "gyro=(%.2f, %.2f, %.2f)\n",
            //     sample.acceleration_ms2.x,
            //     sample.acceleration_ms2.y,
            //     sample.acceleration_ms2.z,
            //     sample.gyro_dps.x,
            //     sample.gyro_dps.y,
            //     sample.gyro_dps.z);
        }
        else
        {
            bno055_compute_accel_attitude(
                sample.acceleration_ms2.x,
                sample.acceleration_ms2.y,
                sample.acceleration_ms2.z,
                &sample.roll_deg, &sample.pitch_deg);
        }

        if ((err1 == ESP_OK) && (err2 == ESP_OK))
        {
            sample.valid = true;
            portENTER_CRITICAL(&s_mux);
            s_data = sample;
            portEXIT_CRITICAL(&s_mux);
        }
        else
        {
            ESP_LOGW(TAG, "Error de lectura: %s # %s", esp_err_to_name(err1), esp_err_to_name(err2));
        }
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(BNO055_PERIOD_MS));
    }
}
/*------------------------------------------------------------*/
/*------------------------------------------------------------*/

esp_err_t BNO055_start(void)
{
    if (s_task != NULL)
        return ESP_ERR_INVALID_STATE;

    const gpio_config_t button_config = {
        .pin_bit_mask = 1ULL << BNO055_MODE_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&button_config), TAG, "No se pudo configurar BOOT");

    /* Si BOOT está pulsado durante el arranque, no cambiamos de modo hasta
       que se suelte y se produzca una nueva pulsación. */
    s_button_raw_level = gpio_get_level(BNO055_MODE_BUTTON_GPIO);
    s_button_stable_level = s_button_raw_level;
    s_button_change_tick = xTaskGetTickCount();

    ESP_RETURN_ON_ERROR(bno055_driver_init(), TAG, "No se pudo iniciar BNO055");

    if (xTaskCreate(bno_task, "bno055", BNO055_TASK_STACK_SIZE, NULL, BNO055_TASK_PRIORITY, &s_task) != pdPASS)
    {
        s_task = NULL;
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "BNO055 iniciado a %u Hz", 1000U / BNO055_PERIOD_MS);
    return ESP_OK;
}

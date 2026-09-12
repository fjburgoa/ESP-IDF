#include <math.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "BNO055.h"
#include "EFIS_I2C.h"
#include "MPU6050.h"
#include "config.h"
#include "settings.h"

#define MPU6050_ADDR 0x68U
#define REG_SMPLRT_DIV 0x19U
#define REG_CONFIG 0x1AU
#define REG_GYRO_CONFIG 0x1BU
#define REG_ACCEL_CONFIG 0x1CU
#define REG_ACCEL_XOUT_H 0x3BU
#define REG_PWR_MGMT_1 0x6BU

// #define ACCEL_LSB_PER_G 16384.0f  //rango acelerómetro: ±2 g
#define ACCEL_LSB_PER_G 8192.0f // rango acelerómetro: ±4 g

#define GYRO_LSB_PER_DPS 131.0f
#define STANDARD_GRAVITY_MS2 9.80665f
#define RAD_TO_DEG 57.29577951308232f
#define CALIBRATION_SAMPLES 300U

static const char *TAG = "MPU6050";
static i2c_master_dev_handle_t s_device;
static TaskHandle_t s_task;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static mpu6050_data_t s_data;
static float s_abx, s_aby, s_abz;
static float s_gbx, s_gby, s_gbz;

static esp_err_t write_reg(uint8_t reg, uint8_t value)
{
    const uint8_t bytes[] = {reg, value};
    return i2c_master_transmit(s_device, bytes, sizeof(bytes), EFIS_I2C_TIMEOUT_MS);
}

static esp_err_t read_raw(int16_t *ax, int16_t *ay, int16_t *az,
                          int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t data[14];
    const uint8_t reg = REG_ACCEL_XOUT_H;
    esp_err_t err = i2c_master_transmit_receive(
        s_device, &reg, 1, data, sizeof(data), EFIS_I2C_TIMEOUT_MS);
    if (err != ESP_OK)
        return err;
    *ax = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
    *ay = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    *az = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    *gx = (int16_t)(((uint16_t)data[8] << 8) | data[9]);
    *gy = (int16_t)(((uint16_t)data[10] << 8) | data[11]);
    *gz = (int16_t)(((uint16_t)data[12] << 8) | data[13]);
    return ESP_OK;
}

static void compute_accel_attitude(float ax, float ay, float az,
                                   float *roll_deg, float *pitch_deg)
{
    *pitch_deg = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
    *roll_deg = atan2f(ay, -az) * RAD_TO_DEG;
}

static esp_err_t calibrate(void)
{
    int64_t sax = 0, say = 0, saz = 0, sgx = 0, sgy = 0, sgz = 0;
    ESP_LOGI(TAG, "Calibrando: mantenga el sensor inmovil y horizontal");

    uint32_t valid_samples = 0;
    uint32_t attempts = 0;
    esp_err_t last_error = ESP_OK;

    while ((valid_samples < CALIBRATION_SAMPLES) &&
           (attempts < CALIBRATION_SAMPLES * 3U))
    {
        int16_t ax, ay, az, gx, gy, gz;
        ++attempts;
        last_error = read_raw(&ax, &ay, &az, &gx, &gy, &gz);
        if (last_error != ESP_OK)
        {
            ESP_LOGW(TAG, "Lectura de calibracion fallida: %s",
                     esp_err_to_name(last_error));
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        sax += ax;
        say += ay;
        saz += az;
        sgx += gx;
        sgy += gy;
        sgz += gz;
        ++valid_samples;
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    if (valid_samples < CALIBRATION_SAMPLES)
    {
        return (last_error == ESP_OK) ? ESP_ERR_TIMEOUT : last_error;
    }
    s_abx = (float)sax / CALIBRATION_SAMPLES;
    s_aby = (float)say / CALIBRATION_SAMPLES;
    s_abz = (float)saz / CALIBRATION_SAMPLES - ACCEL_LSB_PER_G;
    s_gbx = (float)sgx / CALIBRATION_SAMPLES;
    s_gby = (float)sgy / CALIBRATION_SAMPLES;
    s_gbz = (float)sgz / CALIBRATION_SAMPLES;
    ESP_LOGI(TAG, "Calibracion completada con %lu muestras",
             (unsigned long)valid_samples);
    return ESP_OK;
}

mpu6050_data_t MPU6050_get_data(void)
{
    mpu6050_data_t copy;
    portENTER_CRITICAL(&s_mux);
    copy = s_data;
    portEXIT_CRITICAL(&s_mux);
    return copy;
}

static void mpu_task(void *arg)
{
    (void)arg;
    TickType_t wake = xTaskGetTickCount();

    while (1)
    {
        int16_t axr, ayr, azr, gxr, gyr, gzr;

        esp_err_t err = read_raw(&axr, &ayr, &azr, &gxr, &gyr, &gzr);

        if (err == ESP_OK)
        {

            mpu6050_data_t d = {0};

            if (s_mount_mode == MOUNT_VERTICAL)
            {
                d.accel_y_ms2 = -((float)axr - s_abx) * STANDARD_GRAVITY_MS2 / ACCEL_LSB_PER_G;
                d.accel_x_ms2 = -((float)ayr - s_aby) * STANDARD_GRAVITY_MS2 / ACCEL_LSB_PER_G;
                d.accel_z_ms2 = -((float)azr - s_abz) * STANDARD_GRAVITY_MS2 / ACCEL_LSB_PER_G;
                /*
                Los giros positivos se determinan mediante la regla de la mano derecha. Así, con estos ejes:
                    p>0: baja el ala derecha.
                    q>0: sube el morro.
                    r>0: el morro gira hacia la derecha.
                */
                d.gyro_x_dps = ((float)gyr - s_gby) / GYRO_LSB_PER_DPS;
                d.gyro_y_dps = ((float)gxr - s_gbx) / GYRO_LSB_PER_DPS;
                d.gyro_z_dps = -((float)gzr - s_gbz) / GYRO_LSB_PER_DPS;
                d.valid = true;
            }
            else
            {
                d.accel_x_ms2 = -((float)axr - s_abx) * STANDARD_GRAVITY_MS2 / ACCEL_LSB_PER_G;
                d.accel_y_ms2 = ((float)ayr - s_aby) * STANDARD_GRAVITY_MS2 / ACCEL_LSB_PER_G;
                d.accel_z_ms2 = -((float)azr - s_abz) * STANDARD_GRAVITY_MS2 / ACCEL_LSB_PER_G;
                /*
                Los giros positivos se determinan mediante la regla de la mano derecha. Así, con estos ejes:
                    p>0: baja el ala derecha.
                    q>0: sube el morro.
                    r>0: el morro gira hacia la derecha.
                */
                d.gyro_x_dps = ((float)gxr - s_gbx) / GYRO_LSB_PER_DPS;
                d.gyro_y_dps = -((float)gyr - s_gby) / GYRO_LSB_PER_DPS;
                d.gyro_z_dps = -((float)gzr - s_gbz) / GYRO_LSB_PER_DPS;
                d.valid = true;
            }

            compute_accel_attitude(d.accel_x_ms2, d.accel_y_ms2,
                                   d.accel_z_ms2, &d.roll_deg, &d.pitch_deg);

            portENTER_CRITICAL(&s_mux);
            s_data = d;
            portEXIT_CRITICAL(&s_mux);
        }
        else
        {
            ESP_LOGW(TAG, "Error de lectura: %s", esp_err_to_name(err));
        }
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(MPU6050_PERIOD_MS));
    }
}

esp_err_t MPU6050_start(void)
{
    if (s_task != NULL)
        return ESP_ERR_INVALID_STATE;

    ESP_RETURN_ON_ERROR(efis_i2c_add_device(MPU6050_ADDR, &s_device), TAG, "No se pudo registrar MPU6050");
    ESP_RETURN_ON_ERROR(write_reg(REG_PWR_MGMT_1, 0x00), TAG, "Wake failed");

    vTaskDelay(pdMS_TO_TICKS(100));

    /*
     * Configuración del MPU6050
     *
     * 1. Frecuencia de muestreo
     *    REG_SMPLRT_DIV = 9
     *
     *    Con el DLPF activado, la frecuencia interna del giroscopio es 1 kHz:
     *
     *        f_sample = 1000 Hz / (1 + SMPLRT_DIV)
     *                 = 1000 Hz / (1 + 9)
     *                 = 100 Hz
     *
     *    Aunque el sensor genera muestras a 100 Hz, la tarea del programa las
     *    lee cada MPU6050_PERIOD_MS = 40 ms, es decir, a 25 Hz.
     */
    ESP_RETURN_ON_ERROR(
        write_reg(REG_SMPLRT_DIV, 9),
        TAG,
        "Error configurando frecuencia de muestreo a 100 Hz");

    /*
     * 2. Rango del giroscopio
     *    REG_GYRO_CONFIG = 0x00
     *
     *    FS_SEL = 0:
     *
     *        Rango        = ±250 °/s
     *        Sensibilidad = 131 LSB/(°/s)
     *
     *    Conversión:
     *        velocidad_angular_dps = valor_raw / 131.0
     */
    ESP_RETURN_ON_ERROR(
        write_reg(REG_GYRO_CONFIG, 0x00),
        TAG,
        "Error configurando giroscopio a +/-250 grados/s");

    /*
    * 3. Rango del acelerómetro
    *    REG_ACCEL_CONFIG = 0x00
    *
    *    AFS_SEL = 0:
    *     Rango        = ±2 g
    *     Sensibilidad = 16384 LSB/g

    *    AFS_SEL = 1:
    *     Rango        = ±4 g
    *     Sensibilidad = 8192 LSB/g
    *
    *    Conversión a m/s²:
    *        aceleracion_ms2 = valor_raw * 9.80665 / 16384.0  si AFS_SEL = 0
    *        aceleracion_ms2 = valor_raw * 9.80665 / 8192.0   si AFS_SEL = 1
    */
    ESP_RETURN_ON_ERROR(
        // write_reg(REG_ACCEL_CONFIG, 0x00),  //±2 g
        write_reg(REG_ACCEL_CONFIG, 0x08), // ±4 g
        TAG,
        "Error configurando acelerometro a +/-4 g");

    /*
     * 4. Filtro digital paso bajo
     *    REG_CONFIG = 3
     *
     *    DLPF_CFG = 3:
     *
     *                        Ancho de banda    Retardo aproximado
     *        Acelerómetro:       44.8 Hz             4.88 ms
     *        Giroscopio:         41.0 Hz             5.90 ms
     *
     *    El filtro reduce el ruido de las medidas antes de calcular pitch,
     *    roll, velocidades angulares y posición de la bola.
     */
    ESP_RETURN_ON_ERROR(
        write_reg(REG_CONFIG, 3),
        TAG,
        "Error configurando DLPF_CFG=3");

    /* Finaliza antes de que main.c inicialice el BNO055 en el mismo bus. */
    ESP_RETURN_ON_ERROR(calibrate(), TAG, "No se pudo calibrar MPU6050");

    if (xTaskCreate(mpu_task, "mpu6050", 4096, NULL, 5, &s_task) != pdPASS)
    {
        s_task = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "MPU6050 iniciado a %u Hz", 1000U / MPU6050_PERIOD_MS);
    return ESP_OK;
}

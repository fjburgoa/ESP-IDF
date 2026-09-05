/* -------------------------------------------------------------------------- */
/* Prueba del altímetro / variómetro                                           */
/* -------------------------------------------------------------------------- */
static const char *TAG = "MAIN";

#define FEET_TO_METERS 0.3048f
#define SECONDS_PER_MINUTE 60.0f

static void altitude_test_task(void *pvParameters)
{
    (void)pvParameters;

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(ALTITUDE_TEST_PERIOD_MS);

    const float dt_s =
        (float)ALTITUDE_TEST_PERIOD_MS / 1000.0f;

    /* 500 ft/min = 2.54 m/s. */
    const float rate_mps = ALTITUDE_TEST_RATE_FPM * FEET_TO_METERS / SECONDS_PER_MINUTE;

    float fake_altitude_m = ALTITUDE_TEST_MIN_M;
    float direction = 1.0f;
    uint32_t log_divider = 0U;

    for (;;)
    {
        fake_altitude_m += direction * rate_mps * dt_s;

        if (fake_altitude_m >= ALTITUDE_TEST_MAX_M)
        {
            fake_altitude_m = ALTITUDE_TEST_MAX_M;
            direction = -1.0f;
        }
        else if (fake_altitude_m <= ALTITUDE_TEST_MIN_M)
        {
            fake_altitude_m = ALTITUDE_TEST_MIN_M;
            direction = 1.0f;
        }

        /*
         * Solo se inyecta altitud. BMP280.c calcula la velocidad vertical a
         * partir de esta señal exactamente igual que lo hará con la altitud
         * barométrica real.
         */
        bmp280_set_test_altitude(fake_altitude_m, true);

        if (++log_divider >= (1000U / ALTITUDE_TEST_PERIOD_MS))
        {
            log_divider = 0U;

            ESP_LOGI(
                TAG,
                "ALT TEST: %.1f m | consigna VSI=%+.0f ft/min",
                (double)fake_altitude_m,
                (double)(direction * ALTITUDE_TEST_RATE_FPM));
        }

        vTaskDelayUntil(&last_wake, period);
    }
}
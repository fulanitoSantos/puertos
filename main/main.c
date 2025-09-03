#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include <string.h>
#include <esp_log.h>
#include "driver/uart.h"
#define STEP GPIO_NUM_17
#define DIR GPIO_NUM_5
#define RESET GPIO_NUM_16
#define time_step 10

void configurar_gpio()
{
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << STEP) | (1ULL << DIR) | (1ULL << RESET),
        .pull_down_en = 0,
        .pull_up_en = 0,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);
    gpio_set_level(DIR, 0);
    gpio_set_level(STEP, 0);
}
void StepIz(int steps)
{
    gpio_set_level(RESET, 1);
    gpio_set_level(DIR, 0); // Dirección hacia la izquierda
    for (int i = 0; i < steps; i++)
    {
        gpio_set_level(STEP, 1);
        vTaskDelay(pdMS_TO_TICKS(time_step)); // Espera 20 ms
        gpio_set_level(STEP, 0);
        vTaskDelay(pdMS_TO_TICKS(time_step)); // Espera 20 ms
    }

    gpio_set_level(RESET, 0);
}
void StepDer(int steps)
{
    gpio_set_level(RESET, 1);
    gpio_set_level(DIR, 1); // Dirección hacia la derecha
    for (int i = 0; i < steps; i++)
    {
        gpio_set_level(STEP, 1);
        vTaskDelay(pdMS_TO_TICKS(time_step)); // Espera 20 ms
        gpio_set_level(STEP, 0);
        vTaskDelay(pdMS_TO_TICKS(time_step)); // Espera 20 ms
    }
    gpio_set_level(RESET, 0);
}

void app_main(void)
{

    configurar_gpio();

    while (1)
    {

        StepDer(50);
        vTaskDelay(pdMS_TO_TICKS(1000));
        StepIz(50);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

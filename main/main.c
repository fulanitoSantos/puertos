#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include <string.h>
#include <esp_log.h>
#include "driver/uart.h"
#define STEP1 GPIO_NUM_19
#define DIR1 GPIO_NUM_18
#define STEP2 GPIO_NUM_5
#define DIR2 GPIO_NUM_4
#define time_step 10

void configurar_gpio()
{
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << STEP1) | (1ULL << DIR1) | (1ULL << DIR2) | (1ULL << STEP2),
        .pull_down_en = 0,
        .pull_up_en = 0,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);
    gpio_set_level(DIR1, 0);
    gpio_set_level(STEP1, 0);
    gpio_set_level(DIR2, 0);
    gpio_set_level(STEP2, 0);
}
void StepIz1(int steps)
{
    gpio_set_level(DIR1, 0); // Dirección hacia la izquierda
    for (int i = 0; i < steps; i++)
    {
        gpio_set_level(STEP1, 1);
        vTaskDelay(pdMS_TO_TICKS(time_step)); // Espera 20 ms
        gpio_set_level(STEP1, 0);
        vTaskDelay(pdMS_TO_TICKS(time_step)); // Espera 20 ms
    }
}
void StepDer1(int steps)
{
    gpio_set_level(DIR1, 1); // Dirección hacia la derecha
    for (int i = 0; i < steps; i++)
    {
        gpio_set_level(STEP1, 1);
        vTaskDelay(pdMS_TO_TICKS(time_step)); // Espera 20 ms
        gpio_set_level(STEP1, 0);
        vTaskDelay(pdMS_TO_TICKS(time_step)); // Espera 20 ms
    }
}
void StepIz2(int steps)
{
    gpio_set_level(DIR2, 0); // Dirección hacia la izquierda
    for (int i = 0; i < steps; i++)
    {
        gpio_set_level(STEP2, 1);
        vTaskDelay(pdMS_TO_TICKS(time_step)); // Espera 20 ms
        gpio_set_level(STEP2, 0);
        vTaskDelay(pdMS_TO_TICKS(time_step)); // Espera 20 ms
    }
}
void StepDer2(int steps)
{
    gpio_set_level(DIR2, 1); // Dirección hacia la derecha
    for (int i = 0; i < steps; i++)
    {
        gpio_set_level(STEP2, 1);
        vTaskDelay(pdMS_TO_TICKS(time_step)); // Espera 20 ms
        gpio_set_level(STEP2, 0);
        vTaskDelay(pdMS_TO_TICKS(time_step)); // Espera 20 ms
    }
}
void app_main(void)
{

    configurar_gpio();

    while (1)
    {
        StepDer1(50);
        StepDer2(50);
        vTaskDelay(pdMS_TO_TICKS(1000));
        StepIz1(50);
        StepIz2(50);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include <string.h>
#include <esp_log.h>
#include "driver/uart.h"
#include "libreriajson.h"

#define INPUT_PIN GPIO_NUM_39     // Pin digital donde se detectan los pulsos
static QueueHandle_t uart0_queue; // Cola para eventos de UART0
#define UART0 UART_NUM_0
static char buffer_acumulado[256] = {0};
static int buffer_index = 0;
#define BUF_SIZE 1024
#define UART_TX_BUFFER_SIZE 1024
#define BUFFER_SIZE 20000

static void init_uart(uart_port_t uart_num, int tx_pin, int rx_pin, QueueHandle_t *queue)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    uart_param_config(uart_num, &uart_config);
    uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(uart_num, BUF_SIZE, BUF_SIZE, 10, queue, 0);

    printf("UART%d initialized on TX pin %d, RX pin %d\n", uart_num, tx_pin, rx_pin);
}
static void uart0_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t *data = (uint8_t *)malloc(BUF_SIZE);

    while (true)
    {
        // Espera de eventos en la cola UART
        if (xQueueReceive(uart0_queue, (void *)&event, portMAX_DELAY))
        {
            switch (event.type)
            {
            case UART_DATA: // Datos recibidos
                // Leer los datos del buffer UART
                int len = uart_read_bytes(UART0, data, event.size, portMAX_DELAY);
                data[len] = '\0'; // Asegurar que los datos son una cadena

                // Procesar cada carácter recibido
                for (int i = 0; i < len; i++)
                {
                    if (data[i] == '|')
                    {

                        // Encontramos el delimitador, procesar el mensaje completo
                        buffer_acumulado[buffer_index] = '\0'; // Terminar la cadena
                        printf("Mensaje completo recibido: %s\n", buffer_acumulado);
                        // Aquí puedes procesar el mensaje completo
                        verificar_y_procesar_uart(buffer_acumulado);
                        // Limpiar el buffer para el próximo mensaje
                        buffer_index = 0;
                        memset(buffer_acumulado, 0, sizeof(buffer_acumulado));
                    }
                    else
                    {
                        // Acumular el carácter si hay espacio en el buffer
                        if (buffer_index < sizeof(buffer_acumulado) - 1)
                        {
                            buffer_acumulado[buffer_index] = data[i];
                            buffer_index++;
                        }
                        else
                        {
                            printf("Error: Buffer lleno, reiniciando\n");
                            buffer_index = 0;
                            memset(buffer_acumulado, 0, sizeof(buffer_acumulado));
                        }
                    }
                }
                break;

            case UART_FIFO_OVF: // Desbordamiento de FIFO
                printf("¡Desbordamiento de FIFO!\n");
                uart_flush_input(UART0);
                xQueueReset(uart0_queue);
                break;

            case UART_BUFFER_FULL: // Buffer lleno
                printf("¡Buffer lleno!\n");
                uart_flush_input(UART0);
                xQueueReset(uart0_queue);
                break;

            case UART_BREAK:
                printf("UART Break detectado\n");
                break;

            case UART_PARITY_ERR:
                printf("Error de paridad\n");
                break;

            case UART_FRAME_ERR:
                printf("Error de frame\n");
                break;

            default:
                printf("Evento UART no manejado: %d\n", event.type);
                break;
            }
        }
    }

    free(data);
    vTaskDelete(NULL);
}
void uart_printf(uart_port_t uart_num, const char *fmt, ...)
{
    char buffer[UART_TX_BUFFER_SIZE];
    va_list args;

    // Formatear la cadena usando va_list
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args); // Crear el mensaje formateado
    va_end(args);

    // Enviar la cadena formateada por UART
    uart_write_bytes(uart_num, buffer, strlen(buffer));
}
void app_main(void)
{
    // Configurar el pin como entrada
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << INPUT_PIN),   // Selección del pin
        .mode = GPIO_MODE_INPUT,               // Modo entrada
        .pull_up_en = GPIO_PULLUP_DISABLE,     // Sin pull-up interno
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Sin pull-down interno
        .intr_type = GPIO_INTR_DISABLE         // Sin interrupciones (polling)
    };
    gpio_config(&io_conf);

    while (1)
    {
        int level = gpio_get_level(INPUT_PIN); // Leer nivel lógico del pin

        if (level == 1)
        {

            printf("TRUE\n"); // Enviar mensaje por UART/serial
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Pequeño retardo (10 ms)
    }
}

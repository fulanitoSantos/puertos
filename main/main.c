#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include <string.h>
#include <esp_log.h>
#include "driver/uart.h"

static QueueHandle_t uart1_queue; // Cola para eventos de UART1
static QueueHandle_t uart2_queue; // Cola para eventos de UART2

#define UART1 UART_NUM_1
#define UART2 UART_NUM_2

static char buffer_acumulado1[256] = {0};
static int buffer_index1 = 0;

static char buffer_acumulado2[256] = {0};
static int buffer_index2 = 0;

#define BUF_SIZE 1024
#define UART_TX_BUFFER_SIZE 1024

// funciones del uart
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

static void uart1_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t *data = (uint8_t *)malloc(BUF_SIZE);

    while (true)
    {
        // Espera de eventos en la cola UART
        if (xQueueReceive(uart1_queue, (void *)&event, portMAX_DELAY))
        {
            switch (event.type)
            {
            case UART_DATA: // Datos recibidos
                // Leer los datos del buffer UART
                int len = uart_read_bytes(UART1, data, event.size, portMAX_DELAY);
                data[len] = '\0'; // Asegurar que los datos son una cadena

                // Procesar cada carácter recibido
                for (int i = 0; i < len; i++)
                {
                    if (data[i] == '|')
                    {

                        // Encontramos el delimitador, procesar el mensaje completo
                        buffer_acumulado1[buffer_index1] = '\0'; // Terminar la cadena
                        printf("Mensaje completo recibido: %s\n", buffer_acumulado1);
                        // Aquí puedes procesar el mensaje completo
                        // Limpiar el buffer para el próximo mensaje
                        buffer_index1 = 0;
                        memset(buffer_acumulado1, 0, sizeof(buffer_acumulado1));
                    }
                    else
                    {
                        // Acumular el carácter si hay espacio en el buffer
                        if (buffer_index1 < sizeof(buffer_acumulado1) - 1)
                        {
                            buffer_acumulado1[buffer_index1] = data[i];
                            buffer_index1++;
                        }
                        else
                        {
                            printf("Error: Buffer lleno, reiniciando\n");
                            buffer_index1 = 0;
                            memset(buffer_acumulado1, 0, sizeof(buffer_acumulado1));
                        }
                    }
                }
                break;

            case UART_FIFO_OVF: // Desbordamiento de FIFO
                printf("¡Desbordamiento de FIFO!\n");
                uart_flush_input(UART1);
                xQueueReset(uart1_queue);
                break;

            case UART_BUFFER_FULL: // Buffer lleno
                printf("¡Buffer lleno!\n");
                uart_flush_input(UART1);
                xQueueReset(uart1_queue);
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

static void uart2_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t *data = (uint8_t *)malloc(BUF_SIZE);

    while (true)
    {
        // Espera de eventos en la cola UART
        if (xQueueReceive(uart2_queue, (void *)&event, portMAX_DELAY))
        {
            switch (event.type)
            {
            case UART_DATA: // Datos recibidos
                // Leer los datos del buffer UART
                int len = uart_read_bytes(UART2, data, event.size, portMAX_DELAY);
                data[len] = '\0'; // Asegurar que los datos son una cadena

                // Procesar cada carácter recibido
                for (int i = 0; i < len; i++)
                {
                    if (data[i] == '|')
                    {

                        // Encontramos el delimitador, procesar el mensaje completo
                        buffer_acumulado2[buffer_index2] = '\0'; // Terminar la cadena
                        printf("Mensaje completo recibido: %s\n", buffer_acumulado2);
                        // Aquí puedes procesar el mensaje completo
                        // Limpiar el buffer para el próximo mensaje
                        buffer_index2 = 0;
                        memset(buffer_acumulado2, 0, sizeof(buffer_acumulado2));
                    }
                    else
                    {
                        // Acumular el carácter si hay espacio en el buffer
                        if (buffer_index2 < sizeof(buffer_acumulado2) - 1)
                        {
                            buffer_acumulado2[buffer_index2] = data[i];
                            buffer_index2++;
                        }
                        else
                        {
                            printf("Error: Buffer lleno, reiniciando\n");
                            buffer_index2 = 0;
                            memset(buffer_acumulado2, 0, sizeof(buffer_acumulado2));
                        }
                    }
                }
                break;

            case UART_FIFO_OVF: // Desbordamiento de FIFO
                printf("¡Desbordamiento de FIFO!\n");
                uart_flush_input(UART2);
                xQueueReset(uart2_queue);
                break;

            case UART_BUFFER_FULL: // Buffer lleno
                printf("¡Buffer lleno!\n");
                uart_flush_input(UART2);
                xQueueReset(uart2_queue);
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

void app_main()
{
    init_uart(UART1, 16, 17, &uart1_queue);
    init_uart(UART2, 9, 10, &uart2_queue);
    xTaskCreate(uart1_event_task, "uart1_event_task", 2048, NULL, 5, NULL);
    xTaskCreate(uart2_event_task, "uart2_event_task", 2048, NULL, 6, NULL);
    printf("iniciado todo bien \r\n");
    while (true)
    {
        printf("enviando \r\n");
        uart_printf(UART1, "hola desde uart1\n|");
        uart_printf(UART2, "hola desde uart2\n|");
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
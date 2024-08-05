#include <driver/gpio.h>
#include <driver/uart.h>
#include <esp_log.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#define RS485_CONTROL_PIN GPIO_NUM_8
#define UART_PORT_NUM UART_NUM_1
#define BUF_SIZE (1024)
#define EX_UART_NUM UART_NUM_0

static const char *TAG = "RS485";

static QueueHandle_t uart1_queue;

uint8_t calculateCRC8(uint8_t* data, uint8_t length) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < length; i++) {
        uint8_t extract = data[i];
        for (uint8_t tempI = 8; tempI; tempI--) {
            uint8_t sum = (crc ^ extract) & 0x01;
            crc >>= 1;
            if (sum) {
                crc ^= 0x8C;
            }
            extract >>= 1;
        }
    }
    return crc;
}

void preTransmission() {
    gpio_set_level(RS485_CONTROL_PIN, 1);
}

void postTransmission() {
    gpio_set_level(RS485_CONTROL_PIN, 0);
}

void uart_event_task(void *pvParameters) {
    uart_event_t event;
    uint8_t data[BUF_SIZE];
    while (1) {
        if (xQueueReceive(uart1_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA:
                    if (event.size) {
                        int len = uart_read_bytes(UART_PORT_NUM, data, event.size, 100 / portTICK_PERIOD_MS);
                        ESP_LOGI(TAG, "Received: ");
                        for (int i = 0; i < len; i++) {
                            ESP_LOGI(TAG, "%02X", data[i]);
                        }
                    }
                    break;
                default:
                    break;
            }
        }
    }
}

void app_main(void) {
    // Configure UART1 (RS485)
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_set_pin(UART_PORT_NUM, GPIO_NUM_17, GPIO_NUM_18, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart1_queue, 0);

    // Configure RS485 control pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RS485_CONTROL_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    postTransmission(); // Set initial state to listening

    // Configure UART0 (serial monitor)
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0);
    uart_param_config(EX_UART_NUM, &uart_config);
    uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Create UART event task
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 10, NULL);

    // Main loop to read from serial monitor and send over RS485
    uint8_t* data = (uint8_t*) malloc(BUF_SIZE);
    while (1) {
        // Read user input from the serial monitor
        int len = uart_read_bytes(EX_UART_NUM, data, BUF_SIZE, 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = 0; // Null-terminate the input string
            ESP_LOGI(TAG, "Read from UART0: %s", data);

            // Convert input string to bytes
            uint8_t frame[10];
            for (int i = 0; i < 10; i++) {
                if (i * 3 + 2 < len) {
                    frame[i] = strtol((char*)data + i * 3, NULL, 16);
                } else {
                    frame[i] = 0;
                }
            }
            
            // CRC8 Calculation
            frame[9] = calculateCRC8(frame, 9);

            ESP_LOGI(TAG, "Sending: ");
            for (int j = 0; j < 10; j++) {
                ESP_LOGI(TAG, "%02X", frame[j]);
            }

            // Send the frame
            preTransmission();
            uart_write_bytes(UART_PORT_NUM, (const char*)frame, 10);
            uart_wait_tx_done(UART_PORT_NUM, 10 / portTICK_PERIOD_MS);
            postTransmission();
        }

        vTaskDelay(100 / portTICK_PERIOD_MS); // Delay before next iteration
    }
}
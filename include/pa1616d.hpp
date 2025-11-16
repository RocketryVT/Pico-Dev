#pragma once

#include "serial.hpp"
#include "pico/stdlib.h"
#include "stdio.h"
#include <stdint.h>
#include "hardware/uart.h"

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "portmacro.h"
#include "projdefs.h"
#include "task.h"
#include "semphr.h"

#define GPS_UART_BAUD 9600
#define GPS_UART_DATA_BITS 8
#define GPS_UART_STOP_BITS 1
#define GPS_UART_PARITY UART_PARITY_NONE

void pa1616d_uart_task(void *pvParameters);

class PA1616D {
    public:
        PA1616D(uart_inst_t* uart) : uart {uart} {};

        void initialize();
        void checkAndDumpPacket();
        float getFloatField(int index, int fieldCharCount);

#if (USE_FREERTOS == 1)
        static void update_pa1616d_task(void* pvParameters);

        TaskHandle_t update_task_handle = NULL;
#endif

    private:
        uart_inst_t* uart;   //UART must be static as its implementation is more specific with FIFO buffers
        char buffer[128] = {};
        uint8_t fifoIndex = 0;
};


#include "pa1616d.hpp"
#include <cstdlib>

//PA1616D::PA1616D(uart_inst_t* uart) {
//    //Set internal uart instance?
//    uart = uart1;
//}

//Startup routine, configure passed UART instance for GPS comms and wait for first receive
void PA1616D::initialize() {
    //Configure passed UART instance
    uint32_t baud = uart_set_baudrate(uart, GPS_UART_BAUD);     //This may not be exact, returns actual
    uart_set_hw_flow(uart, false, false);   //Disable CTS/RTS??
    uart_set_format(uart, GPS_UART_DATA_BITS, GPS_UART_STOP_BITS, GPS_UART_PARITY);
    uart_set_fifo_enabled(uart, true);  //We will NEED serial-parallel conversion to have any shot at this
    
    //Status print once initialization is complete
    printf("Configured Baud: [%6.2i]", (uint32_t)baud);
    printf("Enabled: [");
    printf((uart_is_enabled(uart) ? "YES" : "NO"));
    printf("]\n");

}

//Just fucking kill me at this point, my nuts are static they exist in all contexts
void PA1616D::checkAndDumpPacket() {
    printf("\nChecking UART - [");
    if (uart_is_readable(uart)) {
        printf("Y!] - [");
        //uart_read_blocking(uart, buffer, 1);

        //Read fifo to buffer by individual characters
        /*uint8_t rx_count = 0;
        uint8_t num_readable = 0;
        uint8_t rx_data = 0;
        while ((num_readable = uart_is_readable(uart)) && rx_count < 16) {
            buffer[rx_count] = uart_getc(uart);
            rx_count++;
        }

        //Print buffer over stdout
        size_t packetNum = 0;
       // while (packetNum < rx_count/16 && packetNum < 4) {            
            for (int i = packetNum * 16; i < (packetNum + 1)*16; i++) printf("%c ", (char)buffer[i]);
            printf("]\n");
       //     packetNum++;
       // }*/
        
        //Intake the incoming packet from FIFO
        int32_t rxCount = 0;
        //while (uart_is_readable(uart)) {
        while (rxCount < 16) {
            //char dumpChar = uart_getc(uart);
            //try {
            buffer[fifoIndex]; 
            char c = uart_getc(uart);
             
            if (c == '$') fifoIndex = 0;
            buffer[fifoIndex] = c;
            //} catch (std::exception e) {
                //printf("ERROR ENCOUNTERED! ");
            //}
            //if (buffer[rxCount] == '$') rxCount = 0;
            fifoIndex++;
            rxCount++;
            if (!uart_is_readable(uart)) break;
        }
        
        for (int i = 0; i < 16; i++) stdio_putchar(buffer[i]);
        printf("] - rxCount: [%u]", rxCount);
    } else {
        printf("N]");
    }
}

float PA1616D::getFloatField(int index, int fieldCharCount) {
    //Iterate over buffer by number of commas
    int numCommas = 0;
    int fieldStartingIndex = 0;
    while (numCommas < index) {
        if (buffer[fieldStartingIndex] == ',') numCommas++;
        fieldStartingIndex++;
    }

    //With starting index found, parse based on number of characters
    char fieldArray[fieldCharCount];
    for (int i = fieldStartingIndex; i < fieldStartingIndex + fieldCharCount; i++) 
        fieldArray[i-fieldStartingIndex] = buffer[i];
    printf("\n - ARRAY: [");
    for (int p = 0; p < fieldCharCount; p++) printf("%c", fieldArray[p]);
    float field = static_cast<float>(std::atof(fieldArray));    //Returns double by default
    printf("] - FIELD (%i): [%4.4f]", index, field);

    return field;
}

//Update task for listening(?) during computation
#if (USE_FREERTOS == 1)
void PA1616D::update_pa1616d_task(void* pvParameters) {
    TickType_t xLastWakeTime;

    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / 25); //TODO: Figure out how the fuck this sampling is gonna work

    xLastWakeTime = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        taskENTER_CRITICAL();

        PA1616D* gps = (PA1616D*) pvParameters;
        gps->checkAndDumpPacket();
        gps->getFloatField(2, 9);         
        taskEXIT_CRITICAL();
        if ((xLastWakeTime + xFrequency) < xTaskGetTickCount()) {
            xLastWakeTime = xTaskGetTickCount();
        }
    }
}
#endif

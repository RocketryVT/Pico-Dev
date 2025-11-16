#include "bno055.hpp"

void BNO055::initialize() {
    //Configure settings for all sensor channels
    //?
    
    //Configure operating mode
    buffer[0] = 0x3D;   //OPR_MODE
    buffer[1] = 0x0C;   //NDoF -> 0bxxxx1100
    i2c_write_blocking(i2c, addr, buffer, 2, false);
    sleep_ms(10);       //7ms delay on switching from config mode, per datasheet

    //Configure to use external oscillator
    buffer[0] = 0x3F;   //SYS_TRIGGER
    buffer[1] = 0x80;   //10000000, CLK_SEL -> HIGH
    i2c_write_blocking(i2c, addr, buffer, 2, false);
}

void BNO055::sample_accel_gyro_mag() {
    //Actually sample data
    buffer[0] = 0x08;   //ACC_DATA_X_LSB
    i2c_write_blocking(i2c, addr, buffer, 1, true);
    i2c_read_blocking(i2c, addr, buffer, 18, false);

    //Parse data from buffer and set internal fields
    raw_ax = (((int16_t) buffer[1]) << 8) | ((int16_t)buffer[0]);
    raw_ay = (((int16_t) buffer[3]) << 8) | ((int16_t)buffer[2]);
    raw_az = (((int16_t) buffer[5]) << 8) | ((int16_t)buffer[4]);
    raw_gx = (((int16_t) buffer[13]) << 8) | ((int16_t)buffer[12]);
    raw_gy = (((int16_t) buffer[15]) << 8) | ((int16_t)buffer[14]);
    raw_gz = (((int16_t) buffer[17]) << 8) | ((int16_t)buffer[16]);
    raw_mx = (((int16_t) buffer[7]) << 8) | ((int16_t)buffer[6]);
    raw_my = (((int16_t) buffer[9]) << 8) | ((int16_t)buffer[8]);
    raw_mz = (((int16_t) buffer[11]) << 8) | ((int16_t)buffer[10]);
}

void BNO055::sample_attitude_quat() {
    //Actually sample data
    buffer[0] = 0x20;   //QUA_Data_w_LSB
    i2c_write_blocking(i2c, addr, buffer, 1, true);
    i2c_read_blocking(i2c, addr, buffer, 8, false);

    //Parse data from buffer and set internal fields
    qw = (((int16_t)buffer[1]) << 8) | ((int16_t)buffer[0]);
    qx = (((int16_t)buffer[3]) << 8) | ((int16_t)buffer[2]);
    qy = (((int16_t)buffer[5]) << 8) | ((int16_t)buffer[4]);
    qz = (((int16_t)buffer[7]) << 8) | ((int16_t)buffer[6]);
}

//void BNO055::set_page(uint8_t page) {
    
//}

#if (USE_FREERTOS == 1)
void BNO055::update_bno055_task(void* pvParameters) { 
    TickType_t xLastWakeTime;

    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / 10); //TODO: Define sample rate

    xLastWakeTime = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        taskENTER_CRITICAL();

        BNO055* bno055 = (BNO055*) pvParameters;
        bno055->sample_accel_gyro_mag();
        bno055->sample_attitude_quat();
        
        taskEXIT_CRITICAL();
        if ((xLastWakeTime + xFrequency) < xTaskGetTickCount()) {
            xLastWakeTime = xTaskGetTickCount();
        }
    }
}
#endif

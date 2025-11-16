#pragma once

#include "serial.hpp"
#include <stdint.h>

#include <hardware/i2c.h>

#if (USE_FREERTOS == 1)
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "portmacro.h"
#include "projdefs.h"
#include "serial.hpp"
#include "task.h"
#include "semphr.h"
#endif

#define BNO055_I2C_ADDR 0x28
#define BNO055_SAMPLE_RATE_HZ 500

#define S_BNO055_ACCEL_SCALE_FACTOR_16BIT 1000  //1LSB/mg
#define S_BNO055_GYRO_SCALE_FACTOR_16BIT 16   //16LSB/deg/s
#define S_BNO055_MAG_SCALE_FACTOR_16BIT 16
#define S_BNO055_QUAT_SCALE_FACTOR_16BIT 16384.0    //16,384 --> quat coeff = 1.0

class BNO055 {
    public:
        BNO055(i2c_inst_t* i2c) : i2c {i2c} {};

        void initialize();

        void sample_accel_gyro_mag();
        void sample_attitude_quat();

        int16_t get_ax() { return raw_ax; }
        int16_t get_ay() { return raw_ay; }
        int16_t get_az() { return raw_az; }
        int16_t get_gx() { return raw_gx; }
        int16_t get_gy() { return raw_gy; }
        int16_t get_gz() { return raw_gz; }
        int16_t get_mx() { return raw_mx; }
        int16_t get_my() { return raw_my; }
        int16_t get_mz() { return raw_mz; }
        int16_t get_qw() { return qw; }
        int16_t get_qx() { return qx; }
        int16_t get_qy() { return qy; }
        int16_t get_qz() { return qz; }

        static float scale_accel(int16_t unscaled) { return ((float)unscaled) / S_BNO055_ACCEL_SCALE_FACTOR_16BIT; }
        static float scale_gyro(int16_t unscaled) { return ((float)unscaled) / S_BNO055_GYRO_SCALE_FACTOR_16BIT; }
        static float scale_mag(int16_t unscaled) { return ((float)unscaled) / S_BNO055_MAG_SCALE_FACTOR_16BIT; }
        static float scale_quat(int16_t unscaled) { return ((float)unscaled) / S_BNO055_QUAT_SCALE_FACTOR_16BIT; }

#if (USE_FREERTOS == 1)
        static void update_bno055_task(void* pvParameters);

        TaskHandle_t update_task_handle = NULL;
#endif

    private:

        const uint8_t addr = BNO055_I2C_ADDR;
        i2c_inst_t* i2c;
        uint8_t buffer[32];

        //Internal data fields
        int16_t raw_ax, raw_ay, raw_az;
        int16_t raw_gx, raw_gy, raw_gz;
        int16_t raw_mx, raw_my, raw_mz;
        int16_t qw, qx, qy, qz;
};

// test_madgwick_filter.cpp
#include <iostream>
#include <cmath>
#include "madgwick_filter.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

int main() {
    madgwick_filter filter(0.1f);
    
    float roll, pitch, yaw;
    const float dt = 0.01f;   // 100 Hz
    const int steps = 1000;
    
    // Test 1: Stationary IMU
    std::cout << "=== Test 1: Stationary (accel = 0,0,1g; gyro = 0,0,0) ===\n";
    for (int i = 0; i < steps; i++) {
        filter.update_imu(
            0.0f, 0.0f, 0.0f,   // gx, gy, gz (rad/s)
            0.0f, 0.0f, 1.0f,   // ax, ay, az (g)
            dt
        );
        if (i % 200 == 0) {
            filter.get_euler(&roll, &pitch, &yaw);
            std::cout << "step " << i
                      << " roll=" << roll
                      << " pitch=" << pitch
                      << " yaw=" << yaw << "\n";
        }
    }
    
    // Test 2: Constant yaw rotation (90 deg/sec)
    std::cout << "\n=== Test 2: Constant yaw at 90 deg/s ===\n";
    filter.reset();  // start fresh
    
    float expectedYaw = 0.0f;
    float gz = 90.0f * M_PI / 180.0f;  // convert deg/s → rad/s
    
    for (int i = 0; i < steps; i++) {
        filter.update_imu(
            0.0f, 0.0f, gz,    // yaw rotation
            0.0f, 0.0f, 1.0f,  // gravity vector
            dt
        );
        expectedYaw += gz * dt;
        
        // Wrap expected yaw to [-pi, pi] for comparison
        while (expectedYaw > M_PI) expectedYaw -= 2.0f * M_PI;
        while (expectedYaw < -M_PI) expectedYaw += 2.0f * M_PI;
        
        if (i % 200 == 0) {
            filter.get_euler(&roll, &pitch, &yaw);
            float error = std::fabs(yaw - expectedYaw);
            std::cout << "step " << i
                      << " yaw_est=" << yaw
                      << " yaw_expected=" << expectedYaw
                      << " error=" << error
                      << "\n";
        }
    }
    
    std::cout << "\nAll tests completed.\n";
    return 0;
}
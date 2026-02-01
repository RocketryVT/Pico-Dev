// madgwick_filter.hpp  (only the new/changed parts)
#pragma once
#include <cmath>

class madgwick_filter {
public:
    madgwick_filter();
    explicit madgwick_filter(float beta);

    void reset();

    // New: let you control whether your gx/gy/gz inputs are deg/s (typical logs) or rad/s.
    void set_gyro_units_deg_per_s(bool enabled);
    bool get_gyro_units_deg_per_s() const;

    void update_imu(float gx, float gy, float gz,
                    float ax, float ay, float az,
                    float dt);

    void get_quaternion(float* w, float* x, float* y, float* z) const;
    void get_euler(float* roll, float* pitch, float* yaw) const;

    void set_beta(float beta);
    float get_beta() const;

private:
    void integrate_gyro(float gx, float gy, float gz, float dt);

    float q0, q1, q2, q3;
    float beta;

    // New:
    bool gyro_in_degs;
};

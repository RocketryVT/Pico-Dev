// madgwick_filter.cpp
#include "madgwick_filter.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

namespace {
constexpr float DEG2RAD = 0.017453292519943295f;

// Tunable: how quickly we fade out accel correction as |a| deviates from 1g.
// For recovery/under-chute, 0.35g is a good start.
constexpr float ACC_FADE_WIDTH_G = 0.35f;

// Don't bother correcting if accel confidence is basically zero.
constexpr float MIN_ACC_WEIGHT = 0.05f;

inline float clamp01(float x) {
    if (x < 0.0f) return 0.0f;
    if (x > 1.0f) return 1.0f;
    return x;
}
} // namespace

void madgwick_filter::integrate_gyro(float gx, float gy, float gz, float dt) {
    float q0_ = q0;
    float q1_ = q1;
    float q2_ = q2;
    float q3_ = q3;

    float half_gx = 0.5f * gx;
    float half_gy = 0.5f * gy;
    float half_gz = 0.5f * gz;

    float qDot0 = -half_gx*q1_ - half_gy*q2_ - half_gz*q3_;
    float qDot1 =  half_gx*q0_ + half_gy*q3_ - half_gz*q2_;
    float qDot2 = -half_gx*q3_ + half_gy*q0_ + half_gz*q1_;
    float qDot3 =  half_gx*q2_ - half_gy*q1_ + half_gz*q0_;

    q0_ += qDot0 * dt;
    q1_ += qDot1 * dt;
    q2_ += qDot2 * dt;
    q3_ += qDot3 * dt;

    float qNorm = std::sqrt(q0_*q0_ + q1_*q1_ + q2_*q2_ + q3_*q3_);
    if (qNorm > 0.0f) {
        q0 = q0_ / qNorm;
        q1 = q1_ / qNorm;
        q2 = q2_ / qNorm;
        q3 = q3_ / qNorm;
    }
}

madgwick_filter::madgwick_filter()
: q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f), beta(0.1f), gyro_in_degs(true) {}

madgwick_filter::madgwick_filter(float beta)
: q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f), beta(beta), gyro_in_degs(true) {}

void madgwick_filter::reset() {
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
}

void madgwick_filter::set_gyro_units_deg_per_s(bool enabled) {
    gyro_in_degs = enabled;
}

bool madgwick_filter::get_gyro_units_deg_per_s() const {
    return gyro_in_degs;
}

void madgwick_filter::update_imu(float gx, float gy, float gz,
                                 float ax, float ay, float az,
                                 float dt) {
    // 1) Gyro units: Madgwick math expects rad/s.
    if (gyro_in_degs) {
        gx *= DEG2RAD;
        gy *= DEG2RAD;
        gz *= DEG2RAD;
    }

    // 2) Compute accel magnitude (assumes ax,ay,az are in "g" units; still works if scaled)
    float aNorm = std::sqrt(ax*ax + ay*ay + az*az);

    // If accel is basically zero, skip correction (gyro only)
    if (aNorm < 1e-6f) {
        integrate_gyro(gx, gy, gz, dt);
        return;
    }

    // 3) Adaptive accel trust (good for recovery and any non-1g periods)
    // Weight is 1 when |a| is 1g and fades to 0 as it deviates.
    float aErr = std::fabs(aNorm - 1.0f);
    float wAcc = clamp01(1.0f - (aErr / ACC_FADE_WIDTH_G));

    // If we basically don't trust accel, integrate gyro only
    if (wAcc < MIN_ACC_WEIGHT) {
        integrate_gyro(gx, gy, gz, dt);
        return;
    }

    // Normalize accelerometer (direction of gravity)
    ax /= aNorm;
    ay /= aNorm;
    az /= aNorm;

    // Local copy of quaternion
    float q0_ = q0;
    float q1_ = q1;
    float q2_ = q2;
    float q3_ = q3;

    // --- Gradient descent correction term ---
    float f1 = 2.0f*(q1_*q3_ - q0_*q2_) - ax;
    float f2 = 2.0f*(q0_*q1_ + q2_*q3_) - ay;
    float f3 = 2.0f*(0.5f - q1_*q1_ - q2_*q2_) - az;

    float s0 = 2.0f * (-q2_ * f1 + q1_ * f2);
    float s1 = 2.0f * ( q3_ * f1 + q0_ * f2 - 2.0f * q1_ * f3);
    float s2 = 2.0f * (-q0_ * f1 + q3_ * f2 - 2.0f * q2_ * f3);
    float s3 = 2.0f * ( q1_ * f1 + q2_ * f2);

    float sNorm = std::sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
    if (sNorm > 1e-9f) {
        s0 /= sNorm;
        s1 /= sNorm;
        s2 /= sNorm;
        s3 /= sNorm;
    }

    // --- Quaternion derivative from gyroscope: qDot_omega = 0.5 * q ⊗ omega ---
    float half_gx = 0.5f * gx;
    float half_gy = 0.5f * gy;
    float half_gz = 0.5f * gz;

    float qDot0 = -half_gx*q1_ - half_gy*q2_ - half_gz*q3_;
    float qDot1 =  half_gx*q0_ + half_gy*q3_ - half_gz*q2_;
    float qDot2 = -half_gx*q3_ + half_gy*q0_ + half_gz*q1_;
    float qDot3 =  half_gx*q2_ - half_gy*q1_ + half_gz*q0_;

    // 4) Scale correction by accel confidence (adaptive beta)
    float beta_eff = beta * wAcc;

    qDot0 -= beta_eff * s0;
    qDot1 -= beta_eff * s1;
    qDot2 -= beta_eff * s2;
    qDot3 -= beta_eff * s3;

    q0_ += qDot0 * dt;
    q1_ += qDot1 * dt;
    q2_ += qDot2 * dt;
    q3_ += qDot3 * dt;

    float qNorm = std::sqrt(q0_*q0_ + q1_*q1_ + q2_*q2_ + q3_*q3_);
    if (qNorm > 0.0f) {
        q0 = q0_ / qNorm;
        q1 = q1_ / qNorm;
        q2 = q2_ / qNorm;
        q3 = q3_ / qNorm;
    }
}

void madgwick_filter::get_quaternion(float* w, float* x, float* y, float* z) const {
    *w = q0;
    *x = q1;
    *y = q2;
    *z = q3;
}

void madgwick_filter::get_euler(float* roll, float* pitch, float* yaw) const {
    float qw = q0;
    float qx = q1;
    float qy = q2;
    float qz = q3;

    float sinr_cosp = 2.0f * (qw*qx + qy*qz);
    float cosr_cosp = 1.0f - 2.0f * (qx*qx + qy*qy);
    *roll = std::atan2(sinr_cosp, cosr_cosp);

    float sinp = 2.0f * (qw*qy - qz*qx);
    if (std::fabs(sinp) >= 1.0f) {
        *pitch = (sinp >= 0.0f) ? (0.5f * M_PI) : (-0.5f * M_PI);
    } else {
        *pitch = std::asin(sinp);
    }

    float siny_cosp = 2.0f * (qw*qz + qx*qy);
    float cosy_cosp = 1.0f - 2.0f * (qy*qy + qz*qz);
    *yaw = std::atan2(siny_cosp, cosy_cosp);
}

void madgwick_filter::set_beta(float beta) {
    this->beta = beta;
}

float madgwick_filter::get_beta() const {
    return beta;
}

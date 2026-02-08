// madgwick_filter.cpp
// Madgwick IMU orientation filter (gyro + accelerometer)

#include "madgwick_filter.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

namespace {

// Unit Conversion
constexpr float DEG2RAD = 0.017453292519943295f;

// Accelerometer Confidence Tuning
constexpr float ACC_FADE_WIDTH_G = 0.35f;   // accel trust fades as |norm(acc)-1g| grows
constexpr float MIN_ACC_WEIGHT   = 0.05f;   // minimum trust required to apply accel correction

// Numerical Safety Thresholds
constexpr float MIN_ACC_NORM   = 1e-6f;     // avoid normalizing near-zero accel vector
constexpr float MIN_GRAD_NORM  = 1e-9f;     // avoid normalizing near-zero gradient
constexpr float MIN_Q_NORM     = 1e-12f;    // avoid divide-by-zero in quaternion normalization

// dt Safety
constexpr float MAX_DT = 0.05f;             // clamp dt to avoid large integration steps

// Clamp value to [0, 1]
inline float clamp01(float x) {
    if (x < 0.0f) return 0.0f;
    if (x > 1.0f) return 1.0f;
    return x;
}

} // namespace

// Gyro-only quaternion integration (used when accel is invalid/untrusted)
void madgwick_filter::integrate_gyro(float gx, float gy, float gz, float dt) {
    // Local working quaternion
    float q0_ = q0;
    float q1_ = q1;
    float q2_ = q2;
    float q3_ = q3;

    // Quaternion derivative from gyro
    const float half_gx = 0.5f * gx;
    const float half_gy = 0.5f * gy;
    const float half_gz = 0.5f * gz;

    const float qDot0 = -half_gx*q1_ - half_gy*q2_ - half_gz*q3_;
    const float qDot1 =  half_gx*q0_ + half_gy*q3_ - half_gz*q2_;
    const float qDot2 = -half_gx*q3_ + half_gy*q0_ + half_gz*q1_;
    const float qDot3 =  half_gx*q2_ - half_gy*q1_ + half_gz*q0_;

    // Integrate
    q0_ += qDot0 * dt;
    q1_ += qDot1 * dt;
    q2_ += qDot2 * dt;
    q3_ += qDot3 * dt;

    // Normalize quaternion
    const float qNorm = std::sqrt(q0_*q0_ + q1_*q1_ + q2_*q2_ + q3_*q3_);
    if (qNorm > MIN_Q_NORM) {
        q0 = q0_ / qNorm;
        q1 = q1_ / qNorm;
        q2 = q2_ / qNorm;
        q3 = q3_ / qNorm;
    }
}

// Constructors
madgwick_filter::madgwick_filter()
: q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f),
  beta(0.1f),
  gyro_in_degs(true)
{}

madgwick_filter::madgwick_filter(float beta)
: q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f),
  beta(beta),
  gyro_in_degs(true)
{}

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

// IMU update (gyro + accelerometer)
void madgwick_filter::update_imu(float gx, float gy, float gz,
                                 float ax, float ay, float az,
                                 float dt) {
    // dt validation / clamp
    if (!(dt > 0.0f)) {
        return;
    }
    if (dt > MAX_DT) {
        dt = MAX_DT;
    }

    // Convert gyro units if input is deg/s
    if (gyro_in_degs) {
        gx *= DEG2RAD;
        gy *= DEG2RAD;
        gz *= DEG2RAD;
    }

    // Accelerometer magnitude (validity + confidence)
    const float aNorm = std::sqrt(ax*ax + ay*ay + az*az);

    // Gyro-only if accel is invalid
    if (aNorm < MIN_ACC_NORM) {
        integrate_gyro(gx, gy, gz, dt);
        return;
    }

    // Confidence weight based on deviation from 1g
    const float aErr = std::fabs(aNorm - 1.0f);
    float wAcc = clamp01(1.0f - (aErr / ACC_FADE_WIDTH_G));
    wAcc = wAcc * wAcc; // smooth weight curve

    // Gyro-only if accel confidence is too low
    if (wAcc < MIN_ACC_WEIGHT) {
        integrate_gyro(gx, gy, gz, dt);
        return;
    }

    // Normalize accelerometer direction
    ax /= aNorm;
    ay /= aNorm;
    az /= aNorm;

    // Local working quaternion
    float q0_ = q0;
    float q1_ = q1;
    float q2_ = q2;
    float q3_ = q3;

    // Objective function: gravity direction error
    const float f1 = 2.0f*(q1_*q3_ - q0_*q2_) - ax;
    const float f2 = 2.0f*(q0_*q1_ + q2_*q3_) - ay;
    const float f3 = 2.0f*(0.5f - q1_*q1_ - q2_*q2_) - az;

    // Gradient of objective function (correction direction)
    float s0 = 2.0f * (-q2_ * f1 + q1_ * f2);
    float s1 = 2.0f * ( q3_ * f1 + q0_ * f2 - 2.0f * q1_ * f3);
    float s2 = 2.0f * (-q0_ * f1 + q3_ * f2 - 2.0f * q2_ * f3);
    float s3 = 2.0f * ( q1_ * f1 + q2_ * f2);

    // Normalize gradient (or skip correction if too small)
    const float sNorm = std::sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
    if (sNorm > MIN_GRAD_NORM) {
        s0 /= sNorm;
        s1 /= sNorm;
        s2 /= sNorm;
        s3 /= sNorm;
    } else {
        s0 = s1 = s2 = s3 = 0.0f;
    }

    // Quaternion derivative from gyro
    const float half_gx = 0.5f * gx;
    const float half_gy = 0.5f * gy;
    const float half_gz = 0.5f * gz;

    float qDot0 = -half_gx*q1_ - half_gy*q2_ - half_gz*q3_;
    float qDot1 =  half_gx*q0_ + half_gy*q3_ - half_gz*q2_;
    float qDot2 = -half_gx*q3_ + half_gy*q0_ + half_gz*q1_;
    float qDot3 =  half_gx*q2_ - half_gy*q1_ + half_gz*q0_;

    // Apply accelerometer correction (adaptive gain)
    const float beta_eff = beta * wAcc;
    qDot0 -= beta_eff * s0;
    qDot1 -= beta_eff * s1;
    qDot2 -= beta_eff * s2;
    qDot3 -= beta_eff * s3;

    // Integrate quaternion
    q0_ += qDot0 * dt;
    q1_ += qDot1 * dt;
    q2_ += qDot2 * dt;
    q3_ += qDot3 * dt;

    // Normalize quaternion
    const float qNorm = std::sqrt(q0_*q0_ + q1_*q1_ + q2_*q2_ + q3_*q3_);
    if (qNorm > MIN_Q_NORM) {
        q0 = q0_ / qNorm;
        q1 = q1_ / qNorm;
        q2 = q2_ / qNorm;
        q3 = q3_ / qNorm;
    }
}

// Outputs
void madgwick_filter::get_quaternion(float* w, float* x, float* y, float* z) const {
    *w = q0;
    *x = q1;
    *y = q2;
    *z = q3;
}

void madgwick_filter::get_euler(float* roll, float* pitch, float* yaw) const {
    const float qw = q0;
    const float qx = q1;
    const float qy = q2;
    const float qz = q3;

    // Roll
    const float sinr_cosp = 2.0f * (qw*qx + qy*qz);
    const float cosr_cosp = 1.0f - 2.0f * (qx*qx + qy*qy);
    *roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (clamp for numerical safety)
    const float sinp = 2.0f * (qw*qy - qz*qx);
    if (std::fabs(sinp) >= 1.0f) {
        *pitch = (sinp >= 0.0f) ? (0.5f * M_PI) : (-0.5f * M_PI);
    } else {
        *pitch = std::asin(sinp);
    }

    // Yaw
    const float siny_cosp = 2.0f * (qw*qz + qx*qy);
    const float cosy_cosp = 1.0f - 2.0f * (qy*qy + qz*qz);
    *yaw = std::atan2(siny_cosp, cosy_cosp);
}

// Parameters
void madgwick_filter::set_beta(float beta) {
    this->beta = beta;
}

float madgwick_filter::get_beta() const {
    return beta;
}
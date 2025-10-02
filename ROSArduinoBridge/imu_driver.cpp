// #include <Arduino.h>

// #include <Wire.h>

// #define MPU_ADDR 0x68
// #define ACCEL_SCALE 16384.0f     // LSB/g for ±2g
// #define GYRO_SCALE 131.0f        // LSB/(°/s) for ±250°/s
// #define DEG2RAD 0.017453292519943295f
// #define GRAVITY 9.80665f

// // biases (in SI units)
// static float gyro_bias_x = 0.0f, gyro_bias_y = 0.0f, gyro_bias_z = 0.0f;
// static float accel_bias_x = 0.0f, accel_bias_y = 0.0f, accel_bias_z = 0.0f;

// void initIMU() {
//   Wire.begin();
//   // wake up
//   Wire.beginTransmission(MPU_ADDR);
//   Wire.write(0x6B);
//   Wire.write(0x00); // clear sleep bit
//   Wire.endTransmission(true);
//   delay(100);

//   // Set DLPF to 44Hz (CONFIG = 3)
//   Wire.beginTransmission(MPU_ADDR);
//   Wire.write(0x1A);
//   Wire.write(0x03);
//   Wire.endTransmission(true);

//   // Set gyro full scale to ±250 °/s (GYRO_CONFIG = 0)
//   Wire.beginTransmission(MPU_ADDR);
//   Wire.write(0x1B);
//   Wire.write(0x00);
//   Wire.endTransmission(true);

//   // Set accel full scale to ±2g (ACCEL_CONFIG = 0)
//   Wire.beginTransmission(MPU_ADDR);
//   Wire.write(0x1C);
//   Wire.write(0x00);
//   Wire.endTransmission(true);

//   delay(50);

//   // Simple bias calibration (do this while robot is stationary)
//   const int N = 200;
//   long sum_ax=0, sum_ay=0, sum_az=0;
//   long sum_gx=0, sum_gy=0, sum_gz=0;

//   for (int i=0; i < N; ++i) {
//     Wire.beginTransmission(MPU_ADDR);
//     Wire.write(0x3B); // ACCEL_XOUT_H
//     Wire.endTransmission(false);
//     Wire.requestFrom(MPU_ADDR, 14, true);

//     int16_t raw_ax = (Wire.read() << 8) | Wire.read();
//     int16_t raw_ay = (Wire.read() << 8) | Wire.read();
//     int16_t raw_az = (Wire.read() << 8) | Wire.read();
//     Wire.read(); Wire.read(); // temp
//     int16_t raw_gx = (Wire.read() << 8) | Wire.read();
//     int16_t raw_gy = (Wire.read() << 8) | Wire.read();
//     int16_t raw_gz = (Wire.read() << 8) | Wire.read();

//     sum_ax += raw_ax; sum_ay += raw_ay; sum_az += raw_az;
//     sum_gx += raw_gx; sum_gy += raw_gy; sum_gz += raw_gz;

//     delay(5);
//   }

//   // convert biases to SI units
//   accel_bias_x = (sum_ax / (float)N) / ACCEL_SCALE * GRAVITY;
//   accel_bias_y = (sum_ay / (float)N) / ACCEL_SCALE * GRAVITY;
//   accel_bias_z = (sum_az / (float)N) / ACCEL_SCALE * GRAVITY - GRAVITY; // remove gravity

//   gyro_bias_x = (sum_gx / (float)N) / GYRO_SCALE * DEG2RAD;
//   gyro_bias_y = (sum_gy / (float)N) / GYRO_SCALE * DEG2RAD;
//   gyro_bias_z = (sum_gz / (float)N) / GYRO_SCALE * DEG2RAD;
// }

// // read IMU raw registers, convert, subtract biases
// void readIMU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
//   Wire.beginTransmission(MPU_ADDR);
//   Wire.write(0x3B);
//   Wire.endTransmission(false);
//   Wire.requestFrom(MPU_ADDR, 14, true);

//   int16_t raw_ax = (Wire.read() << 8) | Wire.read();
//   int16_t raw_ay = (Wire.read() << 8) | Wire.read();
//   int16_t raw_az = (Wire.read() << 8) | Wire.read();
//   Wire.read(); Wire.read(); // temp
//   int16_t raw_gx = (Wire.read() << 8) | Wire.read();
//   int16_t raw_gy = (Wire.read() << 8) | Wire.read();
//   int16_t raw_gz = (Wire.read() << 8) | Wire.read();

//   ax = (raw_ax / ACCEL_SCALE) * GRAVITY - accel_bias_x;
//   ay = (raw_ay / ACCEL_SCALE) * GRAVITY - accel_bias_y;
//   az = (raw_az / ACCEL_SCALE) * GRAVITY - accel_bias_z;

//   gx = (raw_gx / GYRO_SCALE) * DEG2RAD - gyro_bias_x;
//   gy = (raw_gy / GYRO_SCALE) * DEG2RAD - gyro_bias_y;
//   gz = (raw_gz / GYRO_SCALE) * DEG2RAD - gyro_bias_z;
// }

#include "imu_driver.h"
#include <Wire.h>

#define MPU_ADDR 0x68
#define ACCEL_SCALE 16384.0f     // LSB/g for ±2g
#define GYRO_SCALE 131.0f        // LSB/(°/s) for ±250°/s
#define DEG2RAD 0.017453292519943295f
#define GRAVITY 9.80665f

// biases (in SI units)
static float gyro_bias_x = 0.0f, gyro_bias_y = 0.0f, gyro_bias_z = 0.0f;
static float accel_bias_x = 0.0f, accel_bias_y = 0.0f, accel_bias_z = 0.0f;

// Madgwick quaternion state (qw, qx, qy, qz)
static volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

// Madgwick gain (beta). Tune for noise vs responsiveness.
static float madgwickBeta = 0.1f;

// For timing
static unsigned long lastUpdateMicros = 0;

// Forward declaration for the Madgwick IMU update
static void MadgwickAHRSupdateIMU(float gx, float gy, float gz,
                                  float ax, float ay, float az, float dt);

// Public: set filter gain
void setMadgwickBeta(float b) {
  madgwickBeta = b;
}

// Public: get quaternion
void getQuaternion(float &qw, float &qx, float &qy, float &qz) {
  qw = q0;
  qx = q1;
  qy = q2;
  qz = q3;
}

void initIMU() {
  Wire.begin();
  // wake up
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00); // clear sleep bit
  Wire.endTransmission(true);
  delay(100);

  // Set DLPF to 44Hz (CONFIG = 3)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission(true);

  // Set gyro full scale to ±250 °/s (GYRO_CONFIG = 0)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  // Set accel full scale to ±2g (ACCEL_CONFIG = 0)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission(true);

  delay(50);

  // Simple bias calibration (do this while robot is stationary)
  const int N = 200;
  long sum_ax=0, sum_ay=0, sum_az=0;
  long sum_gx=0, sum_gy=0, sum_gz=0;

  for (int i=0; i < N; ++i) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); // ACCEL_XOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14, true);

    int16_t raw_ax = (Wire.read() << 8) | Wire.read();
    int16_t raw_ay = (Wire.read() << 8) | Wire.read();
    int16_t raw_az = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read(); // temp
    int16_t raw_gx = (Wire.read() << 8) | Wire.read();
    int16_t raw_gy = (Wire.read() << 8) | Wire.read();
    int16_t raw_gz = (Wire.read() << 8) | Wire.read();

    sum_ax += raw_ax; sum_ay += raw_ay; sum_az += raw_az;
    sum_gx += raw_gx; sum_gy += raw_gy; sum_gz += raw_gz;

    delay(5);
  }

  // convert biases to SI units
  accel_bias_x = (sum_ax / (float)N) / ACCEL_SCALE * GRAVITY;
  accel_bias_y = (sum_ay / (float)N) / ACCEL_SCALE * GRAVITY;
  accel_bias_z = (sum_az / (float)N) / ACCEL_SCALE * GRAVITY - GRAVITY; // remove gravity

  gyro_bias_x = (sum_gx / (float)N) / GYRO_SCALE * DEG2RAD;
  gyro_bias_y = (sum_gy / (float)N) / GYRO_SCALE * DEG2RAD;
  gyro_bias_z = (sum_gz / (float)N) / GYRO_SCALE * DEG2RAD;

  // initialize timing
  lastUpdateMicros = micros();

  // Initialize quaternion from accelerometer (simple approximation):
  // if device is roughly level, set q to identity. Otherwise, you could compute roll/pitch from accel
  q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
}

// read IMU raw registers, convert, subtract biases and update Madgwick filter
void readIMU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  int16_t raw_ax = (Wire.read() << 8) | Wire.read();
  int16_t raw_ay = (Wire.read() << 8) | Wire.read();
  int16_t raw_az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // temp
  int16_t raw_gx = (Wire.read() << 8) | Wire.read();
  int16_t raw_gy = (Wire.read() << 8) | Wire.read();
  int16_t raw_gz = (Wire.read() << 8) | Wire.read();

  // Convert to SI and subtract biases
  ax = (raw_ax / ACCEL_SCALE) * GRAVITY - accel_bias_x;
  ay = (raw_ay / ACCEL_SCALE) * GRAVITY - accel_bias_y;
  az = (raw_az / ACCEL_SCALE) * GRAVITY - accel_bias_z;

  gx = (raw_gx / GYRO_SCALE) * DEG2RAD - gyro_bias_x;
  gy = (raw_gy / GYRO_SCALE) * DEG2RAD - gyro_bias_y;
  gz = (raw_gz / GYRO_SCALE) * DEG2RAD - gyro_bias_z;

  // Time delta
  unsigned long nowMicros = micros();
  float dt = (nowMicros - lastUpdateMicros) * 1e-6f;
  if (dt <= 0.0f || dt > 1.0f) {
    // protect against bad dt (on startup or after a long delay)
    dt = 0.01f; // assume 100 Hz as default
  }
  lastUpdateMicros = nowMicros;

  // Update Madgwick (inputs: gx,gy,gz in rad/s and ax,ay,az in m/s^2)
  MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az, dt);
}

/*
 * Madgwick AHRS - IMU only (no magnetometer)
 * Minimal implementation based on Madgwick's algorithm.
 * - gx,gy,gz are body angular rates in rad/s
 * - ax,ay,az are acceleration in m/s^2 (gravity dominated)
 * - dt is time step in seconds
 */
static void MadgwickAHRSupdateIMU(float gx, float gy, float gz,
                                  float ax, float ay, float az, float dt) {
  // Short name local vars
  float q0l = q0, q1l = q1, q2l = q2, q3l = q3;
  float recipNorm;
  float s0, s1, s2, s3;
  float vx, vy, vz;
  float _2q0 = 2.0f * q0l;
  float _2q1 = 2.0f * q1l;
  float _2q2 = 2.0f * q2l;
  float _2q3 = 2.0f * q3l;

  // Normalize accelerometer measurement (to unit vector)
  float axn = ax, ayn = ay, azn = az;
  float norm = axn*axn + ayn*ayn + azn*azn;
  if (norm == 0.0f) return; // invalid data
  recipNorm = 1.0f / sqrtf(norm);
  axn *= recipNorm; ayn *= recipNorm; azn *= recipNorm;

  // Estimated direction of gravity (v = q * (0,0,1) * q^-1)
  vx = _2q1 * q3l - _2q0 * q2l;
  vy = _2q0 * q1l + _2q2 * q3l;
  vz = q0l*q0l - q1l*q1l - q2l*q2l + q3l*q3l;

  // Error is cross product between estimated and measured direction of gravity
  float ex = (ayn * vz - azn * vy);
  float ey = (azn * vx - axn * vz);
  float ez = (axn * vy - ayn * vx);

  // Gradient (approx) of the objective function
  s0 = -_2q2 * (2.0f*(q1l*q3l - q0l*q2l) - axn)
       + _2q1 * (2.0f*(q0l*q1l + q2l*q3l) - ayn);
  s1 = _2q3 * (2.0f*(q1l*q3l - q0l*q2l) - axn)
       + _2q0 * (2.0f*(q0l*q1l + q2l*q3l) - ayn)
       - 4.0f * q1l * (1.0f - 2.0f*(q1l*q1l + q2l*q2l) - azn);
  s2 = -_2q0 * (2.0f*(q1l*q3l - q0l*q2l) - axn)
       + _2q3 * (2.0f*(q0l*q1l + q2l*q3l) - ayn)
       - 4.0f * q2l * (1.0f - 2.0f*(q1l*q1l + q2l*q2l) - azn);
  s3 = _2q1 * (2.0f*(q1l*q3l - q0l*q2l) - axn)
       + _2q2 * (2.0f*(q0l*q1l + q2l*q3l) - ayn);

  // Normalize step magnitude
  norm = s0*s0 + s1*s1 + s2*s2 + s3*s3;
  if (norm != 0.0f) {
    recipNorm = 1.0f / sqrtf(norm);
    s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;
  }

  // Rate of change of quaternion from gyroscope
  float qDot0 = 0.5f * (-q1l * gx - q2l * gy - q3l * gz);
  float qDot1 = 0.5f * ( q0l * gx + q2l * gz - q3l * gy);
  float qDot2 = 0.5f * ( q0l * gy - q1l * gz + q3l * gx);
  float qDot3 = 0.5f * ( q0l * gz + q1l * gy - q2l * gx);

  // Apply feedback step (gradient descent)
  qDot0 -= madgwickBeta * s0;
  qDot1 -= madgwickBeta * s1;
  qDot2 -= madgwickBeta * s2;
  qDot3 -= madgwickBeta * s3;

  // Integrate to yield quaternion
  q0 += qDot0 * dt;
  q1 += qDot1 * dt;
  q2 += qDot2 * dt;
  q3 += qDot3 * dt;

  // Normalize quaternion
  norm = q0*q0 + q1*q1 + q2*q2 + q3*q3;
  if (norm == 0.0f) {
    // reset to identity on error
    q0 = 1.0f; q1 = q2 = q3 = 0.0f;
    return;
  }
  recipNorm = 1.0f / sqrtf(norm);
  q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
}

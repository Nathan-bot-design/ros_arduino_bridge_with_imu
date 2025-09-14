#include "imu_driver.h"
#include <Wire.h>
#include "MPU6050_6Axis_MotionApps20.h"

// IMU object
MPU6050 imu;

// DMP state
bool dmpReady = false;
uint16_t packetSize;
uint8_t fifoBuffer[64];  // FIFO storage buffer

// Orientation containers
Quaternion q;
VectorFloat gravity;
float ypr[3];   // yaw, pitch, roll

// Raw accel + gyro
VectorInt16 aa;   // accel (X,Y,Z)
VectorInt16 gg;   // gyro  (X,Y,Z)

// Yaw tracking
float yaw_prev = 0.0;
float yaw_current = 0.0;

// Filtered accel/gyro values
float accel_x_filtered = 0.0;
float omega_z_filtered = 0.0;

// --- Init IMU ---
void initIMU() {
  Wire.begin();
  imu.initialize();

  if (!imu.testConnection()) {
    Serial.println("IMU connection failed!");
    return;
  } else {
    Serial.println("IMU connected.");
  }

  // Initialize DMP
  int devStatus = imu.dmpInitialize();
  if (devStatus == 0) {
    imu.setDMPEnabled(true);
    packetSize = imu.dmpGetFIFOPacketSize();
    dmpReady = true;
    Serial.println("DMP ready.");
  } else {
    Serial.print("DMP init failed (code ");
    Serial.print(devStatus);
    Serial.println(")");
    dmpReady = false;
  }

  resetYaw();
}

// --- Update yaw + refresh accel/gyro ---
float getYaw() {
  if (!dmpReady) return yaw_current;

  if (imu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    imu.dmpGetQuaternion(&q, fifoBuffer);
    imu.dmpGetGravity(&gravity, &q);
    imu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    yaw_current = ypr[0];  // yaw in radians
  }

  // Update raw accel + gyro
  imu.getAcceleration(&aa.x, &aa.y, &aa.z);
  imu.getRotation(&gg.x, &gg.y, &gg.z);

  return yaw_current;  // last valid yaw
}

// --- Delta yaw between calls ---
float deltaYaw() {
  float yaw_new = getYaw();
  float d = yaw_new - yaw_prev;

  // Normalize to [-PI, PI]
  if (d > PI) d -= 2 * PI;
  else if (d < -PI) d += 2 * PI;

  yaw_prev = yaw_new;
  return d;
}

// --- Reset yaw tracking ---
void resetYaw() {
  yaw_prev = 0.0;
  yaw_current = 0.0;
}

// --- Get accel X (m/s^2) with filtering ---
float getIMUAccelX() {
  float raw = (float)aa.x / 16384.0 * 9.81;   // convert raw to m/s^2
  accel_x_filtered = 0.8 * accel_x_filtered + 0.2 * raw;
  return accel_x_filtered;
}

// --- Get yaw rate (rad/s) with filtering ---
float getIMUYawRate() {
  float raw = (float)gg.z / 131.0;            // deg/s
  float rad_s = raw * (PI / 180.0);           // convert to rad/s
  omega_z_filtered = 0.8 * omega_z_filtered + 0.2 * rad_s;
  return omega_z_filtered;
}

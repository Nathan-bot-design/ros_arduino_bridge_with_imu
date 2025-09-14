#include "imu_driver.h"

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

// Yaw tracking
float yaw_prev = 0.0;
float yaw_current = 0.0;

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

float getYaw() {
  if (!dmpReady) return yaw_current;

  // Try to get the latest DMP packet
  if (imu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    imu.dmpGetQuaternion(&q, fifoBuffer);
    imu.dmpGetGravity(&gravity, &q);
    imu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    yaw_current = ypr[0];  // yaw in radians
  }

  return yaw_current;  // last valid yaw
}

float deltaYaw() {
  float yaw_new = getYaw();
  float d = yaw_new - yaw_prev;

  // Normalize to [-PI, PI]
  if (d > PI) d -= 2 * PI;
  else if (d < -PI) d += 2 * PI;

  yaw_prev = yaw_new;
  return d;
}

void resetYaw() {
  yaw_prev = 0.0;
  yaw_current = 0.0;
}

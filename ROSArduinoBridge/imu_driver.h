// #ifndef IMU_DRIVER_H
// #define IMU_DRIVER_H

// // #include <Wire.h>
// // #include <stdint.h>

// // struct IMUData {
// //   float ax, ay, az;   // m/s^2
// //   float gx, gy, gz;   // rad/s
// //   float qw, qx, qy, qz; // orientation quaternion
// // };
// void readIMU(float &ax, float &ay, float &az,
//              float &gx, float &gy, float &gz);


// void initIMU();
// void readIMU();

// #endif


#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include <Arduino.h>

// Existing API: readIMU fills ax,ay,az (m/s^2) and gx,gy,gz (rad/s)
// After calling readIMU(...), quaternion values are available via getQuaternion().
void initIMU();
void readIMU(float &ax, float &ay, float &az,
             float &gx, float &gy, float &gz);

// Quaternion getter (unit quaternion)
void getQuaternion(float &qw, float &qx, float &qy, float &qz);

// Optionally set Madgwick beta (filter gain). Typical: 0.1 - 0.5
void setMadgwickBeta(float b);

#endif


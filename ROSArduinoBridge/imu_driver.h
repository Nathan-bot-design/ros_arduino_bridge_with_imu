#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include <Wire.h>
#include "MPU6050_6Axis_MotionApps20.h"

// Function prototypes
void initIMU();
float getYaw();
float deltaYaw();
void resetYaw();

#endif


// /* *************************************************************
//    Encoder definitions

//    Add an "#ifdef" block to this file to include support for
//    a particular encoder board or library. Then add the appropriate
//    #define near the top of the main ROSArduinoBridge.ino file.

//    ************************************************************ */

// #ifdef USE_BASE

// #ifdef ROBOGAIA
//   /* The Robogaia Mega Encoder shield */
//   #include "MegaEncoderCounter.h"

//   /* Create the encoder shield object */
//   MegaEncoderCounter encoders = MegaEncoderCounter(4); // Initializes the Mega Encoder Counter in the 4X Count mode

//   /* Wrap the encoder reading function */
//   long readEncoder(int i) {
//     if (i == LEFT) return encoders.YAxisGetCount();
//     else return encoders.XAxisGetCount();
//   }

//   /* Wrap the encoder reset function */
//   void resetEncoder(int i) {
//     if (i == LEFT) return encoders.YAxisReset();
//     else return encoders.XAxisReset();
//   }
// #elif defined(ARDUINO_ENC_COUNTER)
//   volatile long left_enc_pos = 0L;
//   volatile long right_enc_pos = 0L;
//   static const int8_t ENC_STATES [] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};  //encoder lookup table

//   /* Interrupt routine for LEFT encoder, taking care of actual counting */
//   ISR (PCINT2_vect){
//   	static uint8_t enc_last=0;

// 	enc_last <<=2; //shift previous state two places
// 	enc_last |= (PIND & (3 << 2)) >> 2; //read the current state into lowest 2 bits

//   	left_enc_pos += ENC_STATES[(enc_last & 0x0f)];
//   }

//   /* Interrupt routine for RIGHT encoder, taking care of actual counting */
//   ISR (PCINT1_vect){
//         static uint8_t enc_last=0;

// 	enc_last <<=2; //shift previous state two places
// 	enc_last |= (PINC & (3 << 4)) >> 4; //read the current state into lowest 2 bits

//   	right_enc_pos += ENC_STATES[(enc_last & 0x0f)];
//   }

//   /* Wrap the encoder reading function */
//   long readEncoder(int i) {
//     if (i == LEFT) return left_enc_pos;
//     else return right_enc_pos;
//   }

//   /* Wrap the encoder reset function */
//   void resetEncoder(int i) {
//     if (i == LEFT){
//       left_enc_pos=0L;
//       return;
//     } else {
//       right_enc_pos=0L;
//       return;
//     }
//   }
// #else
//   #error A encoder driver must be selected!
// #endif

// /* Wrap the encoder reset function */
// void resetEncoders() {
//   resetEncoder(LEFT);
//   resetEncoder(RIGHT);
// }

// #endif

/* *************************************************************
   Encoder definitions with IMU fusion (MPU6050)
   
   Add an "#ifdef" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   ************************************************************ */

#ifdef USE_BASE

#ifdef ROBOGAIA
/* The Robogaia Mega Encoder shield */
#include "MegaEncoderCounter.h"

/* Create the encoder shield object */
MegaEncoderCounter encoders = MegaEncoderCounter(4);  // Initializes the Mega Encoder Counter in the 4X Count mode

/* Wrap the encoder reading function */
long readEncoder(int i) {
  if (i == LEFT) return encoders.YAxisGetCount();
  else return encoders.XAxisGetCount();
}

/* Wrap the encoder reset function */
void resetEncoder(int i) {
  if (i == LEFT) return encoders.YAxisReset();
  else return encoders.XAxisReset();
}

#elif defined(ARDUINO_ENC_COUNTER)

#include "imu_driver.h"        // <-- add your MPU6050 helper here

volatile long left_enc_pos = 0L;
volatile long right_enc_pos = 0L;
static const int8_t ENC_STATES[] = { 0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0 };  // encoder lookup table

// --- Robot geometry constants (adjust these!) ---
const float TICKS_PER_REV = 1135.0;  // encoder ticks per wheel revolution
const float WHEEL_RADIUS = 0.0425;     // meters
const float WHEEL_CIRC = 2.0 * 3.14159 * WHEEL_RADIUS;
const float WHEEL_BASE = 0.15;      // meters (distance between wheels)
// const float YAW_THRESHOLD = 0.087;  // ~5 degrees in radians
// Slip detection thresholds
#define YAW_THRESHOLD 0.087f   // ~5 degrees (radians)
#define DIST_THRESHOLD 0.01f   // meters per update, ignore smaller moves
#define ACCEL_THRESHOLD 0.05f  // m/s^2, must detect some accel for real motion


/* Interrupt routine for LEFT encoder */
ISR(PCINT2_vect) {
  static uint8_t enc_last = 0;
  enc_last <<= 2;                      // shift previous state two places
  enc_last |= (PIND & (3 << 2)) >> 2;  // read the current state
  left_enc_pos += ENC_STATES[(enc_last & 0x0f)];
}

/* Interrupt routine for RIGHT encoder */
ISR(PCINT1_vect) {
  static uint8_t enc_last = 0;
  enc_last <<= 2;
  enc_last |= (PINC & (3 << 4)) >> 4;
  right_enc_pos += ENC_STATES[(enc_last & 0x0f)];
}

/* Wrap the encoder reading function with IMU fusion */
long readEncoder(int i) {
  static long last_left = 0;
  static long last_right = 0;

  long val;
  if (i == LEFT) val = left_enc_pos;
  else val = right_enc_pos;

  // Only check fusion once (when RIGHT is requested)
  if (i == RIGHT) {
    long delta_left = left_enc_pos - last_left;
    long delta_right = right_enc_pos - last_right;

    // Distances from encoders
    float dist_left = (delta_left / TICKS_PER_REV) * WHEEL_CIRC;
    float dist_right = (delta_right / TICKS_PER_REV) * WHEEL_CIRC;

    // Encoder-based motion estimates
    float delta_theta_enc = (dist_right - dist_left) / WHEEL_BASE;
    float delta_s_enc = (dist_right + dist_left) / 2.0;

    // IMU-based readings
    float delta_theta_imu = deltaYaw();  // from imu.h
    float accel_x = getIMUAccelX();      // you expose this in imu_driver.ino
    float omega_z = getIMUYawRate();

    bool slip = false;

    // --- TURN SLIP CHECK ---
    if (fabs(delta_theta_enc - delta_theta_imu) > YAW_THRESHOLD) {
      slip = true;
    }

    // --- FORWARD / BACKWARD SLIP CHECK ---
    if (fabs(delta_s_enc) > DIST_THRESHOLD && fabs(accel_x) < ACCEL_THRESHOLD) {
      slip = true;
    }

    if (slip) {
      // Reject this update → reset encoder counts
      left_enc_pos = last_left;
      right_enc_pos = last_right;
      val = (i == LEFT) ? last_left : last_right;
    } else {
      // Accept update
      last_left = left_enc_pos;
      last_right = right_enc_pos;
    }
  }

  return val;
}


/* Wrap the encoder reset function */
void resetEncoder(int i) {
  if (i == LEFT) {
    left_enc_pos = 0L;
    return;
  } else {
    right_enc_pos = 0L;
    return;
  }
}

#else
#error A encoder driver must be selected!
#endif

/* Wrap the encoder reset function */
void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}

#endif

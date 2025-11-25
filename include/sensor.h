#ifdef BUILD_TX

#pragma once
#include "MPU6050.h"
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>

#define CLEAR_MARGIN 35     // what we consider "clear"
#define TURN_SPEED 128      // motor PWM for turning
#define TURN_CHECK_DELAY 10 // ms per integration step
#define GYRO_SCALE 131.0    // from MPU6050 datasheet (LSB per °/s)
#define THRESHOLD 30        // Threshold distance for obstacle detection and decision
#define ALPHA 0.98          // MPU Trust Factor
#define ZUPT_WIN 20         // window size for ZUPT detection (samples)
#define ZUPT_VAR_THRESH 0.02f
#define ZUPT_MAG_THRESH 0.25f

#define FRONT_US_TRIG 23 // BLACK
#define FRONT_US_ECHO 21 // GRAY

#define LEFT_US_TRIG 4  // PURPLE
#define LEFT_US_ECHO 34 // GRAY

#define RIGHT_US_TRIG 5  // RED
#define RIGHT_US_ECHO 35 // BROWN

#define MPU_SCL 22 // ORANGE
#define MPU_SDA 19 // YELLOW

#define GPS_RX 33 // connect to NEO-6M TX YELLOW
#define GPS_TX 32 // connect to NEO-6M RX BLUE

struct Angle {
        double pitch;
        double yaw;
        double roll;
};

double getDistance(int TRIG, int ECHO);
void getAccValues();
float updateYaw();
void sensorInit();
void calibrateGyro(int samples);
void getAngle();
void sensorUpdate();
void resetYaw();

extern MPU6050 mpu;
extern float accArr[3];
extern unsigned long lastGyroTime;
extern float gyroXoffset;
extern float gyroYoffset;
extern float gyroZoffset;
extern Angle old;
extern TinyGPSPlus gps;
extern HardwareSerial GPS_Serial;

extern float posX;
extern float posY;
extern float posX_imu;
extern float posY_imu;
extern float velX;
extern float velY;
#endif

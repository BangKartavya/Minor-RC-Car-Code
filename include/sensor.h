#pragma once
#include "MPU6050.h"

#define FRONT_US_TRIG 23 // BLACK
#define FRONT_US_ECHO 21 // GRAY

#define LEFT_US_TRIG 17 // PURPLE
#define LEFT_US_ECHO 18 // GRAY

#define RIGHT_US_TRIG 15 // RED
#define RIGHT_US_ECHO 2  // BROWN

#define THRESHOLD 30 // Threshold distance for obstacle detection and decision

#define MPU_SCL 22 // ORANGE
#define MPU_SDA 19 // YELLOW

double getDistance(int TRIG, int ECHO);
void getAccValues();

extern MPU6050 mpu;
extern float accArr[3];

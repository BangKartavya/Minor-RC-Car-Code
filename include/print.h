#ifdef BUILD_TX
#pragma once

#include <Arduino.h>
#include "sensor.h"

// --- ANSI helpers ---
#define ESC "\x1B"
#define CSI ESC "["

// Colours
#define RED CSI "31m"
#define GREEN CSI "32m"
#define YELLOW CSI "33m"
#define CYAN CSI "36m"
#define MAGENTA CSI "35m"
#define WHITE CSI "37m"
#define RESET CSI "0m"

void printTableRow(double f, double l, double r, float ax, float ay, float az, int spd);
void printTableHeader();
void ansiMoveCursor(int row, int col);
void ansiClear();

extern double frontVal;
extern double leftVal;
extern double rightVal;
extern float ax;
extern float ay;
extern float az;
extern int speedVal;

#endif

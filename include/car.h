#pragma once

#define IN1 14 // BLACK
#define IN2 27 // WHITE
#define IN3 26 // GRAY
#define IN4 25 // PURPLE
#define ENA 12 // BLUE
#define ENB 13 // GREEN

extern int SPEED;
extern bool shouldBeAutomated;

void left();
void right();
void forward();
void backward();
void stop();
void handleCommand(char cmd);
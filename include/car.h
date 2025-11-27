#ifdef BUILD_TX

#pragma once

#define IN1 14 // BLACK
#define IN2 27 // WHITE
#define IN3 26 // GRAY
#define IN4 25 // PURPLE
#define ENA 12 // BLUE
#define ENB 13 // GREEN

extern int SPEED;
extern bool shouldBeAutomated;
extern bool turningFastLoop;

enum TurnState {
    TURN_IDLE,
    TURN_START,
    TURN_EXECUTE,
    TURN_FINISH
};

extern TurnState turnState;

void left();
void carInit();
void right();
void forward();
void backward();
void stop();
void handleCommand(char cmd);
void resetYaw();
void turnAngle(float targetAngle, bool leftTurn);
void updateTurnStateMachine();
void startTurn(float angleDeg, bool left);
void updateTurn();

#endif
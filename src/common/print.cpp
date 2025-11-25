#include "print.h"
// ============================
// Beautiful live dashboard for PlatformIO Serial Monitor
// ============================

// Example sensor values (replace with your actual reads)
double frontVal = 0.0, leftVal = 0.0, rightVal = 0.0;
float ax = 0, ay = 0, az = 0;
int speedVal = 0;

// --- ANSI helper functions ---
void ansiClear() {
    Serial.write(27);
    Serial.print("[2J"); // Clear screen
    Serial.write(27);
    Serial.print("[H"); // Cursor to home
}

void ansiMoveCursor(int row, int col) {
    Serial.write(27);
    Serial.print("[");
    Serial.print(row);
    Serial.print(";");
    Serial.print(col);
    Serial.print("H");
}

// --- Print table header once ---
void printTableHeader() {
    ansiClear();
    Serial.println("+--------------------------------------------------------------------------------------------------------------+");
    Serial.println("|  Front(cm) |  Left(cm) |  Right(cm) |   AccX(g) |   AccY(g) |   AccZ(g) | Spd | Angle(X) | Angle(Y) | Angle(Z)");
    Serial.println("+--------------------------------------------------------------------------------------------------------------+");
    Serial.println("|            |           |            |           |           |           |     |          |          |         ");
    Serial.println("+--------------------------------------------------------------------------------------------------------------+");
}
void printTableRow(double f, double l, double r,
                   float ax, float ay, float az, int spd) {
    ansiMoveCursor(4, 3); // row 4, column 3
    char buf[200];

    // Colour logic
    const char* fColor = (f < 30) ? RED : (f < 60) ? YELLOW
                                                   : GREEN;
    const char* lColor = (l < 30) ? RED : (l < 60) ? YELLOW
                                                   : GREEN;
    const char* rColor = (r < 30) ? RED : (r < 60) ? YELLOW
                                                   : GREEN;
    const char* spdColor = (spd > 200) ? MAGENTA : (spd > 100) ? CYAN
                                                               : WHITE;

    snprintf(buf, sizeof(buf),
             "%s%8.2f%s | %s%8.2f%s | %s%9.2f%s | %s%9.3f%s | %s%9.3f%s | %s%9.3f%s | %s%3d%s ",
             fColor, f, RESET,
             lColor, l, RESET,
             rColor, r, RESET,
             CYAN, ax, RESET,
             CYAN, ay, RESET,
             CYAN, az, RESET,
             spdColor, spd, RESET,
             CYAN, old.roll, RESET,
             CYAN, old.pitch, RESET,
             CYAN, old.yaw, RESET);

    Serial.print(buf);
    Serial.print("   "); // erase leftovers
}
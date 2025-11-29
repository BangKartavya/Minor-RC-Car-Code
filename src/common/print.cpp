#include "print.h"

LiveColumn* LiveTable::Cols = nullptr;
int LiveTable::RowY = 0;
int LiveTable::Count = 0;

void LiveTable::AnsiClear() {
    Serial.write(27);
    Serial.print("[2J"); // Clear screen
    Serial.write(27);
    Serial.print("[H"); // Cursor to home
}

void LiveTable::AnsiMoveCursor(int row, int col) {
    Serial.write(27);
    Serial.print("[");
    Serial.print(row);
    Serial.print(";");
    Serial.print(col);
    Serial.print("H");
}

LiveTable*& LiveTable::Instance() {
    static LiveTable* liveTable;
    return liveTable;
}

void LiveTable::Configure(LiveColumn* column, int count, int row) {
    Cols = column;
    LiveTable::Count = count;
    RowY = row;
}

void LiveTable::Init(double vals[]) {
    AnsiClear();

    Serial.print("+");
    for(int i = 0; i < Instance()->Count; i++) {
        for(int j = 0; j < Instance()->Cols[i].width + 2; j++)
            Serial.print("-");
        Serial.print("+");
    }
    Serial.println();

    Serial.print("|");
    for(int i = 0; i < Instance()->Count; i++) {
        Serial.printf(" %-*s |", Instance()->Cols[i].width, Instance()->Cols[i].header);
    }
    Serial.println();

    Serial.print("+");
    for(int i = 0; i < Instance()->Count; i++) {
        for(int j = 0; j < Instance()->Cols[i].width + 2; j++)
            Serial.print("-");
        Serial.print("+");
    }
    Serial.println();
}

void LiveTable::Update(double vals[]) {
    AnsiMoveCursor(Instance()->RowY, 1);
    Serial.print("|");

    for(int i = 0; i < Instance()->Count; i++) {
        char buf[32];
        snprintf(buf, sizeof(buf), Instance()->Cols[i].fmt, vals[i]);

        const char* c = Instance()->Cols[i].color ? Instance()->Cols[i].color(vals[i]) : RESET;

        Serial.print(c);
        Serial.printf(" %-*s ", Instance()->Cols[i].width, buf);
        Serial.print(RESET);
        Serial.print("|");
    }

    Serial.println();
}
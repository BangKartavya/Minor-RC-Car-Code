#pragma once

#include <Arduino.h>

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

typedef const char* (*ColourFunction)(double v);

struct LiveColumn {
        const char* header; // column title
        const char* fmt;    // printf format string, e.g. "%.2f"
        int width;          // column width
        ColourFunction color;
};

class LiveTable {
    private:
        LiveTable();

    private:
        static void AnsiClear();
        static void AnsiMoveCursor(int row, int col);

    public:
        static LiveTable*& Instance();
        static void Configure(LiveColumn* column, int count, int row);
        static void Init(double vals[]);
        static void Update(double vals[]);

    public:
        static LiveColumn* Cols;
        static int Count;
        static int RowY; // which row to print the live row on
};
#ifndef MAIN_H
#define MAIN_H

enum class Mode
{
    kRover,
    kBase
};

void PrintEEPROM();
void PrintNarwin();
void PrintStatus();

#endif
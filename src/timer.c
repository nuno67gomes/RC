#include "timer.h"
#include <stdio.h>

int alarmEnabled = 0;
int alarmCount = 0;

void alarmHandler(int signal) {
    alarmEnabled = 0;
    alarmCount++;
    printf("[TIMER] Alarm #%d triggered\n", alarmCount);
}

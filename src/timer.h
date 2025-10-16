#ifndef TIMER_H
#define TIMER_H

#include <signal.h>

extern int alarmEnabled;
extern int alarmCount;

void alarmHandler(int signal);

#endif

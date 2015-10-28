// schedule the sensor module to achieve a certain sampling rate and low power consumption
#ifndef RESET_H
#define RESET_H

#include "routines.h"

void timer_init(void);
void start_scheduler(void);
void update_status(int status);
int check_status(void);
void timer_rst(void);
void setDutyCycle(int dutyCycle, int frequency);


extern unsigned int done;
extern unsigned int curr_channel;
extern unsigned int freqCounter;
extern int high;
extern int low;
extern int totalTicks;
extern int cycles;

#endif
// schedule the sensor module to achieve a certain sampling rate and low power consumption
#ifndef RESET_H
#define RESET_H

#include "routines.h"

void timer_init(void);
void start_scheduler(void);
void update_status(int status);
int check_status(void);
void timer_rst(void);

extern unsigned int done;
extern unsigned int curr_channel;
extern unsigned int freqCounter;
void init_delay(void);
#endif
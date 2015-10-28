
#ifndef MOTOR_DRIVE_H
#define	MOTOR_DRIVE_H
#include "routines.h"

#define SPEED_REPORT_RATE 60
#define THROTTLE_LOW 4000
#define ZERO_SPEED_THROTTLE 700 // throttle that just enough to keep to motors still
#define MAX_THROTTLE 999999999
#define SPEED2THROTTLE_FACTOR 1


#define MOTOR_TIMER_PRESCALE 0b10

extern unsigned int motorAll, high1, high2, high3, high4;
extern unsigned int totalTicks;
extern unsigned char motor_on;

int setDutyCycle(unsigned int throttle, unsigned int motor_num);
void motors_init(void);
void start_motor_timers(void);
void motors_on(unsigned char on_noff);
unsigned int controlled_speed(void);
unsigned int normalize_speed(double speed);
#endif




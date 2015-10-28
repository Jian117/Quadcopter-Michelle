#ifndef STABILIZER_H
#define STABILIZER_H

#include "routines.h"

/****************************************************
*            EE478 quadcopter stabilizer            *
* The motor speeds parameters are stabilized here   *
****************************************************/

typedef struct 
{
    double roll_error;
    double pitch_error;
    double yaw_error;
    double altitude_error;
} kinetic_errors;

typedef struct
{
    double en0;
    double en1;
    double en2;        // error[n], error[n-1], error[n-2]
    double fn0;
    double fn1;
    double fn2;        // output[n-1], output[n-1], output[n-2]
} PID_storage  ;

extern kinetic_errors* kinetic_errs;
extern PID_storage* roll_PID;
extern PID_storage* pitch_PID;
extern PID_storage* yaw_PID;

void stabilize_init(kinetic_errors*, PID_storage*,  PID_storage*, PID_storage*);
unsigned int stablize_routine(void);
unsigned char roll_stablize(void);
unsigned char pitch_stablize (void);
unsigned char yaw_stablize (void);
unsigned char altitude_stabilize (void);
void update_errors(void);
void inverted_motion_matrix(void);
unsigned int stabilize_routine(void);
void pid_clr(void);
#endif


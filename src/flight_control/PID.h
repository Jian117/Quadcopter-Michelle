#ifndef PID_H
#define PID_H

#include "routines.h"
/****************************************************
*            EE478 quadcopter PID controller        *
*                   PID controller                  *
****************************************************/

typedef struct 
{ 
    // Constants for the PID controller 
    unsigned char Kp; // Proportion gain
    unsigned char Ki; // Integral gain
    unsigned char Kd; // Differential gain

	// Variables for the PID controller
    unsigned int c_error;   // current error
    unsigned int a_error; // accumulative error
    unsigned int p_error; // previous error
    unsigned char error_limit1;
    unsigned char error_limit2;
	
    short long derivative;
    short long integral;
    short long proportion;
    short long PID_result;
	
        // Status bits
    unsigned f_pid_sign  :  1 ; // sign bit for final PID result
    unsigned d_pid_sign  :  1 ; // sign bit for the derivative term
    unsigned p_err_sign  :  1 ; // sign bit for p_error
    unsigned a_err_sign  :  1 ; // sign bit for a_error
    unsigned a_err_zero  :  1 ; // sign bit for a_error
    unsigned d_err_z     :  1 ; // indicate if the data error is zero
    unsigned err_zero    :  1 ; // indicate if the error is zero
    unsigned mag         :  1 ; // indicate which variable is greater in magnitude (AARGB, BARGB)
} PID_storage; 



#endif
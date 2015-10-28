#include "stabilizer.h""

kinetic_errors* kinetic_errs;
PID_storage* roll_PID;
PID_storage* pitch_PID;
PID_storage* yaw_PID;

void stabilize_init(kinetic_errors* kinetic_errors_ptr, PID_storage* roll,
                    PID_storage* pitch, PID_storage* yaw)
{
    kinetic_errs = kinetic_errors_ptr;
    // initialize structs
    kinetic_errs->altitude_error = 0;
    kinetic_errs->pitch_error    = 0;
    kinetic_errs->roll_error     = 0;
    kinetic_errs->yaw_error      = 0;

    // initilize PIDs
    roll_PID = roll;
    pitch_PID = pitch;
    yaw_PID = yaw;
    pid_clr();
}

void pid_clr(void)
{
    roll_PID->en0 = 0.0;
    roll_PID->en1 = 0.0;
    roll_PID->en2 = 0.0;
    roll_PID->fn0 = 0.0;
    roll_PID->fn1 = 0.0;
    roll_PID->fn2 = 0.0;

    pitch_PID->en0 = 0.0;
    pitch_PID->en1 = 0.0;
    pitch_PID->en2 = 0.0;
    pitch_PID->fn0 = 0.0;
    pitch_PID->fn1 = 0.0;
    pitch_PID->fn2 = 0.0;

    yaw_PID->en0 = 0.0;
    yaw_PID->en1 = 0.0;
    yaw_PID->en2 = 0.0;
    yaw_PID->fn0 = 0.0;
    yaw_PID->fn1 = 0.0;
    yaw_PID->fn2 = 0.0;
}

// lower level stabilizers to approximate hovering state 
unsigned int stabilize_routine(void)
{
    update_errors();
    roll_stablize();
    pitch_stablize();
    altitude_stabilize();
    yaw_stablize();
    inverted_motion_matrix();
    return 1;
}

unsigned char altitude_stabilize(void)
{
    // TODO: make the altitude controlled
    ctrl_data->U1 = controller_param->lift;
}

unsigned char roll_stablize(void)
{ 
    roll_PID->en0 = kinetic_errs->roll_error;
    // compute next output
    roll_PID->fn0 = 1.055* roll_PID->en0 - 1.007*roll_PID->en1 + 0.7917*roll_PID->fn1;
    ctrl_data->U2 = roll_PID->fn0;
    // store prev data
    roll_PID->en2 = roll_PID->en1;
    roll_PID->en1 = roll_PID->en0;
    roll_PID->fn2 = roll_PID->fn1;
    roll_PID->fn1 = roll_PID->fn0;
    return 1;
} 
  
unsigned char pitch_stablize (void)
{
    pitch_PID->en0 = (double)kinetic_errs->pitch_error;
    // compute next output
    roll_PID->fn0 = 1.055* roll_PID->en0 - 1.007*roll_PID->en1 + 0.7917*roll_PID->fn1;
    ctrl_data->U3 = pitch_PID->fn0;
    // store prev data
    pitch_PID->en2 = pitch_PID->en1;
    pitch_PID->en1 = pitch_PID->en0;
    pitch_PID->fn2 = pitch_PID->fn1;
    pitch_PID->fn1 = pitch_PID->fn0;
    return 1;
}

unsigned char yaw_stablize (void)
{
    yaw_PID->en0 = (double)kinetic_errs->yaw_error;
    // compute next output
    roll_PID->fn0 = 1.105* roll_PID->en0 - 1.07*roll_PID->en1 + 0.8535*roll_PID->fn1;
    ctrl_data->U4 = yaw_PID->fn0;
    // store prev data
    yaw_PID->en2 = yaw_PID->en1;
    yaw_PID->en1 = yaw_PID->en0;
    yaw_PID->fn2 = yaw_PID->fn1;
    yaw_PID->fn1 = yaw_PID->fn0;
    return 1;
} 

// compute the system error 
void update_errors(void)
{
    kinetic_errs->altitude_error = 0;
    kinetic_errs->pitch_error = controller_param->pitch_target - ctrl_data->pitch;
    kinetic_errs->roll_error = controller_param->roll_target - ctrl_data->roll;
    kinetic_errs->yaw_error = controller_param->yaw_target - ctrl_data->yaw;
}
 
// inverted motion matrix operation
void inverted_motion_matrix(void)
{
    double temp1, temp2, temp3, temp4, temp5;
    temp5 = (quad_intrinsics->l*quad_intrinsics->b);
    temp1 = ctrl_data->U1/4/quad_intrinsics->b;
    temp2 = ctrl_data->U2/2/temp5;
    temp3 = ctrl_data->U3/2/temp5;
    temp4 = ctrl_data->U4/4/quad_intrinsics->d;

    ctrl_data->omega1square = temp1 - temp3 - temp4;
    ctrl_data->omega2square = temp1 - temp2 + temp4;
    ctrl_data->omega3square = temp1 + temp3 - temp4;
    ctrl_data->omega4square = temp1 + temp2 + temp4;
}

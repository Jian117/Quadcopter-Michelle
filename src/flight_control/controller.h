#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "routines.h"

#define MANUAL_CTRL 0
#define AUTOPILOT  1

// flight controller state
#define FCS_PWRON    0
#define FCS_ZERO     1
#define FCS_TAKEOFF  2
#define FCS_HOVERING 3
#define FCS_CRASH    4    //emergency landing
#define FCS_OFFSET   5    
#define FCS_LANDING  6    //land gracefully
#define FCS_AUTO_F   7    //auto forward
#define FCS_AUTO_B   8    //auto backward
#define FCS_AUTO_L   9    //auto left
#define FCS_AUTO_R   10   //auto right
#define FCS_AUTO_U   11   //auto up
#define FCS_AUTO_D   12   //auto down
#define FCS_AUTO_YL  13   //auto yaw left
#define FCS_AUTO_YR  14   //auto yaw right
#define FCS_MOTOR_TEST   15

#define YAW_LOW_BOND -180.0   // -180 degree < Yaw < 180 degree
#define YAW_HIGH_BOND 180.0
#define PITCH_LOW_BOND -10  // -10 degree < Pitch < 10 degree
#define PITCH_HIGH_BOND 10
#define ROLL_LOW_BOND -10   //  -10 degree < Roll < 10 degree
#define ROLL_HIGH_BOND 10
#define MAX_LIFT 99999999 // No max lift limit
#define YAW_UNIT_CHANGE 30 // rotate the yaw by 30 degrees every time you press it.
#define ROLL_UNIT_CHANGE 1 // rotate the roll by 1 degrees every time you press it.
#define PITCH_UNIT_CHANGE 1 // rotate the pitch by 1 degrees every time you press it.
#define LIFT_UNIT_CHANGE 1 // increase 1 N everytime we press it.
#define MOTOR_TEST_STEP 50

typedef struct 
{
    unsigned char  FCS; // Flight Controller State
    unsigned int  controller_counter; // used to time maneuvers
    double  roll_target; // -10 degrees to 10 degrees
    double  pitch_target; // -10 degrees to 10 degrees
    double  yaw_target;   // -179 degrees to 180 degrees
    double  lift; // 0 to MAX lift
    unsigned int  auto_manual_sel; // selecting autopilot or manual control
    unsigned char manual_is_pressed;
    unsigned char manual_key_pressed;
} controller_parameters;

typedef union
{
    struct {
        double U1;
        double U2;
        double U3;
        double U4;
    };
    struct {
        double roll;
        double pitch;
        double yaw;
        double altitude;
    };
    struct {
        double omega1square;
        double omega2square;
        double omega3square;
        double omega4square;
    };
} controller_data;

typedef struct 
{
    double  m;  //mass of the quad
    double IXX; //moment of inertia of the quad around the roll axis
    double IYY; //moment of inertia of the quad around the pitch axis
    double IZZ; //moment of inertia of the quad around the yaw axis
    double d;   // drag factor
    double b;   // trust factor
    double l;   // distance between the center of the rotor to the center of the quadcopter fram
    double hl;  // hovering lift
} quadcopter_intrinsics;

extern controller_parameters* controller_param;
extern controller_data* ctrl_data;
extern quadcopter_intrinsics* quad_intrinsics;
extern unsigned int* test_speed;

unsigned int flight_control_routine(void);
void process_command(unsigned char byte_instruction);
void controller_init(quadcopter_intrinsics*, controller_data*, controller_parameters*, unsigned int*);
void update_target_roll(double new_roll);
void update_target_pitch(double new_pitch);
void update_target_yaw(double new_yaw);
void update_lift(double new_lift);


// autopilot
void zero_quadcopter(void);
void take_off(void);
void crash_landing(void);
void controlled_landing(void);
void panorama_maneuver(void);
void foward(unsigned int distance);
void backward(unsigned int distance);
void left(unsigned int distance);
void right(unsigned int distance);
void up(unsigned int distance);
void down(unsigned int distance);
void yaw_CW(unsigned int degrees);
void yaw_CCW(unsigned int degrees);

// manuel control
void offset_clear(void);
void motor_test(unsigned int sign, unsigned int value, unsigned int motor_num);
void lift_offset(unsigned int sign, unsigned int value);
void roll_offset(unsigned int sign, unsigned int value);
void pitch_offset(unsigned int sign, unsigned int value);
void yaw_offset(unsigned int sign, unsigned int value);
void inverted_motion_matrix(void);


#endif
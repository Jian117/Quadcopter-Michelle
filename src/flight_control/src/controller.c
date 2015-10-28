#include "controller.h"

/****************************************************
*            EE478 quadcopter controller            *
* The motor speeds parameters are offsetted here    *
****************************************************/

controller_parameters* controller_param;
quadcopter_intrinsics* quad_intrinsics;
controller_data* ctrl_data;
unsigned int* test_speed;

void controller_init(quadcopter_intrinsics* qi,
        controller_data* cd,
        controller_parameters* cp,
        unsigned int* t)
{
    int i;
    
   //init the controller data storage
    quad_intrinsics = qi;
    ctrl_data = cd;
    controller_param = cp;
    test_speed = t;
    
    ctrl_data->U1 = 0;
    ctrl_data->U2 = 0;
    ctrl_data->U3 = 0;
    ctrl_data->U4 = 0;

    // init quadintrinsics data
    quad_intrinsics->IXX = 0.1296;
    quad_intrinsics->IYY = 0.1296;
    quad_intrinsics->IZZ = 0.2272;
    // total mass : 1800 g
    quad_intrinsics->m = 1.8;
    // Motor / Center distance: 32 cm
    quad_intrinsics->l = 0.32;
    quad_intrinsics->b = 0.538;
    quad_intrinsics->d = 0.011;
    quad_intrinsics->hl = 17.64;
    // Force needed to lift to quad: 17.64 N

    controller_param->FCS = 0;
    controller_param->auto_manual_sel = MANUAL_CTRL;
    controller_param->controller_counter = 0;
    controller_param->pitch_target = 0;
    controller_param->roll_target = 0;
    controller_param->yaw_target = 0;
    controller_param->manual_is_pressed = 0;

    for (i = 0; i < 4; i++) {
        test_speed[i] = ZERO_SPEED_THROTTLE;
    }
}

// PROCESS_COMMAND
//         switch the controller state according to input state
//         and reset the controller_counter. The point is to 
//         achieve the state diagram for the controller(see doc)
//
// PARAM   byte_command  : command as requested by the PI
void process_command(unsigned char byte_instruction)
{
    // auto manual switch
    if (byte_instruction == BYTE_INSTRUCTION_AUTO_MANUAL) {
        if (controller_param->auto_manual_sel == AUTOPILOT) {
            printf("\n\rSwitch to manual control.\n\r");
            controller_param->auto_manual_sel = MANUAL_CTRL;
        } else {
            printf("\n\rSwitch to autopilot\n\r");
            controller_param->auto_manual_sel = AUTOPILOT;
            if (controller_param->FCS == FCS_OFFSET) {
                controller_param->FCS = FCS_HOVERING;
                controller_param->controller_counter = 0;
            }
        }
    // abort mission
    } else if (byte_instruction == BYTE_INSTRUCTION_ABORT){
        printf("\n\rAbort mission, start crash landing sequence\n\r");
        controller_param->FCS = FCS_CRASH;
        controller_param->controller_counter = 0;
    } else if (byte_instruction == BYTE_INSTRUCTION_PIC) {
        printf("\n\rCheese!\n\r");
    } else if (byte_instruction == BYTE_INSTRUCTION_VID) {
        printf("\n\The big brother is watching you!\n\r");
    } else if (byte_instruction == BYTE_INSTRUCTION_CLR) {
        printf("\n\rCleaning multimedia storage.\n\r");
    } else {
            // state FCS_PWRON
        if (controller_param->FCS == FCS_PWRON) {
            // power on to zero
            if (byte_instruction == BYTE_INSTRUCTION_ZERO) {
                controller_param->controller_counter = 0;
                controller_param->FCS = FCS_ZERO;
                printf("\n\rZeroing quadcopter\n\r");
            } else if (byte_instruction == BYTE_INSTRUCTION_MOTOR_TEST) {
                controller_param->FCS = FCS_MOTOR_TEST;
                controller_param->controller_counter = 0;
                printf("\n\rEnter motor testing mode\n\r");
            } else {
                error = INVALID_COMMAND;
            }
        // state FCS_ZREO
        } else if (controller_param->FCS == FCS_ZERO) {
            // rezero the quadcopter
            if (byte_instruction == BYTE_INSTRUCTION_ZERO) {
                controller_param->controller_counter = 0;
                controller_param->FCS = FCS_ZERO;
                printf("\n\rRezeroing quadcopter\n\r");
            // zero to take-off
            } else if (byte_instruction == BYTE_INSTRUCTION_TAKEOFF) {
                controller_param->controller_counter = 0;
                motor_on = 1;
                controller_param->FCS = FCS_TAKEOFF;
                printf("\n\rInit take-off sequence\n\r");
            } else {
                error = INVALID_COMMAND;
            }
        // state FCS_TAKEOFF
        } else if (controller_param->FCS == FCS_TAKEOFF) {
            // Ain't no command mess with take-off sequence. Ain't no command.
            // http://static.fjcdn.com/pictures/Ain+t+no+one+fucks+with+tiny+hippo_7541d9_3298343.jpg
            error = INVALID_COMMAND;
        // state FCS_HOVERING
        } else if (controller_param->FCS == FCS_HOVERING) {
            if (byte_command == BYTE_INSTRUCTION_CONTROLLED_LANDING) {
                controller_param->controller_counter = 0;
                controller_param->FCS = FCS_LANDING;
                printf("\n\rInit landing sequence\n\r");
            } else {
                // Autopilot manuvers
                if (controller_param->auto_manual_sel == AUTOPILOT) {
                    if (byte_instruction == BYTE_INSTRUCTION_W) {
                        controller_param->controller_counter = 0;
                        controller_param->FCS = FCS_AUTO_F;
                        printf("Autopilot moving forward.\n\r");
                    } else if (byte_instruction == BYTE_INSTRUCTION_S) {
                        controller_param->controller_counter = 0;
                        controller_param->FCS = FCS_AUTO_B;
                        printf("Autopilot moving backward.\n\r");
                    } else if (byte_instruction == BYTE_INSTRUCTION_A) {
                        controller_param->controller_counter = 0;
                        controller_param->FCS = FCS_AUTO_L;
                        printf("Autopilot moving left.\n\r");
                    } else if (byte_instruction == BYTE_INSTRUCTION_D) {
                        controller_param->controller_counter = 0;
                        controller_param->FCS = FCS_AUTO_R;
                        printf("Autopilot moving right.\n\r");
                    } else if (byte_instruction == BYTE_INSTRUCTION_Q) {
                        controller_param->controller_counter = 0;
                        controller_param->FCS = FCS_AUTO_YL;
                        printf("Autopilot yaw left.\n\r");
                    } else if (byte_instruction == BYTE_INSTRUCTION_E) {
                        controller_param->controller_counter = 0;
                        controller_param->FCS = FCS_AUTO_YR;
                        printf("Autopilot yaw right.\n\r");
                    } else if (byte_instruction == BYTE_INSTRUCTION_R) {
                        controller_param->controller_counter = 0;
                        controller_param->FCS = FCS_AUTO_U;
                        printf("Autopilot elevation up.\n\r");
                    } else if (byte_instruction == BYTE_INSTRUCTION_F) {
                        controller_param->controller_counter = 0;
                        controller_param->FCS = FCS_AUTO_D;
                        printf("Autopilot elevation down.\n\r");
                    } else {
                    error = INVALID_COMMAND;
                    }
               // Manual manuvers
               } else {
                    if (byte_instruction == BYTE_INSTRUCTION_W ||
                        byte_instruction == BYTE_INSTRUCTION_A ||
                        byte_instruction == BYTE_INSTRUCTION_S ||
                        byte_instruction == BYTE_INSTRUCTION_D ||
                        byte_instruction == BYTE_INSTRUCTION_E ||
                        byte_instruction == BYTE_INSTRUCTION_Q ||
                        byte_instruction == BYTE_INSTRUCTION_R ||
                        byte_instruction == BYTE_INSTRUCTION_F) {
                        controller_param->controller_counter = 0;
                        controller_param->FCS = FCS_OFFSET;
                        controller_param->manual_is_pressed = 1;
                        controller_param->manual_key_pressed = byte_instruction;
                    } else {
                        error = INVALID_COMMAND;
                    }
                }
            }
        // state FCS_OFFSET
        } else if (controller_param->FCS == FCS_OFFSET){
            if (byte_command == BYTE_INSTRUCTION_AUTO_BALANCE) {
                controller_param->controller_counter = 0;
                controller_param->FCS = FCS_HOVERING;
                offset_clear();
                printf("Auto balance quadcopter.\n\r");
            } else if (byte_instruction == BYTE_INSTRUCTION_W ||
                        byte_instruction == BYTE_INSTRUCTION_A ||
                        byte_instruction == BYTE_INSTRUCTION_S ||
                        byte_instruction == BYTE_INSTRUCTION_D ||
                        byte_instruction == BYTE_INSTRUCTION_E ||
                        byte_instruction == BYTE_INSTRUCTION_Q ||
                        byte_instruction == BYTE_INSTRUCTION_R ||
                        byte_instruction == BYTE_INSTRUCTION_F) {
                controller_param->manual_is_pressed = 1;
                controller_param->manual_key_pressed = byte_instruction;
            } else {
               error = INVALID_COMMAND;
            }
        // state FCS_LANDING
        } else if (controller_param->FCS == FCS_OFFSET){
            // Ain't no command mess with landing sequence. Ain't no command.
            // http://static.fjcdn.com/pictures/Ain+t+no+one+fucks+with+tiny+hippo_7541d9_3298343.jpg
            error = INVALID_COMMAND;
        // state FCS_CRASH
        } else if (controller_param->FCS == FCS_CRASH){
            // Ain't no command mess with crash landing sequence. Ain't no command.
            // http://static.fjcdn.com/pictures/Ain+t+no+one+fucks+with+tiny+hippo_7541d9_3298343.jpg
            error = INVALID_COMMAND;
        } else if (controller_param->FCS == FCS_MOTOR_TEST) {
            if (byte_instruction == BYTE_INSTRUCTION_W ||
                        byte_instruction == BYTE_INSTRUCTION_A ||
                        byte_instruction == BYTE_INSTRUCTION_S ||
                        byte_instruction == BYTE_INSTRUCTION_D ||
                        byte_instruction == BYTE_INSTRUCTION_E ||
                        byte_instruction == BYTE_INSTRUCTION_Q ||
                        byte_instruction == BYTE_INSTRUCTION_R ||
                        byte_instruction == BYTE_INSTRUCTION_F) {
                controller_param->manual_is_pressed = 1;
                motor_on = 1;
                controller_param->manual_key_pressed = byte_instruction;
            } else {
               error = INVALID_COMMAND;
            }
        }
    }
}

// TERMINATE_SEQUENCE
//         Terminate the current sequence and change states accordingly
//         The point is to achieve the state diagram for the controller(see doc)
void terminate_sequence(void)
{
    // terminating take-off sequence
    if (controller_param->FCS == FCS_TAKEOFF) {
        printf("Take-off Sequence complete\n\r");
        controller_param->FCS = FCS_HOVERING;
        controller_param->controller_counter = 0;
    // terminating crash-landing sequence
    } else if (controller_param->FCS == FCS_CRASH){
        printf("Crash landing sequence complete, good luck.\n\r");
        controller_param->FCS = FCS_PWRON;
        controller_param->controller_counter = 0;
    // terminating controlled-landing sequence
    } else if (controller_param->FCS == FCS_LANDING) {
        printf("Landing sequence complete.\n\r");
        controller_param->FCS = FCS_PWRON;
        controller_param->controller_counter = 0;
    // terminating autopilot manuvers
    } else if (controller_param->FCS == FCS_AUTO_F || 
               controller_param->FCS == FCS_AUTO_B ||
               controller_param->FCS == FCS_AUTO_L ||
               controller_param->FCS == FCS_AUTO_R ||
               controller_param->FCS == FCS_AUTO_U ||
               controller_param->FCS == FCS_AUTO_D ||
               controller_param->FCS == FCS_AUTO_YL||
               controller_param->FCS == FCS_AUTO_YR) {
        printf("Autopilot Maneuvering complete\n\r");
        controller_param->FCS = FCS_HOVERING;
        controller_param->controller_counter = 0;
    } else {
        // the current state is not terminable, cast error
        error = CANNOT_TERMINATE_SEQUENCE;
    }
}

// take down initial position of the quadcopter
// prepare for take off.
void zero_quadcopter(void)
{
    if (!controller_param->controller_counter) {
        // record the current pitch 
        update_target_yaw(ctrl_data->yaw);

        printf("reset yaw: ");
        printFloat(controller_param->yaw_target);
        printf(" degrees \n\r");
        // zero the lift, yaw, pitch
        controller_param->lift = 0;
        offset_clear();
        controller_param->controller_counter = 1;
    }
}

// flight control routine that goes to the scheduler
unsigned int flight_control_routine(void)
{
    if (controller_param->FCS == FCS_OFFSET) {
        if (controller_param->manual_is_pressed) {
            if (controller_param->manual_key_pressed == 'r') {
                lift_offset(1, LIFT_UNIT_CHANGE);
            } else if (controller_param->manual_key_pressed == 'f') {
                lift_offset(0, LIFT_UNIT_CHANGE);
            } else if (controller_param->manual_key_pressed == 'w') {
                pitch_offset(1, PITCH_UNIT_CHANGE);
            } else if (controller_param->manual_key_pressed == 's') {
                pitch_offset(0, PITCH_UNIT_CHANGE);
            } else if (controller_param->manual_key_pressed == 'a') {
                roll_offset(1, ROLL_UNIT_CHANGE);
            } else if (controller_param->manual_key_pressed == 'd') {
                roll_offset(0, ROLL_UNIT_CHANGE);
            } else if (controller_param->manual_key_pressed == 'q') {
                yaw_offset(1, YAW_UNIT_CHANGE);
            } else if (controller_param->manual_key_pressed == 'e') {
                yaw_offset(0, YAW_UNIT_CHANGE);
            } else if (controller_param->manual_key_pressed == ' ') {
                offset_clear();
            }
            //printf("%d.\n\r", ctrl_data->lift);
            controller_param->manual_is_pressed = 0;
        }
    } else if (controller_param->FCS == FCS_TAKEOFF) {
        take_off();
    } else if (controller_param->FCS == FCS_CRASH) {
        crash_landing();
    } else if (controller_param->FCS == FCS_MOTOR_TEST) {
         if (controller_param->manual_is_pressed) {
            if (controller_param->manual_key_pressed == 'q') {
                motor_test(1, MOTOR_TEST_STEP, 1);
            } else if (controller_param->manual_key_pressed == 'a') {
                motor_test(0, MOTOR_TEST_STEP, 1);
            } else if (controller_param->manual_key_pressed == 'w') {
                motor_test(1, MOTOR_TEST_STEP, 2);
            } else if (controller_param->manual_key_pressed == 's') {
                motor_test(0, MOTOR_TEST_STEP, 2);
            } else if (controller_param->manual_key_pressed == 'e') {
                motor_test(1, MOTOR_TEST_STEP, 3);
            } else if (controller_param->manual_key_pressed == 'd') {
                motor_test(0, MOTOR_TEST_STEP, 3);
            } else if (controller_param->manual_key_pressed == 'r') {
                motor_test(1, MOTOR_TEST_STEP, 4);
            } else if (controller_param->manual_key_pressed == 'f') {
                motor_test(0, MOTOR_TEST_STEP, 4);
            }
            controller_param->manual_is_pressed = 0;
         }
    } else if (controller_param->FCS == FCS_ZERO) {
        zero_quadcopter();
    } else if (controller_param->FCS == FCS_PWRON) {
        //TODO:
    } else if (controller_param->FCS == FCS_HOVERING) {`
        //TODO:
    } else if (controller_param->FCS == FCS_LANDING) {
        //TODO:
        controlled_landing();
    // AUTO PILOT CONTROLS
    } else if (controller_param->FCS == FCS_AUTO_F) {
        foward(1);
    } else if (controller_param->FCS == FCS_AUTO_B) {
        backward(1);
    } else if (controller_param->FCS == FCS_AUTO_L) {
        left(1);
    } else if (controller_param->FCS == FCS_AUTO_R) {
        right(1);
    } else if (controller_param->FCS == FCS_AUTO_U) {
        up(1);
    } else if (controller_param->FCS == FCS_AUTO_D) {
        down(1);
    } else if (controller_param->FCS == FCS_AUTO_YL) {
        yaw_CW(1);
    } else if (controller_param->FCS == FCS_AUTO_YR) {
        yaw_CCW(1);
    } else {
        error = 1;
    }
    // modify U1', U2', U3', U4' from the flight control routine
    return 1;
}

// Manual control
void offset_clear(void) {
    controller_param->pitch_target = 0;
    controller_param->roll_target  = 0;
}

// Increment or decrement the trottle value for all motors
void lift_offset(unsigned int sign, unsigned int value){
    if (sign) {
        update_lift(controller_param->lift + value);
    } else {
        update_lift(controller_param->lift - value);
    }
}

// increase or decrease the lift value for specific motor by a certain amount
void motor_test(unsigned int sign, unsigned int value, unsigned int motor_num) {
    unsigned int result;
    result = sign ? test_speed[motor_num-1] + value : test_speed[motor_num-1] - value;
    test_speed[motor_num-1] = result;
    printf("M%d: %u.\n\r", motor_num, result);
    setDutyCycle(result,motor_num);
}

// offset four motors
void roll_offset(unsigned int sign, unsigned int value) {
    if (sign) {
        update_target_pitch(controller_param->roll_target + (double)value);
    } else {
        update_target_pitch(controller_param->roll_target - (double)value);
    }
}

// offset the pitch
void pitch_offset(unsigned int sign, unsigned int value)
{
    if (sign) {
        update_target_pitch(controller_param->pitch_target + (double)value);
    } else {
        update_target_pitch(controller_param->pitch_target - (double)value);
    }
}

// offset the yaw
void yaw_offset(unsigned int sign, unsigned int value)
{
    if (sign) {
        update_target_yaw(controller_param->yaw_target + (double)value);
    } else {
        update_target_yaw(controller_param->yaw_target - (double)value);
    }
}

// Autopilot routines
void take_off(void)
{
    // TODO:
    // Slowly increase throttle to HOVERING_THROTTLE
    pid_clr();
    if (controller_param->controller_counter < 100) {
        motors_on(1);
        (controller_param->controller_counter)++;
    } else if (controller_param->controller_counter < 200 ){
        update_lift(10.0);
        (controller_param->controller_counter)++;
    } else if (controller_param->controller_counter < 300) {
        update_lift(20.0);
        (controller_param->controller_counter)++;
    } else if (controller_param->controller_counter < 400) {
        update_lift(30.0);
        (controller_param->controller_counter)++;
        // terminate the motors for debugging
    } else {
        terminate_sequence();
    }
}

// THE PANIC BUTTON
void crash_landing()
{
    // Temp crash landing solution:
    // Turn off the motors on the fly
    motors_on(0);
    test_speed[0] = ZERO_SPEED_THROTTLE;
    test_speed[1] = ZERO_SPEED_THROTTLE;
    test_speed[2] = ZERO_SPEED_THROTTLE;
    test_speed[3] = ZERO_SPEED_THROTTLE;
    terminate_sequence();
}

// safely land the aircraft
void controlled_landing()
{

}

// panorama maneuver
void panorama_maneuver()
{

}

// fly forward
void foward(unsigned int distance)
{

}

// fly backward
void backward(unsigned int distance)
{

}

// fly to the left
void left(unsigned int distance)
{

}

// fly to the right
void right(unsigned int distance)
{

}

void up(unsigned int distance)
{

}

void down(unsigned int distance)
{

}

void yaw_CW(unsigned int degree)
{

}

void yaw_CCW(unsigned int degree)
{

}

// return the new yaw
// update the yaw to the new yaw, report errors if limits are exceeded
void update_target_yaw(double new_yaw)
{
    double period;
    period = YAW_HIGH_BOND - YAW_LOW_BOND;
    while (new_yaw < YAW_LOW_BOND || new_yaw > YAW_HIGH_BOND) {
        if (new_yaw < YAW_LOW_BOND) {
            new_yaw + period;
        } else {
            new_yaw - period;
        }
    }
    controller_param->yaw_target =  new_yaw;
}


// return the new yaw
// update the yaw to the new yaw, report errors if limits are exceeded
void update_target_pitch(double new_pitch)
{
    double period;
    period = PITCH_HIGH_BOND - PITCH_LOW_BOND;
    if (new_pitch < PITCH_LOW_BOND) {
        controller_param->pitch_target = PITCH_LOW_BOND;
    } else if (new_pitch > PITCH_HIGH_BOND) {
        controller_param->pitch_target = PITCH_HIGH_BOND;
    } else {
        controller_param->pitch_target = new_pitch;
    }
}


// return the new yaw
// update the yaw to the new yaw, report errors if limits are exceeded
void update_target_roll(double new_roll)
{
    double period;
    period = ROLL_HIGH_BOND - ROLL_LOW_BOND;
    if (new_roll < ROLL_LOW_BOND) {
        controller_param->roll_target = ROLL_LOW_BOND;
    } else if (new_roll > ROLL_HIGH_BOND) {
        controller_param->roll_target = ROLL_HIGH_BOND;
    } else {
        controller_param->roll_target = new_roll;
    }
}

void update_lift(double new_lift)
{
    if (new_lift >= MAX_LIFT) {
        controller_param->lift = MAX_LIFT;
    } else if (new_lift < 0){
        controller_param->lift = 0;
    } else {
        controller_param->lift = new_lift;
    }
}
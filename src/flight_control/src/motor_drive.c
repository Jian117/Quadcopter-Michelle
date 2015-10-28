#include "pwm.h"
#include "motor_drive.h"

unsigned int speed_report_count;
unsigned char motor_on;

void motors_init(void)
{
    // enable timer0,1,3,5 for motor control
    T0CONbits.T08BIT = 0;   // set timer0 in 16-bit mode
    T0CONbits.T0CS = 0;     // set timer0 clk source as instruction cycle
    T0CONbits.PSA  = 0;     // enable timer0 pre-scale
    T0CONbits.T0SE = 0;     // increment on low to high transition
    T0CONbits.T0PS = 0b001; // choose a pre-scale of 4
    // turn on Timer0
    T0CONbits.TMR0ON = 0;

    // enable timer1
    T1CONbits.TMR1CS = 0b00; // CLK src FOSC/4
    T1CONbits.T1CKPS = MOTOR_TIMER_PRESCALE;
    T1CONbits.T1SOSCEN = 0; // disable secondary osc
    T1CONbits.T1RD16 = 1;   // sing 16-bit-read/write operation enable
    T1CONbits.TMR1ON = 0;   // enable tmr1

    // enable timer3
    T3CONbits.TMR3CS = 0b00; // CLK src FOSC/4
    T3CONbits.T3CKPS = MOTOR_TIMER_PRESCALE;
    T3CONbits.T3SOSCEN = 0; // disable secondary osc
    T3CONbits.T3RD16 = 1;   // sing 16-bit-read/write operation enable
    T3CONbits.TMR3ON = 0;   // enable tmr3

    // enable timer5
    T5CONbits.TMR5CS = 0b00; // CLK src FOSC/4
    T5CONbits.T5CKPS = MOTOR_TIMER_PRESCALE;
    T5CONbits.T5SOSCEN = 0; // disable secondary osc
    T5CONbits.T5RD16 = 1;   // sing 16-bit-read/write operation enable
    T5CONbits.TMR5ON = 1;   // enable tmr5

    motor_on = 0;
    setDutyCycle (6, 1);      // dutyCycle = 10%, frequency = 50Hz
    setDutyCycle (6, 2);      // dutyCycle = 10%, frequency = 50Hz
    setDutyCycle (6, 3);      // dutyCycle = 10%, frequency = 50Hz
    setDutyCycle (6, 4);      // dutyCycle = 10%, frequency = 50Hz

    PORTBbits.RB0 = 1;
    PORTBbits.RB1 = 1;
    PORTBbits.RB2 = 1;
    PORTBbits.RB3 = 1;
    speed_report_count = 1;
}

// 1: start the engines. 0: stop the engines.
void motors_on(unsigned char on_noff)
{
    motor_on = on_noff;
}

// throttle is a number between 0 and 16000
int setDutyCycle(unsigned int throttle, unsigned int motor_num)
{
    if (throttle < 16001 && throttle > 0) {
        if (motor_num == 1 || motor_num == 0){
            high1 = throttle + THROTTLE_LOW;
        }
        if (motor_num == 2 || motor_num == 0){
            high2 = throttle + THROTTLE_LOW;
        }
        if (motor_num == 3 || motor_num == 0){
            high3 = throttle + THROTTLE_LOW;
        }
       if (motor_num == 4 || motor_num == 0){
            high4 = throttle + THROTTLE_LOW;
        }
        return 1;
    }else {
        return 0;
    }
}

void start_motor_timers(void)
{
    unsigned int temp;
    INTCONbits.TMR0IF = 0;
    temp = ~high1;
    TMR0H = temp >> 8;
    TMR0L = temp;
    T0CONbits.TMR0ON = 1;
    
    PIR1bits.TMR1IF = 0;
    temp = ~high2;
    TMR1H = temp >> 8;
    TMR1L = temp;
    T1CONbits.TMR1ON = 1;

    PIR2bits.TMR3IF = 0;
    temp = ~high3;
    TMR3H = temp >> 8;
    TMR3L = temp;
    T3CONbits.TMR3ON = 1;

    PIR5bits.TMR5IF = 0;
    temp = ~high4;
    TMR5H = temp >> 8;
    TMR5L = temp;
    T5CONbits.TMR5ON = 1;
}

//translate omega1,2,3,4 to pwm1,2,3,4 and actuate motors accordingly
unsigned int controlled_speed(void)
{
    unsigned int t1;
    unsigned int t2;
    unsigned int t3;
    unsigned int t4;
    if (controller_param->FCS == FCS_MOTOR_TEST) {
        setDutyCycle(test_speed[0], 1);
        setDutyCycle(test_speed[1], 2);
        setDutyCycle(test_speed[2], 3);
        setDutyCycle(test_speed[3], 4);
    } else {
        //actuate the motors
        if (motor_on) {
            t1 = normalize_speed(ctrl_data->omega1square);
            t2 = normalize_speed(ctrl_data->omega2square);
            t3 = normalize_speed(ctrl_data->omega3square);
            t4 = normalize_speed(ctrl_data->omega4square);
            setDutyCycle(normalize_speed(ZERO_SPEED_THROTTLE + ctrl_data->omega1square), 1);
            setDutyCycle(normalize_speed(ZERO_SPEED_THROTTLE + ctrl_data->omega2square), 2);
            setDutyCycle(normalize_speed(ZERO_SPEED_THROTTLE + ctrl_data->omega3square), 3);
            setDutyCycle(normalize_speed(ZERO_SPEED_THROTTLE + ctrl_data->omega4square), 4);
            if (speed_report_count == SPEED_REPORT_RATE) {

                // printf("T1: %u, T2: %u, T3: %u, T4: %u.\n\r", t1, t2, t3, t4);
                 //print the motor speeds
                /*
                printf("M1: ");
                printFloat(ctrl_data->omega1square);
                printf(", M2: ");
                printFloat(ctrl_data->omega2square);
                printf(", M3: ");
                printFloat(ctrl_data->omega3square);
                printf(", M4: ");
                printFloat(ctrl_data->omega4square);
                printf(".\n\r");
               
                printf("e_roll: ");
                printFloat(kinetic_errs->roll_error);
                printf("e_pitch: ");
                printFloat(kinetic_errs->pitch_error);
                printf(", e_yaw: ");
                printFloat(kinetic_errs->yaw_error);
                printf(", e_alt: ");
                printFloat(kinetic_errs->altitude_error);
                printf(".\n\r");
                printf(".\n\r");
*/
                speed_report_count = 0;
            } else {
                speed_report_count++;
            }

        } else {
            // gives zero throttle to all
            setDutyCycle(6, 0);
        }
    }
}

// normalize omega2 into throttle input
unsigned int normalize_speed(double speed) {
    int output;
    output = (int)SPEED2THROTTLE_FACTOR*speed;
    output = (output < 0) ? 0:
             (output > MAX_THROTTLE) ?  MAX_THROTTLE :
              output;
    return (unsigned int)output;
}
/*
void pwm_init(void)
{
    SetOutputEPWM1(SINGLE_OUT, PWM_MODE_1);
    OpenEPWM1(0xFF, ECCP_1_SEL_TMR12);
    SetDCEPWM1(0x6FF);
}
*/
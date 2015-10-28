#include "scheduler.h" 

unsigned int done;
unsigned int motorAll, high1, high2, high3, high4;


void timer_init(void) {
    // Timer2, used as the scheduler timer
    // choose a prescale of 4
    T2CONbits.T2CKPS = 0b01;
    // set period for the scheduler counter
    PR2 = 165;
    // turn on Timer2
    done = 0;
    curr_channel = 0;
    T2CONbits.TMR2ON = 1;
}

void start_scheduler(void) {
    while (1) {
        
        //timer interrupt
        if (sampling_flag == 1) {
            timer_rst();
            //settlingTime++;
            if (curr_channel == 1) {
                update_status(SYSTEM_MOTOR);
                PORTAbits.RA5 = !PORTAbits.RA5;
                PORTB = 0xFF;
                start_motor_timers();
            }

            if (curr_channel == 100) {
                update_status(SYSTEM_I2C_R);
                //done = read_sensor_dummy();
                done = read_sensor();
            }

            if (curr_channel == 400) {
                update_status(SYSTEM_CONTROLLER);
                done = flight_control_routine();
            }
            if (curr_channel == 500){
                update_status(SYSTEM_STABILIZER);
                done = stabilize_routine();
            }

            if (curr_channel == 600) {
                update_status(SYSTEM_UART);
                done = uart_task();
            }
            
            if (curr_channel == 700) {
                update_status(SYSTEM_MOTOR);
                done = controlled_speed();
            }


            //update status if a task terminate
        } else {
            if ((check_status() == SYSTEM_UART) && done && TXSTAbits.TRMT) {
               update_status(SYSTEM_IDLE);
               done = 0;
            } else if ( done && (check_status() == SYSTEM_I2C_R ||
                        check_status() == SYSTEM_STABILIZER ||
                        check_status() == SYSTEM_CONTROLLER)) {
                update_status(SYSTEM_IDLE);
                done = 0;
            }
        }
    }
}

// update system status to R5 RC2 RC1 RC0

void update_status(int status) {
    int status_out = 0;
    status_out = status_out | (status & 0x01) | (status & 0x02) | status & 0x04 | ((status & 0x08) << 2);
    PORTC = status_out;
}

int check_status(void) {
    return PORTCbits.RC0
            | (PORTCbits.RC1 << 1)
            | (PORTCbits.RC2 << 2)
            | (PORTCbits.RC5 << 5);
}

void timer_rst(void) {
    sampling_flag = 0;
    curr_channel = (curr_channel >= TOTAL_TICKS) ? 0: curr_channel+1;
}

// delay n clk_cycle, update curr_chan;

void timer0_delay(int clk_cycle) {
    int count = 0;
    while (count < clk_cycle) {
        if (sampling_flag) {
            timer_rst();
            count++;
        }
    }
}
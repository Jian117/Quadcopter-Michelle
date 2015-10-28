#include "scheduler.h" 

unsigned int done;

void timer_init(void) {
    // Timer0, used to time motor 1
    T0CONbits.T08BIT = 1; // set timer0 in 8-bit mode
    T0CONbits.T0CS = 0; // set timer0 clk source as instruction cycle
    T0CONbits.PSA = 0; // enable timer0 pre-scale
    T0CONbits.T0SE = 0; // increment on low to high transition
    T0CONbits.T0PS = 0b001; // choose a pre-scale of 4
    // turn on Timer0
    T0CONbits.TMR0ON = 1;

    // Timer2, used as the scheduler timer

    // choose a prescale of 4
    T2CONbits.T2CKPS = 0b11;
    // turn on Timer2
    T2CONbits.TMR2ON = 1;
}

void start_scheduler(void) {
    while (1) {
        //timer interrupt
        if (sampling_flag == 1) {
            timer_rst();
            if (curr_channel == 1) {
                // used to indicate scheduler cycle in GPIO pin RA5
                update_status(SYSTEM_START);
                PORTAbits.RA5 = !PORTAbits.RA5;
            } else if (curr_channel == 100) {
                update_status(SYSTEM_GYRO);
                
            } else if (curr_channel == 200) {
                update_status(SYSTEM_ACCELERO);
                done = Get_Accel_Values();
            
            } else if (curr_channel == 300) {
               update_status(SYSTEM_COMPASS);
               done = get_compass();
              //done = Compass_test_single();
            } else if (curr_channel == 450) {
                update_status(SYSTEM_FUSION);
                //Calibrate_Gyros();
                ComplementaryFilter();
                fusion_final();
            } else if (curr_channel == 800) {
                update_status(SYSTEM_UART);
               // done = uart_task();
            }
        } else {
             //update status if a task terminate
            if (done && (check_status() == SYSTEM_UART) && TXSTAbits.TRMT) {
                update_status(SYSTEM_IDLE);
                done = 0;
            } else if (done && ((check_status() == SYSTEM_GYRO) ||
                    check_status() == SYSTEM_ACCELERO ||
                    check_status() == SYSTEM_COMPASS  ||
                    check_status() == SYSTEM_FUSION
                    )) {
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
    TMR0H = 0x00;
    TMR0L = 180;
    curr_channel = (curr_channel >= 1000) ? 0: curr_channel+1;
    // take data from analog channels
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

// delay operations until the master is ready.
void init_delay(void)
{
    timer0_delay(100000);
}
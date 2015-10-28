#include "scheduler.h" 

unsigned int done;

void timer_init(void) {
    // enable timer0, configured as 8-bit, internal clock source, tick 
    // on rising edge, change pre-scale according to PRESCALE 
    T0CONbits.T08BIT = 1;   // set timer0 in 8-bit mode
    T0CONbits.T0CS = 0;     // set timer0 clk source as instruction cycle
    T0CONbits.PSA = 0;      // enable timer0 pre-scale
    T0CONbits.T0SE = 0;     // increment on low to high transition
    T0CONbits.T0PS = 0b001; // choose a pre-scale of 4
    T2CON = 0b00000101;     // sampling at 10kHz
    // turn on Timer0
    T0CONbits.TMR0ON = 1;
}

void start_scheduler(void) {
    setDutyCycle (50, 50);      // dutyCycle = 50%, frequency = 50Hz
    while (1) {
//  OpenEPWM1(0xff);
//  SetDCEPWM1(0);
//  SetOutputEPWM1(FULL_OUT_FWD, PWM_MODE_1);
//  CloseEPWM1();
        
        //timer interrupt
        if (sampling_flag == 1) {
            timer_rst();
            if (curr_channel < high) {
                PORTBbits.RB0 = 1;
                PORTBbits.RB1 = 1;
                PORTBbits.RB2 = 0;
                PORTBbits.RB3 = 0;
            } else if (curr_channel >= high){
                PORTBbits.RB0 = 0;
                PORTBbits.RB1 = 0;
                PORTBbits.RB2 = 1;
                PORTBbits.RB3 = 1;
            } else if (curr_channel == totalTicks){
                update_status(SYSTEM_UART);
                done = uart_task();
            }
            //update status if a task terminate
        } else {
            if ((check_status() == SYSTEM_UART) && done && TXSTAbits.TRMT) {
               update_status(SYSTEM_IDLE);
               done = 0;
            }
        }
    }
}

void setDutyCycle(int dutyCycle, int frequency){
    totalTicks = 10000/frequency;
    high = dutyCycle*totalTicks/100;
    low = totalTicks - high;
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
    TMR0H = 0;
    TMR0L = 188;
    // take data from analog channels
    curr_channel = (curr_channel + 1) % totalTicks;
}

// delay n clk_cycle, update curr_chan;

void timer0_delay(int clk_cycle) {
    int count = 0;
    while (count < clk_cycle) {
        if (sampling_flag) {
            timer_rst();
            curr_channel++;
            count++;
        }
    }
}
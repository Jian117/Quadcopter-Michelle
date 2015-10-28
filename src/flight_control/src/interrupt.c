#include "interrupts.h" 

void interrupt_init(void) 
{
    // disable all interrupts except for external interrupt on INT2/RA2 
    // and timer0/1 interrupt 
    // disable interrupt priority
    RCONbits.IPEN = 1; 
    INTCON =  0b00100000;       // timer0 overflow interrupt
    INTCON2 = 0b11000100;	// Pull-ups disabled/Interrupt0 Rising edge
    INTCON3 = 0b00000000; 	// external to zero

    PIE1bits.RC1IE = 1;   // serial RX interrupt
    PIE1bits.TMR2IE = 1;  // enable timer2 interrupt

    PIE1bits.TMR1IE = 1;  // enable timer1 interrupt
    PIE2bits.TMR3IE = 1;  // enable timer3 interrupt
    PIE5bits.TMR5IE = 1;  // enable timer5 interrupt
    INTCONbits.T0IE = 1;  // enable timer0 interrupt
    
    IPR1bits.RC1IP = 1;
    IPR1bits.TMR2IP = 1;
    IPR2bits.TMR3IP = 1;
    IPR1bits.TMR1IP = 1;
    IPR5bits.TMR5IP = 1;
    
    INTCONbits.GIE_GIEH = 1;            /* Enable the interrupts */ 
    INTCONbits.PEIE_GIEL = 1;           /* Enable peripheral interrupts */
}

#pragma code high_vector=0x08
void global_interrupt(void) {
    _asm GOTO myISR _endasm;
}
#pragma code

#pragma interrupt myISR
void myISR(void)
{
    // check frame error
    if (PIR1bits.TMR2IF) {
        // restart the counter
        TMR2 = 0;
        PIR1bits.TMR2IF = 0;
        sampling_flag = 1;   
    }
    
    if (PIR1bits.RC1IF) {
       PIR1bits.RC1IF = 0;
       byte_command = RCREG1;
       byte_command = RCREG1;
       
       remaining_buffer_size--;
       if (!remaining_buffer_size) {
              RX_buffer_full = 1;
              RCSTA1bits.CREN1 = 0;
              PIE1bits.RC1IE = 0;
       }
       INTCONbits.GIE = 1;
    }

    if (INTCONbits.T0IF) {
        TMR0H:TMR0L = 0;
        INTCONbits.TMR0IF = 0;
        T0CONbits.TMR0ON = 0;
        PORTBbits.RB0 = 0;
    }

    if (PIR1bits.TMR1IF) {
        TMR1H:TMR1L = 0;
        PIR1bits.TMR1IF = 0;
        T1CONbits.TMR1ON = 0;
        PORTBbits.RB5 = 0;
    }

    if (PIR2bits.TMR3IF) {
        TMR3H:TMR3L = 0;
        PIR2bits.TMR3IF = 0;
        T3CONbits.TMR3ON = 0;
        PORTBbits.RB2 = 0;
    }

    if (PIR5bits.TMR5IF) {
        TMR5H:TMR5L = 0;
        PIR5bits.TMR5IF = 0;
        T5CONbits.TMR5ON = 0;
        PORTBbits.RB3 = 0;
    }
}
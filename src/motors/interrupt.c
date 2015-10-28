#include "interrupts.h" 

void interrupt_init(void) 
{
    // disable all interrupts except for external interrupt on INT2/RA2 
    // and timer0/1 interrupt 
      
    // disable interrupt priority 
	
    RCONbits.IPEN = 1; 		// Enable Interrupt Priority Enable Bits
    INTCON =  0b00100000;   // timer0 overflow interrupt
    INTCON2 = 0b11000100;	// Pull-ups disabled/Interrupt0 Rising edge
    INTCON3 = 0b00000000; 	// external to zero
	
    PIE1 =    0b00100000; 	// enable timer2
    PIE2 =    0b00000000;        
    PIE3 =    0b00000000;
    PIE4 =    0b00000000;
    PIE5 =    0b00000000;
    IPR1 =    0b00100000;
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
	// *note: was T0IF
    if (INTCONbits.INT0IF) {
        INTCONbits.INT0IF = 0;
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
}
#include "interrupts.h" 
#define I2CADDR 0x55
#define TRANSMIT_SIZE 17

unsigned char ne;

void interrupt_init(void) 
{
    // disable all interrupts except for external interrupt on INT2/RA2 
    // and timer0/1 interrupt 
    // disable interrupt priority
    RCONbits.IPEN = 1;

    INTCON =  0b00100000;       // timer0 overflow interrupt
    INTCON2 = 0b11000100;	// Pull-ups disabled/Interrupt0 Rising edge
    INTCON3 = 0b00000000; 	// external to zero

    PIR3bits.SSP2IF = 0; // disable the SSP2 interrupt
    IPR3bits.SSP2IP = 1;
    PIE3bits.SSP2IE = 1; // SSP2 interrupt enable


    INTCONbits.GIE_GIEH = 1;            /* Enable the interrupts */
    INTCONbits.PEIE_GIEL = 1;
    
    ne = 0;
    /* Enable peripheral interrupts */
} 

#pragma code high_vector=0x08
void global_interrupt(void) {
    _asm GOTO myISR _endasm;
}
#pragma code

#pragma interrupt myISR
void myISR(void)
{

    char count;
    char buffer;
    PORTC = 0;
    // check frame error
    if (INTCONbits.T0IF) {
        INTCONbits.T0IF = 0;
        sampling_flag = 1;
    }
    if (PIR3bits.SSP2IF) {        // Check for SSP interrupt
//        PORTCbits.RC2= 1;
        count = (SSP2STAT & 0x2d);
        // last byte received is master write addr
        if (count == 0x09){
            PORTC = 1;
            buffer = SSP2BUF;
            //*(command_ptr + no) = buffer;
            SSP2CON2bits.ACKDT = 0; //ack
            SSP2CON1bits.CKP = 1;
        //state2, write/last data
        } else if(count == 0x29){
            PORTC = 2;
            SSP2CON2bits.ACKDT = 0; //ack
            buffer = SSP2BUF;
            SSP2CON1bits.CKP =1;
        }
        // last byte received is master read addr
        else if(count == 0x0D){
            PORTC = 3;
            ne = 0;
            buffer = SSP2BUF;
            SSP2CON1bits.CKP = 0;
            SSP2BUF = 0x55;
            SSP2CON1bits.CKP = 1;
            ne++;
        //state4, master read, last byte data
        }else if(count ==  0x2C){
            PORTC = 4;
            buffer = SSP2BUF;
            SSP2CON1bits.CKP = 0;

//            SSP2BUF = (ne == 1) ? 1://sensor_result_ptr->roll:
//                      (ne == 2) ? 2:// sensor_result_ptr->pitch:
//                      (ne == 3) ? 3:0;// sensor_result_ptr->yaw:0;
            SSP2BUF = (ne == 1) ?  sensor_result_ptr->roll >> 24:
                      (ne == 2) ?  sensor_result_ptr->roll >> 16:
                      (ne == 3) ?  sensor_result_ptr->roll >> 8 :
                      (ne == 4) ?  sensor_result_ptr->roll      :
                      (ne == 5) ?  sensor_result_ptr->pitch>> 24:
                      (ne == 6) ?  sensor_result_ptr->pitch>> 16:
                      (ne == 7) ?  sensor_result_ptr->pitch>> 8 :
                      (ne == 8) ?  sensor_result_ptr->pitch     :
                      (ne == 9) ?  sensor_result_ptr->yaw  >> 24:
                      (ne == 10) ? sensor_result_ptr->yaw  >> 16:
                      (ne == 11) ? sensor_result_ptr->yaw  >> 8 :
                      (ne == 12) ? sensor_result_ptr->yaw       :
//                      (ne == 13) ? sensor_result_ptr->altitude>>24:
//                      (ne == 14) ? sensor_result_ptr->altitude>>16:
//                      (ne == 15) ? sensor_result_ptr->altitude>>8:
//                      (ne == 16) ? sensor_result_ptr->altitude:
                       0;


            SSP2CON1bits.CKP = 1;
            ne++;
        }else if ((count ==  0x21) ||(count ==  0x20)){
            PORTC = 5;
             buffer = SSP2BUF;
             SSP2CON1bits.CKP = 0;
             SSP2BUF = 0x01;//data_ptr[ne];
             SSP2CON1bits.CKP = 1;
        }else{
            buffer = SSP2BUF;
            SSP2CON1bits.CKP = 1;
        }
        PIR3bits.SSP2IF = 0;  // Clear the interrupt flag
        PORTCbits.RC2 = 0;
    }

    INTCONbits.GIE = 1;
    INTCONbits.T0IF = 0;

}
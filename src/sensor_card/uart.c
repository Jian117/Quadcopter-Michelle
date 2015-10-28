#include "uart.h"

// constraints:
 int count_sensor = 0;
int uart_task(void)
{
    double n;
   
    //n = ;
    //baud_test();
   // printf("\033[2J");

    if (error == NO_ERROR) {
 
        if(count_sensor == 10)
        {
            printFloat(sensor_data_ptr->roll);
            printFloat(sensor_data_ptr->pitch);
            printFloat(sensor_data_ptr->yaw);
            count_sensor = 0;
            printf("\r\n");
        }
        count_sensor++;
       //printFloat(sensor_data_ptr->compass_X);
       //printFloat(sensor_data_ptr->compass_Y);
       //printFloat(sensor_data_ptr->compass_Z);

    } else if (error == ERROR_I2C_READ_TIMEOUT) {
        printf("\n\r Error: Read I2C timeout.");
    }

    return 1;
}

//transmit 0x55 to test baud rate of TX
void baud_test(void)
{
	char test_byte = 't';
	_user_putc(test_byte);
}


//Low level protocols

//enable the receiver
void RX_open(void) {
    RCSTA1bits.CREN1 = 1;
    PIE1bits.RC1IE = 1;
    PIR1bits.RC1IF = 0;
    remaining_buffer_size = BUFFER_SIZE;
}

// close RX port when there are command to be executed
void RX_close(void) {
    RCSTA1bits.CREN1 = 0;
    PIE1bits.RC1IE = 0;
}

// pull the a byte out of the RX register and append it
// to the command string if there is no frame error
void RX_buffer_push(char data_byte) {
    *(RX_buffer_ptr + BUFFER_SIZE - remaining_buffer_size) = data_byte;
    remaining_buffer_size--;
    if (!remaining_buffer_size) {
        RX_close();
    }
}

void uart_init(void) {
	// Asynchronous mode, high speed, 9th bit-disabled
	TXSTA1 =   0b10100101;
	// 8-bit reception, clear framing and overrun error
	RCSTA1 =   0b10011000;
	// TX/RX high true. 1 as idle bit. 8-bit baud
	// rate generator, auto baud disabled
	BAUDCON1 = 0b01000000;
	SPBRGH1= 0x00;
	SPBRG1 = 0xCF;
        RX_open();
}

// send out a byte via uart or LCD, used as stdoutput
void _user_putc(char TX_byte)
{
	// turning on transmitting LED
	while(!TXSTAbits.TRMT) {
            if (sampling_flag) {
                timer_rst();
            }
        }
	TXREG1 = TX_byte;
}



void printFloat(double fInput)
{
    //The number is converted to two parts.
    long lWhole=(long)((double)fInput);
    long ulPart=(long)((double)fInput*100)-lWhole*100;

    printf(" %li.%li",lWhole,ulPart);

}

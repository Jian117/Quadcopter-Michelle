#include "uart.h"

// UART high-level protocols:

// modifying flight control signal flow according to
// the task given
// Echo that command is received
int uart_task(void)
{
    char test = 0x55;
    //baud_test();
    if (RX_buffer_full) {
        RX_command_process();
        RX_buffer_full = 0;
    }
    if (error == NO_ERROR) {
        //printf("\n\rFCS: %d\n\r", controller_param->FCS);
       //printf("%li, %li, %li, %li\n\r", ctrl_data->roll, ctrl_data->pitch, ctrl_data->yaw, ctrl_data->altitude);


    } else if (error == SERIAL_TRANSMISSION_ERROR) {
        printf("\n\rSERIAL_ERROR\n\r");
		error = NO_ERROR;
    } else if (error == I2C_TRANSMISSION_ERROR) {
        printf("\n\rI2C_ERROR\n\r");
		error = NO_ERROR;
    } else if (error == INVALID_COMMAND) {
	printf("\n\rInvalid command\n\r");
	error = NO_ERROR;
	}

    return 1;
}

//transmit 0x55 to test baud rate of TX
void baud_test(void)
{
	char test_byte = 't';
	_user_putc(test_byte);
}

void RX_command_process(void) {
    int i;
    unsigned char command;
    // pharsing the command
    for (i = 0; i < BUFFER_SIZE; i++) {
        command = (unsigned char)byte_command;
        process_command(command);
    }
    RX_open();
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
        byte_command = '0';
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
    unsigned long ulPart=(unsigned long)((double)fInput*100)-lWhole*100;

    printf(" %li.%lu",lWhole,ulPart);

}
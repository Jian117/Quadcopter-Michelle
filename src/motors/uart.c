#include "uart.h"

int uart_task(void)
{
    baud_test();
    return 1;
}

//transmit 0x55 to test baud rate of TX
void baud_test(void)
{
	char test_byte = 0x55;
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
	while(!TXSTAbits.TRMT);
	TXREG1 = TX_byte;
}

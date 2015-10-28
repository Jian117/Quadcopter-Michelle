#ifndef UART_H
#define UART_H

#define BUFFER_SIZE 1

// REPORT_FLAG indicate what the terminal
// is going to output

// The terminal sends the welcome msg
#define REPORT_FLAG_WELCOME 0
// The terminal is waiting for changes
#define REPORT_FLAG_WAIT    1
// REPORT_FLAG_REPORT means the terminal is scanning things
#define REPORT_FLAG_REPORT  2
#define REPORT_FLAG_IDLE    3
#define REPORT_FLAG_T    4
#define REPORT_FLAG_C    5
#define REPORT_FLAG_S    6
#define REPORT_FLAG_F    7
#define REPORT_FLAG_SCAN    8

#define BYTE_INSTRUCTION_REPORT 'r'
#define BYTE_INSTRUCTION_REPORT_ALL 'a'
#define BYTE_INSTRUCTION_SCAN 's'
#define BYTE_INSTRUCTION_TEMP_UNIT 't'
#define BYTE_INSTRUCTION_T '1'
#define BYTE_INSTRUCTION_C '2'
#define BYTE_INSTRUCTION_S '3'
#define BYTE_INSTRUCTION_F '4'
#define BYTE_INSTRUCTION_NEXT_UNIT 'k'
#define BYTE_INSTRUCTION_PREV_UNIT 'j'
#define BYTE_INSTRUCTION_NEXT_SIZE 'm'
#define BYTE_INSTRUCTION_PREV_SIZE 'n'

#define ACCURACY 2

#include "routines.h"

//global variables
extern char* RX_buffer_ptr;
extern unsigned int report_flag;
extern volatile int remaining_buffer_size;
extern volatile unsigned int RX_buffer_full;
extern volatile unsigned char byte_command;
extern unsigned int report_type;

// public functions
void uart_init(void);
void baud_test(void);
int uart_task(void);
void RX_ISR(void);

void printFloat(double fInput);


// private functions
void RX_buffer_push(char RX_buffer_push);
void _user_putc(char TX_byte);
void RX_open(void);
void RX_close(void);
unsigned int verify_data(char* data_ptr);



#endif
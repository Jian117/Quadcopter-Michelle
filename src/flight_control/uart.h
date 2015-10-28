#ifndef UART_H
#define UART_H

#define BUFFER_SIZE 1

// REPORT_FLAG indicate what the terminal
// is going to output

// The terminal sends the welcome msg
#define REPORT_FLAG_ECHO                     1

#define BYTE_INSTRUCTION_ZERO               'z'
#define BYTE_INSTRUCTION_TAKEOFF            't'
#define BYTE_INSTRUCTION_CONTROLLED_LANDING 'l'
#define BYTE_INSTRUCTION_ABORT              'c'
#define BYTE_INSTRUCTION_AUTO_MANUAL        'x'
#define BYTE_INSTRUCTION_AUTO_BALANCE       ' '
#define BYTE_INSTRUCTION_W                  'w'
#define BYTE_INSTRUCTION_A                  'a'
#define BYTE_INSTRUCTION_S                  's'
#define BYTE_INSTRUCTION_D                  'd'
#define BYTE_INSTRUCTION_Q                  'q'
#define BYTE_INSTRUCTION_E                  'e'
#define BYTE_INSTRUCTION_R                  'r'
#define BYTE_INSTRUCTION_F                  'f'
#define BYTE_INSTRUCTION_SYSINFO            'i'
#define BYTE_INSTRUCTION_MOTOR_TEST         'm'
#define BYTE_INSTRUCTION_PIC                'p'
#define BYTE_INSTRUCTION_VID                'v'
#define BYTE_INSTRUCTION_CLR                'u'


#include "routines.h"

typedef struct
{
    unsigned char overflow;
    char sign;
    unsigned int decimal_left;
    unsigned int decimal_right;
} double_printable;

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
void RX_command_process(void);
void printFloat(double fInput);

// private functions
void RX_buffer_push(char RX_buffer_push);
void _user_putc(char TX_byte);
void RX_open(void);
void RX_close(void);
unsigned int verify_data(char* data_ptr);


extern int count_sensor;

#endif
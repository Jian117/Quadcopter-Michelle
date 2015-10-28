#ifndef ROUTINES_H
#define ROUTINES_H

#include "uart.h"
#include "scheduler.h"
#include "interrupts.h"
#include "intra_comm.h"
#include "motor_drive.h"
#include "stabilizer.h"
#include "controller.h"
#include "PID.h"
#include <stdio.h>
#include <p18F25K22.h>

extern unsigned int TX_buffer_size;
extern unsigned int RX_buffer_size;

// definitions
#define TIMER0_COUNTTO 125

#define SYSTEM_STATUS_MASK 0x27

// system status
#define SYSTEM_IDLE        0x00
#define SYSTEM_STABILIZER  0x01
#define SYSTEM_CONTROLLER  0x02
#define SYSTEM_UART        0x03
#define SYSTEM_I2C_R       0x04
#define SYSTEM_MOTOR       0x05

// error
#define NO_ERROR                    0
#define SERIAL_TRANSMISSION_ERROR   1
#define I2C_TRANSMISSION_ERROR      2
#define INVALID_COMMAND             3
#define CANNOT_TERMINATE_SEQUENCE   4
#define HOLY_SHIT_ITS_GONNA_GO_DOWN 5

extern char* RX_buffer_ptr;
extern unsigned int report_flag;
extern volatile int remaining_buffer_size;
extern volatile unsigned int RX_buffer_full;
extern volatile unsigned char byte_command;
extern unsigned int report_type;
extern unsigned int error;

extern unsigned int sampling_flag;

void check_ack(void);

// public functions
void uart_init(void);
int uart_task(void);
void baud_test(void);

void interrupt_init(void);
void global_interrupt(void);
void myISR(void);

void timer_init(void);
void start_scheduler(void);
void timer0_delay(int clk_cycle);

void I2C_slave_init(void);
void I2C_master_init(char* TX_buffer, char* RX_buffer);

unsigned int bytes2int(unsigned char upper,unsigned char lower);
#endif
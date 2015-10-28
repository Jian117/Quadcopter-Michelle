#ifndef ROUTINES_H
#define ROUTINES_H

#include "adc.h"
#include "uart.h"
#include "scheduler.h" 
#include "interrupts.h"
#include "intra_comm.h"
#include "sensor_accelerometer.h"
#include "sensor_gyroscope.h"
#include "sensor_magnetometer.h"
#include "sensor_pressure.h"
#include "bmp_85_bst.h"
#include <stdio.h>
#include <p18F25K22.h>

extern unsigned int TX_buffer_size;
extern unsigned int RX_buffer_size;

// definitions
#define TIMER0_COUNTTO 125

#define SYSTEM_STATUS_MASK 0x27

#define SYSTEM_IDLE        0x00
#define SYSTEM_ADC         0x01
#define SYSTEM_FREQ_S      0x02
#define SYSTEM_UART        0x03
#define SYSTEM_LCD_I_FIRE  0x04
#define SYSTEM_FREQ_E      0x05
#define SYSTEM_SRAM_W      0x06
#define SYSTEM_SRAM_R      0x07
#define SYSTEM_LCD_INSTR   0x09
#define SYSTEM_LCD_DATA    0x0A
#define SYSTEM_LCD_D_FIRE  0x0B
#define SYSTEM_I2C         0x08

extern char* RX_buffer_ptr;
extern unsigned int report_flag;
extern volatile int remaining_buffer_size;
extern volatile unsigned int RX_buffer_full;
extern volatile unsigned char byte_command;
extern unsigned int report_type;

extern unsigned int sampling_flag;

extern char* I2C_TX_buffer_ptr;
extern char* I2C_RX_buffer_ptr;

void check_ack(void);
void sensors_init(void);

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
void I2C_master_init(void);
void setDutyCycle(int dutyCycle, int frequency);
unsigned int bytes2int(char upper, char lower);
#endif
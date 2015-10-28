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
#include "complementary_filter.h"
#include <stdio.h>
#include <p18F25K22.h>

extern unsigned int TX_buffer_size;
extern unsigned int RX_buffer_size;

// definitions
#define TIMER0_COUNTTO 125

#define SYSTEM_STATUS_MASK 0x27

// system status
#define SYSTEM_IDLE        0x00
#define SYSTEM_ACCELERO    0x01
#define SYSTEM_GYRO        0x02
#define SYSTEM_COMPASS     0x06
#define SYSTEM_START       0x07
#define SYSTEM_FUSION      0x05
#define SYSTEM_MASTER_READ 0x04
#define SYSTEM_UART        0x03
// error
#define NO_ERROR               0
#define ERROR_I2C_READ_TIMEOUT 1


// define sensor data storage block
typedef struct
{
    float angular_velocity_X;
    float angular_velocity_Y;
    float angular_velocity_Z;
    float compass_X;
    float compass_Y;
    float compass_Z;
    float accel_X;
    float accel_Y;
    float accel_Z;
    float roll;
    float pitch;
    //float roll_gy;
    //float pitch_gy;
    float roll_ac;
    float pitch_ac;
    float yaw;

    char data_size;
    unsigned int secret_code;
    
} sensor_data_struct;

// final result for the sensor processing unit
typedef struct
{
    long pitch;
    long roll;
    long yaw;
    long altitude;
} sensor_result_struct;

extern char* RX_buffer_ptr;
extern sensor_data_struct* sensor_data_ptr;
extern sensor_result_struct* sensor_result_ptr;
extern unsigned int report_flag;
extern volatile int remaining_buffer_size;
extern volatile unsigned int RX_buffer_full;
extern volatile unsigned char byte_command;
extern unsigned int report_type;
extern unsigned int error;

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

unsigned int bytes2int(unsigned char upper,unsigned char lower);
#endif
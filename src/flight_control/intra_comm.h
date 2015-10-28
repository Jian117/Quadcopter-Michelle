#ifndef INTRA_H
#define INTRA_H

#include "routines.h"

#define TIMEOUT 35
#define SENSOR_CARD_ADDR 0x55
#define THROUGHPUT 17

extern char* I2C_TX_buffer_ptr;
extern char* I2C_RX_buffer_ptr;

// low level I2C functions
void I2C_master_init(char* TX_buffer, char* RX_buffer);
void master_update_bit_field(unsigned int device_ID, unsigned char sub_addr,
                             unsigned char bit_field, unsigned char mask);
void master_I2C_read(unsigned int device_ID, int sub_addr);
void master_I2C_write(unsigned int device_ID, int registerAddr);

unsigned char master_I2C_read_byte(unsigned int device_ID, int registerAddr);
void master_I2C_write_byte(unsigned int device_ID, int registerAddr,
                           unsigned char data);
void c_WriteI2C( unsigned char data_out );
unsigned char c_ReadI2C( void );

// high level I2C protocols
void I2C_test(char* buffer);
unsigned int read_sensor(void);
unsigned int read_sensor_dummy(void);

void c_StartI2C(void);
// private functions
unsigned int mask2shift(unsigned char mask);
void c_NackI2C( void );
void c_RestartI2C(void);
void c_ackI2C(void);
void c_stopI2C(void);
void c_idleI2C(void);

#endif
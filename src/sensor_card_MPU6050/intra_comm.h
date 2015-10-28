#ifndef INTRA_H
#define INTRA_H

#include "routines.h"

#define TIMEOUT 999

// define message packets
typedef struct
{
    unsigned ID:3;
    unsigned type:2;
    unsigned undefined:3;
    char data_size;
    unsigned int secret_code;
} request_msg;

typedef struct
{
    unsigned PACKETSIZE:7;
    unsigned request:1;
    unsigned undefined:8;
    unsigned int parameter;
    unsigned int new_value;
    unsigned int secret_code;
} config_msg;

extern char* I2C_TX_buffer_ptr;
extern char* I2C_RX_buffer_ptr;

// low level I2C functions
void I2C_master_init(void);
void master_update_bit_field(unsigned int device_ID, unsigned char sub_addr,
                             unsigned char bit_field, unsigned char mask);
void master_I2C_read(unsigned int device_ID, int sub_addr);
void master_I2C_write(unsigned int device_ID, int registerAddr);
void c_RestartI2C( void );

unsigned char master_I2C_read_byte(unsigned int device_ID, int registerAddr);
void master_I2C_write_byte(unsigned int device_ID, int registerAddr,
                           unsigned char data);
unsigned char c_WriteI2C( unsigned char data_out );
unsigned char c_ReadI2C( void );

// high level I2C protocols
void I2C_test(char* buffer);
unsigned int compute_8bit_checksum(char* start, unsigned int size);
char* master_request_data(unsigned int device_ID,
unsigned int dataSize, unsigned int type);
void c_ackI2C(void);
void c_stopI2C(void);

// private functions
unsigned int mask2shift(unsigned char mask);
void c_NackI2C( void );
void c_idleI2C(void);
#endif
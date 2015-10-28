#include "intra_comm.h"
#include "i2c.h"

unsigned int TX_buffer_size;
unsigned int RX_buffer_size;
unsigned int secret_code;

/******************************************************
 Low-level intrasystem comm protocols
 *****************************************************/

// MASTER_I2C_WRITE_BYTE
//         write a byte to a specific location of a slave device
//
// PARAM   device_ID   : I2C address of the slave
//         registerAddr: register address for a specific variable
//         data        : the data type being written.
void master_update_bit_field(unsigned int device_ID, unsigned char sub_addr,
                             unsigned char bit_field, unsigned char mask)
{
    unsigned char temp;
    temp = master_I2C_read_byte(device_ID, sub_addr);
    temp = (temp & ~mask) | (bit_field << (mask2shift(mask)) & mask);
    master_I2C_write_byte(device_ID, sub_addr, temp);
}

unsigned int mask2shift(unsigned char mask)
{
    unsigned int count;
    unsigned char last_bit_mask;
    count = 0;
    last_bit_mask = 0x01;
    while(!(mask & last_bit_mask)) {
        mask = mask >> 1;
        count++;
    }
    return count;
}

// MASTER_I2C_READ_BYTE 
//         wait for device to reply, read a byte into the first
//         byte of the data buffer.
//
// RETURNS the byte being read as unsigned char.
// PARAM   device_ID:    I2C address of the slave
//         registerAddr: register address of the
//         variable being read.
unsigned char master_I2C_read_byte(unsigned int device_ID, int registerAddr)
{
    RX_buffer_size = 1;
    master_I2C_read(device_ID, registerAddr);
    RX_buffer_size = 0;
    return I2C_RX_buffer_ptr[0];
}

// MASTER_I2C_WRITE_BYTE
//         write a byte to a specific location of a slave device
//
// PARAM   device_ID   : I2C address of the slave
//         registerAddr: register address for a specific variable
//         data        : the data type being written.
void master_I2C_write_byte(unsigned int device_ID, int registerAddr,
                           unsigned char data)
{
    TX_buffer_size = 1;
    I2C_TX_buffer_ptr[0] = data;
    master_I2C_write(device_ID, registerAddr);
    TX_buffer_size = 0;
}

// MASTER_I2C_WRITE
//         write whatever's inside the TX_buffer to the slave device
//
// PARAM   device_ID   : I2C address of the slave
//         registerAddr: register address for a specific variable
void master_I2C_write(unsigned int device_ID, int registerAddr)
{
    int i;
    SSP2CON2bits.RCEN = 0;

    IdleI2C();                         // Wait until the bus is idle
    StartI2C();                        // Send START condition
    IdleI2C();                         // Wait for the end of the START condition
    WriteI2C( device_ID << 1 | 0x00 );  // Send address with R/W cleared for write
    IdleI2C();                         // Wait for ACK
    check_ack();

    WriteI2C(registerAddr);               // Write first byte of data
    IdleI2C();  // Wait for ACK
    check_ack();

    for (i = 0; i < TX_buffer_size; i++)
    {
        WriteI2C(I2C_TX_buffer_ptr[i]);               // Write first byte of data
	IdleI2C();  // Wait for ACK
        check_ack();
    }
    StopI2C();                         // Hang up, send STOP condition
}


// MASTER_I2C_READ
//         read an array of data from the save device to the data_buffer
//
// PARAM   device_ID   : I2C address of the slave
//         registerAddr: register address for a specific variable
//         size        : size of the array being read

void master_I2C_read(unsigned int device_ID, int registerAddr)
{
    // write addr and subaddress
    int i;
    //SSP2CON2bits.RCEN = 1;

    IdleI2C();                         // Wait until the bus is idle
    StartI2C();                        // Send START condition
    IdleI2C();                         // Wait for the end of the START condition
    WriteI2C( device_ID <<1 | 0x00 );  // Send address with R/W cleared for write
    IdleI2C();                         // Wait for ACK
  //  check_ack();

    WriteI2C(registerAddr);               // Write first byte of data
    IdleI2C();  // Wait for ACK
  //  check_ack();

    //restart conditinon
    c_RestartI2C();

    //receive reading
    IdleI2C();                         // Wait for the end of the START condition
    WriteI2C( device_ID << 1 | 0x01 ); // Send address with R/W set for read
    check_ack();
    IdleI2C();
    
    // wait for data sent and acked
    for(i=0; i< RX_buffer_size-1; i++)
    {
        SSP2CON2bits.RCEN = 1;
        check_ack();	//sec_ackndc_ack ACK
	I2C_RX_buffer_ptr[i] = c_ReadI2C();

        // Read first byte of data

    }
    SSP2CON2bits.RCEN = 1;
    I2C_RX_buffer_ptr[RX_buffer_size - 1] = ReadI2C(); // Read nth byte of data

    NotAckI2C();                       // Send NACK
    c_stopI2C();                         // Hang up, send STOP condition
}

void I2C_slave_init(void)
{
    // set 7bit slave addr
//    SSP1ADDbits.SSP1ADD = 0;
//    SSP1ADDbits.SSP1ADD << 1;
    // slew rate control desabled for 100kHz
    SSP2STATbits.SMP = 1;
    // disable SMbus input level
    SSP2STATbits.CKE = 0;
    // set clock stretch as initially off
    SSP2CON1bits.CKP = 1;
    // set RB0 and RB1 as open drain I2C ports
    SSP2CON1bits.SSPEN = 1;
    // I2C slave mode, 7-bit address with start and stop
    // bit interrupt enabled
    SSP2CON1bits.SSPM = 0b1110;
    // enable general call address interrupt
    SSP2CON2bits.GCEN = 1;
    // SSP1BUF could only be updated when SSP1OV is cleared
    SSP2CON3bits.BOEN = 0;
    // disable data/addr hold
    SSP2CON3bits.AHEN = 0;
    SSP2CON3bits.DHEN = 0;
    // enable start/stop condition interrupt enable
    SSP2CON3bits.PCIE = 1;
    SSP2CON3bits.SCIE = 1;
    // 100ns hold time
    SSP2CON3bits.SDAHT = 0;
    // disable bus collision
    SSP2CON3bits.SBCDE = 0;
    SSP2MSK = 0xFF;
    
    SSP2ADD = (0x55<<1);
    
}


void I2C_master_init(void)
{
    //FOSC = 64MHz, I2C clock = 100kHz
    SSP1ADDbits.SSP1ADD = 0x9F;
    //slew rate control desabled for 100kHz
    SSP1STATbits.SMP = 0;
    // disable SMbus input level
    SSP1STATbits.CKE = 0;
    // set RC4 and RC3 as open drain I2C ports
    SSP1CON1bits.SSPEN = 1;
    // I2C master mode, clock = FOSC/(4*(SSP1ADD+1))
    SSP1CON1bits.SSPM = 0b1000;
    // enable receive mode for I2C master
    SSP1MSK = 0xFF;
    // enable general call
    SSP1CON2bits.GCEN = 1;
    // transmit mode
    SSP2CON2bits.RCEN = 0;
    // disable data/addr hold
    SSP1CON3bits.AHEN = 0;
    SSP1CON3bits.DHEN = 0;
    // enable start/stop condition interrupt enable
    SSP1CON3bits.PCIE = 1;
    SSP1CON3bits.SCIE = 1;
    // 100ns hold time
    SSP1CON3bits.SDAHT = 0;
    // disable bus collision
    SSP1CON3bits.SBCDE = 0;
}

// wait until ack arrived or ack timeout
void check_ack(void)
{
    int time_out = 0;
    int ack = SSPCON2bits.ACKSTAT;
    while (ack && time_out < TIMEOUT) {
        ack = SSPCON2bits.ACKSTAT;
        if (sampling_flag) {
            timer_rst();
            time_out++;
        }
    }
}



unsigned char c_ReadI2C( void )
{
    int time_out = 0;
    unsigned char result;
    while ( !SSP1STATbits.BF && time_out < TIMEOUT ){
        if (sampling_flag) {
            timer_rst();
            time_out++;
        }
    }      // wait until byte received
    if (time_out >= TIMEOUT) {
        error = ERROR_I2C_READ_TIMEOUT;
    }
    result = SSP1BUF;
    return result;              // return with read byte
}

unsigned char c_WriteI2C( unsigned char data_out )
{
    int time_out = 0;
    SSP1BUF = data_out;           // write single byte to SSPBUF
    if ( SSP1CON1bits.WCOL)      // test if write collision occurred
        return ( -1 );              // if WCOL bit is set return negative #
    else {
    while( SSP1STATbits.BF && time_out < TIMEOUT);   // wait until write cycle is complete
        if (sampling_flag) {
            timer_rst();
            time_out++;
        }
    }
    if (time_out >= TIMEOUT) {
        printf("\n\r Error: Write I2C timeout.");
    }
    return  0;
}

void c_NackI2C( void )
{
  SSP1CON2bits.ACKDT = 1;          // set acknowledge bit for not ACK
  SSP1CON2bits.ACKEN = 1;          // initiate bus acknowledge sequence
}

//ack the I2C
void c_ackI2C(void)
{
    SSP1CON2bits.ACKDT=0;
    SSP1CON2bits.ACKEN=1;
}

void c_stopI2C(void)
{
    SSP1CON2bits.PEN = 1;
}

void c_RestartI2C( void )
{
  SSP1CON2bits.RSEN = 1;           // initiate bus restart condition
}
/******************************************************
High-level intrasystem comm protocols
 *****************************************************/
// functions for masters

void I2C_test(char* buffer_ptr)
{
    buffer_ptr = I2C_TX_buffer_ptr;
    //send hello to device 0xFF at 0xFF
    //sprintf(I2C_TX_buffer_ptr, "Hello");
    buffer_ptr[0] = 'H';
    buffer_ptr[1] = 'e';
    buffer_ptr[2] = 'l';
    buffer_ptr[3] = 'l';
    buffer_ptr[4] = 'o';
    TX_buffer_size = 5;
    master_I2C_write(0x55, 0x33);
}

// check the first integer of RX_buffer
// return 1 if data is valid
unsigned int verify_data(char* buffer)
{
    unsigned int code;
    unsigned int result;
    code = bytes2int(buffer[0], buffer[1]);
    //printf("\n\rsecret is %d\n\r", secret_code);
    //update secret code
    result = (code == secret_code) ? 1:0;
    secret_code = (secret_code + 271) % 51;
    return result;
}

void c_idleI2C(void)
{
  while ( ( SSP1CON2 & 0x1F ) || ( SSPSTATbits.R_W ) )
     if (sampling_flag) {
         timer_rst();
     }
}




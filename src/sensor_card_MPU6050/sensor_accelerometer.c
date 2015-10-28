#include "sensor_accelerometer.h"

#include <math.h>

#define PI  3.14
#define alpha 0.005

//offset over 1000 data points/1000
#define OFFSET_X -5.70
#define OFFSET_Y 35.31
#define OFFSET_Z -2.63



int get_acceleration(){


    sensor_data_ptr->accel_X = (get_acceler_X() + OFFSET_X)/256;
    sensor_data_ptr->accel_Y = (get_acceler_Y() + OFFSET_Y)/256;
    sensor_data_ptr->accel_Z = (get_acceler_Z() + OFFSET_Z)/256;
 
     return 1;
}

void acceler_init(){

     //Put the ADXL345 into +/- 2G range by writing the value 0x01 to the DATA_FORMAT register.
     master_I2C_write_byte(ADXL345_ADDRESS_ALT_LOW, ADXL345_RA_DATA_FORMAT, 0b00001011);
    //set data rate to be 50 Hz
     
    
     //master_I2C_write_byte(ADXL345_ADDRESS_ALT_LOW, 0x2E, 0x80);
    
     master_I2C_write_byte(ADXL345_ADDRESS_ALT_LOW, ADXL345_RA_FIFO_CTL, 0x00);
      //master_I2C_write_byte(ADXL345_ADDRESS_ALT_LOW, 0x1E, -1350);
      //master_I2C_write_byte(ADXL345_ADDRESS_ALT_LOW, 0x1F, 2225);
      //master_I2C_write_byte(ADXL345_ADDRESS_ALT_LOW, 0x20, -561);
     //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
     master_I2C_write_byte(ADXL345_ADDRESS_ALT_LOW, ADXL345_RA_BW_RATE, 0x09);
     master_I2C_write_byte(ADXL345_ADDRESS_ALT_LOW, ADXL345_RA_POWER_CTL, 0x08);
}

int get_acceler_X(){
     int  acceler_X_raw;

     acceler_X_raw = master_I2C_read_byte(ADXL345_ADDRESS_ALT_LOW, ADXL345_RA_DATAX1);
     acceler_X_raw = (acceler_X_raw << 8) | master_I2C_read_byte(ADXL345_ADDRESS_ALT_LOW, ADXL345_RA_DATAX0);

    return acceler_X_raw;
}

int get_acceler_Y(){
    int acceler_Y_raw;
    acceler_Y_raw = master_I2C_read_byte(ADXL345_ADDRESS_ALT_LOW, ADXL345_RA_DATAY1);
    acceler_Y_raw = (acceler_Y_raw << 8) | master_I2C_read_byte(ADXL345_ADDRESS_ALT_LOW, ADXL345_RA_DATAY0);

    return acceler_Y_raw;
}

int get_acceler_Z(){
    int acceler_Z_raw = 0;
    char neg = 0;
    acceler_Z_raw =master_I2C_read_byte(ADXL345_ADDRESS_ALT_LOW, ADXL345_RA_DATAZ1);
    if (acceler_Z_raw == 3) {
        acceler_Z_raw = 0;
    } else if (acceler_Z_raw == 4) {
        acceler_Z_raw = 1;
    } else if (acceler_Z_raw == 2){
        acceler_Z_raw = 0xFF;
        neg = 1;
    }
    if (neg == 0) {
        acceler_Z_raw = (acceler_Z_raw << 8) | master_I2C_read_byte(ADXL345_ADDRESS_ALT_LOW, ADXL345_RA_DATAZ0);
    } else {
        acceler_Z_raw = (acceler_Z_raw << 8) | ((master_I2C_read_byte(1-ADXL345_ADDRESS_ALT_LOW, ADXL345_RA_DATAZ0)));
    }

    return acceler_Z_raw;
} 
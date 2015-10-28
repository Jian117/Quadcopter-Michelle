#include "sensor_magnetometer.h"
#include "routines.h"
#include "i2c.h"
#include<math.h>

#define COMPASS_SENSITIVITY 0.00256410256

int get_compass(){
    
    sensor_data_ptr->compass_X = get_compass_X() * COMPASS_SENSITIVITY;
    sensor_data_ptr->compass_Y = get_compass_Y() * COMPASS_SENSITIVITY;
    sensor_data_ptr->compass_Z = get_compass_Z() * COMPASS_SENSITIVITY;


   return 1;
}

void magneto_init(){    
    //15Hz
    master_I2C_write_byte(HMC5883L_ADDRESS, HMC5883L_RA_CONFIG_A, 0x70);
    //Gain = 5
    master_I2C_write_byte(HMC5883L_ADDRESS, HMC5883L_RA_CONFIG_B, 0xA0);
    //coninuous measurement
    master_I2C_write_byte(HMC5883L_ADDRESS, HMC5883L_RA_MODE, 0x00);
    //timer0_delay(300);
}

int get_compass_X(){
     int  compass_X_raw;
     
     compass_X_raw = master_I2C_read_byte(HMC5883L_ADDRESS, HMC5883L_RA_DATAX_H);
     compass_X_raw = (compass_X_raw << 8) | master_I2C_read_byte(HMC5883L_ADDRESS, HMC5883L_RA_DATAX_L);

    return compass_X_raw; 
}

int get_compass_Y(){
    int compass_Y_raw; 
    compass_Y_raw = master_I2C_read_byte(HMC5883L_ADDRESS, HMC5883L_RA_DATAY_H);
    compass_Y_raw = (compass_Y_raw << 8) | master_I2C_read_byte(HMC5883L_ADDRESS, HMC5883L_RA_DATAY_L);

    return compass_Y_raw; 
}

int get_compass_Z(){
    int compass_Z_raw; 
    compass_Z_raw = master_I2C_read_byte(HMC5883L_ADDRESS, HMC5883L_RA_DATAZ_H);
    compass_Z_raw = (compass_Z_raw << 8) | master_I2C_read_byte(HMC5883L_ADDRESS, HMC5883L_RA_DATAZ_L);

    return compass_Z_raw; 
}


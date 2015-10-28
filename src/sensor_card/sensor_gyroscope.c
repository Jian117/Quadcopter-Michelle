#include "sensor_gyroscope.h"
//int count;
#define OFF_G_X -0.4
#define OFF_G_Y 0.46
#define OFF_G_Z -0.74


int gyro_task(void)





{
    
  sensor_data_ptr->angular_velocity_X = (gyro_read_X_raw())*0.00875 +OFF_G_X;
  sensor_data_ptr->angular_velocity_Y = (gyro_read_Y_raw())*0.00875 +OFF_G_Y;
  sensor_data_ptr->angular_velocity_Z = (gyro_read_Z_raw())*0.00875 +OFF_G_Z;

    return 1;
}
















int gyro_read_X_raw(void)
{
    unsigned int gyro_X_raw;
    gyro_X_raw = master_I2C_read_byte(L3G4200D_ADDRESS, L3G4200D_REG_OUT_X_H);
    gyro_X_raw = (gyro_X_raw << 8) | master_I2C_read_byte(L3G4200D_ADDRESS, L3G4200D_REG_OUT_X_L);
    return gyro_X_raw;
}

int gyro_read_Y_raw(void)
{
    unsigned int gyro_Y_raw;
    gyro_Y_raw = master_I2C_read_byte(L3G4200D_ADDRESS, L3G4200D_REG_OUT_Y_H);
    gyro_Y_raw = (gyro_Y_raw << 8) | master_I2C_read_byte(L3G4200D_ADDRESS, L3G4200D_REG_OUT_Y_L);
    return gyro_Y_raw;
}

int gyro_read_Z_raw(void)
{
    unsigned int gyro_Z_raw;
    gyro_Z_raw = master_I2C_read_byte(L3G4200D_ADDRESS, L3G4200D_REG_OUT_Z_H);
    gyro_Z_raw = (gyro_Z_raw << 8) | master_I2C_read_byte(L3G4200D_ADDRESS, L3G4200D_REG_OUT_Z_L);
    return gyro_Z_raw;
}

void gyro_init(void)
{
    sensor_data_ptr->roll = 0;
    sensor_data_ptr->pitch = 0;
    // set CTRL_REG1 in L3G4200D
    // bit 7-6  output data rate
    // bit 5-4  bandwidth selection
    // bit 3    power down enable
    // bit 2    Z axis enable
    // bit 1    Y axis enable
    // bit 0    X axis enable
    master_I2C_write_byte(L3G4200D_ADDRESS, L3G4200D_REG_CTRL_REG1, 0b01101111);

    // set CTRL_REG3 in L3G4200D
    // bit 7    INT1 pin interrupt enable
    // bit 6    Boot status available on INT1
    // bit 5    Interrupt active config on INT1
    // bit 4    Push - pull / Open drain for I2C
    // bit 3    Data ready on RDY/INT2
    // bit 2    FIFO watermark interrupt RDY/INT2
    // bit 1    FIFO overrun interrupt RDY/INT2
    // bit 0    FIFO empty interrupt RDY/INT2
    master_I2C_write_byte(L3G4200D_ADDRESS, L3G4200D_REG_CTRL_REG3, 0b00010000);

    // set CTRL_REG3 in L3G4200D
    // bit 7    INT1 pin interrupt enable
    // bit 6    Boot status available on INT1
    // bit 5    Interrupt active config on INT1
    // bit 4    Push - pull / Open drain for I2C
    // bit 3    Data ready on RDY/INT2
    // bit 2    FIFO watermark interrupt RDY/INT2
    // bit 1    FIFO overrun interrupt RDY/INT2
    // bit 0    FIFO empty interrupt RDY/INT2
    master_I2C_write_byte(L3G4200D_ADDRESS, L3G4200D_REG_CTRL_REG3, 0b00010000);
}

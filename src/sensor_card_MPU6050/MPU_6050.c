#include "MPU_6050.h"

#include <math.h>

#define GYRO_SENSITIVITY 65.5
#define GYRO_XOUT_OFFSET 4
#define GYRO_YOUT_OFFSET -11
#define GYRO_ZOUT_OFFSET 11
void Setup_MPU6050(void)
{

    //Sets sample rate to 8000/1+7 = 1000Hz
    master_I2C_write_byte(MPU6050_ADDRESS, MPU6050_RA_SMPLRT_DIV, 0x07);
    //Disable FSync, 256Hz DLPF, DLPF_CFG =4
    master_I2C_write_byte(MPU6050_ADDRESS, MPU6050_RA_CONFIG, 0x00);
    //Disable gyro self tests, scale of 500 degrees/s
    master_I2C_write_byte(MPU6050_ADDRESS, MPU6050_RA_GYRO_CONFIG, 0b00001000);
    //Disable accel self tests, scale of +-2g, no DHPF
    master_I2C_write_byte(MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 0x00);

    //Freefall threshold of |0mg|
    master_I2C_write_byte(MPU6050_ADDRESS, MPU6050_RA_FF_THR, 0x00);
    //Freefall duration limit of 0
    master_I2C_write_byte(MPU6050_ADDRESS, MPU6050_RA_FF_DUR, 0x00);
    //Motion threshold of 0mg
    master_I2C_write_byte(MPU6050_ADDRESS, MPU6050_RA_MOT_THR, 0x00);
    //Motion duration of 0s
    master_I2C_write_byte(MPU6050_ADDRESS, MPU6050_RA_MOT_DUR, 0x00);
    //Zero motion threshold
    master_I2C_write_byte(MPU6050_ADDRESS, MPU6050_RA_ZRMOT_THR, 0x00);
    //Zero motion duration threshold
    master_I2C_write_byte(MPU6050_ADDRESS, MPU6050_RA_ZRMOT_DUR, 0x00);
    //Disable sensor output to FIFO buffer
    master_I2C_write_byte(MPU6050_ADDRESS, MPU6050_RA_FIFO_EN, 0x00);

    //AUX I2C setup
    //Sets AUX I2C to single master control, plus other config
    master_I2C_write_byte(MPU6050_ADDRESS, MPU6050_RA_I2C_MST_CTRL, 0x00);
    //Setup AUX I2C slaves
    master_I2C_write_byte(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_ADDR, 0x00);
    master_I2C_write_byte(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_REG, 0x00);
    master_I2C_write_byte(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_CTRL, 0x00);
    master_I2C_write_byte(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_ADDR, 0x00);
    master_I2C_write_byte(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_REG, 0x00);
    master_I2C_write_byte(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_CTRL, 0x00);
    master_I2C_write_byte(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_ADDR, 0x00);
    master_I2C_write_byte(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_REG, 0x00);
    master_I2C_write_byte(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_CTRL, 0x00);
    master_I2C_write_byte(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_ADDR, 0x00);
    master_I2C_write_byte(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_REG, 0x00);
    master_I2C_write_byte(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_CTRL, 0x00);
    master_I2C_write_byte(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_ADDR, 0x00);
    master_I2C_write_byte(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_REG, 0x00);
    master_I2C_write_byte(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_DO, 0x00);
    master_I2C_write_byte(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_CTRL, 0x00);
    master_I2C_write_byte(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_DI, 0x00);

    //MPU6050_RA_I2C_MST_STATUS //Read-only
    //Setup INT pin and AUX I2C pass through
    master_I2C_write_byte(MPU6050_ADDRESS, MPU6050_RA_INT_PIN_CFG, 0x00);
    //Enable data ready interrupt
    master_I2C_write_byte(MPU6050_ADDRESS, MPU6050_RA_INT_ENABLE, 0x00);

    //Slave out, dont care
    master_I2C_write_byte(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_DO, 0x00);
    master_I2C_write_byte(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_DO, 0x00);
    master_I2C_write_byte(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_DO, 0x00);
    master_I2C_write_byte(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_DO, 0x00);
    //More slave config
    master_I2C_write_byte(MPU6050_ADDRESS, MPU6050_RA_I2C_MST_DELAY_CTRL, 0x00);
    //Reset sensor signal paths
    master_I2C_write_byte(MPU6050_ADDRESS, MPU6050_RA_SIGNAL_PATH_RESET, 0x00);
    //Motion detection control
    master_I2C_write_byte(MPU6050_ADDRESS, MPU6050_RA_MOT_DETECT_CTRL, 0x00);
    //Disables FIFO, AUX I2C, FIFO and I2C reset bits to 0
    master_I2C_write_byte(MPU6050_ADDRESS, MPU6050_RA_USER_CTRL, 0x00);
    //Sets clock source to gyro reference w/ PLL
    master_I2C_write_byte(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0b00000010);
    //Controls frequency of wakeups in accel low power mode plus the sensor standby modes
    master_I2C_write_byte(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_2, 0x00);
   
    //Data transfer to and from the FIFO buffer
    master_I2C_write_byte(MPU6050_ADDRESS, MPU6050_RA_FIFO_R_W, 0x00);
    //MPU6050_RA_WHO_AM_I             //Read-only, I2C address
}



void Calibrate_Gyros(void)
{
    int GYRO_XOUT_H,GYRO_XOUT_L,GYRO_YOUT_H, GYRO_YOUT_L, GYRO_ZOUT_H, GYRO_ZOUT_L;
    int OFFSET_X, OFFSET_Y, OFFSET_Z;
    int x = 0;

    for(x = 0; x<1000; x++)
    {
	GYRO_XOUT_H = master_I2C_read_byte(MPU6050_ADDRESS, MPU6050_RA_GYRO_XOUT_H);
	GYRO_XOUT_L =  master_I2C_read_byte(MPU6050_ADDRESS, MPU6050_RA_GYRO_XOUT_L);
	GYRO_YOUT_H = master_I2C_read_byte(MPU6050_ADDRESS, MPU6050_RA_GYRO_YOUT_H);
	GYRO_YOUT_L = master_I2C_read_byte(MPU6050_ADDRESS, MPU6050_RA_GYRO_YOUT_L);
	GYRO_ZOUT_H = master_I2C_read_byte(MPU6050_ADDRESS, MPU6050_RA_GYRO_ZOUT_H);
	GYRO_ZOUT_L = master_I2C_read_byte(MPU6050_ADDRESS, MPU6050_RA_GYRO_ZOUT_L);

        OFFSET_X += ((GYRO_XOUT_H<<8)|GYRO_XOUT_L);
	OFFSET_Y += ((GYRO_YOUT_H<<8)|GYRO_YOUT_L);
	OFFSET_Z += ((GYRO_ZOUT_H<<8)|GYRO_ZOUT_L);
    }
	OFFSET_X = OFFSET_X/1000;
	OFFSET_Y = OFFSET_Y/1000;
	OFFSET_Z = OFFSET_Z/1000;


         sensor_data_ptr->angular_velocity_X = OFFSET_X;
         sensor_data_ptr->angular_velocity_Y = OFFSET_Y;
         sensor_data_ptr->angular_velocity_Z = OFFSET_Z;
}

//Gets raw accelerometer data, performs no processing
int Get_Accel_Values(void)
{
    int ACCEL_XOUT_H,ACCEL_XOUT_L,ACCEL_YOUT_H, ACCEL_YOUT_L,ACCEL_ZOUT_H,ACCEL_ZOUT_L;

	ACCEL_XOUT_H = master_I2C_read_byte(MPU6050_ADDRESS, MPU6050_RA_ACCEL_XOUT_H);
	ACCEL_XOUT_L = master_I2C_read_byte(MPU6050_ADDRESS, MPU6050_RA_ACCEL_XOUT_L);
	ACCEL_YOUT_H = master_I2C_read_byte(MPU6050_ADDRESS, MPU6050_RA_ACCEL_YOUT_H);
	ACCEL_YOUT_L = master_I2C_read_byte(MPU6050_ADDRESS, MPU6050_RA_ACCEL_YOUT_L);
	ACCEL_ZOUT_H = master_I2C_read_byte(MPU6050_ADDRESS, MPU6050_RA_ACCEL_ZOUT_H);
	ACCEL_ZOUT_L = master_I2C_read_byte(MPU6050_ADDRESS, MPU6050_RA_ACCEL_ZOUT_L);

	sensor_data_ptr->accel_X = ((ACCEL_XOUT_H<<8)|ACCEL_XOUT_L);
	sensor_data_ptr->accel_Y = ((ACCEL_YOUT_H<<8)|ACCEL_YOUT_L);
	sensor_data_ptr->accel_Z = ((ACCEL_ZOUT_H<<8)|ACCEL_ZOUT_L);

        return 1;
}
//Converts the already acquired accelerometer data into 3D euler angles
//void Get_Accel_Angles(void){
//	sensor_data_ptr->roll = 57.295*atan((float)sensor_data_ptr->accel_Y/ sqrt(pow((float)sensor_data_ptr->accel_Z,2)+pow((float)sensor_data_ptr->accel_X,2)));
//
//        sensor_data_ptr->pitch = 57.295*atan((float)-sensor_data_ptr->accel_X/ sqrt(pow((float)sensor_data_ptr->accel_Z,2)+pow((float)sensor_data_ptr->accel_Y,2)));
//
//}

//Function to read the gyroscope rate data and convert it into degrees/s

int Get_Gyro_Rates(void)
{
    int GYRO_XOUT_H, GYRO_XOUT_L, GYRO_YOUT_H, GYRO_YOUT_L, GYRO_ZOUT_H, GYRO_ZOUT_L;
    int GYRO_XOUT, GYRO_YOUT, GYRO_ZOUT;
    
	GYRO_XOUT_H = master_I2C_read_byte(MPU6050_ADDRESS, MPU6050_RA_GYRO_XOUT_H);
	GYRO_XOUT_L = master_I2C_read_byte(MPU6050_ADDRESS, MPU6050_RA_GYRO_XOUT_L);
	GYRO_YOUT_H = master_I2C_read_byte(MPU6050_ADDRESS, MPU6050_RA_GYRO_YOUT_H);
	GYRO_YOUT_L = master_I2C_read_byte(MPU6050_ADDRESS, MPU6050_RA_GYRO_YOUT_L);
	GYRO_ZOUT_H = master_I2C_read_byte(MPU6050_ADDRESS, MPU6050_RA_GYRO_ZOUT_H);
	GYRO_ZOUT_L = master_I2C_read_byte(MPU6050_ADDRESS, MPU6050_RA_GYRO_ZOUT_L);

	GYRO_XOUT = ((GYRO_XOUT_H<<8)|GYRO_XOUT_L) - GYRO_XOUT_OFFSET;
	GYRO_YOUT = ((GYRO_YOUT_H<<8)|GYRO_YOUT_L) - GYRO_YOUT_OFFSET;
	GYRO_ZOUT = ((GYRO_ZOUT_H<<8)|GYRO_ZOUT_L) - GYRO_ZOUT_OFFSET;


	//GYRO_XRATE = (float)GYRO_XOUT/gyro_xsensitivity;
	//GYRO_YRATE = (float)GYRO_YOUT/gyro_ysensitivity;
	//GYRO_ZRATE = (float)GYRO_ZOUT/gyro_zsensitivity;

        sensor_data_ptr->angular_velocity_X  = (float)((GYRO_XOUT_H<<8)|GYRO_XOUT_L)/GYRO_SENSITIVITY;
        sensor_data_ptr->angular_velocity_Y  = (float)((GYRO_YOUT_H<<8)|GYRO_YOUT_L)/GYRO_SENSITIVITY;
        sensor_data_ptr->angular_velocity_Z  = (float)((GYRO_ZOUT_H<<8)|GYRO_ZOUT_L)/GYRO_SENSITIVITY;
        
        return 1;
}








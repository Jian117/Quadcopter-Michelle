
#include "complementary_filter.h"
#include "i2c.h"
#include <math.h>

#define GYROSCOPE_SENSITIVITY 0.00875

#define M_PI 3.14159265

#define dt 0.05

#define alpha 0.15

#define round(x) ((x)>=0?(int)((x)+0.5):(int)((x)-0.5))

float fXg = 0;
float fYg = 0;
float fZg = 0;

void ComplementaryFilter()
{

   //using low pass-filter
   fXg  = sensor_data_ptr->accel_X * alpha + ( fXg * (1.0 - alpha));
   fYg  = sensor_data_ptr->accel_Y * alpha + ( fYg * (1.0 - alpha));
   fZg  = sensor_data_ptr->accel_Z  * alpha + ( fZg * (1.0 - alpha));

   sensor_data_ptr->roll  = (atan2(-fXg, fZg))*180/M_PI;
   //sensor_data_ptr->pitch  = (atan2(fYg, fZg))*180/M_PI;
   sensor_data_ptr->pitch = (atan2(fYg, sqrt(fXg*fXg + fZg*fZg)))*180/M_PI;

   //heading in Z
    sensor_data_ptr->yaw = atan2(sensor_data_ptr->compass_X,sensor_data_ptr->compass_Y)
            * 180 / M_PI +180 ;






//   if(sensor_data_ptr->compass_Y < 0){
//        sensor_data_ptr->yaw = 270 - atan2(sensor_data_ptr->compass_X,sensor_data_ptr->compass_Y)
//            * 180 / M_PI ;
//   }
//   else if (sensor_data_ptr->compass_Y > 0){
//       sensor_data_ptr->yaw = 90 - atan2(sensor_data_ptr->compass_X,sensor_data_ptr->compass_Y)
//            * 180 / M_PI ;
//   }
//   else if ((sensor_data_ptr->compass_Y == 0) &&  (sensor_data_ptr->compass_X < 0)){
//        sensor_data_ptr->yaw = 180;
//   }
//   else if ((sensor_data_ptr->compass_Y == 0) && (sensor_data_ptr->compass_X > 0)){
//        sensor_data_ptr->yaw = 0;
//   }
   
     
   //Direction (y>0) = 90 - [arcTAN(x/y)]*180/Pi
   //Direction (y<0) = 270 - [arcTAN(x/y)]*180/Pi


    // Integrate the gyroscope data -> int(angularSpeed) = angle
    //sensor_data_ptr->pitch_gy += ((float)sensor_data_ptr->angular_velocity_X  ) * dt; // Angle around the X-axis
    //sensor_data_ptr->roll_gy  -= ((float)sensor_data_ptr->angular_velocity_Y  ) * dt;    // Angle around the Y-axis


    //sensor_data_ptr->pitch = Round(sensor_data_ptr->pitch_g);
    //->roll =  Round(sensor_data_ptr->roll_g);
	// Turning around the X axis results in a vector on the Y-axis
  // sensor_data_ptr->pitch = atan2((float)sensor_data_ptr->accel_Y, (float)sensor_data_ptr->accel_Z) * 180 / M_PI;
    //  sensor_data_ptr->pitch_g = sensor_data_ptr->pitch_g * 0.98 + pitchAcc * 0.02;

	// Turning around the Y axis results in a vector on the X-axis
    
   // sensor_data_ptr->roll = atan2((float)sensor_data_ptr->accel_X, (float)sensor_data_ptr->accel_Z) * 180 / M_PI;
      
    //sensor_data_ptr->roll = sensor_data_ptr->roll_gy * 0.70 + sensor_data_ptr->roll_ac * 0.30;

}

int Round(float myfloat)
{
  return (int)(myfloat + 0.5);
}

// fusion final
//         Compute the final sensor outputs that feed into I2C to the flight controller
//
// PARAM   sensor_data_ptr: Pointer to the sensor data storage
//         sensor_result_ptr: Pointer to the sensor final result struct
void fusion_final(void)
{

    sensor_result_ptr->pitch =  (long)(sensor_data_ptr->pitch * 1000);
    sensor_result_ptr->roll =  (long)(sensor_data_ptr->roll * 1000);
    sensor_result_ptr->yaw =  (long)(sensor_data_ptr->yaw * 1000);
    
}
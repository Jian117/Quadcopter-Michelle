
#ifndef GYRO_H
#define GYRO_H

#include "routines.h"

#define L3G4200D_ADDRESS           0x69
#define L3G4200D_REG_WHO_AM_I       0x0F
#define L3G4200D_REG_CTRL_REG1      0x20
#define L3G4200D_REG_CTRL_REG2      0x21
#define L3G4200D_REG_CTRL_REG3      0x22
#define L3G4200D_REG_CTRL_REG4      0x23
#define L3G4200D_REG_CTRL_REG5      0x24
#define L3G4200D_REG_REFERENCE      0x25
#define L3G4200D_REG_OUT_TEMP       0x26
#define L3G4200D_REG_STATUS         0x27
#define L3G4200D_REG_OUT_X_L        0x28
#define L3G4200D_REG_OUT_X_H        0x29
#define L3G4200D_REG_OUT_Y_L        0x2A
#define L3G4200D_REG_OUT_Y_H        0x2B
#define L3G4200D_REG_OUT_Z_L        0x2C
#define L3G4200D_REG_OUT_Z_H        0x2D
#define L3G4200D_REG_FIFO_CTRL      0x2E
#define L3G4200D_REG_FIFO_SRC       0x2F
#define L3G4200D_REG_INT1_CFG       0x30
#define L3G4200D_REG_INT1_SRC       0x31
#define L3G4200D_REG_INT1_THS_XH    0x32
#define L3G4200D_REG_INT1_THS_XL    0X33
#define L3G4200D_REG_INT1_THS_YH    0X34
#define L3G4200D_REG_INT1_THS_YL    0x35
#define L3G4200D_REG_INT1_THS_ZH    0X36
#define L3G4200D_REG_INT1_THS_ZL    0x37
#define L3G4200D_REG_INT1_DURATION  0X38

#define L3G4200D_FS_MASK 0b11001111
#define L3G4200D_FS_250  0b00
#define L3G4200D_FS_500  0b01
#define L3G4200D_FS_2000 0b11

//extern int count;
int gyro_task(void);
int gyro_read_X_raw(void);
int gyro_read_Y_raw(void);
int gyro_read_Z_raw(void);
void gyro_init(void);

#endif
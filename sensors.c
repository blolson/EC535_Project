#include <stdio.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string.h>
#include <sys/time.h>
#include "i2c.h"
#include "sensors.h"
#include <math.h>
#include "MahonyAHRS.h"

//=============BLADE: THIS IS FOR THE PRESSURE SENSOR ADC==================
/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    #define ADS1015_ADDRESS                 (0x48)    // 1001 000 (ADDR = GND)
/*=========================================================================*/

/*=========================================================================
    CONVERSION DELAY (in mS)
    -----------------------------------------------------------------------*/
    #define ADS1015_CONVERSIONDELAY         (1)
    #define ADS1115_CONVERSIONDELAY         (8)
/*=========================================================================*/

/*=========================================================================
    POINTER REGISTER
    -----------------------------------------------------------------------*/
    #define ADS1015_REG_POINTER_MASK        (0x03)
    #define ADS1015_REG_POINTER_CONVERT     (0x00)
    #define ADS1015_REG_POINTER_CONFIG      (0x01)
    #define ADS1015_REG_POINTER_LOWTHRESH   (0x02)
    #define ADS1015_REG_POINTER_HITHRESH    (0x03)
/*=========================================================================*/

/*=========================================================================
    CONFIG REGISTER
    -----------------------------------------------------------------------*/
    #define ADS1015_REG_CONFIG_OS_MASK      (0x8000)
    #define ADS1015_REG_CONFIG_OS_SINGLE    (0x8000)  // Write: Set to start a single-conversion
    #define ADS1015_REG_CONFIG_OS_BUSY      (0x0000)  // Read: Bit = 0 when conversion is in progress
    #define ADS1015_REG_CONFIG_OS_NOTBUSY   (0x8000)  // Read: Bit = 1 when device is not performing a conversion

    #define ADS1015_REG_CONFIG_MUX_MASK     (0x7000)
    #define ADS1015_REG_CONFIG_MUX_DIFF_0_1 (0x0000)  // Differential P = AIN0, N = AIN1 (default)
    #define ADS1015_REG_CONFIG_MUX_DIFF_0_3 (0x1000)  // Differential P = AIN0, N = AIN3
    #define ADS1015_REG_CONFIG_MUX_DIFF_1_3 (0x2000)  // Differential P = AIN1, N = AIN3
    #define ADS1015_REG_CONFIG_MUX_DIFF_2_3 (0x3000)  // Differential P = AIN2, N = AIN3
    #define ADS1015_REG_CONFIG_MUX_SINGLE_0 (0x4000)  // Single-ended AIN0
    #define ADS1015_REG_CONFIG_MUX_SINGLE_1 (0x5000)  // Single-ended AIN1
    #define ADS1015_REG_CONFIG_MUX_SINGLE_2 (0x6000)  // Single-ended AIN2
    #define ADS1015_REG_CONFIG_MUX_SINGLE_3 (0x7000)  // Single-ended AIN3

    #define ADS1015_REG_CONFIG_PGA_MASK     (0x0E00)
    #define ADS1015_REG_CONFIG_PGA_6_144V   (0x0000)  // +/-6.144V range = Gain 2/3
    #define ADS1015_REG_CONFIG_PGA_4_096V   (0x0200)  // +/-4.096V range = Gain 1
    #define ADS1015_REG_CONFIG_PGA_2_048V   (0x0400)  // +/-2.048V range = Gain 2 (default)
    #define ADS1015_REG_CONFIG_PGA_1_024V   (0x0600)  // +/-1.024V range = Gain 4
    #define ADS1015_REG_CONFIG_PGA_0_512V   (0x0800)  // +/-0.512V range = Gain 8
    #define ADS1015_REG_CONFIG_PGA_0_256V   (0x0A00)  // +/-0.256V range = Gain 16

    #define ADS1015_REG_CONFIG_MODE_MASK    (0x0100)
    #define ADS1015_REG_CONFIG_MODE_CONTIN  (0x0000)  // Continuous conversion mode
    #define ADS1015_REG_CONFIG_MODE_SINGLE  (0x0100)  // Power-down single-shot mode (default)

    #define ADS1015_REG_CONFIG_DR_MASK      (0x00E0)  
    #define ADS1015_REG_CONFIG_DR_128SPS    (0x0000)  // 128 samples per second
    #define ADS1015_REG_CONFIG_DR_250SPS    (0x0020)  // 250 samples per second
    #define ADS1015_REG_CONFIG_DR_490SPS    (0x0040)  // 490 samples per second
    #define ADS1015_REG_CONFIG_DR_920SPS    (0x0060)  // 920 samples per second
    #define ADS1015_REG_CONFIG_DR_1600SPS   (0x0080)  // 1600 samples per second (default)
    #define ADS1015_REG_CONFIG_DR_2400SPS   (0x00A0)  // 2400 samples per second
    #define ADS1015_REG_CONFIG_DR_3300SPS   (0x00C0)  // 3300 samples per second

    #define ADS1015_REG_CONFIG_CMODE_MASK   (0x0010)
    #define ADS1015_REG_CONFIG_CMODE_TRAD   (0x0000)  // Traditional comparator with hysteresis (default)
    #define ADS1015_REG_CONFIG_CMODE_WINDOW (0x0010)  // Window comparator

    #define ADS1015_REG_CONFIG_CPOL_MASK    (0x0008)
    #define ADS1015_REG_CONFIG_CPOL_ACTVLOW (0x0000)  // ALERT/RDY pin is low when active (default)
    #define ADS1015_REG_CONFIG_CPOL_ACTVHI  (0x0008)  // ALERT/RDY pin is high when active

    #define ADS1015_REG_CONFIG_CLAT_MASK    (0x0004)  // Determines if ALERT/RDY pin latches once asserted
    #define ADS1015_REG_CONFIG_CLAT_NONLAT  (0x0000)  // Non-latching comparator (default)
    #define ADS1015_REG_CONFIG_CLAT_LATCH   (0x0004)  // Latching comparator

    #define ADS1015_REG_CONFIG_CQUE_MASK    (0x0003)
    #define ADS1015_REG_CONFIG_CQUE_1CONV   (0x0000)  // Assert ALERT/RDY after one conversions
    #define ADS1015_REG_CONFIG_CQUE_2CONV   (0x0001)  // Assert ALERT/RDY after two conversions
    #define ADS1015_REG_CONFIG_CQUE_4CONV   (0x0002)  // Assert ALERT/RDY after four conversions
    #define ADS1015_REG_CONFIG_CQUE_NONE    (0x0003)  // Disable the comparator and put ALERT/RDY in high state (default)
/*=========================================================================*/





//=============BLADE: THIS IS FOR THE 9-DOF IMU==================
//=========================================================================
#define SENSORS_GRAVITY_STANDARD (9.80665F) /**< Earth's gravity in m/s^2 */

// Linear Acceleration: mg per LSB
#define LSM9DS0_ACCEL_MG_LSB_2G (0.061F)
#define LSM9DS0_ACCEL_MG_LSB_4G (0.122F)
#define LSM9DS0_ACCEL_MG_LSB_6G (0.183F)
#define LSM9DS0_ACCEL_MG_LSB_8G (0.244F)
#define LSM9DS0_ACCEL_MG_LSB_16G (0.732F) // Is this right? Was expecting 0.488F

// Magnetic Field Strength: gauss range
#define LSM9DS0_MAG_MGAUSS_2GAUSS      (0.08F)
#define LSM9DS0_MAG_MGAUSS_4GAUSS      (0.16F)
#define LSM9DS0_MAG_MGAUSS_8GAUSS      (0.32F)
#define LSM9DS0_MAG_MGAUSS_12GAUSS     (0.48F)

// Angular Rate: dps per LSB
#define LSM9DS0_GYRO_DPS_DIGIT_245DPS      (0.00875F)
#define LSM9DS0_GYRO_DPS_DIGIT_500DPS      (0.01750F)
#define LSM9DS0_GYRO_DPS_DIGIT_2000DPS (0.07000F)


//Gyrometer registers
#define LSM9DS0_REGISTER_WHO_AM_I_G             0x0F
#define LSM9DS0_REGISTER_CTRL_REG1_G            0x20
#define LSM9DS0_REGISTER_CTRL_REG3_G            0x22
#define LSM9DS0_REGISTER_CTRL_REG4_G            0x23
#define LSM9DS0_REGISTER_OUT_X_L_G              0x28
#define LSM9DS0_REGISTER_OUT_X_H_G              0x29
#define LSM9DS0_REGISTER_OUT_Y_L_G              0x2A
#define LSM9DS0_REGISTER_OUT_Y_H_G              0x2B
#define LSM9DS0_REGISTER_OUT_Z_L_G              0x2C
#define LSM9DS0_REGISTER_OUT_Z_H_G              0x2D

//Accelerometer and Magnetometer registers
#define LSM9DS0_REGISTER_TEMP_OUT_L_XM          0x05
#define LSM9DS0_REGISTER_TEMP_OUT_H_XM          0x06
#define LSM9DS0_REGISTER_STATUS_REG_M           0x07
#define LSM9DS0_REGISTER_OUT_X_L_M              0x08
#define LSM9DS0_REGISTER_OUT_X_H_M              0x09
#define LSM9DS0_REGISTER_OUT_Y_L_M              0x0A
#define LSM9DS0_REGISTER_OUT_Y_H_M              0x0B
#define LSM9DS0_REGISTER_OUT_Z_L_M              0x0C
#define LSM9DS0_REGISTER_OUT_Z_H_M              0x0D
#define LSM9DS0_REGISTER_WHO_AM_I_XM            0x0F
#define LSM9DS0_REGISTER_INT_CTRL_REG_M         0x12
#define LSM9DS0_REGISTER_INT_SRC_REG_M          0x13
#define LSM9DS0_REGISTER_CTRL_REG1_XM           0x20
#define LSM9DS0_REGISTER_CTRL_REG2_XM           0x21
#define LSM9DS0_REGISTER_CTRL_REG5_XM           0x24
#define LSM9DS0_REGISTER_CTRL_REG6_XM           0x25
#define LSM9DS0_REGISTER_CTRL_REG7_XM           0x26
#define LSM9DS0_REGISTER_OUT_X_L_A              0x28
#define LSM9DS0_REGISTER_OUT_X_H_A              0x29
#define LSM9DS0_REGISTER_OUT_Y_L_A              0x2A
#define LSM9DS0_REGISTER_OUT_Y_H_A              0x2B
#define LSM9DS0_REGISTER_OUT_Z_L_A              0x2C
#define LSM9DS0_REGISTER_OUT_Z_H_A              0x2D

/*---------=IMU #1=-----------*/
//Angular rate SAD+read/write patterns
#define LSM9DS0_ADDRESS_1_GYRO_READ               0xd7 //(11010111) - 107 - Read
#define LSM9DS0_ADDRESS_1_GYRO_WRITE              0xd6 //(11010110) - Write

//Linear acceleration and magnetic sensor SAD+read/write patterns
#define LSM9DS0_ADDRESS_1_ACCELMAG_READ           0x3b //(00111011) - 29 - Read
#define LSM9DS0_ADDRESS_1_ACCELMAG_WRITE          0x3a //(00111010) - Write


/*---------=IMU #2=-----------*/
//Angular rate SAD+read/write patterns
#define LSM9DS0_ADDRESS_2_GYRO_READ               0xd5 //(11010101) - Read
#define LSM9DS0_ADDRESS_2_GYRO_WRITE              0xd4 //(11010100) - Write

//Linear acceleration and magnetic sensor SAD+read/write patterns
#define LSM9DS0_ADDRESS_2_ACCELMAG_READ           0x3d //(00111011) - Read
#define LSM9DS0_ADDRESS_2_ACCELMAG_WRITE          0x3c //(00111010) - Write


coords getMag(int i2c_file)
{
  int address = LSM9DS0_ADDRESS_1_ACCELMAG_READ;
  unsigned char mag_X_L,mag_X_H,mag_Y_L,mag_Y_H,mag_Z_L,mag_Z_H;
  short mag_X,mag_Y,mag_Z;
  float mX, mY, mZ;
  get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_X_L_M, &mag_X_L);
  get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_X_H_M, &mag_X_H);
  get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_Y_L_M, &mag_Y_L);
  get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_Y_H_M, &mag_Y_H);
  get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_Z_L_M, &mag_Z_L);
  get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_Z_H_M, &mag_Z_H);
  mag_X = mag_X_L + (mag_X_H << 8);
  mag_Y = mag_Y_L + (mag_Y_H << 8);
  mag_Z = mag_Z_L + (mag_Z_H << 8);

  mX = mag_X * LSM9DS0_MAG_MGAUSS_4GAUSS;
  mX /= 1000;
  mY = mag_Y * LSM9DS0_MAG_MGAUSS_4GAUSS;
  mY /= 1000;
  mZ = mag_Z * LSM9DS0_MAG_MGAUSS_4GAUSS;
  mZ /= 1000;
  coords xyz = {MAG, mX, mY, mZ};
  return xyz;
}


coords getGyro(int i2c_file)
{
  int address = LSM9DS0_ADDRESS_1_GYRO_READ;
  unsigned char gyro_X_L,gyro_X_H,gyro_Y_L,gyro_Y_H,gyro_Z_L,gyro_Z_H;
  short gyro_X,gyro_Y,gyro_Z;
  float gX, gY, gZ;
  get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_X_L_G, &gyro_X_L);
  get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_X_H_G, &gyro_X_H);
  get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_Y_L_G, &gyro_Y_L);
  get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_Y_H_G, &gyro_Y_H);
  get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_Z_L_G, &gyro_Z_L);
  get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_Z_H_G, &gyro_Z_H);
  gyro_X = gyro_X_L + (gyro_X_H << 8);
  gyro_Y = gyro_Y_L + (gyro_Y_H << 8);
  gyro_Z = gyro_Z_L + (gyro_Z_H << 8);
  gX = gyro_X * LSM9DS0_GYRO_DPS_DIGIT_245DPS;
  gY = gyro_Y * LSM9DS0_GYRO_DPS_DIGIT_245DPS;
  gZ = gyro_Z * LSM9DS0_GYRO_DPS_DIGIT_245DPS;
  coords xyz = {GYRO, gX, gY, gZ};
  return xyz;
}

coords getAccel(int i2c_file)
{
  int address = LSM9DS0_ADDRESS_1_ACCELMAG_READ;
  unsigned char accel_X_L,accel_X_H,accel_Y_L,accel_Y_H,accel_Z_L,accel_Z_H;
  short accel_X,accel_Y,accel_Z;
  float aX, aY, aZ;

  get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_X_L_A, &accel_X_L);
  get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_X_H_A, &accel_X_H);
  get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_Y_L_A, &accel_Y_L);
  get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_Y_H_A, &accel_Y_H);
  get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_Z_L_A, &accel_Z_L);
  get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_Z_H_A, &accel_Z_H);
	
  accel_X = (accel_X_L + (accel_X_H << 8));
  aX = (float)accel_X * LSM9DS0_ACCEL_MG_LSB_2G;
  aX /= 1000;
  aX *= SENSORS_GRAVITY_STANDARD;

  accel_Y = accel_Y_L + (accel_Y_H << 8);
  aY = (float)accel_Y * LSM9DS0_ACCEL_MG_LSB_2G;
  aY /= 1000;
  aY *= SENSORS_GRAVITY_STANDARD;

  accel_Z = accel_Z_L + (accel_Z_H << 8);
  aZ = (float)accel_Z * LSM9DS0_ACCEL_MG_LSB_2G;
  aZ /= 1000;
  aZ *= SENSORS_GRAVITY_STANDARD;
  coords xyz = {ACCEL, aX, aY, aZ};
  
  return xyz;
}

short getPressure(int i2c_file)
{
  short ADC_value;	
  get_i2c_register_16(i2c_file, ADS1015_ADDRESS, ADS1015_REG_POINTER_CONVERT, &ADC_value);
  //printf("ADC Value: %d\n", ADC_value >> 4);
  return ADC_value >> 4;
}


void setupADC_Comparator(int i2c_file, int channel, short threshold)
{
  // Start with default values
  short config =	ADS1015_REG_CONFIG_CQUE_1CONV   | // Comparator enabled and asserts on 1 match
                    ADS1015_REG_CONFIG_CLAT_LATCH   | // Non-Latching mode
                    ADS1015_REG_CONFIG_CPOL_ACTVHI | // Alert/Rdy active high
		    ADS1015_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
                    ADS1015_REG_CONFIG_DR_1600SPS   | // 1600 samples per second (default)
                    ADS1015_REG_CONFIG_MODE_CONTIN;   // Continuous conversion mode

  // Set PGA/voltage range
  config |= ADS1015_REG_CONFIG_PGA_6_144V;
                    
  printf("Value of config is: %d (%x)", config, config);

  // Set single-ended input channel
  switch (channel)
  {
    case (0):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_0;
      break;
    case (1):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_1;
      break;
    case (2):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_2;
      break;
    case (3):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_3;
      break;
  }

  // Set the high threshold register
  // Shift 12-bit results left 4 bits for the ADS1015
  set_i2c_register_16(i2c_file, ADS1015_ADDRESS, ADS1015_REG_POINTER_HITHRESH, threshold << 4);

  // Write config register to the ADC
  set_i2c_register_16(i2c_file, ADS1015_ADDRESS, ADS1015_REG_POINTER_CONFIG, config);
}



void setupGyro(int i2c_file, int address)
{
  //enable gyro continuous update
  set_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_CTRL_REG1_G, 15); //0x0F
}

void setupAccelMag(int i2c_file, int address)
{

  //enable accel to update @ 100Hz
  set_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_CTRL_REG1_XM, 103); //0x67

  //enable mag continuous
  set_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_CTRL_REG7_XM, 0);
}


void setup_all_sensors(int i2c_file)
{
  setupAccelMag(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_WRITE);
  setupGyro(i2c_file, LSM9DS0_ADDRESS_1_GYRO_WRITE);
  setupADC_Comparator(i2c_file, 0, 1000);
}


void print(coords c) {
  char *sensor = (c.type == ACCEL) ? "ACC" : (c.type == MAG) ? "MAG" : (c.type == EULER) ? "EULER" : "GYR";
  printf("%s, %f, %f, %f, \n", sensor, c.X, c.Y, c.Z);
}

short getGyroX(int i2c_file, int address)
{
        unsigned char _L,_H;
        get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_X_L_G, &_L);
        get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_X_H_G, &_H);
        return _L + (_H << 8);
}
short getGyroY(int i2c_file, int address)
{
        unsigned char _L,_H;
        get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_Y_L_G, &_L);
        get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_Y_H_G, &_H);
        return _L + (_H << 8);
}
short getGyroZ(int i2c_file, int address)
{
        unsigned char _L,_H;
        get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_Z_L_G, &_L);
        get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_Z_H_G, &_H);
        return _L + (_H << 8);
}

short getMagX(int i2c_file, int address)
{
        unsigned char _L,_H;
        get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_X_L_M, &_L);
        get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_X_H_M, &_H);
        return _L + (_H << 8);
}
short getMagY(int i2c_file, int address)
{
        unsigned char _L,_H;
        get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_Y_L_M, &_L);
        get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_Y_H_M, &_H);
        return _L + (_H << 8);
}
short getMagZ(int i2c_file, int address)
{
        unsigned char _L,_H;
        get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_Z_L_M, &_L);
        get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_Z_H_M, &_H);
        return _L + (_H << 8);
}


short getAccelX(int i2c_file, int address)
{
        unsigned char _L,_H;
        get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_X_L_A, &_L);
        get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_X_H_A, &_H);
        return _L + (_H << 8);
}
short getAccelY(int i2c_file, int address)
{
        unsigned char _L,_H;
        get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_Y_L_A, &_L);
        get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_Y_H_A, &_H);
        return _L + (_H << 8);
}
short getAccelZ(int i2c_file, int address)
{
        unsigned char _L,_H;
        get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_Z_L_A, &_L);
        get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_Z_H_A, &_H);
        return _L + (_H << 8);
}


void getRollPitchYaw(int i2c_file, float *roll, float *pitch, float *yaw) {
  float aX,aY,aZ,gX,gY,gZ,mX,mY,mZ;
  float bias[3];
  aX = getAccelX(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_READ) * LSM9DS0_ACCEL_MG_LSB_2G;
  aX /= 1000;
  aX *= SENSORS_GRAVITY_STANDARD;
  aY = getAccelY(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_READ) * LSM9DS0_ACCEL_MG_LSB_2G;
  aY /= 1000;
  aY *= SENSORS_GRAVITY_STANDARD;
  aZ = getAccelZ(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_READ) * LSM9DS0_ACCEL_MG_LSB_2G;
  aZ /= 1000;
  aZ *= SENSORS_GRAVITY_STANDARD;
  gX = getGyroX(i2c_file, LSM9DS0_ADDRESS_1_GYRO_READ) * LSM9DS0_GYRO_DPS_DIGIT_245DPS;
  gY = getGyroY(i2c_file, LSM9DS0_ADDRESS_1_GYRO_READ) * LSM9DS0_GYRO_DPS_DIGIT_245DPS;
  gZ = getGyroZ(i2c_file, LSM9DS0_ADDRESS_1_GYRO_READ) * LSM9DS0_GYRO_DPS_DIGIT_245DPS;
  mX = getMagX(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_READ) * LSM9DS0_MAG_MGAUSS_4GAUSS;
  mX /= 1000;
  mX -= bias[0];
  mY = getMagY(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_READ) * LSM9DS0_MAG_MGAUSS_4GAUSS;
  mY /= 1000;
  mY -= bias[1];
  mZ = getMagZ(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_READ) * LSM9DS0_MAG_MGAUSS_4GAUSS;
  mZ /= 1000;
  mZ -= bias[2];
  MahonyAHRSupdate(gX, gY, gZ, aX, aY, aZ, mX, mY, mZ);
  *roll = getRoll();
  *pitch = getPitch();
  *yaw = getYaw();
}


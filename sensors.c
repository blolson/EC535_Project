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

#define LSM9DS0_ADDRESS_1_ACCELMAG_READ           0x3b //(00111011) - 29 - Read
#define LSM9DS0_REGISTER_OUT_X_L_M              0x08
#define LSM9DS0_REGISTER_OUT_X_H_M              0x09
#define LSM9DS0_REGISTER_OUT_Y_L_M              0x0A
#define LSM9DS0_REGISTER_OUT_Y_H_M              0x0B
#define LSM9DS0_REGISTER_OUT_Z_L_M              0x0C
#define LSM9DS0_REGISTER_OUT_Z_H_M              0x0D
#define LSM9DS0_REGISTER_OUT_X_L_G              0x28
#define LSM9DS0_REGISTER_OUT_X_H_G              0x29
#define LSM9DS0_REGISTER_OUT_Y_L_G              0x2A
#define LSM9DS0_REGISTER_OUT_Y_H_G              0x2B
#define LSM9DS0_REGISTER_OUT_Z_L_G              0x2C
#define LSM9DS0_REGISTER_OUT_Z_H_G              0x2D
#define LSM9DS0_REGISTER_OUT_X_L_A              0x28
#define LSM9DS0_REGISTER_OUT_X_H_A              0x29
#define LSM9DS0_REGISTER_OUT_Y_L_A              0x2A
#define LSM9DS0_REGISTER_OUT_Y_H_A              0x2B
#define LSM9DS0_REGISTER_OUT_Z_L_A              0x2C
#define LSM9DS0_REGISTER_OUT_Z_H_A              0x2D
#define LSM9DS0_ADDRESS_1_GYRO_READ               0xd7 //(11010111) - 107 - Read
#define LSM9DS0_ADDRESS_1_GYRO_WRITE              0xd6 //(11010110) - Write
#define LSM9DS0_ADDRESS_1_ACCELMAG_READ           0x3b //(00111011) - 29 - Read
#define LSM9DS0_ADDRESS_1_ACCELMAG_WRITE          0x3a //(00111010) - Write
#define LSM9DS0_REGISTER_CTRL_REG1_G            0x20
#define LSM9DS0_REGISTER_CTRL_REG1_XM           0x20
#define LSM9DS0_REGISTER_CTRL_REG7_XM           0x26

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

coords getMag(int i2c_file)
{
  unsigned char gyro_X_L,gyro_X_H,gyro_Y_L,gyro_Y_H,gyro_Z_L,gyro_Z_H;
  short gyro_X,gyro_Y,gyro_Z;
  get_i2c_register(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_READ >> 1, LSM9DS0_REGISTER_OUT_X_L_M, &gyro_X_L);
  get_i2c_register(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_READ >> 1, LSM9DS0_REGISTER_OUT_X_H_M, &gyro_X_H);
  get_i2c_register(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_READ >> 1, LSM9DS0_REGISTER_OUT_Y_L_M, &gyro_Y_L);
  get_i2c_register(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_READ >> 1, LSM9DS0_REGISTER_OUT_Y_H_M, &gyro_Y_H);
  get_i2c_register(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_READ >> 1, LSM9DS0_REGISTER_OUT_Z_L_M, &gyro_Z_L);
  get_i2c_register(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_READ >> 1, LSM9DS0_REGISTER_OUT_Z_H_M, &gyro_Z_H);
  gyro_X = gyro_X_L + (gyro_X_H << 8);
  gyro_Y = gyro_Y_L + (gyro_Y_H << 8);
  gyro_Z = gyro_Z_L + (gyro_Z_H << 8);
  coords xyz = {MAG, gyro_X, gyro_Y, gyro_Z};
  return xyz;
}


coords getGyro(int i2c_file)
{
  unsigned char gyro_X_L,gyro_X_H,gyro_Y_L,gyro_Y_H,gyro_Z_L,gyro_Z_H;
  short gyro_X,gyro_Y,gyro_Z;
  get_i2c_register(i2c_file, LSM9DS0_ADDRESS_1_GYRO_READ >> 1, LSM9DS0_REGISTER_OUT_X_L_G, &gyro_X_L);
  get_i2c_register(i2c_file, LSM9DS0_ADDRESS_1_GYRO_READ >> 1, LSM9DS0_REGISTER_OUT_X_H_G, &gyro_X_H);
  get_i2c_register(i2c_file, LSM9DS0_ADDRESS_1_GYRO_READ >> 1, LSM9DS0_REGISTER_OUT_Y_L_G, &gyro_Y_L);
  get_i2c_register(i2c_file, LSM9DS0_ADDRESS_1_GYRO_READ >> 1, LSM9DS0_REGISTER_OUT_Y_H_G, &gyro_Y_H);
  get_i2c_register(i2c_file, LSM9DS0_ADDRESS_1_GYRO_READ >> 1, LSM9DS0_REGISTER_OUT_Z_L_G, &gyro_Z_L);
  get_i2c_register(i2c_file, LSM9DS0_ADDRESS_1_GYRO_READ >> 1, LSM9DS0_REGISTER_OUT_Z_H_G, &gyro_Z_H);
  gyro_X = gyro_X_L + (gyro_X_H << 8);
  gyro_Y = gyro_Y_L + (gyro_Y_H << 8);
  gyro_Z = gyro_Z_L + (gyro_Z_H << 8);
  coords xyz = {GYRO, gyro_X, gyro_Y, gyro_Z};
  return xyz;
}

coords getAccel(int i2c_file)
{
  unsigned char accel_X_L,accel_X_H,accel_Y_L,accel_Y_H,accel_Z_L,accel_Z_H;
  short accel_X,accel_Y,accel_Z;
  get_i2c_register(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_READ >> 1, LSM9DS0_REGISTER_OUT_X_L_A, &accel_X_L);
  get_i2c_register(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_READ >> 1, LSM9DS0_REGISTER_OUT_X_H_A, &accel_X_H);
  get_i2c_register(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_READ >> 1, LSM9DS0_REGISTER_OUT_Y_L_A, &accel_Y_L);
  get_i2c_register(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_READ >> 1, LSM9DS0_REGISTER_OUT_Y_H_A, &accel_Y_H);
  get_i2c_register(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_READ >> 1, LSM9DS0_REGISTER_OUT_Z_L_A, &accel_Z_L);
  get_i2c_register(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_READ >> 1, LSM9DS0_REGISTER_OUT_Z_H_A, &accel_Z_H);
  accel_X = accel_X_L + (accel_X_H << 8);
  accel_Y = accel_Y_L + (accel_Y_H << 8);
  accel_Z = accel_Z_L + (accel_Z_H << 8);
  coords xyz = {ACCEL, accel_X, accel_Y, accel_Z};
  return xyz;
}

short getPressure(int i2c_file)
{
  short ADC_value;	
  get_i2c_register_16(i2c_file, ADS1015_ADDRESS, ADS1015_REG_POINTER_CONVERT, &ADC_value);
  //printf("ADC Value: %d\n", ADC_value >> 4);
  return ADC_value;
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
  char *sensor = (c.type == ACCEL) ? "ACC" : (c.type == MAG) ? "MAG" : "GYR";
  printf("%s %d %d %d \n", sensor, c.X, c.Y, c.Z);
}


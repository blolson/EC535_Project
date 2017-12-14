/*
 This software uses a BSD license.

Copyright (c) 2010, Sean Cross / chumby industries
All rights reserved.
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 * Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the
   distribution.  
 * Neither the name of Sean Cross / chumby industries nor the names
   of its contributors may be used to endorse or promote products
   derived from this software without specific prior written
   permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
 WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
 
 */

#include <stdio.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string.h>
#include "MahonyAHRS.h"
#include <signal.h>
#include <ctype.h>
#include <sys/time.h>
#include <math.h>

//=============BLADE: THIS IS FOR THE PRESSURE SENSOR ADC==================
//https://github.com/adafruit/Adafruit_ADS1X15
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
//https://github.com/adafruit/Adafruit_LSM9DS0_Library/blob/master/Adafruit_LSM9DS0.cpp
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



#define I2C_FILE_NAME "/dev/i2c-0"
#define USAGE_MESSAGE \
    "Usage:\n" \
    "  %s r [addr] [register]   " \
        "to read value from [register]\n" \
    "  %s w [addr] [register] [value]   " \
        "to write a value [value] to register [register]\n" \
    ""

int i2c_file;

void sighandler(int);
void setupGyro(int, int);
void setupAccelMag(int, int);
void setupADC_Comparator(int, int, short);
void readADC(int, int);
void printADC(int, int);
void printAccel(int, int);
void printGyro(int, int);
void printMag(int, int);
void getAccelMag(int, int);
short getAccelX(int, int);
short getAccelY(int, int);
short getAccelZ(int, int);
short getGyroX(int, int);
short getGyroY(int, int);
short getGyroZ(int, int);
short getMagX(int, int);
short getMagY(int, int);
short getMagZ(int, int);
void calibrateMag(float *);
long long current_timestamp();

//Write a 1-byte register over i2c
static int set_i2c_register(int file,
                            unsigned char addr,
                            unsigned char reg,
                            unsigned char value) {

    unsigned char outbuf[2];
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[1];

    messages[0].addr  = addr;
    messages[0].flags = 0;
    messages[0].len   = sizeof(outbuf);
    messages[0].buf   = outbuf;

    /* The first byte indicates which register we'll write */
    outbuf[0] = reg;

    /* 
     * The second byte indicates the value to write.  Note that for many
     * devices, we can write multiple, sequential registers at once by
     * simply making outbuf bigger.
     */
    outbuf[1] = value;

    /* Transfer the i2c packets to the kernel and verify it worked */
    packets.msgs  = messages;
    packets.nmsgs = 1;
    if(ioctl(file, I2C_RDWR, &packets) < 0) {
        perror("Unable to send data");
        return 1;
    }

    return 0;
}

//Write a 2-byte register over i2c
static int set_i2c_register_16(int file,
                            unsigned char addr,
                            unsigned char reg,
                            short value) {

    unsigned char outbuf[3];
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[1];

    messages[0].addr  = addr;
    messages[0].flags = 0;
    messages[0].len   = sizeof(outbuf);
    messages[0].buf   = outbuf;

    /* The first byte indicates which register we'll write */
    outbuf[0] = reg;

    /* 
     * The second byte indicates the value to write.  Note that for many
     * devices, we can write multiple, sequential registers at once by
     * simply making outbuf bigger.
     */
    outbuf[1] = value >> 8;
    outbuf[2] = value;

    /* Transfer the i2c packets to the kernel and verify it worked */
    packets.msgs  = messages;
    packets.nmsgs = 1;
    if(ioctl(file, I2C_RDWR, &packets) < 0) {
        perror("Unable to send data");
        return 1;
    }

    return 0;
}

//Read a 1-byte register over i2c
static int get_i2c_register(int file,
                            unsigned char addr,
                            unsigned char reg,
                            unsigned char *val) {
    unsigned char inbuf, outbuf;
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];

    /*
     * In order to read a register, we first do a "dummy write" by writing
     * 0 bytes to the register we want to read from.  This is similar to
     * the packet in set_i2c_register, except it's 1 byte rather than 2.
     */
    outbuf = reg;
    messages[0].addr  = addr;
    messages[0].flags = 0;
    messages[0].len   = sizeof(outbuf);
    messages[0].buf   = &outbuf;

    /* The data will get returned in this structure */
    messages[1].addr  = addr;
    messages[1].flags = I2C_M_RD/* | I2C_M_NOSTART*/;
    messages[1].len   = sizeof(inbuf);
    messages[1].buf   = &inbuf;

    /* Send the request to the kernel and get the result back */
    packets.msgs      = messages;
    packets.nmsgs     = 2;
    if(ioctl(file, I2C_RDWR, &packets) < 0) {
        perror("Unable to send data");
        return 1;
    }
    *val = inbuf;

    return 0;
}

//Read a 2-byte register over i2c
static int get_i2c_register_16(int file,
                            unsigned char addr,
                            unsigned char reg,
                            short *val) {
    unsigned char outbuf;
    short inbuf;
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];

    /*
     * In order to read a register, we first do a "dummy write" by writing
     * 0 bytes to the register we want to read from.  This is similar to
     * the packet in set_i2c_register, except it's 1 byte rather than 2.
     */

    //printf("Size of inbuf %d\n", sizeof(inbuf));

    outbuf = reg;
    messages[0].addr  = addr;
    messages[0].flags = 0;
    messages[0].len   = sizeof(outbuf);
    messages[0].buf   = &outbuf;

    /* The data will get returned in this structure */
    messages[1].addr  = addr;
    messages[1].flags = I2C_M_RD/* | I2C_M_NOSTART*/;
    messages[1].len   = sizeof(inbuf);
    messages[1].buf   = &inbuf;

    /* Send the request to the kernel and get the result back */
    packets.msgs      = messages;
    packets.nmsgs     = 2;
    if(ioctl(file, I2C_RDWR, &packets) < 0) {
        perror("Unable to send data");
        return 1;
    }
    //printf("testing the inbuf: %x\n", inbuf);
    *val = ((inbuf & 0xff) << 8) + ((inbuf >> 8) & 0xff);

    return 0;
}


int main(int argc, char **argv) {
    // Open a connection to the I2C userspace control file.
    if ((i2c_file = open(I2C_FILE_NAME, O_RDWR)) < 0) {
        perror("Unable to open i2c control file");
        exit(1);
    }
   
    //Test read a register
    if(argc > 3 && !strcmp(argv[1], "r")) {
        int addr = strtol(argv[2], NULL, 0);
        int reg = strtol(argv[3], NULL, 0);
	int size = strtol(argv[4], NULL, 0);

	//test read 2 bytes
	if(size == 2)
	{
		short value = 0;
        	if(get_i2c_register_16(i2c_file, addr, reg, &value)) {
            		printf("Unable to get register %x at address %x!\n", reg, addr);
        	}
        	else {
            		printf("Register %d: (%x)\n", reg, (short)value);
        	}
	}
	else
	{
	//test read 1 byte
		unsigned char value;
		if(get_i2c_register(i2c_file, addr, reg, &value)) {
            		printf("Unable to get register %x at address %x!\n", reg, addr);
        	}
        	else {
            		printf("Register %d: %d (%x)\n", reg, (int)value, (int)value);
        	}
	}

    }
    else if(argc > 4 && !strcmp(argv[1], "w")) {
        int addr = strtol(argv[2], NULL, 0);
        int reg = strtol(argv[3], NULL, 0);
        int value = strtol(argv[4], NULL, 0);
	int size = strtol(argv[4], NULL, 0);

	//test write 2 bytes
	if(size == 2)
	        if(set_i2c_register_16(i2c_file, addr, reg, value)) {
        	    printf("Unable to get register!\n");
        	}
       		else {
            		printf("Set register %x: %d (%x)\n", reg, value, value);
       		}
	else
	{
	//test write 1 byte
	        if(set_i2c_register(i2c_file, addr, reg, value)) {
        	    printf("Unable to get register!\n");
        	}
       		else {
            		printf("Set register %x: %d (%x)\n", reg, value, value);
       		}
	}
    }
    else if(argc > 1 && !strcmp(argv[1], "accel")) {
	//read accel values over i2c
	setupAccelMag(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_WRITE);
	while(1)	
	{
	        //printAccel(i2c_file, LSM9DS0_ADDRESS_2_ACCELMAG_READ);
	        printAccel(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_READ);
	}  	
    }
    else if(argc > 1 && !strcmp(argv[1], "accel_mag")) {
	//read accel values over i2c and perform magnitude math
	setupAccelMag(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_WRITE);
	while(1)	
	{
	        getAccelMag(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_READ);
	}  	
    }
    else if(argc > 1 && !strcmp(argv[1], "gyro")) {
	//read gyro over i2c
	setupGyro(i2c_file, LSM9DS0_ADDRESS_1_GYRO_WRITE);
	while(1)	
	{
	        printGyro(i2c_file, LSM9DS0_ADDRESS_1_GYRO_READ);
	}  	
    }
    else if(argc > 1 && !strcmp(argv[1], "mag")) {
	//read magnetometer over i2c
	setupAccelMag(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_WRITE);
	while(1)	
	{
	        printMag(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_READ);
	}  	
    }
    else if(argc > 1 && !strcmp(argv[1], "ADC")) {
        //get values in a continuous stream off the ADC
	setupADC_Comparator(i2c_file, 0, 1000);
	//readADC(i2c_file, 0);

	int pFile, oflags;
        struct sigaction action;
	
	// Opens to device file
	pFile = open("/dev/ADC", O_RDWR);

        if (pFile < 0) {
                printf ("ADC module isn't loaded\n");
		//return 1;
        }

	// Setup signal handler
        memset(&action, 0, sizeof(action));
        action.sa_handler = sighandler;
        action.sa_flags = SA_SIGINFO;
        sigemptyset(&action.sa_mask);
        sigaction(SIGIO, &action, NULL);
        fcntl(pFile, F_SETOWN, getpid());
        oflags = fcntl(pFile, F_GETFL);
        fcntl(pFile, F_SETFL, oflags | FASYNC);

	short ADC_value;	
	while(1)
	{
		// Read the conversion results
		// Shift 12-bit results right 4 bits for the ADS1015
		get_i2c_register_16(i2c_file, ADS1015_ADDRESS, ADS1015_REG_POINTER_CONVERT, &ADC_value);
		printf("ADC Value: %d\n", ADC_value >> 4);  
	}
    }
    else if(argc > 1 && !strcmp(argv[1], "all")) {
	//get accel, gyro, and mag from IMU over i2c
	setupAccelMag(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_WRITE);
	setupGyro(i2c_file, LSM9DS0_ADDRESS_1_GYRO_WRITE);

	while(1)	
	{
		printAccel(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_READ);
		printGyro(i2c_file, LSM9DS0_ADDRESS_1_GYRO_READ);
	        printMag(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_READ);		
	}  	
    }
    else if(argc > 1 && !strcmp(argv[1], "euler")) {
	//calculat euler angles using MahonyAHRS algorithm
        setupAccelMag(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_WRITE);
        setupGyro(i2c_file, LSM9DS0_ADDRESS_1_GYRO_WRITE);
	
	float aX,aY,aZ,gX,gY,gZ,mX,mY,mZ;
	float roll, pitch, yaw;	
	float bias[3];
	
	//printf("--CALIBRATION--\nWave device in a figure 8 for ~5-10 seconds\n");
	//calibrateMag(bias);

        while(1)
        {
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

		//printf("AccelX:\t%.2f\tAccelY:\t%.2f\tAccelZ:\t%.2f\n", aX, aY, aZ);
		//printf("GyroX:\t%.2f\tGyroY:\t%.2f\tGyroZ:\t%.2f\n", gX, gY, gZ);		
		//printf("MagX:\t%.2f\tMagY:\t%.2f\tMagZ:\t%.2f\n", mX, mY, mZ);

		//MahonyAHRSupdateIMU(gX, gY, gZ, aX, aY, aZ);

		
		MahonyAHRSupdate(gX, gY, gZ, aX, aY, aZ, mX, mY, mZ);

		//https://github.com/adafruit/Adafruit_AHRS/blob/master/Adafruit_Simple_AHRS.cpp
		//https://learn.adafruit.com/ahrs-for-adafruits-9-dof-10-dof-breakout?view=all#software
		//https://learn.adafruit.com/ahrs-for-adafruits-9-dof-10-dof-breakout/sensor-fusion-algorithms		

		roll = getRoll();
		pitch = getPitch();
		yaw = getYaw();
		//current_timestamp();
		printf("Roll:\t%.2f\tPitch\t%.2f\tYaw:\t%.2f\n", roll, pitch, yaw);
        }
    }
    else {
        fprintf(stderr, USAGE_MESSAGE, argv[0], argv[0]);
    }


    close(i2c_file);


    return 0;
}

//used when testing Hz of data recording from ADC/IMU
long long current_timestamp() {
    struct timeval te; 
    gettimeofday(&te, NULL); // get current time
    long long milliseconds = te.tv_sec*1000LL + te.tv_usec/1000; // calculate milliseconds
    printf("milliseconds: %lld\n", milliseconds);
    return milliseconds;
}

//this never got used in the end
void sighandler(int signio)
{
	printf("sighandler called\n");
	printADC(i2c_file, ADS1015_ADDRESS);
}


  /* setup commands found in the arduino .cpp library for LSM9DS0
	found here: https://github.com/adafruit/Adafruit_LSM9DS0_Library/blob/master/Adafruit_LSM9DS0.cpp
  write8(XMTYPE, LSM9DS0_REGISTER_CTRL_REG5_XM, 0b11110000);
  // enable mag continuous
  write8(XMTYPE, LSM9DS0_REGISTER_CTRL_REG7_XM, 0b00000000);
  // enable gyro continuous
  write8(GYROTYPE, LSM9DS0_REGISTER_CTRL_REG1_G, 0x0F); // on XYZ
  // enable the temperature sensor (output rate same as the mag sensor)
  uint8_t tempReg = read8(XMTYPE, LSM9DS0_REGISTER_CTRL_REG5_XM);
  write8(XMTYPE, LSM9DS0_REGISTER_CTRL_REG5_XM, tempReg | (1<<7));
  */


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

void getAccelMag(int i2c_file, int address)
{
	unsigned char accel_X_L,accel_X_H,accel_Y_L,accel_Y_H,accel_Z_L,accel_Z_H;
	short accel_X,accel_Y,accel_Z;
	float aX, aY, aZ, aMag;

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

	aMag = sqrt(aX*aX + aY*aY + aZ*aZ);

	printf("AccelMag:\t%.2f\n", aMag);
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

/**************************************************************************/
/*!
    @brief  Sets up the comparator to operate in basic mode, causing the
            ALERT/RDY pin to assert (go from high to low) when the ADC
            value exceeds the specified threshold.
            This will also set the ADC in continuous conversion mode.
*/
/**************************************************************************/
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

void readADC(int i2c_file, int channel)
{  
  // Start with default values
  short config = ADS1015_REG_CONFIG_CQUE_NONE    | // Disable the comparator (default val)
                    ADS1015_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)
                    ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
                    ADS1015_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
                    ADS1015_REG_CONFIG_DR_2400SPS   | // 1600 samples per second (default)
                    ADS1015_REG_CONFIG_MODE_CONTIN;   // Continuous conversion mode

  // Set PGA/voltage range
  config |= ADS1015_REG_CONFIG_PGA_6_144V;

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

  // Set 'start single-conversion' bit
  config |= ADS1015_REG_CONFIG_OS_SINGLE;

  // Write config register to the ADC
  set_i2c_register_16(i2c_file, ADS1015_ADDRESS, ADS1015_REG_POINTER_CONFIG, config);

  short ADC_value;
	
  // Read the conversion results
  // Shift 12-bit results right 4 bits for the ADS1015
  get_i2c_register_16(i2c_file, ADS1015_ADDRESS, ADS1015_REG_POINTER_CONVERT, &ADC_value);
  printf("ADC Value: %d", ADC_value >> 4);  
}

void printADC(int i2c_file, int address)
{
        unsigned char ADC_return;
        short ADC_value;

	// Read the conversion results
	get_i2c_register(i2c_file, address, ADS1015_REG_POINTER_CONVERT, &ADC_return);
	ADC_value = ADC_return >> 4;
    	// Shift 12-bit results right 4 bits for the ADS1015,
    	// making sure we keep the sign bit intact
    	//if (ADC_value > 0x07FF)
    	//{
      		// negative number - extend the sign to 16th bit
      		//ADC_value |= 0xF000;
    	//}
        printf("ADC Value:\t%d\n", ADC_value);
}


void printAccel(int i2c_file, int address)
{
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
	printf("AccelX:\t%.2f\tAccelY:\t%.2f\tAccelZ:\t%.2f\n", aX, aY, aZ);
}

void printGyro(int i2c_file, int address)
{
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
	printf("GyroX:\t%.2f\tGyroY:\t%.2f\tGyroZ:\t%.2f\n", gX, gY, gZ);
}

void printMag(int i2c_file, int address)
{
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

	printf("MagX:\t%.2f\tMagY:\t%.2f\tMagZ:\t%.2f\n", mX, mY, mZ);
}

void calibrateMag(float * bias) 
{
	//https://github.com/kriswiner/MPU9250/blob/master/MPU9250_BMP280_BasicAHRS_t3.ino
	
	short i = 0;
	short j = 0;
	short sample_count = 5000;
	int mag_bias[3] = {0, 0, 0};
	short mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

	printf("Mag Calibration: Wave device in a figure eight until done!");

	// shoot for ~fifteen seconds of mag data
	for(i = 0; i < sample_count; i++)
	{
		mag_temp[0] = getMagX(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_READ);
		mag_temp[1] = getMagX(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_READ);
		mag_temp[2] = getMagX(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_READ);

		for (j = 0; j < 3; j++)
		{
			if(mag_temp[j] > mag_max[j]) mag_max[j] = mag_temp[j];
			if(mag_temp[j] < mag_min[j]) mag_min[j] = mag_temp[j];
		}
	}

	// Get hard iron correction
	mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
	mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
	mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

	// save mag biases for main program
	bias[0] = (float) mag_bias[0]*LSM9DS0_MAG_MGAUSS_4GAUSS;
	bias[0] /= 1000;  
	bias[1] = (float) mag_bias[1]*LSM9DS0_MAG_MGAUSS_4GAUSS;
	bias[1] /= 1000;  
	bias[2] = (float) mag_bias[2]*LSM9DS0_MAG_MGAUSS_4GAUSS;
	bias[2] /= 1000;  

	printf("Mag Calibration done!");
}

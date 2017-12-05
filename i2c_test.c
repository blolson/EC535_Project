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
#define LSM9DS0_ADDRESS_1_GYRO_READ               0xd7 //(11010111) - Read
#define LSM9DS0_ADDRESS_1_GYRO_WRITE              0xd6 //(11010110) - Write

//Linear acceleration and magnetic sensor SAD+read/write patterns
#define LSM9DS0_ADDRESS_1_ACCELMAG_READ           0x3b //(00111011) - Read
#define LSM9DS0_ADDRESS_1_ACCELMAG_WRITE          0x3a //(00111010) - Write


/*---------=IMU #2=-----------*/
//Angular rate SAD+read/write patterns
#define LSM9DS0_ADDRESS_2_GYRO_READ               0xd5 //(11010111) - Read
#define LSM9DS0_ADDRESS_2_GYRO_WRITE              0xd4 //(11010110) - Write

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

void setupGyro(int, int);
void setupAccelMag(int, int);
void printAccel(int, int);
void printGyro(int, int);
void printMag(int, int);

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


int main(int argc, char **argv) {
    int i2c_file;

    // Open a connection to the I2C userspace control file.
    if ((i2c_file = open(I2C_FILE_NAME, O_RDWR)) < 0) {
        perror("Unable to open i2c control file");
        exit(1);
    }
   
    if(argc > 3 && !strcmp(argv[1], "r")) {
        int addr = strtol(argv[2], NULL, 0);
        int reg = strtol(argv[3], NULL, 0);
        unsigned char value;

        if(get_i2c_register(i2c_file, addr, reg, &value)) {
            printf("Unable to get register %x at address %x!\n", reg, addr);
        }
        else {
            printf("Register %d: %d (%x)\n", reg, (int)value, (int)value);
        }
    }
    else if(argc > 4 && !strcmp(argv[1], "w")) {
        int addr = strtol(argv[2], NULL, 0);
        int reg = strtol(argv[3], NULL, 0);
        int value = strtol(argv[4], NULL, 0);
        if(set_i2c_register(i2c_file, addr, reg, value)) {
            printf("Unable to get register!\n");
        }
        else {
            printf("Set register %x: %d (%x)\n", reg, value, value);
        }
    }
    else if(argc > 1 && !strcmp(argv[1], "accel")) {
	setupAccelMag(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_WRITE);
	while(1)	
	{
	        //printAccel(i2c_file, LSM9DS0_ADDRESS_2_ACCELMAG_READ);
	        printAccel(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_READ);
	}  	
    }
    else if(argc > 1 && !strcmp(argv[1], "gyro")) {
	setupGyro(i2c_file, LSM9DS0_ADDRESS_1_GYRO_WRITE);
	while(1)	
	{
	        printGyro(i2c_file, LSM9DS0_ADDRESS_1_GYRO_READ);
	}  	
    }
    else if(argc > 1 && !strcmp(argv[1], "mag")) {
	setupAccelMag(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_WRITE);
	while(1)	
	{
	        printMag(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_READ);		
	}  	
    }
    else {
        fprintf(stderr, USAGE_MESSAGE, argv[0], argv[0]);
    }


    close(i2c_file);


    return 0;
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

void printAccel(int i2c_file, int address)
{
	unsigned char accel_X_L,accel_X_H,accel_Y_L,accel_Y_H,accel_Z_L,accel_Z_H;
	int accel_X,accel_Y,accel_Z;
	get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_X_L_A, &accel_X_L);
	get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_X_H_A, &accel_X_H);
	get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_Y_L_A, &accel_Y_L);
	get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_Y_H_A, &accel_Y_H);
	get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_Z_L_A, &accel_Z_L);
	get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_Z_H_A, &accel_Z_H);
	accel_X = accel_X_L + (accel_X_H << 8);
	accel_Y = accel_Y_L + (accel_Y_H << 8);
	accel_Z = accel_Z_L + (accel_Z_H << 8);
	printf("AccelX:\t%d\tAccelY:\t%d\tAccelZ:\t%d\n", accel_X, accel_Y, accel_Z);
}

void printGyro(int i2c_file, int address)
{
	unsigned char gyro_X_L,gyro_X_H,gyro_Y_L,gyro_Y_H,gyro_Z_L,gyro_Z_H;
	int gyro_X,gyro_Y,gyro_Z;
	get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_X_L_G, &gyro_X_L);
	get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_X_H_G, &gyro_X_H);
	get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_Y_L_G, &gyro_Y_L);
	get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_Y_H_G, &gyro_Y_H);
	get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_Z_L_G, &gyro_Z_L);
	get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_Z_H_G, &gyro_Z_H);
	gyro_X = gyro_X_L + (gyro_X_H << 8);
	gyro_Y = gyro_Y_L + (gyro_Y_H << 8);
	gyro_Z = gyro_Z_L + (gyro_Z_H << 8);
	printf("GyroX:\t%d\tGyroY:\t%d\tGyroZ:\t%d\n", gyro_X, gyro_Y, gyro_Z);
}

void printMag(int i2c_file, int address)
{
	unsigned char gyro_X_L,gyro_X_H,gyro_Y_L,gyro_Y_H,gyro_Z_L,gyro_Z_H;
	int gyro_X,gyro_Y,gyro_Z;
	get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_X_L_M, &gyro_X_L);
	get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_X_H_M, &gyro_X_H);
	get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_Y_L_M, &gyro_Y_L);
	get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_Y_H_M, &gyro_Y_H);
	get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_Z_L_M, &gyro_Z_L);
	get_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_OUT_Z_H_M, &gyro_Z_H);
	gyro_X = gyro_X_L + (gyro_X_H << 8);
	gyro_Y = gyro_Y_L + (gyro_Y_H << 8);
	gyro_Z = gyro_Z_L + (gyro_Z_H << 8);
	printf("MagX:\t%d\tMagY:\t%d\tMagZ:\t%d\n", gyro_X, gyro_Y, gyro_Z);
}

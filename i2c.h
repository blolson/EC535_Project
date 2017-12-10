#ifndef I2C_H
#define I2C_H

int init_i2c_file();
void set_i2c_register(int file, unsigned char addr, unsigned char reg, unsigned char value);
void get_i2c_register(int file, unsigned char addr, unsigned char reg, unsigned char *value);
int get_i2c_register_16(int file, unsigned char addr, unsigned char reg, short *val);
int set_i2c_register_16(int file, unsigned char addr, unsigned char reg, short value);
#endif


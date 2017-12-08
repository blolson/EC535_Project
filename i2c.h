#ifndef I2C_H
#define I2C_H

int init_i2c_file();
void set_i2c_register(int file, unsigned char addr, unsigned char reg, unsigned char value);
void get_i2c_register(int file, unsigned char addr, unsigned char reg, unsigned char *value);

#endif


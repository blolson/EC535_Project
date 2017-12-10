#ifndef SENSORS_H
#define SENSORS_H

typedef enum  {MAG, GYRO, ACCEL} sensor_type;

typedef struct {
  sensor_type type;
  short X;
  short Y;
  short Z;
} coords;


coords getMag(int i2c_file);
coords getGyro(int i2c_file);
coords getAccel(int i2c_file);
short getPressure(int i2c_file);
void setup_all_sensors(int i2c_file);
void print(coords c);
#endif


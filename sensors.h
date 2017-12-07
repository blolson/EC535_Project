#ifndef SENSORS_H
#define SENSORS_H

typedef enum  {MAG, GYRO, ACCEL} sensor_type;

typedef struct {
  sensor_type type;
  int X;
  int Y;
  int Z;
} coords;


coords getMag(int i2c_file);
coords getGyro(int i2c_file);
coords getAccel(int i2c_file);
void setup_all_sensors(int i2c_file);
void print(coords c);
#endif


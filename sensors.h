#ifndef SENSORS_H
#define SENSORS_H

typedef enum  {MAG, GYRO, ACCEL, EULER} sensor_type;

typedef struct {
  sensor_type type;
  float X;
  float Y;
  float Z;
} coords;


coords getMag(int i2c_file);
coords getGyro(int i2c_file);
coords getAccel(int i2c_file);
short getPressure(int i2c_file);
void getRollPitchYaw(int i2c_file, float *roll, float *pitch, float *yaw);
void setup_all_sensors(int i2c_file);
void print(coords c);
#endif


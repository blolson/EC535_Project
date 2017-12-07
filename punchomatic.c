#include <stdio.h>
#include "i2c.h"
#include "sensors.h"

int main()
{
  int i2c_file = init_i2c_file();
  setup_all_sensors(i2c_file);
  while (1) {
    coords accel = getAccel(i2c_file);
    print(accel);
  }
}

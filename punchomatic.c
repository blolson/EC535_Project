#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdbool.h>
#include "i2c.h"
#include "sensors.h"
#include "ring_buffer.h"

#define PRESSURE_THRESHOLD 200
#define PRESSURE_SAMPLES 20

void sighandler(int);
void collectData();

ring_buffer acc_queue;
ring_buffer mag_queue;
ring_buffer gyr_queue;
int i2c_file;
bool punchInProgress;
short maxPunchPressure;
int pressureSampleCounter;

int main(int argc, char **argv)
{
  i2c_file = init_i2c_file();
  setup_all_sensors(i2c_file);

  //  char line[256];
  //int ii, count = 0;
	
  FILE * ledFile;
  ledFile = fopen("/dev/led", "r+");
  if (ledFile==NULL) {
    fputs("led module isn't loaded\n",stderr);
    return -1;
  }

  fputs("1 0 1 0", ledFile);

  //do proc file for leds
  
  collectData();
}

void collectData()
{
  coords acc_data[RING_BUFFER_SIZE];
  coords mag_data[RING_BUFFER_SIZE];
  coords gyr_data[RING_BUFFER_SIZE];
  maxPunchPressure = 0;
  pressureSampleCounter = 0;
  while (1) {
    coords accel = getAccel(i2c_file);
    coords mag = getMag(i2c_file);
    coords gyro = getGyro(i2c_file);
    short pressure = getPressure(i2c_file);
    put(&acc_queue, accel);
    put(&mag_queue, mag);
    put(&gyr_queue, gyro);
    
    if (!punchInProgress & pressure > PRESSURE_THRESHOLD) {
      get_all(&acc_queue, acc_data);
      get_all(&mag_queue, mag_data);
      get_all(&gyr_queue, gyr_data);
      maxPunchPressure = pressure;
      punchInProgress = true;
      pressureSampleCounter++;
    } else if (punchInProgress) {
      maxPunchPressure = (pressure > maxPunchPressure) ? pressure : maxPunchPressure;
      printf("%d \n", pressure);
      pressureSampleCounter++;
    }
    
    if (punchInProgress && pressureSampleCounter >= PRESSURE_SAMPLES) {
      punchInProgress = false;
      pressureSampleCounter = 0;
        for (int i = 0; i < RING_BUFFER_SIZE; i++) {
	  // if (i == 0 || i == RING_BUFFER_SIZE - 1)
	  print(acc_data[i]);
	}

      //Do analysis here and light up LEDs
      
    }

    
  }  
}
  


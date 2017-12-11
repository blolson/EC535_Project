#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdbool.h>
#include "i2c_punchomatic.h"
#include "sensors.h"
#include "ring_buffer.h"

#define PRESSURE_THRESHOLD 750
#define PRESSURE_SAMPLES 20

void sighandler(int);
void collectData();

ring_buffer acc_queue;
ring_buffer mag_queue;
ring_buffer gyr_queue;
ring_buffer eul_queue;
int i2c_file;
bool punchInProgress;
short maxPunchPressure;
int pressureSampleCounter;
int punchCounter;
volatile int ledFile;

int main(int argc, char **argv)
{
  i2c_file = init_i2c_file();
  setup_all_sensors(i2c_file);

  //  char line[256];
  //int ii, count = 0;


     /* FILE * ledFile; */
     /*  ledFile = fopen("/dev/led", "r+"); */
     /*  if (ledFile==NULL) { */
     /* 	fputs("led module isn't loaded\n",stderr); */
     /* 	return -1; */
     /*  } */
  
	  
    /*   //Do analysis here and light up LEDs */
    /*   printf("About to write! \n"); */
    /*   if (punchCounter % 2 == 0) { */
    /* 	fputs("2", ledFile); */
    /*   } else { */
    /* 	fputs("4", ledFile); */
    /*   } */
    /*   printf("Closing \n"); */
    /*   fclose(ledFile);       */
    /* } */


  ledFile = open("/dev/led", O_RDWR);
  if (ledFile < 0) {
    fputs("led module isn't loaded\n",stderr);
    return -1;
  }
  
  coords acc_data[RING_BUFFER_SIZE];
  coords mag_data[RING_BUFFER_SIZE];
  coords gyr_data[RING_BUFFER_SIZE];
  coords eul_data[RING_BUFFER_SIZE];
  maxPunchPressure = 0;
  pressureSampleCounter = 0;
  punchCounter = 0;
  while (1) {
    coords accel = getAccel(i2c_file);
    coords mag = getMag(i2c_file);
    coords gyro = getGyro(i2c_file);
    short pressure = getPressure(i2c_file);
    float roll, pitch, yaw;
    getRollPitchYaw(i2c_file, &roll, &pitch, &yaw);
    put(&acc_queue, accel);
    put(&mag_queue, mag);
    put(&gyr_queue, gyro);
    coords angles = {EULER, roll, pitch, yaw};
    put(&eul_queue, angles);
    //print(accel);
    //printf("Pressure %d \n", pressure);
    
    if (!punchInProgress && pressure > PRESSURE_THRESHOLD) {
      //printf("Detecting punch \n");
      get_all(&acc_queue, acc_data);
      get_all(&mag_queue, mag_data);
      get_all(&gyr_queue, gyr_data);
      get_all(&eul_queue, eul_data);
      //      printf("Loaded data \n");
      maxPunchPressure = pressure;
      punchInProgress = true;
      pressureSampleCounter++;
    } else if (punchInProgress) {
      maxPunchPressure = (pressure > maxPunchPressure) ? pressure : maxPunchPressure;
      //printf("Checking max pressure %d %d \n", pressure, maxPunchPressure);
      pressureSampleCounter++;
    }
    
    if (punchInProgress && pressureSampleCounter >= PRESSURE_SAMPLES) {
      punchInProgress = false;
      pressureSampleCounter = 0;
      printf("Punch with pressure %d \n", maxPunchPressure);
      maxPunchPressure = 0;
      printf("===================PUNCH %d ===================\n", punchCounter++);
      /* for (int i = 0; i < RING_BUFFER_SIZE; i++) print(acc_data[i]); */
      /* for (int i = 0; i < RING_BUFFER_SIZE; i++)  print(eul_data[i]); */
      /* for (int i = 0; i < RING_BUFFER_SIZE; i++)  print(mag_data[i]); */
      /* for (int i = 0; i < RING_BUFFER_SIZE; i++)  print(gyr_data[i]); */

	  
	//Do analysis here and light up LEDs
	//printf("About to write!\n");
	if (punchCounter % 2 == 0)
	{
		write(ledFile, "2,0", 3);
	}
	else 
	{
      		write(ledFile, "1,0", 3);
	}
	//printf("Closing \n");

    }

    
  }
  close(ledFile);      
}
  


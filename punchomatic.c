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
#include <math.h>

#define PRESSURE_THRESHOLD 750
#define PRESSURE_SAMPLES 20

void sighandler(int);
int CalculateHit(coords[RING_BUFFER_SIZE], coords[RING_BUFFER_SIZE]);

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
coords aP_data[RING_BUFFER_SIZE];
coords gP_data[RING_BUFFER_SIZE];
coords aU_data[RING_BUFFER_SIZE];
coords gU_data[RING_BUFFER_SIZE];
coords aH_data[RING_BUFFER_SIZE];
coords gH_data[RING_BUFFER_SIZE];

int main(int argc, char **argv)
{
  i2c_file = init_i2c_file();
  setup_all_sensors(i2c_file);

	bool punchesCalculated = false;
	bool uppercutsCalculated = false;
	bool hooksCalculated = false;


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
get_all(&acc_queue, acc_data);    put(&mag_queue, mag);
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
      for (int i = 0; i < RING_BUFFER_SIZE; i++) print(acc_data[i]);
      for (int i = 0; i < RING_BUFFER_SIZE; i++)  print(eul_data[i]);
      for (int i = 0; i < RING_BUFFER_SIZE; i++)  print(mag_data[i]);
      for (int i = 0; i < RING_BUFFER_SIZE; i++)  print(gyr_data[i]);

	  
	if(!punchesCalculated)
	{
		punchesCalculated = true;
		get_all(&acc_queue, aP_data);
		get_all(&gyr_queue, gP_data);
	}
	else if(!uppercutsCalculated)
	{
		uppercutsCalculated = true;
		get_all(&acc_queue, aU_data);
		get_all(&gyr_queue, gU_data);
	}
	else if(!hooksCalculated)
	{
		hooksCalculated = true;
		get_all(&acc_queue, aH_data);
		get_all(&gyr_queue, gH_data);
	}
	else
	{
		int hit = CalculateHit(acc_data, gyr_data);
		if(hit == 1)
			write(ledFile, "1,0", 3);
		else if(hit == 2)
			write(ledFile, "2,0", 3);
		else
			write(ledFile, "4,0", 3);

		/*
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
		*/

	}	

    }

    
  }
  close(ledFile);      
}
  
int CalculateHit(coords a[100], coords g[100])
{

	short i;
	int e_aP[3] = {0,0,0};
	int e_aU[3] = {0,0,0};
	int e_aH[3] = {0,0,0};
	int te_P, te_U, te_H = 0;
	int bestError = 0;

	//Blade: Yes, I know it's gross to do this three times copy+pasted, and yet here we are...
	for(i = 0; i < RING_BUFFER_SIZE; i++)
	{
		e_aP[0] += abs((int)aP_data[i].X - (int)a[i].X);
	} 

	for(i = 0; i < RING_BUFFER_SIZE; i++)
	{
		e_aU[0] += abs((int)aU_data[i].X - (int)a[i].X);
	} 

	for(i = 0; i < RING_BUFFER_SIZE; i++)
	{
		e_aH[0] += abs((int)aH_data[i].X - (int)a[i].X);
	} 

	te_P = e_aP[0] + e_aP[1] + e_aP[2];
	te_U = e_aU[0] + e_aU[1] + e_aU[2];
	te_H = e_aH[0] + e_aH[1] + e_aH[2];

	bestError = te_P < te_U ? te_P : te_U;	
	bestError = te_H < bestError ? te_H : bestError;

	if(bestError == te_P)
	{
		printf("Punch detected\n");
		return 1;
	}	
	else if(bestError == te_U)
	{
		printf("Uppercut detected\n");	
		return 2;
	}
	else
	{
		printf("Hook detected\n");
		return 3;
	}
}

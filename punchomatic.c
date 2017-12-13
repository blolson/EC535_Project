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

  //Turn off every LED
  write(ledFile, "7,0,0,0,0", 10);

  //write(ledFile, "led,mode,milli,delay,cont", characterCount);
  //led = 1-9
  //mode == 1 - turn off
  //mode == 2 - blink
  //mode == 3 - turn on
  //milli - duration
  //delay - how long to stagger start
  //cont == 0 - once
  //cont == 1 - loop

  //do cool lighting effect
  write(ledFile, "1,1,0300,0000,1", 16);

  //stores snapshot of pre-punch movements for evaluation
  coords acc_data[RING_BUFFER_SIZE];
  coords mag_data[RING_BUFFER_SIZE];
  coords gyr_data[RING_BUFFER_SIZE];
  coords eul_data[RING_BUFFER_SIZE];

  maxPunchPressure = 0;
  pressureSampleCounter = 0;
  punchCounter = 0;

  while (1) {
    //Constantly store sensor data in ring buffer
    //to represent moment before punch
    coords accel = getAccel(i2c_file);
    coords mag = getMag(i2c_file);
    coords gyro = getGyro(i2c_file);
    short pressure = getPressure(i2c_file);
    float roll, pitch, yaw;
    getRollPitchYaw(i2c_file, &roll, &pitch, &yaw);
    put(&acc_queue, accel);
    get_all(&acc_queue, acc_data);
    put(&mag_queue, mag);
    put(&gyr_queue, gyro);
    coords angles = {EULER, roll, pitch, yaw};
    put(&eul_queue, angles);

    //Checking whether to begin sampling pressure sensor
    //to determine hardest impact in this punch
    if (!punchInProgress && pressure > PRESSURE_THRESHOLD) {
      //Make the snapshot of the motion leading up to the punch
      get_all(&acc_queue, acc_data);
      get_all(&mag_queue, mag_data);
      get_all(&gyr_queue, gyr_data);
      get_all(&eul_queue, eul_data);

      maxPunchPressure = pressure;
      punchInProgress = true;

      //Flash all the lights because we got a punch
      write(ledFile, "7,0,0,0,0", 10);
      write(ledFile, "1,1,0100,0000,1", 16);
      write(ledFile, "2,1,0100,0000,1", 16);
      write(ledFile, "4,1,0100,0000,1", 16);

      pressureSampleCounter++;
    } else if (punchInProgress) {
      //Now evaluating the samples to determine the max pressure for this particular punch
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
      //for (int i = 0; i < RING_BUFFER_SIZE; i++) print(acc_data[i]);
      //for (int i = 0; i < RING_BUFFER_SIZE; i++)  print(eul_data[i]);
      //for (int i = 0; i < RING_BUFFER_SIZE; i++)  print(mag_data[i]);
      //for (int i = 0; i < RING_BUFFER_SIZE; i++)  print(gyr_data[i]);

	//write(ledFile, "led,mode,milli,delay,cont", characterCount);
	  
	if(!punchesCalculated)
	{
		punchesCalculated = true;
		get_all(&acc_queue, aP_data);
		get_all(&gyr_queue, gP_data);

		write(ledFile, "7,0,0,0,0", 10);		
		write(ledFile, "1,1,3000,0,0", 13);
		write(ledFile, "2,1,0300,2000,1", 16);
	}
	else if(!uppercutsCalculated)
	{
		uppercutsCalculated = true;
		get_all(&acc_queue, aU_data);
		get_all(&gyr_queue, gU_data);

		write(ledFile, "7,0,0,0,0", 10);		
		write(ledFile, "2,1,3000,0,0", 13);
		write(ledFile, "4,1,0300,2000,1", 16);
	}
	else if(!hooksCalculated)
	{
		hooksCalculated = true;
		get_all(&acc_queue, aH_data);
		get_all(&gyr_queue, gH_data);

		write(ledFile, "7,0,0,0,0", 10);		
		write(ledFile, "4,2,0,0,0", 10);

		write(ledFile, "1,1,0300,2000,1", 16);
		write(ledFile, "2,1,0300,2100,1", 16);
		write(ledFile, "4,1,0300,2200,1", 16);
	}
	else
	{
		int hit = CalculateHit(acc_data, gyr_data);
		if(hit == 1)
		{
			write(ledFile, "7,0,0,0,0", 10);		
			write(ledFile, "1,2,0,0,0", 10);
		}
		else if(hit == 2)
		{
			write(ledFile, "7,0,0,0,0", 10);		
			write(ledFile, "2,2,0,0,0", 10);
		}
		else
		{
			write(ledFile, "7,0,0,0,0", 10);		
			write(ledFile, "4,2,0,0,0", 10);
		}

		write(ledFile, "1,1,0300,2000,1", 16);
		write(ledFile, "2,1,0300,2100,1", 16);
		write(ledFile, "4,1,0300,2200,1", 16);

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

// classifies the new punch against the saved punches
int CalculateHit(coords a[100], coords g[100])
{

	short i;
	int e_aP[3] = {0,0,0};
	int e_aU[3] = {0,0,0};
	int e_aH[3] = {0,0,0};
	int te_P, te_U, te_H = 0;
	int bestError = 0;

	// Go through the saved punches to determine which has the
	// least distance between the new punches
	// Complexity is linear.
	for(i = 0; i < RING_BUFFER_SIZE; i++)
	{
		e_aP[0] += abs((int)aP_data[i].X - (int)a[i].X);
		e_aP[1] += abs((int)aP_data[i].Y - (int)a[i].Y);
		e_aP[2] += abs((int)aP_data[i].Z - (int)a[i].Z);
	} 

	for(i = 0; i < RING_BUFFER_SIZE; i++)
	{
		e_aU[0] += abs((int)aU_data[i].X - (int)a[i].X);
		e_aU[1] += abs((int)aU_data[i].Y - (int)a[i].Y);
		e_aU[2] += abs((int)aU_data[i].Z - (int)a[i].Z);
	} 

	for(i = 0; i < RING_BUFFER_SIZE; i++)
	{
		e_aH[0] += abs((int)aH_data[i].X - (int)a[i].X);
		e_aH[1] += abs((int)aH_data[i].Y - (int)a[i].Y);
		e_aH[2] += abs((int)aH_data[i].Z - (int)a[i].Z);
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

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "ring_buffer.h"

int main() {
  ring_buffer queue;
  coords data[RING_BUFFER_SIZE];
  srand(time(NULL));

  for (int i = 0; i < 2000; i++) {
    coords mag = {MAG, rand(), rand(), rand()};
    if ((i == 1999) || (i % (2000 - RING_BUFFER_SIZE) == 0))
      print(mag);
    put(&queue, mag);
  }
  printf("========================= \n");
  get_all(&queue, data);

  for (int i = 0; i < RING_BUFFER_SIZE; i++) {
    if (i == 0 || i == RING_BUFFER_SIZE - 1)
      print(data[i]);
  }
  
}

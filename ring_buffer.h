#ifndef RINGBUF_H
#define RINGBUF_H

#include <stdbool.h>
#include "sensors.h"

#define RING_BUFFER_SIZE 50

typedef struct {
  coords data[RING_BUFFER_SIZE];
  int in_index;
  //int out_index;
  bool wrapped;
  float f;
} ring_buffer;

//void init_ring_buffer(ring_buffer queue);
//coords pop(ring_buffer *queue);
bool get_all(ring_buffer *queue, coords *output);
void put(ring_buffer *queue, coords c);
void put_float(ring_buffer *queue, float f);
bool get_all_floats(ring_buffer *queue, float *floats);

#endif

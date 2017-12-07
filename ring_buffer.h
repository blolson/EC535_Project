#ifndef RINGBUF_H
#define RINGBUF_H

#define RING_BUFFER_SIZE

typedef struct {
  coords data[RING_BUFFER_SIZE];
  int in_index;
  int out_index;
} ring_buffer;

void init_ring_buffer(ring_buffer queue);
coords pop();
void insert(coords c);

#endif

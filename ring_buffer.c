#include "ring_buffer.h"

void init_ring_buffer(ring_buffer *queue) {
  queue->in_index = 0;
  queue->out_index = 0;
}

coords pop(*ring_buffer queue) {
  if (queue->in_index != queue->out_index) {
    if (queue->out_index == RING_BUFFER_SIZE-1) queue->out_index = 0;
    else queue->out_index++;
  } 
  
}
void insert(*ring_buffer queue, coords c);

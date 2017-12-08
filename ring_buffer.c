#include "ring_buffer.h"

//Is any of this necessary?
void init_ring_buffer(ring_buffer *queue) {
  queue->in_index = 0;
  queue->wrapped = false;
  //Need to initialize array?
}

//coords pop(*ring_buffer queue) {}

void put(ring_buffer *queue, coords c) {
  if (queue->in_index == RING_BUFFER_SIZE) {
    queue->wrapped = true;
    queue->in_index = 0;
  }
  queue->data[queue->in_index++] = c;
}

// Right now only returning if buffer has been filled
// Depending on how quickly data collects, this may
// never be an issue
bool get_all(ring_buffer *queue, coords *output) {
  if (!queue->wrapped) return false;
  int i = 0;
  for (int j = queue->in_index; j < RING_BUFFER_SIZE; j++) {
    output[i++] =  queue->data[j]; //I really hope this works!
  }
  if (queue->in_index == 0) return true; //just for efficiency
  for (int j = 0; j < RING_BUFFER_SIZE; j++) {
    output[i++] = queue->data[j];
  }

  return true;
}

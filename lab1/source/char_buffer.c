#include "char_buffer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int char_buffer_init(char_buffer *cb, int initSize) {
  if (cb == NULL) return -1;

  cb->buffer = (unsigned char *)malloc(initSize * sizeof(unsigned char));
  if (cb->buffer == NULL) return -1;
  cb->size = 0;
  cb->capacity = initSize * sizeof(unsigned char);
  return 0;
}

int char_buffer_push(char_buffer *cb, char bt) {
  if (cb == NULL || cb->buffer == NULL) return -1;

  // Grow if at capacity
  if (cb->size * sizeof(unsigned char) >= cb->capacity) {
    cb->buffer = (unsigned char *)realloc(cb->buffer, cb->capacity * 2);
    if (cb->buffer == NULL) return -1;
    cb->capacity *= 2;
  }

  cb->buffer[cb->size] = bt;
  ++cb->size;

  return 0;
}

int char_buffer_remove(char_buffer *cb, unsigned int pos) {
  if (cb == NULL || cb->buffer == NULL || pos > cb->size + 1) return -1;

  // Pop last element
  if (pos == cb->size + 1) {
    cb->buffer[pos] = (unsigned char)0x00;
    --cb->size;
    return 0;
  }

  memmove(cb->buffer + pos * sizeof(unsigned char),
          cb->buffer + (pos + 1) * sizeof(unsigned char),
          cb->capacity - (pos + 1) * sizeof(unsigned char));

  --cb->size;
  return 0;
}

void char_buffer_print(char_buffer *cb) {
  if (cb == NULL || cb->buffer == NULL) return;
  printf("Buffer Content: ");
  for (unsigned int i = 0; i < cb->size; ++i) printf("%c ", cb->buffer[i]);
  printf(" Elements: %d Capacity: %d", cb->size, cb->capacity);
  printf("\n\n");
}

void char_buffer_printHex(char_buffer *cb) {
  if (cb == NULL || cb->buffer == NULL) return;
  printf("Buffer Content: ");
  for (unsigned int i = 0; i < cb->size; ++i) printf("%x ", cb->buffer[i]);
  printf(" Elements: %d Capacity: %d", cb->size, cb->capacity);
  printf("\n\n");
}

void char_buffer_destroy(char_buffer *cb) {
  if (cb == NULL || cb->buffer == NULL) return;
  free(cb->buffer);
}

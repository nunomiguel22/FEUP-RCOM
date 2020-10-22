#include "CharBuffer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int CharBuffer_init(CharBuffer *cb, int initSize) {
  if (cb == NULL) return -1;

  cb->buffer = (char *)malloc(initSize * sizeof(char));
  if (cb->buffer == NULL) return -1;
  cb->size = 0;
  cb->capacity = initSize * sizeof(char);
  return 0;
}

int CharBuffer_push(CharBuffer *cb, char bt) {
  if (cb == NULL || cb->buffer == NULL) return -1;

  // Grow if at capacity
  if (cb->size * sizeof(char) >= cb->capacity) {
    cb->buffer = (char *)realloc(cb->buffer, cb->capacity * 2);
    if (cb->buffer == NULL) return -1;
    cb->capacity *= 2;
  }

  cb->buffer[cb->size] = bt;
  ++cb->size;

  return 0;
}

int CharBuffer_remove(CharBuffer *cb, unsigned int pos) {
  if (cb == NULL || cb->buffer == NULL || pos > cb->size + 1) return -1;

  // Pop last element
  if (pos == cb->size + 1) {
    cb->buffer[pos] = (char)0x00;
    --cb->size;
    return 0;
  }

  memmove(cb->buffer + pos * sizeof(char),
          cb->buffer + (pos + 1) * sizeof(char),
          cb->capacity - (pos + 1) * sizeof(char));

  --cb->size;
  return 0;
}

void CharBuffer_print(CharBuffer *cb) {
  if (cb == NULL || cb->buffer == NULL) return;
  printf("Buffer Content: ");
  for (unsigned int i = 0; i < cb->size; ++i) printf("%c ", cb->buffer[i]);
  printf(" Elements: %d Capacity: %d", cb->size, cb->capacity);
  printf("\n\n");
}

void CharBuffer_printHex(CharBuffer *cb) {
  if (cb == NULL || cb->buffer == NULL) return;
  printf("Buffer Content: ");
  for (unsigned int i = 0; i < cb->size; ++i) printf("%x ", cb->buffer[i]);
  printf(" Elements: %d Capacity: %d", cb->size, cb->capacity);
  printf("\n\n");
}

void CharBuffer_destroy(CharBuffer *cb) {
  if (cb == NULL || cb->buffer == NULL) return;
  free(cb->buffer);
}

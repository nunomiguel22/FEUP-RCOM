#ifndef CHAR_BUFFER_H
#define CHAR_BUFFER_H

typedef struct {
  unsigned char *buffer;
  unsigned int size;
  unsigned int capacity;
} char_buffer;

int char_buffer_init(char_buffer *cb, int initSize);
int char_buffer_push(char_buffer *cb, char bt);
int char_buffer_remove(char_buffer *cb, unsigned int pos);
void char_buffer_print(char_buffer *cb);
void char_buffer_printHex(char_buffer *cb);
void char_buffer_destroy(char_buffer *cb);

#endif

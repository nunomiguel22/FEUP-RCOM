#ifndef _CHARBUFFER_H
#define _CHARBUFFER_H

typedef struct {
  char *buffer;
  unsigned int size;
  unsigned int capacity;
} CharBuffer;

int CharBuffer_init(CharBuffer *cb, int initSize);
int CharBuffer_push(CharBuffer *cb, char bt);
int CharBuffer_remove(CharBuffer *cb, unsigned int pos);
void CharBuffer_print(CharBuffer *cb);
void CharBuffer_printHex(CharBuffer *cb);
void CharBuffer_destroy(CharBuffer *cb);

#endif

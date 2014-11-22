#ifndef __MDFN_SIMPLEFIFO_H
#define __MDFN_SIMPLEFIFO_H

typedef struct
{
 uint32 *data;
 uint32 size;
 uint32 read_pos; // Read position
 uint32 write_pos; // Write position
 uint32 in_count; // Number of units in the FIFO
} SimpleFIFOU32;

typedef struct
{
 uint8  *data;
 uint32 size;
 uint32 read_pos; // Read position
 uint32 write_pos; // Write position
 uint32 in_count; // Number of units in the FIFO
} SimpleFIFOU8;

#define FIFO_CAN_WRITE(fifo) ((fifo)->size - (fifo)->in_count)

#define SimpleFIFO_WriteUnit(fifo, wr_data) \
   (fifo)->data[(fifo)->write_pos] = wr_data; \
   (fifo)->write_pos = ((fifo)->write_pos + 1) & ((fifo)->size - 1); \
   (fifo)->in_count++

#define SimpleFIFO_ReadUnit(fifo) ((fifo)->data[(fifo)->read_pos])

#define SimpleFIFO_Write(fifo, happy_data, happy_count) \
  while(happy_count) \
  { \
   SimpleFIFO_WriteUnit(fifo, *happy_data); \
   happy_data++; \
   happy_count--; \
  }

#define SimpleFIFO_Flush(fifo) \
  (fifo)->read_pos = 0; \
  (fifo)->write_pos = 0; \
  (fifo)->in_count = 0

#define SimpleFIFO_SaveStatePostLoad(fifo) \
    (fifo)->read_pos  %=  (fifo)->size; \
    (fifo)->write_pos %=  (fifo)->size; \
    (fifo)->in_count  %= ((fifo)->size + 1)

#define SimpleFIFO_ReadUnitIncrement(fifo) \
  (fifo)->read_pos = ((fifo)->read_pos + 1) & ((fifo)->size - 1); \
  (fifo)->in_count--

#define SimpleFIFO_New(fifo, fifotype, type, the_size) \
   (fifo) = (fifotype*)calloc(1, sizeof(fifotype)); \
   if (fifo) \
   { \
    (fifo)->data = (type*)malloc(the_size * sizeof(type)); \
    (fifo)->size = the_size; \
    (fifo)->read_pos = 0; \
    (fifo)->write_pos = 0; \
    (fifo)->in_count = 0; \
   }

#define SimpleFIFO_Free(fifo) \
   if (fifo) \
   { \
      if ((fifo)->data) \
         free((fifo)->data); \
      free((fifo)); \
   } \
   (fifo) = NULL;

#endif

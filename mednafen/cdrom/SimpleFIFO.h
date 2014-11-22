#ifndef __MDFN_SIMPLEFIFO_H
#define __MDFN_SIMPLEFIFO_H

#include <vector>
#include <assert.h>

#include "../math_ops.h"

#define FIFO_CAN_WRITE(fifo) ((fifo).size - (fifo).in_count)

#define SimpleFIFO_WriteByte(fifo, wr_data) \
   (fifo).data[(fifo).write_pos] = wr_data; \
   (fifo).write_pos = ((fifo).write_pos + 1) & ((fifo).size - 1)

#define SimpleFIFO_WriteUnit(fifo, wr_data) \
   (fifo).data[(fifo).write_pos] = wr_data; \
   (fifo).write_pos = ((fifo).write_pos + 1) & ((fifo).size - 1); \
   (fifo).in_count++

#define SimpleFIFO_ReadUnitPeek(fifo) ((fifo).data[(fifo).read_pos])

#define SimpleFIFO_Write(fifo, happy_data, happy_count) \
  while(happy_count) \
  { \
   SimpleFIFO_WriteUnit(fifo, *happy_data); \
   happy_data++; \
   happy_count--; \
  }

#define SimpleFIFO_Flush(fifo) \
  (fifo).read_pos = 0; \
  (fifo).write_pos = 0; \
  (fifo).in_count = 0

template<typename T>
class SimpleFIFO
{
 public:

 // Constructor
 SimpleFIFO(uint32 the_size)
 {
    /* Size should be a power of 2! */
    assert(the_size && !(the_size & (the_size - 1)));

    data = (T*)malloc(the_size * sizeof(T));
    size = the_size;
    read_pos = 0;
    write_pos = 0;
    in_count = 0;
 }

 // Destructor
 INLINE ~SimpleFIFO()
 {
    if (data)
       free(data);
 }

 INLINE void SaveStatePostLoad(void)
 {
    read_pos %= size;
    write_pos %= size;
    in_count %= (size + 1);
 }


 INLINE T ReadUnit()
 {
  uint32 cur_read_pos = read_pos;

  assert(in_count > 0);

  read_pos = (read_pos + 1) & (size - 1);
  in_count--;

  return data[cur_read_pos];
 }

 T* data;
 uint32 size;
 uint32 read_pos; // Read position
 uint32 write_pos; // Write position
 uint32 in_count; // Number of units in the FIFO
};


#endif

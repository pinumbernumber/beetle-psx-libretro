#ifndef __MDFN_SIMPLEFIFO_H
#define __MDFN_SIMPLEFIFO_H

#include <vector>
#include <assert.h>

#include "../math_ops.h"

#define FIFO_CAN_WRITE_INTERNAL() (size - in_count)
#define FIFO_CAN_WRITE(fifo) ((fifo).size - (fifo).in_count)

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

 INLINE T ReadUnitPeek()
 {
  assert(in_count > 0);
  return data[read_pos];
 }

 INLINE T ReadUnit()
 {
  uint32 cur_read_pos = read_pos;

  assert(in_count > 0);

  read_pos = (read_pos + 1) & (size - 1);
  in_count--;

  return data[cur_read_pos];
 }

 INLINE void Write(const T *happy_data, uint32 happy_count)
 {
  assert(FIFO_CAN_WRITE_INTERNAL() >= happy_count);

  while(happy_count)
  {
   data[write_pos] = *happy_data;

   write_pos = (write_pos + 1) & (size - 1);
   in_count++;
   happy_data++;
   happy_count--;
  }
 }

 INLINE void WriteUnit(const T wr_data)
 {
   assert(FIFO_CAN_WRITE_INTERNAL() >= 1);
   data[write_pos] = wr_data;
   write_pos = (write_pos + 1) & (size - 1);
   in_count++;
 }

 INLINE void WriteByte(const T wr_data)
 {
   assert(sizeof(T) == 1);
   assert(FIFO_CAN_WRITE_INTERNAL() >= 1);
   data[write_pos] = wr_data;
   write_pos = (write_pos + 1) & (size - 1);
 }


 INLINE void Flush(void)
 {
  read_pos = 0;
  write_pos = 0;
  in_count = 0;
 }

 T* data;
 uint32 size;
 uint32 read_pos; // Read position
 uint32 write_pos; // Write position
 uint32 in_count; // Number of units in the FIFO
};


#endif

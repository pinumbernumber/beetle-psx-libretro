#ifndef _STATE_COMMON_H
#define _STATE_COMMON_H

#include "mednafen-types.h"

typedef struct
{
   uint8 *data;
   uint32 loc;
   uint32 len;
   uint32 malloced;
   uint32 initial_malloc; // A setting!
} StateMem;

typedef struct
{
   void *v;		// Pointer to the variable/array
   uint32 size;		// Length, in bytes, of the data to be saved EXCEPT:
   //  In the case of MDFNSTATE_BOOL, it is the number of bool elements to save(bool is not always 1-byte).
   // If 0, the subchunk isn't saved.
   uint32 flags;	// Flags
   const char *name;	// Name
   //uint32 struct_size;	// Only used for MDFNSTATE_ARRAYOFS, sizeof(struct) that members of the linked SFORMAT struct are in.
} SFORMAT;

#ifdef __cplusplus
extern "C" {
#endif

int MDFNSS_StateAction(void *st, int load, SFORMAT *sf, const char *name);

#ifdef __cplusplus
}
#endif

#endif

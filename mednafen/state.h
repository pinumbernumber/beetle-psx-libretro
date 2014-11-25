#ifndef _STATE_H
#define _STATE_H

#include "state-common.h"

// Eh, we abuse the smem_* in-memory stream code
// in a few other places. :)
int32 smem_read(StateMem *st, void *buffer, uint32 len);
int32 smem_write(StateMem *st, void *buffer, uint32 len);
int32 smem_putc(StateMem *st, int value);
int32 smem_tell(StateMem *st);
int32 smem_seek(StateMem *st, uint32 offset, int whence);
int smem_write32le(StateMem *st, uint32 b);
int smem_read32le(StateMem *st, uint32 *b);

int MDFNSS_SaveSM(void *st, int, int, const void*, const void*, const void*);
int MDFNSS_LoadSM(void *st, int, int);

// Flag for a single, >= 1 byte native-endian variable
#define MDFNSTATE_RLSB            0x80000000

// 32-bit native-endian elements
#define MDFNSTATE_RLSB32          0x40000000

// 16-bit native-endian elements
#define MDFNSTATE_RLSB16          0x20000000

// 64-bit native-endian elements
#define MDFNSTATE_RLSB64          0x10000000

#define MDFNSTATE_BOOL		  0x08000000

#endif

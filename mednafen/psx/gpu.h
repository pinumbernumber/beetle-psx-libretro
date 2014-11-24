// WARNING WARNING WARNING:  ONLY use CanRead() method of BlitterFIFO, and NOT CanWrite(), since the FIFO is larger than the actual PS1 GPU FIFO to accommodate
// our lack of fancy superscalarish command sequencer.

#ifndef __MDFN_PSX_GPU_H
#define __MDFN_PSX_GPU_H

#include "../cdrom/SimpleFIFO.h"

/* Powers of 2 for faster 
 * multiple equality testing
 * (just for multi-testing; 
 * InCmd itself will only contain 0, 
 * or a power of 2).
 */
enum
{
   INCMD_NONE     = 0,
   INCMD_PLINE    = (1 << 0),
   INCMD_QUAD     = (1 << 1),
   INCMD_FBWRITE  = (1 << 2),
   INCMD_FBREAD   = (1 << 3)
};


void GPU_New(bool pal_clock_and_tv, int sls, int sle);
void GPU_Free();

void GPU_FillVideoParams(MDFNGI* gi);

void GPU_Power(void);

void GPU_ResetTS(void);

int GPU_StateAction(StateMem *sm, int load, int data_only);

void GPU_StartFrame(EmulateSpecStruct *espec);

int32_t GPU_Update(const int32_t timestamp);

bool GPU_DMACanWrite(void);

void GPU_Write(const int32_t timestamp, uint32 A, uint32 V);

void GPU_WriteDMA(uint32 V);

uint32 GPU_ReadDMA(void);

uint32 GPU_Read(const int32_t timestamp, uint32 A);

int32 GPU_GetScanlineNum(void);

uint16 GPU_PeekRAM(uint32 A);

void GPU_PokeRAM(uint32 A, uint16 V);

#endif

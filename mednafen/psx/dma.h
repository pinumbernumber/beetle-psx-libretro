#ifndef __MDFN_PSX_DMA_H
#define __MDFN_PSX_DMA_H

bool DMA_GPUWriteActive(void);

int32_t DMA_Update(const int32_t timestamp);
void DMA_Write(const int32_t timestamp, uint32_t A, uint32_t V);
uint32_t DMA_Read(const int32_t timestamp, uint32_t A);

void DMA_ResetTS(void);

void DMA_Power(void);

void DMA_New(void);
void DMA_Free(void);

int DMA_StateAction(void *data, int load, int data_only);

#endif

#ifndef __MDFN_PSX_MDEC_H
#define __MDFN_PSX_MDEC_H

#ifdef __cplusplus
extern "C" {
#endif

void MDEC_DMAWrite(uint32_t V);

uint32_t MDEC_DMARead(int32_t *offs);

void MDEC_Write(const int32_t timestamp, uint32_t A, uint32_t V);
uint32_t MDEC_Read(const int32_t timestamp, uint32_t A);

void MDEC_Power(void);

bool MDEC_DMACanWrite(void);
bool MDEC_DMACanRead(void);
void MDEC_Run(int32 clocks);

int MDEC_StateAction(void *data, int load, int data_only);

#ifdef __cplusplus
}
#endif

#endif

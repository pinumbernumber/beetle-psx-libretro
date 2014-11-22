#ifndef __MDFN_PSX_TIMER_H
#define __MDFN_PSX_TIMER_H

enum
{
 TIMER_GSREG_COUNTER0 = 0x00,
 TIMER_GSREG_MODE0,
 TIMER_GSREG_TARGET0,

 TIMER_GSREG_COUNTER1 = 0x10,
 TIMER_GSREG_MODE1,
 TIMER_GSREG_TARGET1,

 TIMER_GSREG_COUNTER2 = 0x20,
 TIMER_GSREG_MODE2,
 TIMER_GSREG_TARGET2,
};

#ifdef __cplusplus
extern "C" {
#endif

uint32 TIMER_GetRegister(unsigned int which, char *special, const uint32 special_len);
void TIMER_SetRegister(unsigned int which, uint32 value);

void TIMER_Write(const int32_t timestamp, uint32 A, uint16 V);
uint16 TIMER_Read(const int32_t timestamp, uint32 A);

void TIMER_AddDotClocks(uint32 count);
void TIMER_ClockHRetrace(void);
void TIMER_SetHRetrace(bool status);
void TIMER_SetVBlank(bool status);

int32_t TIMER_Update(const int32_t unused);
void TIMER_ResetTS(void);

void TIMER_Power(void);
int TIMER_StateAction(void *data, int load, int data_only);
int32_t TIMER_CalcNextEvent(int32_t next_event);

#ifdef __cplusplus
}
#endif

#endif

#ifndef __MDFN_PSX_SPU_H
#define __MDFN_PSX_SPU_H

extern uint32_t IntermediateBufferPos;
extern int16_t IntermediateBuffer[4096][2];

enum
{
   ADSR_ATTACK = 0,
   ADSR_DECAY = 1,
   ADSR_SUSTAIN = 2,
   ADSR_RELEASE = 3
};

enum
{
   GSREG_SPUCONTROL = 0,

   GSREG_FM_ON,
   GSREG_NOISE_ON,
   GSREG_REVERB_ON,

   GSREG_CDVOL_L,
   GSREG_CDVOL_R,

   GSREG_DRYVOL_CTRL_L,
   GSREG_DRYVOL_CTRL_R,

   GSREG_DRYVOL_L,
   GSREG_DRYVOL_R,

   GSREG_WETVOL_L,
   GSREG_WETVOL_R,

   GSREG_RWADDR,

   GSREG_IRQADDR,

   GSREG_REVERBWA,

   GSREG_VOICEON,
   GSREG_VOICEOFF,
   GSREG_BLOCKEND,

   // Note: the order of these should match the reverb reg array
   GSREG_FB_SRC_A,
   GSREG_FB_SRC_B,
   GSREG_IIR_ALPHA,
   GSREG_ACC_COEF_A,
   GSREG_ACC_COEF_B,
   GSREG_ACC_COEF_C,
   GSREG_ACC_COEF_D,
   GSREG_IIR_COEF,
   GSREG_FB_ALPHA,
   GSREG_FB_X,
   GSREG_IIR_DEST_A0,
   GSREG_IIR_DEST_A1,
   GSREG_ACC_SRC_A0,
   GSREG_ACC_SRC_A1,
   GSREG_ACC_SRC_B0,
   GSREG_ACC_SRC_B1,
   GSREG_IIR_SRC_A0,
   GSREG_IIR_SRC_A1,
   GSREG_IIR_DEST_B0,
   GSREG_IIR_DEST_B1,
   GSREG_ACC_SRC_C0,
   GSREG_ACC_SRC_C1,
   GSREG_ACC_SRC_D0,
   GSREG_ACC_SRC_D1,
   GSREG_IIR_SRC_B1,
   GSREG_IIR_SRC_B0,
   GSREG_MIX_DEST_A0,
   GSREG_MIX_DEST_A1,
   GSREG_MIX_DEST_B0,
   GSREG_MIX_DEST_B1,
   GSREG_IN_COEF_L,
   GSREG_IN_COEF_R,


   // Multiply v * 256 for each extra voice
   GSREG_V0_VOL_CTRL_L  = 0x8000,
   GSREG_V0_VOL_CTRL_R,
   GSREG_V0_VOL_L,
   GSREG_V0_VOL_R,
   GSREG_V0_PITCH,
   GSREG_V0_STARTADDR,
   GSREG_V0_ADSR_CTRL,
   GSREG_V0_ADSR_LEVEL,
   GSREG_V0_LOOP_ADDR,
   GSREG_V0_READ_ADDR
};

// Buffers 44.1KHz samples, should have enough for two(worst-case scenario) video frames(2* ~735 frames NTSC, 2* ~882 PAL) plus jitter plus enough for the resampler leftovers.
// We'll just go with 4096 because powers of 2 are AWESOME and such.

struct SPU_ADSR
{
 uint16_t EnvLevel;	// We typecast it to (int16) in several places, but keep it here as (uint16) to prevent signed overflow/underflow, which compilers
			// may not treat consistently.
 uint32_t Divider;
 uint32_t Phase;

 bool AttackExp;
 bool SustainExp;
 bool SustainDec;
 bool ReleaseExp;

 int32_t AttackRate;	// Ar
 int32_t DecayRate;	// Dr * 4
 int32_t SustainRate;	// Sr
 int32_t ReleaseRate;	// Rr * 4

 int32_t SustainLevel;	// (Sl + 1) << 11
};

struct SPU_Sweep
{
 uint16_t Control;
 uint16_t Current;
 uint32_t Divider;
};

struct SPU_Voice
{
 int16 DecodeBuffer[0x20];
 int16 DecodeM2;
 int16 DecodeM1;

 uint32 DecodePlayDelay;
 uint32 DecodeWritePos;
 uint32 DecodeReadPos;
 uint32 DecodeAvail;

 bool IgnoreSampLA;
 
 uint8 DecodeShift;
 uint8 DecodeWeight;
 uint8_t DecodeFlags;

 SPU_Sweep Sweep[2];

 uint16_t Pitch;
 uint32_t CurPhase;

 uint32_t StartAddr;

 uint32_t CurAddr;

 uint32_t ADSRControl;

 uint32_t LoopAddr;

 int32_t PreLRSample;	// After enveloping, but before L/R volume.  Range of -32768 to 32767

 SPU_ADSR ADSR;
};

namespace MDFN_IEN_PSX
{


void SPU_Sweep_Clock(SPU_Sweep *sweep);


void SPU_New();
void SPU_Free();

uint32_t SPU_GetRegister(unsigned int which, char *special, const uint32_t special_len);
void SPU_SetRegister(unsigned int which, uint32_t value);

uint16_t SPU_PeekSPURAM(uint32_t address);
void SPU_PokeSPURAM(uint32_t address, uint16_t value);

int SPU_StateAction(StateMem *sm, int load, int data_only);

void SPU_Power(void);
void SPU_Write(int32_t timestamp, uint32_t A, uint16_t V);
uint16_t SPU_Read(int32_t timestamp, uint32_t A);

void SPU_StartFrame(double rate, uint32_t quality);
int32_t SPU_EndFrame(int16 *SoundBuf);

int32_t SPU_UpdateFromCDC(int32_t clocks);

}

#ifdef __cplusplus
extern "C" {
#endif

void SPU_WriteDMA(uint32_t V);
uint32 SPU_ReadDMA(void);

#ifdef __cplusplus
}
#endif

#endif

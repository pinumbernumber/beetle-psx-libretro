/* Mednafen - Multi-system Emulator
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/* TODO:
	Note to self: Emulating the SPU at more timing accuracy than sample, and emulating the whole SPU RAM write port FIFO thing and hypothetical periodic FIFO commit to
	SPU RAM(maybe every 32 CPU cycles, with exceptions?) will likely necessitate a much more timing-accurate CPU core, and emulation of the SPU delay register(or at least the
	effects of the standard value written to it), to avoid game glitches.  Probably more trouble than it's worth....

	SPU IRQ emulation isn't totally correct, behavior is kind of complex; run more tests on PS1.

	Test reverb upsampler on the real thing.

	Alter reverb algorithm to process in the pattern of L,R,L,R,L,R on each input sample, instead of doing both L and R on every 2 input samples(make
	sure the real thing does it this way too, I think it at least runs the downsampler this way); and while we're at it, implement the correct buffer
	offset(probably either -39 or -40, the latter is what we have now).

	Alter reverb algorithm to perform saturation more often, as occurs on the real thing.

	See if sample flag & 0x8 does anything weird, like suppressing the program-readable block end flag setting.

	Determine the actual purpose of global register 0x2C(is it REALLY an address multiplier?  And if so, does it affect the reverb offsets too?)

	For ADSR and volume sweep, should the divider be reset to 0 on &0x8000 == true, or should the upper bit be cleared?

	Should shift occur on all stages of ADPCM sample decoding, or only at the end?

	On the real thing, there's some kind of weirdness with ADSR when you voice on when attack_rate(raw) = 0x7F; the envelope level register is repeatedly
	reset to 0, which you can see by manual writes to the envelope level register.  Normally in the attack phase when attack_rate = 0x7F, enveloping is 		effectively stuck/paused such that the value you write is sticky and won't be replaced or reset.  Note that after you voice on, you can write a new 		attack_rate < 0x7F, and enveloping will work "normally" again shortly afterwards.  You can even write an attack_rate of 0x7F at that point to pause 		enveloping 		clocking.  I doubt any games rely on this, but it's something to keep in mind if we ever need greater insight as to how the SPU 	functions at a low-level in 		order to emulate it at cycle granularity rather than sample granularity, and it may not be a bad idea to 		investigate this oddity further and emulate it in 		the future regardless.

	Voice 1 and 3 waveform output writes to SPURAM might not be correct(noted due to problems reading this area of SPU RAM on the real thing
	based on my expectations of how this should work).
*/

/*
 Notes:
	The last half of the noise freq table was confirmed on a real PSX(more or less, number of changes * 0x8000 / samples), but the first half hasn't been yet with sufficient precision.

	All addresses(for 16-bit access, at least) within the SPU address space appear to be fully read/write as if they were RAM, though
	values at some addresses(like the envelope current value) will be "overwritten" by the sound processing at certain times.

	32-bit and 8-bit reads act as if it were RAM(not tested with all addresses, but a few, assuming the rest are the same), but 8-bit writes
	to odd addresses appear to be ignored, and 8-bit writes to even addresses are treated as 16-bit writes(most likely, but, need to code custom assembly to
	fully test the upper 8 bits).  NOTE: the preceding information doesn't necessarily cover accesses with side effects, they still need to be tested; and it
	of course covers reads/writes from the point of view of software running on the CPU.

	It doesn't appear to be possible to enable FM on the first channel/voice(channel/voice 0).

	Lower bit of channel start address appears to be masked out to 0(such that ADPCM block decoding is always 8 16-bit units, 16 bytes, aligned), as far as
	block-decoding and flag-set program-readable loop address go.
*/

/*
 Update() isn't called on Read and Writes for performance reasons, it's called with sufficient granularity from the event
 system, though this will obviously need to change if we ever emulate the SPU with better precision than per-sample(pair).
*/

#include "psx.h"
#include "cdc.h"
#include "spu.h"
#include "../../libretro.h"

uint32_t IntermediateBufferPos;
int16_t IntermediateBuffer[4096][2];

static uint32_t RWAddr;
static uint16_t SPUControl;
static uint16_t SPURAM[524288 / sizeof(uint16)];
static uint32_t IRQAddr;
static bool IRQAsserted;

static SPU_Voice Voices[24];

static uint32_t NoiseDivider;
static uint32_t NoiseCounter;
static uint16_t LFSR;

static uint32_t FM_Mode;
static uint32_t Noise_Mode;
static uint32_t Reverb_Mode;

static int32_t ReverbWA;

static SPU_Sweep GlobalSweep[2];	// Doesn't affect reverb volume!

static int32_t ReverbVol[2];

static int32_t CDVol[2];
static int32_t ExternVol[2];
 
static uint32_t VoiceOn;
static uint32_t VoiceOff;

static uint32_t BlockEnd;

static uint32_t CWA;

union
{
   uint16_t Regs[0x100];
   struct
   {
      uint16_t VoiceRegs[0xC0];
      union
      {
         uint16_t GlobalRegs[0x20];
         struct
         {
            uint16_t _Global0[0x17];
            uint16_t SPUStatus;
            uint16_t _Global1[0x08];
         };
      } globalregs;
      union
      {
         int16 ReverbRegs[0x20];

         struct
         {
            int16 FB_SRC_A;
            int16 FB_SRC_B;
            int16 IIR_ALPHA;
            int16 ACC_COEF_A;
            int16 ACC_COEF_B;
            int16 ACC_COEF_C;
            int16 ACC_COEF_D;
            int16 IIR_COEF;
            int16 FB_ALPHA;
            int16 FB_X;
            int16 IIR_DEST_A0;
            int16 IIR_DEST_A1;
            int16 ACC_SRC_A0;
            int16 ACC_SRC_A1;
            int16 ACC_SRC_B0;
            int16 ACC_SRC_B1;
            int16 IIR_SRC_A0;
            int16 IIR_SRC_A1;
            int16 IIR_DEST_B0;
            int16 IIR_DEST_B1;
            int16 ACC_SRC_C0;
            int16 ACC_SRC_C1;
            int16 ACC_SRC_D0;
            int16 ACC_SRC_D1;
            int16 IIR_SRC_B1;
            int16 IIR_SRC_B0;
            int16 MIX_DEST_A0;
            int16 MIX_DEST_A1;
            int16 MIX_DEST_B0;
            int16 MIX_DEST_B1;
            int16 IN_COEF_L;
            int16 IN_COEF_R;
         };
      } reverbregs;
   };
} unionregs;

static uint16_t AuxRegs[0x10];

static int16 RDSB[2][128];	// [40]
static int32_t RDSB_WP;

static int16 RUSB[2][128];
static int32_t RUSB_WP;

static int32_t ReverbCur;

static int32_t clock_divider;

static int last_rate;
static uint32_t last_quality;

//#define SPUIRQ_DBG(format, ...) { printf("[SPUIRQDBG] " format " -- Voice 22 CA=0x%06x,LA=0x%06x\n", ## __VA_ARGS__, Voices[22].CurAddr, Voices[22].LoopAddr); }

static INLINE void SPUIRQ_DBG(const char *fmt, ...)
{
}

static const int16 FIR_Table[256][4] =
{
 #include "spu_fir_table.inc"
};

void SPU_New()
{
 IntermediateBufferPos = 0;
 memset(IntermediateBuffer, 0, sizeof(IntermediateBuffer));

}

void SPU_Free()
{
}

void SPU_Power(void)
{
 clock_divider = 768;

 memset(SPURAM, 0, sizeof(SPURAM));

 for(int i = 0; i < 24; i++)
 {
  memset(Voices[i].DecodeBuffer, 0, sizeof(Voices[i].DecodeBuffer));
  Voices[i].DecodeM2 = 0;
  Voices[i].DecodeM1 = 0;

  Voices[i].DecodePlayDelay = 0;
  Voices[i].DecodeWritePos = 0;
  Voices[i].DecodeReadPos = 0;
  Voices[i].DecodeAvail = 0;

  Voices[i].DecodeShift = 0;
  Voices[i].DecodeWeight = 0;
  Voices[i].DecodeFlags = 0;

  Voices[i].IgnoreSampLA = false;

  Voices[i].Sweep[0].Control = 0;
  Voices[i].Sweep[0].Current = 0;
  Voices[i].Sweep[0].Divider = 0;
  Voices[i].Sweep[1].Control = 0;
  Voices[i].Sweep[1].Current = 0;
  Voices[i].Sweep[1].Divider = 0;

  Voices[i].Pitch = 0;
  Voices[i].CurPhase = 0;

  Voices[i].StartAddr = 0;

  Voices[i].CurAddr = 0;

  Voices[i].ADSRControl = 0;

  Voices[i].LoopAddr = 0;

  Voices[i].PreLRSample = 0;

  memset(&Voices[i].ADSR, 0, sizeof(SPU_ADSR));
 }

 GlobalSweep[0].Control = 0;
 GlobalSweep[0].Current = 0;
 GlobalSweep[0].Divider = 0;
 GlobalSweep[1].Control = 0;
 GlobalSweep[1].Current = 0;
 GlobalSweep[1].Divider = 0;

 NoiseDivider = 0;
 NoiseCounter = 0;
 LFSR = 0;

 FM_Mode = 0;
 Noise_Mode = 0;
 Reverb_Mode = 0;
 ReverbWA = 0;

 ReverbVol[0] = ReverbVol[1] = 0;

 CDVol[0] = CDVol[1] = 0;

 ExternVol[0] = ExternVol[1] = 0;

 IRQAddr = 0;

 RWAddr = 0;

 SPUControl = 0;

 VoiceOn = 0;
 VoiceOff = 0;

 BlockEnd = 0;

 CWA = 0;

 memset(unionregs.Regs, 0, sizeof(unionregs.Regs));

 memset(RDSB, 0, sizeof(RDSB));
 RDSB_WP = 0;

 memset(RUSB, 0, sizeof(RUSB));
 RUSB_WP = 0;

 ReverbCur = ReverbWA;

 IRQAsserted = false;
}

static INLINE void CalcVCDelta(const uint8 zs, uint8 speed,bool log_mode,
      bool dec_mode, bool inv_increment, int16 Current, int &increment, int &divinco)
{
  increment = (7 - (speed & 0x3));

  if(inv_increment)
   increment = ~increment;

  divinco = 32768;

  if(speed < 0x2C)
   increment <<= (0x2F - speed) >> 2;

  if(speed >= 0x30)
   divinco >>= (speed - 0x2C) >> 2;

  if(log_mode)
  {
   if(dec_mode)	// Log decrement mode
    increment = (Current * increment) >> 15;
   else			// Log increment mode
   {
    if((Current & 0x7FFF) >= 0x6000)
    {
     if(speed < 0x28)
      increment >>= 2;
     else if(speed >= 0x2C)
      divinco >>= 2;
     else	// 0x28 ... 0x2B
     {
      increment >>= 1;
      divinco >>= 1;
     }
    }
   }
  } // end if(log_mode)

  if(divinco == 0 && speed < zs) //0x7F)
   divinco = 1;
}


void SPU_Sweep_Clock(SPU_Sweep *sweep)
{
 if(!(sweep->Control & 0x8000))
 {
  sweep->Current = (sweep->Control & 0x7FFF) << 1;
  return;
 }

 if(sweep->Control & 0x8000) 	// Sweep enabled
 {
  int increment, divinco;
  const bool log_mode = (bool)(sweep->Control & 0x4000);
  const bool dec_mode = (bool)(sweep->Control & 0x2000);
  const bool inv_mode = (bool)(sweep->Control & 0x1000);
  const bool inv_increment = (dec_mode ^ inv_mode) | (dec_mode & log_mode);
  const uint16 vc_cv_xor = (inv_mode & !(dec_mode & log_mode)) ? 0xFFFF : 0x0000;
  const uint16 TestInvert = inv_mode ? 0xFFFF : 0x0000;

  CalcVCDelta(0x7F, sweep->Control & 0x7F, log_mode, dec_mode, inv_increment, (int16)(sweep->Current ^ vc_cv_xor), increment, divinco);
  //printf("%d %d\n", divinco, increment);

  if((dec_mode & !(inv_mode & log_mode)) && ((sweep->Current & 0x8000) == (inv_mode ? 0x0000 : 0x8000) || (sweep->Current == 0)))
  {
   //
   // Not sure if this condition should stop the Divider adding or force the increment value to 0.
   //
   sweep->Current = 0;
  }
  else
  {
   sweep->Divider += divinco;

   if(sweep->Divider & 0x8000)
   {
    sweep->Divider = 0;

    if(dec_mode || ((sweep->Current ^ TestInvert) != 0x7FFF))
    {
     uint16 PrevCurrent = sweep->Current;
     sweep->Current = sweep->Current + increment;

     //printf("%04x %04x\n", PrevCurrent, sweep->Current);

     if(!dec_mode && ((sweep->Current ^ PrevCurrent) & 0x8000) && ((sweep->Current ^ TestInvert) & 0x8000))
      sweep->Current = 0x7FFF ^ TestInvert;
    }
   }
  }
 }
}

//
// Take care not to trigger SPU IRQ for the next block before its decoding start.
//
static void SPU_RunDecoder(SPU_Voice *voice)
{
 // 5 through 0xF appear to be 0 on the real thing.
 static const int32 Weights[16][2] =
 {
  // s-1    s-2
  {   0,    0 },
  {  60,    0 },
  { 115,  -52 },
  {  98,  -55 },
  { 122,  -60 },
 };

 if(voice->DecodeAvail >= 11)
 {
  if(SPUControl & 0x40)
  {
   unsigned test_addr = (voice->CurAddr - 1) & 0x3FFFF;
   if(IRQAddr == test_addr || IRQAddr == (test_addr & 0x3FFF8))
   {
    //SPUIRQ_DBG("SPU IRQ (VDA): 0x%06x", addr);
    IRQAsserted = true;
    IRQ_Assert(IRQ_SPU, IRQAsserted);
   }
  }
  return;
 }

 if((voice->CurAddr & 0x7) == 0)
 {
  // Handle delayed flags from the previously-decoded block.
  if(voice->DecodeFlags & 0x1)
  {
   voice->CurAddr = voice->LoopAddr & ~0x7;

   BlockEnd |= 1 << (voice - Voices);

   if(!(voice->DecodeFlags & 0x2))	// Force enveloping to 0 if not "looping".  TODO: Should we reset the ADSR divider counter too?
   {
    if(!(Noise_Mode & (1 << (voice - Voices))))
    {
     voice->ADSR.Phase = ADSR_RELEASE;
     voice->ADSR.EnvLevel = 0;
    }
   }
  }
 }

 //for(int z = 0; z < 4; z++)
 {

  if(SPUControl & 0x40)
  {
   unsigned test_addr = voice->CurAddr & 0x3FFFF;
   if(IRQAddr == test_addr || IRQAddr == (test_addr & 0x3FFF8))
   {
    //SPUIRQ_DBG("SPU IRQ: 0x%06x", addr);
    IRQAsserted = true;
    IRQ_Assert(IRQ_SPU, IRQAsserted);
   }
  }

  if((voice->CurAddr & 0x7) == 0)
  {
     const uint16 CV = SPURAM[voice->CurAddr];
   voice->DecodeShift = CV & 0xF;
   voice->DecodeWeight = (CV >> 4) & 0xF;
   voice->DecodeFlags = (CV >> 8) & 0xFF;

   if(voice->DecodeFlags & 0x4)
   {
    if(!voice->IgnoreSampLA)
     voice->LoopAddr = voice->CurAddr;
#if 0
    else
    {
     if(voice->LoopAddr != voice->CurAddr)
     {
      PSX_DBG(PSX_DBG_FLOOD, "[SPU] Ignore: LoopAddr=0x%08x, SampLA=0x%08x\n", voice->LoopAddr, voice->CurAddr);
     }
    }
#endif
   }
   voice->CurAddr = (voice->CurAddr + 1) & 0x3FFFF;
  }

  //
  // Don't else this block; we need to ALWAYS decode 4 samples per call to RunDecoder() if DecodeAvail < 11, or else sample playback
  // at higher rates will fail horribly.
  //
  {
     const uint16 CV = SPURAM[voice->CurAddr];
   const unsigned shift = voice->DecodeShift;
   const int32 weight_m1 = Weights[voice->DecodeWeight][0];
   const int32 weight_m2 = Weights[voice->DecodeWeight][1];
   uint32 coded = (uint32)CV << 12;
   int16 *tb = &voice->DecodeBuffer[voice->DecodeWritePos];

   for(int i = 0; i < 4; i++)
   {
    int32 sample = (int16)(coded & 0xF000) >> shift;

    sample += ((voice->DecodeM2 * weight_m2) >> 6);
    sample += ((voice->DecodeM1 * weight_m1) >> 6);

    clamp(&sample, -32768, 32767);

    tb[i] = sample;
    voice->DecodeM2 = voice->DecodeM1;
    voice->DecodeM1 = sample;
    coded >>= 4;
   }
   voice->DecodeWritePos = (voice->DecodeWritePos + 4) & 0x1F;
   voice->DecodeAvail += 4;
   voice->CurAddr = (voice->CurAddr + 1) & 0x3FFFF;
  }
 }
}

#define CacheEnvelope(voice) \
 voice->ADSR.AttackExp = (bool)(voice->ADSRControl & (1 << 15)); \
 voice->ADSR.ReleaseExp = (bool)(voice->ADSRControl & (1 << 21)); \
 voice->ADSR.SustainExp = (bool)(voice->ADSRControl & (1 << 31)); \
 voice->ADSR.SustainDec = (bool)(voice->ADSRControl & (1 << 30)); \
 voice->ADSR.AttackRate =  (voice->ADSRControl >> 8) & 0x7F;         /* Ar */ \
 voice->ADSR.DecayRate =   ((voice->ADSRControl >> 4) & 0x0F) << 2;  /* Dr & 0x0F */ \
 voice->ADSR.SustainRate = (voice->ADSRControl >> 22) & 0x7F;        /* Sr */ \
 voice->ADSR.ReleaseRate = ((voice->ADSRControl >> 16) & 0x1F) << 2; /* Rr << 2 */ \
 voice->ADSR.SustainLevel = (((voice->ADSRControl >> 0) & 0x0F) + 1) << 11         /* (Sl + 1) << 11 */

#define ResetEnvelope(voice) \
 voice->ADSR.EnvLevel = 0; \
 voice->ADSR.Divider = 0; \
 voice->ADSR.Phase = ADSR_ATTACK

#define ReleaseEnvelope(voice) \
 voice->ADSR.Divider = 0; \
 voice->ADSR.Phase = ADSR_RELEASE


static void SPU_RunEnvelope(SPU_Voice *voice)
{
 SPU_ADSR *ADSR = &voice->ADSR;
 int increment;
 int divinco;
 int16 uoflow_reset;

 if(ADSR->Phase == ADSR_ATTACK && ADSR->EnvLevel == 0x7FFF)
  ADSR->Phase++;

 switch(ADSR->Phase)
 {
  default: assert(0);
	   break;

  case ADSR_ATTACK:
	CalcVCDelta(0x7F, ADSR->AttackRate, ADSR->AttackExp, false, false, (int16)ADSR->EnvLevel, increment, divinco);
	uoflow_reset = 0x7FFF;
	break;

  case ADSR_DECAY:
	CalcVCDelta(0x1F << 2, ADSR->DecayRate, true, true, true, (int16)ADSR->EnvLevel, increment, divinco);
	uoflow_reset = 0;
	break;

  case ADSR_SUSTAIN:
	CalcVCDelta(0x7F, ADSR->SustainRate, ADSR->SustainExp, ADSR->SustainDec, ADSR->SustainDec, (int16)ADSR->EnvLevel, increment, divinco);
	uoflow_reset = ADSR->SustainDec ? 0 : 0x7FFF;
	break;

  case ADSR_RELEASE:
	CalcVCDelta(0x1F << 2, ADSR->ReleaseRate, ADSR->ReleaseExp, true, true, (int16)ADSR->EnvLevel, increment, divinco);
	uoflow_reset = 0;
	break;
 }

 ADSR->Divider += divinco;
 if(ADSR->Divider & 0x8000)
 {
  const uint16 prev_level = ADSR->EnvLevel;

  ADSR->Divider = 0;
  ADSR->EnvLevel += increment;

  if(ADSR->Phase == ADSR_ATTACK)
  {
   // If previous the upper bit was 0, but now it's 1, handle overflow.
   if(((prev_level ^ ADSR->EnvLevel) & ADSR->EnvLevel) & 0x8000)
    ADSR->EnvLevel = uoflow_reset;
  }
  else
  {
   if(ADSR->EnvLevel & 0x8000)
    ADSR->EnvLevel = uoflow_reset;
  }
  if(ADSR->Phase == ADSR_DECAY && (uint16)ADSR->EnvLevel < ADSR->SustainLevel)
   ADSR->Phase++;
 }
}

#define CheckIRQAddr(addr) \
   if((SPUControl & 0x40) && (IRQAddr == (addr))) \
   { \
      /* SPUIRQ_DBG("SPU IRQ (ALT): 0x%06x", addr); */ \
      IRQAsserted = true; \
      IRQ_Assert(IRQ_SPU, IRQAsserted); \
   }

static INLINE void WriteSPURAM(uint32 addr, uint16 value)
{
   CheckIRQAddr(addr);

   SPURAM[addr] = value;
}

static INLINE uint16 ReadSPURAM(uint32 addr)
{
   CheckIRQAddr(addr);
   return(SPURAM[addr]);
}

static INLINE int16_t ReverbSat(int32_t samp)
{
   clamp(&samp, -32768, 32767);
   return(samp);
}

static int32_t SPU_Get_Reverb_Offset(int32_t in_offset)
{
   int32_t offset = in_offset & 0x3FFFF;
   int32_t wa_size = 0x40000 - ReverbWA;

   if(offset & 0x20000)
   {
      offset -= ReverbWA;

      if(offset < 0)
      {
         offset = 0;
         //PSX_WARNING("[SPU] A reverb offset is broken(-).");
      }
   }
   else
   {
      if(offset >= wa_size)
      {
         offset = wa_size - 1;
         //PSX_WARNING("[SPU] A reverb offset is broken(+): WASize=0x%04x, 0x%04x.", wa_size >> 2, in_offset >> 2);
      }
   }

   offset += ReverbCur;

   if(offset >= 0x40000)
   offset = (offset & 0x3FFFF) + ReverbWA;

   assert(offset >= ReverbWA && offset < 0x40000);

   return(offset);
}

#define RD_RVB(raw_offs) ((int16)SPURAM[SPU_Get_Reverb_Offset((raw_offs) << 2)])

#define WR_RVB(raw_offs, sample, extra_offs) SPURAM[SPU_Get_Reverb_Offset(((raw_offs) << 2) + (extra_offs))] = ReverbSat((sample))

static INLINE int32_t Reverb4422(const int16_t *src)
{
 static const int16_t ResampTable[40] =
 {
  (int16)0xffff,
  (int16)0x0000,
  (int16)0x0002,
  (int16)0x0000,
  (int16)0xfff6,
  (int16)0x0000,
  (int16)0x0023,
  (int16)0x0000,
  (int16)0xff99,
  (int16)0x0000,
  (int16)0x010a,
  (int16)0x0000,
  (int16)0xfd98,
  (int16)0x0000,
  (int16)0x0534,
  (int16)0x0000,
  (int16)0xf470,
  (int16)0x0000,
  (int16)0x2806,
  (int16)0x4000,
  (int16)0x2806,
  (int16)0x0000,
  (int16)0xf470,
  (int16)0x0000,
  (int16)0x0534,
  (int16)0x0000,
  (int16)0xfd98,
  (int16)0x0000,
  (int16)0x010a,
  (int16)0x0000,
  (int16)0xff99,
  (int16)0x0000,
  (int16)0x0023,
  (int16)0x0000,
  (int16)0xfff6,
  (int16)0x0000,
  (int16)0x0002,
  (int16)0x0000,
  (int16)0xffff,				     
  (int16)0x0000,
 };
 int32_t out = 0;	// 32-bits is adequate(it won't overflow)

 for(int i = 0; i < 40; i += 2)
  out += ResampTable[i] * src[i];

 // Middle non-zero
 out += 0x4000 * src[19];

 out >>= 15;

 clamp(&out, -32768, 32767);
 return(out);
}

static INLINE void SPU_RunReverb(int32_t in_l, int32_t in_r, int32_t &out_l, int32_t &out_r)
{
 int32_t upsampled[2] = { 0, 0 };

 RDSB[0][RDSB_WP] = in_l;
 RDSB[1][RDSB_WP] = in_r;
 RDSB[0][RDSB_WP | 0x40] = in_l;	// So we don't have to &/bounds check in our MAC loop
 RDSB[1][RDSB_WP | 0x40] = in_r;

 RDSB_WP = (RDSB_WP + 1) & 0x3F;

 if(!(RDSB_WP & 1))
 {
  int32_t downsampled[2];

  for(int lr = 0; lr < 2; lr++)
   downsampled[lr] = Reverb4422(&RDSB[lr][(RDSB_WP - 40) & 0x3F]);

  //
  // Run algorithm
  ///
 if(SPUControl & 0x80)
 {
  int32_t ACC0, ACC1;
  int32_t FB_A0, FB_A1, FB_B0, FB_B1;

  int32_t IIR_INPUT_A0 = ((RD_RVB(unionregs.reverbregs.IIR_SRC_A0) * unionregs.reverbregs.IIR_COEF) >> 15) + ((downsampled[0] * unionregs.reverbregs.IN_COEF_L) >> 15);
  int32_t IIR_INPUT_A1 = ((RD_RVB(unionregs.reverbregs.IIR_SRC_A1) * unionregs.reverbregs.IIR_COEF) >> 15) + ((downsampled[1] * unionregs.reverbregs.IN_COEF_R) >> 15);
  int32_t IIR_INPUT_B0 = ((RD_RVB(unionregs.reverbregs.IIR_SRC_B0) * unionregs.reverbregs.IIR_COEF) >> 15) + ((downsampled[0] * unionregs.reverbregs.IN_COEF_L) >> 15);
  int32_t IIR_INPUT_B1 = ((RD_RVB(unionregs.reverbregs.IIR_SRC_B1) * unionregs.reverbregs.IIR_COEF) >> 15) + ((downsampled[1] * unionregs.reverbregs.IN_COEF_R) >> 15);


  int32_t IIR_A0 = (((int64)IIR_INPUT_A0 * unionregs.reverbregs.IIR_ALPHA) >> 15) + ((RD_RVB(unionregs.reverbregs.IIR_DEST_A0) * (32768 - unionregs.reverbregs.IIR_ALPHA)) >> 15);
  int32_t IIR_A1 = (((int64)IIR_INPUT_A1 * unionregs.reverbregs.IIR_ALPHA) >> 15) + ((RD_RVB(unionregs.reverbregs.IIR_DEST_A1) * (32768 - unionregs.reverbregs.IIR_ALPHA)) >> 15);
  int32_t IIR_B0 = (((int64)IIR_INPUT_B0 * unionregs.reverbregs.IIR_ALPHA) >> 15) + ((RD_RVB(unionregs.reverbregs.IIR_DEST_B0) * (32768 - unionregs.reverbregs.IIR_ALPHA)) >> 15);
  int32_t IIR_B1 = (((int64)IIR_INPUT_B1 * unionregs.reverbregs.IIR_ALPHA) >> 15) + ((RD_RVB(unionregs.reverbregs.IIR_DEST_B1) * (32768 - unionregs.reverbregs.IIR_ALPHA)) >> 15);

  WR_RVB(unionregs.reverbregs.IIR_DEST_A0, IIR_A0, 1);
  WR_RVB(unionregs.reverbregs.IIR_DEST_A1, IIR_A1, 1);
  WR_RVB(unionregs.reverbregs.IIR_DEST_B0, IIR_B0, 1);
  WR_RVB(unionregs.reverbregs.IIR_DEST_B1, IIR_B1, 1);

  ACC0 = ((int64)(RD_RVB(unionregs.reverbregs.ACC_SRC_A0) * unionregs.reverbregs.ACC_COEF_A) +
	        (RD_RVB(unionregs.reverbregs.ACC_SRC_B0) * unionregs.reverbregs.ACC_COEF_B) +
	        (RD_RVB(unionregs.reverbregs.ACC_SRC_C0) * unionregs.reverbregs.ACC_COEF_C) +
	        (RD_RVB(unionregs.reverbregs.ACC_SRC_D0) * unionregs.reverbregs.ACC_COEF_D)) >> 15;


  ACC1 = ((int64)(RD_RVB(unionregs.reverbregs.ACC_SRC_A1) * unionregs.reverbregs.ACC_COEF_A) +
	        (RD_RVB(unionregs.reverbregs.ACC_SRC_B1) * unionregs.reverbregs.ACC_COEF_B) +
	        (RD_RVB(unionregs.reverbregs.ACC_SRC_C1) * unionregs.reverbregs.ACC_COEF_C) +
	        (RD_RVB(unionregs.reverbregs.ACC_SRC_D1) * unionregs.reverbregs.ACC_COEF_D)) >> 15;

  FB_A0 = RD_RVB(unionregs.reverbregs.MIX_DEST_A0 - unionregs.reverbregs.FB_SRC_A);
  FB_A1 = RD_RVB(unionregs.reverbregs.MIX_DEST_A1 - unionregs.reverbregs.FB_SRC_A);
  FB_B0 = RD_RVB(unionregs.reverbregs.MIX_DEST_B0 - unionregs.reverbregs.FB_SRC_B);
  FB_B1 = RD_RVB(unionregs.reverbregs.MIX_DEST_B1 - unionregs.reverbregs.FB_SRC_B);

  WR_RVB(unionregs.reverbregs.MIX_DEST_A0, ACC0 - ((FB_A0 * unionregs.reverbregs.FB_ALPHA) >> 15), 0);
  WR_RVB(unionregs.reverbregs.MIX_DEST_A1, ACC1 - ((FB_A1 * unionregs.reverbregs.FB_ALPHA) >> 15), 0);

  WR_RVB(unionregs.reverbregs.MIX_DEST_B0, (((int64)unionregs.reverbregs.FB_ALPHA * ACC0) >> 15) - ((FB_A0 * (int16)(0x8000 ^ unionregs.reverbregs.FB_ALPHA)) >> 15) - ((FB_B0 * unionregs.reverbregs.FB_X) >> 15), 0);
  WR_RVB(unionregs.reverbregs.MIX_DEST_B1, (((int64)unionregs.reverbregs.FB_ALPHA * ACC1) >> 15) - ((FB_A1 * (int16)(0x8000 ^ unionregs.reverbregs.FB_ALPHA)) >> 15) - ((FB_B1 * unionregs.reverbregs.FB_X) >> 15), 0);
 }

  // 
  // Get output samples
  //
//  RUSB[0][RUSB_WP | 0x40] = RUSB[0][RUSB_WP] = (short)rand();
//  RUSB[1][RUSB_WP | 0x40] = RUSB[1][RUSB_WP] = (short)rand();
  RUSB[0][RUSB_WP | 0x40] = RUSB[0][RUSB_WP] = (RD_RVB(unionregs.reverbregs.MIX_DEST_A0) + RD_RVB(unionregs.reverbregs.MIX_DEST_B0)) >> 1;
  RUSB[1][RUSB_WP | 0x40] = RUSB[1][RUSB_WP] = (RD_RVB(unionregs.reverbregs.MIX_DEST_A1) + RD_RVB(unionregs.reverbregs.MIX_DEST_B1)) >> 1;


  ReverbCur = (ReverbCur + 1) & 0x3FFFF;
  if(!ReverbCur)
   ReverbCur = ReverbWA;
 } 
 else
 {
  RUSB[0][RUSB_WP | 0x40] = RUSB[0][RUSB_WP] = 0;
  RUSB[1][RUSB_WP | 0x40] = RUSB[1][RUSB_WP] = 0;

 }
 RUSB_WP = (RUSB_WP + 1) & 0x3F;

 for(int lr = 0; lr < 2; lr++)
  upsampled[lr] = Reverb4422(&RUSB[lr][(RUSB_WP - 40) & 0x3F]);

 out_l = upsampled[0];
 out_r = upsampled[1];
}


static INLINE void SPU_RunNoise(void)
{
   const unsigned rf = ((SPUControl >> 8) & 0x3F);
   uint32 NoiseDividerInc = (2 << (rf >> 2));
   uint32 NoiseCounterInc = 4 + (rf & 0x3);

   if(rf >= 0x3C)
   {
      NoiseDividerInc = 0x8000;
      NoiseCounterInc = 8;
   }

   NoiseDivider += NoiseDividerInc;

   if(NoiseDivider & 0x8000)
   {
      NoiseDivider = 0;

      NoiseCounter += NoiseCounterInc;

      if(NoiseCounter & 0x8)
      {
         NoiseCounter &= 0x7;
         LFSR = (LFSR << 1) | (((LFSR >> 15) ^ (LFSR >> 12) ^ (LFSR >> 11) ^ (LFSR >> 10) ^ 1) & 1);
      }
   }
}

int32 SPU_UpdateFromCDC(int32 clocks)
{
 //int32 clocks = timestamp - lastts;
 int32 sample_clocks = 0;
 //lastts = timestamp;

 clock_divider -= clocks;

 while(clock_divider <= 0)
 {
  clock_divider += 768;
  sample_clocks++;
 }

 while(sample_clocks > 0)
 {
  // Accumulated normal sound output.
  int32 accum_l = 0;
  int32 accum_r = 0;

  // Accumulated sound output for reverb input
  int32 accum_fv_l = 0;
  int32 accum_fv_r = 0;

  // Output of reverb processing.
  int32 reverb_l = 0;
  int32 reverb_r = 0;

  // Final output.
  int32 output_l = 0;
  int32 output_r = 0;

  const uint32 PhaseModCache = FM_Mode & ~ 1;
/*
**
** 0x1F801DAE Notes and Conjecture:
**   -------------------------------------------------------------------------------------
**   |   15   14 | 13 | 12 | 11 | 10  | 9  | 8 |  7 |  6  | 5    4    3    2    1    0   |
**   |      ?    | *13| ?  | ba | *10 | wrr|rdr| df |  is |      c                       |
**   -------------------------------------------------------------------------------------
**
**	c - Appears to be delayed copy of lower 6 bits from 0x1F801DAA.
**
**     is - Interrupt asserted out status. (apparently not instantaneous status though...)
**
**     df - Related to (c & 0x30) == 0x20 or (c & 0x30) == 0x30, at least.
**          0 = DMA busy(FIFO not empty when in DMA write mode?)?
**	    1 = DMA ready?  Something to do with the FIFO?
**
**     rdr - Read(DMA read?) Ready?
**
**     wrr - Write(DMA write?) Ready?
**
**     *10 - Unknown.  Some sort of (FIFO?) busy status?(BIOS tests for this bit in places)
**
**     ba - Alternates between 0 and 1, even when SPUControl bit15 is 0; might be related to CD audio and voice 1 and 3 writing to SPU RAM.
**
**     *13 - Unknown, was set to 1 when testing with an SPU delay system reg value of 0x200921E1(test result might not be reliable, re-run).
*/
  unionregs.globalregs.SPUStatus = SPUControl & 0x3F;
  unionregs.globalregs.SPUStatus |= IRQAsserted ? 0x40 : 0x00;

  if(unionregs.Regs[0xD6] == 0x4)	// TODO: Investigate more(case 0x2C in global regs r/w handler)
   unionregs.globalregs.SPUStatus |= (CWA & 0x100) ? 0x800 : 0x000;

  for(int voice_num = 0; voice_num < 24; voice_num++)
  {
   SPU_Voice *voice = &Voices[voice_num];
   int32 voice_pvs;

   voice->PreLRSample = 0;

   //PSX_WARNING("[SPU] Voice %d CurPhase=%08x, pitch=%04x, CurAddr=%08x", voice_num, voice->CurPhase, voice->Pitch, voice->CurAddr);

   //
   // Decode new samples if necessary.
   //
   SPU_RunDecoder(voice);


   //
   //
   //
   int l, r;

   if(Noise_Mode & (1 << voice_num))
    voice_pvs = (int16)LFSR;
   else
   {
    const int si = voice->DecodeReadPos;
    const int pi = ((voice->CurPhase & 0xFFF) >> 4);

    voice_pvs = ((voice->DecodeBuffer[(si + 0) & 0x1F] * FIR_Table[pi][0]) +
	         (voice->DecodeBuffer[(si + 1) & 0x1F] * FIR_Table[pi][1]) +
	         (voice->DecodeBuffer[(si + 2) & 0x1F] * FIR_Table[pi][2]) +   
 	         (voice->DecodeBuffer[(si + 3) & 0x1F] * FIR_Table[pi][3])) >> 15;
   }

   voice_pvs = (voice_pvs * (int16)voice->ADSR.EnvLevel) >> 15;
   voice->PreLRSample = voice_pvs;

   if(voice_num == 1 || voice_num == 3)
   {
    int index = voice_num >> 1;

    WriteSPURAM(0x400 | (index * 0x200) | CWA, voice_pvs);
   }


   l = (voice_pvs * (int16)voice->Sweep[0].Current) >> 15;
   r = (voice_pvs * (int16)voice->Sweep[1].Current) >> 15;

   accum_l += l;
   accum_r += r;

   if(Reverb_Mode & (1 << voice_num))
   {
    accum_fv_l += l;
    accum_fv_r += r;
   }

   // Run sweep
   for(int lr = 0; lr < 2; lr++)
    SPU_Sweep_Clock(&voice->Sweep[lr]);

   // Increment stuff
   if(!voice->DecodePlayDelay)
   {
    unsigned phase_inc;

    // Run enveloping
    SPU_RunEnvelope(voice);

    if(PhaseModCache & (1 << voice_num))
    {
     // This old formula: phase_inc = (voice->Pitch * ((voice - 1)->PreLRSample + 0x8000)) >> 15;
     // is incorrect, as it does not handle carrier pitches >= 0x8000 properly.
     phase_inc = voice->Pitch + (((int16)voice->Pitch * ((voice - 1)->PreLRSample)) >> 15);
    }
    else
     phase_inc = voice->Pitch;

    if(phase_inc > 0x3FFF)
     phase_inc = 0x3FFF;

    {
     const uint32 tmp_phase = voice->CurPhase + phase_inc;
     const unsigned used = tmp_phase >> 12;

     voice->CurPhase = tmp_phase & 0xFFF;
     voice->DecodeAvail -= used;
     voice->DecodeReadPos = (voice->DecodeReadPos + used) & 0x1F;
    }
   }
   else
    voice->DecodePlayDelay--;

   if(VoiceOff & (1U << voice_num))
   {
    if(voice->ADSR.Phase != ADSR_RELEASE)
    {
     ReleaseEnvelope(voice);
    }
   }

   if(VoiceOn & (1U << voice_num))
   {
    //printf("Voice On: %u\n", voice_num);

    ResetEnvelope(voice);

    voice->DecodeFlags = 0;
    voice->DecodeWritePos = 0;
    voice->DecodeReadPos = 0;
    voice->DecodeAvail = 0;
    voice->DecodePlayDelay = 4;

    BlockEnd &= ~(1 << voice_num);

    //
    // Weight/filter previous value initialization:
    //
    voice->DecodeM2 = 0;
    voice->DecodeM1 = 0;

    voice->CurPhase = 0;
    voice->CurAddr = voice->StartAddr & ~0x7;
    voice->IgnoreSampLA = false;
   }

   if(!(SPUControl & 0x8000))
   {
    voice->ADSR.Phase = ADSR_RELEASE;
    voice->ADSR.EnvLevel = 0;
   }
  }

  VoiceOff = 0;
  VoiceOn = 0; 

  // "Mute" control doesn't seem to affect CD audio(though CD audio reverb wasn't tested...)
  // TODO: If we add sub-sample timing accuracy, see if it's checked for every channel at different times, or just once.
  if(!(SPUControl & 0x4000))
  {
   accum_l = 0;
   accum_r = 0;
   accum_fv_l = 0;
   accum_fv_r = 0;
  }

  // Get CD-DA
  {
   int32 cda_raw[2];
   int32 cdav[2];

   MDFN_IEN_PSX::CDC->GetCDAudio(cda_raw);	// PS_CDC::GetCDAudio() guarantees the variables passed by reference will be set to 0,
				// and that their range shall be -32768 through 32767.

   WriteSPURAM(CWA | 0x000, cda_raw[0]);
   WriteSPURAM(CWA | 0x200, cda_raw[1]);

   for(unsigned i = 0; i < 2; i++)
    cdav[i] = (cda_raw[i] * CDVol[i]) >> 15;

   if(SPUControl & 0x0001)
   {
    accum_l += cdav[0];
    accum_r += cdav[1];

    if(SPUControl & 0x0004)	// TODO: Test this bit(and see if it is really dependent on bit0)
    {
     accum_fv_l += cdav[0];
     accum_fv_r += cdav[1];
    }
   }
  }

  CWA = (CWA + 1) & 0x1FF;

  SPU_RunNoise();

  clamp(&accum_l, -32768, 32767);
  clamp(&accum_r, -32768, 32767);
  clamp(&accum_fv_l, -32768, 32767);
  clamp(&accum_fv_r, -32768, 32767);
  
#if 0
  accum_l = 0;
  accum_r = 0;
  //accum_fv_l = (short)(rand());
  //accum_fv_r = (short)(rand());
#endif

  SPU_RunReverb(accum_fv_l, accum_fv_r, reverb_l, reverb_r);

  //MDFN_DispMessage("%d %d\n", MainVol[0], MainVol[1], ReverbVol[0], ReverbVol[1]);

  // FIXME: Dry volume versus everything else
  output_l = (((accum_l * (int16)GlobalSweep[0].Current) >> 16) + ((reverb_l * ReverbVol[0]) >> 15));
  output_r = (((accum_r * (int16)GlobalSweep[1].Current) >> 16) + ((reverb_r * ReverbVol[1]) >> 15));

  //output_l = reverb_l;
  //output_r = reverb_r;

  clamp(&output_l, -32768, 32767);
  clamp(&output_r, -32768, 32767);

  if(IntermediateBufferPos < 4096)	// Overflow might occur in some debugger use cases.
  {
   IntermediateBuffer[IntermediateBufferPos][0] = output_l;
   IntermediateBuffer[IntermediateBufferPos][1] = output_r;
   IntermediateBufferPos++;
  }

  sample_clocks--;

  // Clock global sweep
  for(int lr = 0; lr < 2; lr++)
   SPU_Sweep_Clock(&GlobalSweep[lr]);
 }

 //assert(clock_divider < 768);

 return clock_divider;
}

#ifdef __cplusplus
extern "C" {
#endif

void SPU_WriteDMA(uint32 V)
{
   //SPUIRQ_DBG("DMA Write, RWAddr after=0x%06x", RWAddr);
   WriteSPURAM(RWAddr, V);
   RWAddr = (RWAddr + 1) & 0x3FFFF;

   WriteSPURAM(RWAddr, V >> 16);
   RWAddr = (RWAddr + 1) & 0x3FFFF;


   CheckIRQAddr(RWAddr);
}

uint32 SPU_ReadDMA(void)
{
 uint32 ret = (uint16)ReadSPURAM(RWAddr);
 RWAddr = (RWAddr + 1) & 0x3FFFF;

 ret |= (uint32)(uint16)ReadSPURAM(RWAddr) << 16;
 RWAddr = (RWAddr + 1) & 0x3FFFF;

 CheckIRQAddr(RWAddr);

 //SPUIRQ_DBG("DMA Read, RWAddr after=0x%06x", RWAddr);

 return(ret);
}

#ifdef __cplusplus
}
#endif


void SPU_Write(int32_t timestamp, uint32 A, uint16 V)
{
 //if((A & 0x3FF) < 0x180)
 // PSX_WARNING("[SPU] Write: %08x %04x", A, V);

 A &= 0x3FF;

 if(A >= 0x200)
 {
  //printf("Write: %08x %04x\n", A, V);
  if(A < 0x260)
  {
   SPU_Voice *voice = &Voices[(A - 0x200) >> 2];
   voice->Sweep[(A & 2) >> 1].Current = V;
  }
  else if(A < 0x280)
   AuxRegs[(A & 0x1F) >> 1] = V;

  return;
 }

 if(A < 0x180)
 {
  SPU_Voice *voice = &Voices[A >> 4];

  switch(A & 0xF)
  {
   case 0x00:
   case 0x02:
	     voice->Sweep[(A & 2) >> 1].Control = V;
	     break;

   case 0x04: voice->Pitch = V;
	      break;

   case 0x06: voice->StartAddr = (V << 2) & 0x3FFFF;
	      break;

   case 0x08: voice->ADSRControl &= 0xFFFF0000;
	      voice->ADSRControl |= V;
	      CacheEnvelope(voice);
	      break;

   case 0x0A: voice->ADSRControl &= 0x0000FFFF;
	      voice->ADSRControl |= V << 16;
	      CacheEnvelope(voice);
	      break;

   case 0x0C: voice->ADSR.EnvLevel = V;
	      break;

   case 0x0E: voice->LoopAddr = (V << 2) & 0x3FFFF;
	      voice->IgnoreSampLA = true;
	      //if((voice - Voices) == 22)
	      //{
	      // SPUIRQ_DBG("Manual loop address setting for voice %d: %04x", (int)(voice - Voices), V);
	      //}
	      break;
  }
 }
 else
 {
  switch(A & 0x7F)
  {
   case 0x00:
   case 0x02:
      GlobalSweep[(A & 2) >> 1].Control = V;
	      break;

   case 0x04: ReverbVol[0] = (int16)V;
	      break;

   case 0x06: ReverbVol[1] = (int16)V;
	      break;

   // Voice ON:
   case 0x08: VoiceOn &= 0xFFFF0000;
	      VoiceOn |= V << 0;
	      break;

   case 0x0a: VoiceOn &= 0x0000FFFF;
              VoiceOn |= (V & 0xFF) << 16;
	      break;

   // Voice OFF:
   case 0x0c: VoiceOff &= 0xFFFF0000;
              VoiceOff |= V << 0;
	      break;

   case 0x0e: VoiceOff &= 0x0000FFFF;
              VoiceOff |= (V & 0xFF) << 16;
	      break;

   case 0x10: FM_Mode &= 0xFFFF0000;
	      FM_Mode |= V << 0;
	      break;

   case 0x12: FM_Mode &= 0x0000FFFF;
	      FM_Mode |= (V & 0xFF) << 16;
	      break;

   case 0x14: Noise_Mode &= 0xFFFF0000;
	      Noise_Mode |= V << 0;
	      break;

   case 0x16: Noise_Mode &= 0x0000FFFF;
	      Noise_Mode |= (V & 0xFF) << 16;
	      break;

   case 0x18: Reverb_Mode &= 0xFFFF0000;
	      Reverb_Mode |= V << 0;
	      break;

   case 0x1A: Reverb_Mode &= 0x0000FFFF;
	      Reverb_Mode |= (V & 0xFF) << 16;
	      break;

   case 0x1C: BlockEnd &= 0xFFFF0000;
	      BlockEnd |= V << 0;
	      break;

   case 0x1E: BlockEnd &= 0x0000FFFF;
	      BlockEnd |= V << 16;
	      break;

   case 0x22: ReverbWA = (V << 2) & 0x3FFFF;
	      ReverbCur = ReverbWA;
	      //PSX_WARNING("[SPU] Reverb WA set: 0x%04x", V);
	      break;

   case 0x24: IRQAddr = (V << 2) & 0x3FFFF;
	      CheckIRQAddr(RWAddr);
	      //SPUIRQ_DBG("Set IRQAddr=0x%06x", IRQAddr);
	      break;

   case 0x26: RWAddr = (V << 2) & 0x3FFFF;	      
	      CheckIRQAddr(RWAddr);
	      //SPUIRQ_DBG("Set RWAddr=0x%06x", RWAddr);
	      break;

   case 0x28: WriteSPURAM(RWAddr, V);
	      RWAddr = (RWAddr + 1) & 0x3FFFF;
	      CheckIRQAddr(RWAddr);
	      break;

   case 0x2A: //if((SPUControl & 0x80) && !(V & 0x80))
	      // printf("\n\n\n\n ************** REVERB PROCESSING DISABLED\n\n\n\n");

	      SPUControl = V;
	      //SPUIRQ_DBG("Set SPUControl=0x%04x -- IRQA=%06x, RWA=%06x", V, IRQAddr, RWAddr);
	      //printf("SPU control write: %04x\n", V);
	      if(!(V & 0x40))
	      {
	       IRQAsserted = false;
	       IRQ_Assert(IRQ_SPU, IRQAsserted);
	      }
	      CheckIRQAddr(RWAddr);
	      break;

   case 0x2C:
         //PSX_WARNING("[SPU] Global reg 0x2c set: 0x%04x", V);
	      break;

   case 0x30: CDVol[0] = V;
	      break;

   case 0x32: CDVol[1] = V;
	      break;

   case 0x34: ExternVol[0] = V;
	      break;

   case 0x36: ExternVol[1] = V;
	      break;

   case 0x38:
   case 0x3A: GlobalSweep[(A & 2) >> 1].Current = V;
	      break;
  }
 }

 unionregs.Regs[(A & 0x1FF) >> 1] = V;
}

uint16 SPU_Read(int32_t timestamp, uint32 A)
{
 //PSX_DBGINFO("[SPU] Read: %08x", A);
 A &= 0x3FF;

 if(A >= 0x200)
 {
  if(A < 0x260)
  {
   SPU_Voice *voice = &Voices[(A - 0x200) >> 2];

   //printf("Read: %08x %04x\n", A, (int16)voice->Sweep[(A & 2) >> 1].Current);

   return voice->Sweep[(A & 2) >> 1].Current;
  }
  else if(A < 0x280)
   return(AuxRegs[(A & 0x1F) >> 1]);

  return(0xFFFF);
 }


 if(A < 0x180)
 {
  SPU_Voice *voice = &Voices[A >> 4];

  switch(A & 0xF)
  {
   case 0x0C: return(voice->ADSR.EnvLevel);
   case 0x0E: return(voice->LoopAddr >> 2);
  }
 }
 else
 {
  switch(A & 0x7F)
  {
   case 0x1C: return(BlockEnd);
   case 0x1E: return(BlockEnd >> 16);

   case 0x26: //PSX_WARNING("[SPU] RWADDR Read");
	      break;

   case 0x28:
	      {
         //PSX_WARNING("[SPU] SPURAM Read port(?) Read");
	       uint16 ret = ReadSPURAM(RWAddr);

	       RWAddr = (RWAddr + 1) & 0x3FFFF;
	       CheckIRQAddr(RWAddr);

	       return(ret);
	      }

   case 0x2a:
	return(SPUControl);

/* FIXME: What is this used for? */
   case 0x3C:
	//PSX_WARNING("[SPU] Read Unknown: %08x", A);
	return(0);

   case 0x38:
   case 0x3A:
   return((int16)GlobalSweep[(A & 2) >> 1].Current);
  }
 }

 return(unionregs.Regs[(A & 0x1FF) >> 1]);
}


void SPU_StartFrame(double rate, uint32 quality)
{
}

int32 SPU_EndFrame(int16 *SoundBuf)
{
   return 0;
}

int SPU_StateAction(StateMem *sm, int load, int data_only)
{
 SFORMAT StateRegs[] =
 {
  { ((&Voices[0].DecodeBuffer[0])), (uint32)(((sizeof(Voices[0].DecodeBuffer) / sizeof(Voices[0].DecodeBuffer[0]))) * sizeof(uint16)), 0x20000000 | 0, "&Voices[0].DecodeBuffer[0]" },
  { &((Voices[0].DecodeM2)), sizeof((Voices[0].DecodeM2)), 0x80000000 | 0, "Voices[0].DecodeM2" },
  { &((Voices[0].DecodeM1)), sizeof((Voices[0].DecodeM1)), 0x80000000 | 0, "Voices[0].DecodeM1" },
  { &((Voices[0].DecodePlayDelay)), sizeof((Voices[0].DecodePlayDelay)), 0x80000000 | 0, "Voices[0].DecodePlayDelay" },
  { &((Voices[0].DecodeWritePos)), sizeof((Voices[0].DecodeWritePos)), 0x80000000 | 0, "Voices[0].DecodeWritePos" },
  { &((Voices[0].DecodeReadPos)), sizeof((Voices[0].DecodeReadPos)), 0x80000000 | 0, "Voices[0].DecodeReadPos" },
  { &((Voices[0].DecodeAvail)), sizeof((Voices[0].DecodeAvail)), 0x80000000 | 0, "Voices[0].DecodeAvail" },
  { &((Voices[0].DecodeShift)), sizeof((Voices[0].DecodeShift)), 0x80000000 | 0, "Voices[0].DecodeShift" },
  { &((Voices[0].DecodeWeight)), sizeof((Voices[0].DecodeWeight)), 0x80000000 | 0, "Voices[0].DecodeWeight" },
  { &((Voices[0].DecodeFlags)), sizeof((Voices[0].DecodeFlags)), 0x80000000 | 0, "Voices[0].DecodeFlags" },
  { &((Voices[0].IgnoreSampLA)), 1, 0x80000000 | 0x08000000, "Voices[0].IgnoreSampLA" },
  { &(((Voices[0].Sweep[0]).Control)), sizeof(((Voices[0].Sweep[0]).Control)), 0x80000000 | 0, "(Voices[0].Sweep[0]).Control" },
  { &(((Voices[0].Sweep[0]).Current)), sizeof(((Voices[0].Sweep[0]).Current)), 0x80000000 | 0, "(Voices[0].Sweep[0]).Current" },
  { &(((Voices[0].Sweep[0]).Divider)), sizeof(((Voices[0].Sweep[0]).Divider)), 0x80000000 | 0, "(Voices[0].Sweep[0]).Divider" },
  { &(((Voices[0].Sweep[1]).Control)), sizeof(((Voices[0].Sweep[1]).Control)), 0x80000000 | 0, "(Voices[0].Sweep[1]).Control" },
  { &(((Voices[0].Sweep[1]).Current)), sizeof(((Voices[0].Sweep[1]).Current)), 0x80000000 | 0, "(Voices[0].Sweep[1]).Current" },
  { &(((Voices[0].Sweep[1]).Divider)), sizeof(((Voices[0].Sweep[1]).Divider)), 0x80000000 | 0, "(Voices[0].Sweep[1]).Divider" },
  { &((Voices[0].Pitch)), sizeof((Voices[0].Pitch)), 0x80000000 | 0, "Voices[0].Pitch" },
  { &((Voices[0].CurPhase)), sizeof((Voices[0].CurPhase)), 0x80000000 | 0, "Voices[0].CurPhase" },
  { &((Voices[0].StartAddr)), sizeof((Voices[0].StartAddr)), 0x80000000 | 0, "Voices[0].StartAddr" },
  { &((Voices[0].CurAddr)), sizeof((Voices[0].CurAddr)), 0x80000000 | 0, "Voices[0].CurAddr" },
  { &((Voices[0].ADSRControl)), sizeof((Voices[0].ADSRControl)), 0x80000000 | 0, "Voices[0].ADSRControl" },
  { &((Voices[0].LoopAddr)), sizeof((Voices[0].LoopAddr)), 0x80000000 | 0, "Voices[0].LoopAddr" },
  { &((Voices[0].PreLRSample)), sizeof((Voices[0].PreLRSample)), 0x80000000 | 0, "Voices[0].PreLRSample" },
  { &((Voices[0].ADSR.EnvLevel)), sizeof((Voices[0].ADSR.EnvLevel)), 0x80000000 | 0, "Voices[0].ADSR.EnvLevel" },
  { &((Voices[0].ADSR.Divider)), sizeof((Voices[0].ADSR.Divider)), 0x80000000 | 0, "Voices[0].ADSR.Divider" },
  { &((Voices[0].ADSR.Phase)), sizeof((Voices[0].ADSR.Phase)), 0x80000000 | 0, "Voices[0].ADSR.Phase" },
  { &((Voices[0].ADSR.AttackExp)), 1, 0x80000000 | 0x08000000, "Voices[0].ADSR.AttackExp" },
  { &((Voices[0].ADSR.SustainExp)), 1, 0x80000000 | 0x08000000 , "Voices[0].ADSR.SustainExp" },
  { &((Voices[0].ADSR.SustainDec)), 1, 0x80000000 | 0x08000000, "Voices[0].ADSR.SustainDec" },
  { &((Voices[0].ADSR.ReleaseExp)), 1, 0x80000000 | 0x08000000, "Voices[0].ADSR.ReleaseExp" },
  { &((Voices[0].ADSR.AttackRate)), sizeof((Voices[0].ADSR.AttackRate)), 0x80000000 | 0, "Voices[0].ADSR.AttackRate" },
  { &((Voices[0].ADSR.DecayRate)), sizeof((Voices[0].ADSR.DecayRate)), 0x80000000 | 0, "Voices[0].ADSR.DecayRate" },
  { &((Voices[0].ADSR.SustainRate)), sizeof((Voices[0].ADSR.SustainRate)), 0x80000000 | 0, "Voices[0].ADSR.SustainRate" },
  { &((Voices[0].ADSR.ReleaseRate)), sizeof((Voices[0].ADSR.ReleaseRate)), 0x80000000 | 0, "Voices[0].ADSR.ReleaseRate" },
  { &((Voices[0].ADSR.SustainLevel)), sizeof((Voices[0].ADSR.SustainLevel)), 0x80000000 | 0, "Voices[0].ADSR.SustainLevel" },
  { ((&Voices[1].DecodeBuffer[0])), (uint32)(((sizeof(Voices[1].DecodeBuffer) / sizeof(Voices[1].DecodeBuffer[0]))) * sizeof(uint16)), 0x20000000 | 0, "&Voices[1].DecodeBuffer[0]" },
  { &((Voices[1].DecodeM2)), sizeof((Voices[1].DecodeM2)), 0x80000000 | 0, "Voices[1].DecodeM2" },
  { &((Voices[1].DecodeM1)), sizeof((Voices[1].DecodeM1)), 0x80000000 | 0, "Voices[1].DecodeM1" },
  { &((Voices[1].DecodePlayDelay)), sizeof((Voices[1].DecodePlayDelay)), 0x80000000 | 0, "Voices[1].DecodePlayDelay" },
  { &((Voices[1].DecodeWritePos)), sizeof((Voices[1].DecodeWritePos)), 0x80000000 | 0, "Voices[1].DecodeWritePos" },
  { &((Voices[1].DecodeReadPos)), sizeof((Voices[1].DecodeReadPos)), 0x80000000 | 0, "Voices[1].DecodeReadPos" },
  { &((Voices[1].DecodeAvail)), sizeof((Voices[1].DecodeAvail)), 0x80000000 | 0, "Voices[1].DecodeAvail" },
  { &((Voices[1].DecodeShift)), sizeof((Voices[1].DecodeShift)), 0x80000000 | 0, "Voices[1].DecodeShift" },
  { &((Voices[1].DecodeWeight)), sizeof((Voices[1].DecodeWeight)), 0x80000000 | 0, "Voices[1].DecodeWeight" },
  { &((Voices[1].DecodeFlags)), sizeof((Voices[1].DecodeFlags)), 0x80000000 | 0, "Voices[1].DecodeFlags" },
  { &((Voices[1].IgnoreSampLA)), 1, 0x80000000 | 0x08000000, "Voices[1].IgnoreSampLA" },
  { &(((Voices[1].Sweep[0]).Control)), SF_IS_BOOL(&(((Voices[1].Sweep[0]).Control))) ? 1 : sizeof(((Voices[1].Sweep[0]).Control)), 0x80000000 | (SF_IS_BOOL(&(((Voices[1].Sweep[0]).Control))) ? 0x08000000 : 0), "(Voices[1].Sweep[0]).Control" },
  { &(((Voices[1].Sweep[0]).Current)), SF_IS_BOOL(&(((Voices[1].Sweep[0]).Current))) ? 1 : sizeof(((Voices[1].Sweep[0]).Current)), 0x80000000 | (SF_IS_BOOL(&(((Voices[1].Sweep[0]).Current))) ? 0x08000000 : 0), "(Voices[1].Sweep[0]).Current" },
  { &(((Voices[1].Sweep[0]).Divider)), SF_IS_BOOL(&(((Voices[1].Sweep[0]).Divider))) ? 1 : sizeof(((Voices[1].Sweep[0]).Divider)), 0x80000000 | (SF_IS_BOOL(&(((Voices[1].Sweep[0]).Divider))) ? 0x08000000 : 0), "(Voices[1].Sweep[0]).Divider" },
  { &(((Voices[1].Sweep[1]).Control)), SF_IS_BOOL(&(((Voices[1].Sweep[1]).Control))) ? 1 : sizeof(((Voices[1].Sweep[1]).Control)), 0x80000000 | (SF_IS_BOOL(&(((Voices[1].Sweep[1]).Control))) ? 0x08000000 : 0), "(Voices[1].Sweep[1]).Control" },
  { &(((Voices[1].Sweep[1]).Current)), SF_IS_BOOL(&(((Voices[1].Sweep[1]).Current))) ? 1 : sizeof(((Voices[1].Sweep[1]).Current)), 0x80000000 | (SF_IS_BOOL(&(((Voices[1].Sweep[1]).Current))) ? 0x08000000 : 0), "(Voices[1].Sweep[1]).Current" },
  { &(((Voices[1].Sweep[1]).Divider)), SF_IS_BOOL(&(((Voices[1].Sweep[1]).Divider))) ? 1 : sizeof(((Voices[1].Sweep[1]).Divider)), 0x80000000 | (SF_IS_BOOL(&(((Voices[1].Sweep[1]).Divider))) ? 0x08000000 : 0), "(Voices[1].Sweep[1]).Divider" },
  { &((Voices[1].Pitch)), SF_IS_BOOL(&((Voices[1].Pitch))) ? 1 : sizeof((Voices[1].Pitch)), 0x80000000 | (SF_IS_BOOL(&((Voices[1].Pitch))) ? 0x08000000 : 0), "Voices[1].Pitch" },
  { &((Voices[1].CurPhase)), SF_IS_BOOL(&((Voices[1].CurPhase))) ? 1 : sizeof((Voices[1].CurPhase)), 0x80000000 | (SF_IS_BOOL(&((Voices[1].CurPhase))) ? 0x08000000 : 0), "Voices[1].CurPhase" },
  { &((Voices[1].StartAddr)), SF_IS_BOOL(&((Voices[1].StartAddr))) ? 1 : sizeof((Voices[1].StartAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[1].StartAddr))) ? 0x08000000 : 0), "Voices[1].StartAddr" },
  { &((Voices[1].CurAddr)), SF_IS_BOOL(&((Voices[1].CurAddr))) ? 1 : sizeof((Voices[1].CurAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[1].CurAddr))) ? 0x08000000 : 0), "Voices[1].CurAddr" },
  { &((Voices[1].ADSRControl)), SF_IS_BOOL(&((Voices[1].ADSRControl))) ? 1 : sizeof((Voices[1].ADSRControl)), 0x80000000 | (SF_IS_BOOL(&((Voices[1].ADSRControl))) ? 0x08000000 : 0), "Voices[1].ADSRControl" },
  { &((Voices[1].LoopAddr)), SF_IS_BOOL(&((Voices[1].LoopAddr))) ? 1 : sizeof((Voices[1].LoopAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[1].LoopAddr))) ? 0x08000000 : 0), "Voices[1].LoopAddr" },
  { &((Voices[1].PreLRSample)), SF_IS_BOOL(&((Voices[1].PreLRSample))) ? 1 : sizeof((Voices[1].PreLRSample)), 0x80000000 | (SF_IS_BOOL(&((Voices[1].PreLRSample))) ? 0x08000000 : 0), "Voices[1].PreLRSample" },
  { &((Voices[1].ADSR.EnvLevel)), SF_IS_BOOL(&((Voices[1].ADSR.EnvLevel))) ? 1 : sizeof((Voices[1].ADSR.EnvLevel)), 0x80000000 | (SF_IS_BOOL(&((Voices[1].ADSR.EnvLevel))) ? 0x08000000 : 0), "Voices[1].ADSR.EnvLevel" },
  { &((Voices[1].ADSR.Divider)), SF_IS_BOOL(&((Voices[1].ADSR.Divider))) ? 1 : sizeof((Voices[1].ADSR.Divider)), 0x80000000 | (SF_IS_BOOL(&((Voices[1].ADSR.Divider))) ? 0x08000000 : 0), "Voices[1].ADSR.Divider" },
  { &((Voices[1].ADSR.Phase)), SF_IS_BOOL(&((Voices[1].ADSR.Phase))) ? 1 : sizeof((Voices[1].ADSR.Phase)), 0x80000000 | (SF_IS_BOOL(&((Voices[1].ADSR.Phase))) ? 0x08000000 : 0), "Voices[1].ADSR.Phase" },
  { &((Voices[1].ADSR.AttackExp)), SF_IS_BOOL(&((Voices[1].ADSR.AttackExp))) ? 1 : sizeof((Voices[1].ADSR.AttackExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[1].ADSR.AttackExp))) ? 0x08000000 : 0), "Voices[1].ADSR.AttackExp" },
  { &((Voices[1].ADSR.SustainExp)), SF_IS_BOOL(&((Voices[1].ADSR.SustainExp))) ? 1 : sizeof((Voices[1].ADSR.SustainExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[1].ADSR.SustainExp))) ? 0x08000000 : 0), "Voices[1].ADSR.SustainExp" },
  { &((Voices[1].ADSR.SustainDec)), SF_IS_BOOL(&((Voices[1].ADSR.SustainDec))) ? 1 : sizeof((Voices[1].ADSR.SustainDec)), 0x80000000 | (SF_IS_BOOL(&((Voices[1].ADSR.SustainDec))) ? 0x08000000 : 0), "Voices[1].ADSR.SustainDec" },
  { &((Voices[1].ADSR.ReleaseExp)), SF_IS_BOOL(&((Voices[1].ADSR.ReleaseExp))) ? 1 : sizeof((Voices[1].ADSR.ReleaseExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[1].ADSR.ReleaseExp))) ? 0x08000000 : 0), "Voices[1].ADSR.ReleaseExp" },
  { &((Voices[1].ADSR.AttackRate)), SF_IS_BOOL(&((Voices[1].ADSR.AttackRate))) ? 1 : sizeof((Voices[1].ADSR.AttackRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[1].ADSR.AttackRate))) ? 0x08000000 : 0), "Voices[1].ADSR.AttackRate" },
  { &((Voices[1].ADSR.DecayRate)), SF_IS_BOOL(&((Voices[1].ADSR.DecayRate))) ? 1 : sizeof((Voices[1].ADSR.DecayRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[1].ADSR.DecayRate))) ? 0x08000000 : 0), "Voices[1].ADSR.DecayRate" },
  { &((Voices[1].ADSR.SustainRate)), SF_IS_BOOL(&((Voices[1].ADSR.SustainRate))) ? 1 : sizeof((Voices[1].ADSR.SustainRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[1].ADSR.SustainRate))) ? 0x08000000 : 0), "Voices[1].ADSR.SustainRate" },
  { &((Voices[1].ADSR.ReleaseRate)), SF_IS_BOOL(&((Voices[1].ADSR.ReleaseRate))) ? 1 : sizeof((Voices[1].ADSR.ReleaseRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[1].ADSR.ReleaseRate))) ? 0x08000000 : 0), "Voices[1].ADSR.ReleaseRate" },
     { &((Voices[1].ADSR.SustainLevel)), SF_IS_BOOL(&((Voices[1].ADSR.SustainLevel))) ? 1 : sizeof((Voices[1].ADSR.SustainLevel)), 0x80000000 | (SF_IS_BOOL(&((Voices[1].ADSR.SustainLevel))) ? 0x08000000 : 0), "Voices[1].ADSR.SustainLevel" },
  { ((&Voices[2].DecodeBuffer[0])), (uint32)(((sizeof(Voices[2].DecodeBuffer) / sizeof(Voices[2].DecodeBuffer[0]))) * sizeof(uint16)), 0x20000000 | SF_FORCE_A16((&Voices[2].DecodeBuffer[0])), "&Voices[2].DecodeBuffer[0]" },
  { &((Voices[2].DecodeM2)), SF_IS_BOOL(&((Voices[2].DecodeM2))) ? 1 : sizeof((Voices[2].DecodeM2)), 0x80000000 | (SF_IS_BOOL(&((Voices[2].DecodeM2))) ? 0x08000000 : 0), "Voices[2].DecodeM2" },
  { &((Voices[2].DecodeM1)), SF_IS_BOOL(&((Voices[2].DecodeM1))) ? 1 : sizeof((Voices[2].DecodeM1)), 0x80000000 | (SF_IS_BOOL(&((Voices[2].DecodeM1))) ? 0x08000000 : 0), "Voices[2].DecodeM1" },
  { &((Voices[2].DecodePlayDelay)), SF_IS_BOOL(&((Voices[2].DecodePlayDelay))) ? 1 : sizeof((Voices[2].DecodePlayDelay)), 0x80000000 | (SF_IS_BOOL(&((Voices[2].DecodePlayDelay))) ? 0x08000000 : 0), "Voices[2].DecodePlayDelay" },
  { &((Voices[2].DecodeWritePos)), SF_IS_BOOL(&((Voices[2].DecodeWritePos))) ? 1 : sizeof((Voices[2].DecodeWritePos)), 0x80000000 | (SF_IS_BOOL(&((Voices[2].DecodeWritePos))) ? 0x08000000 : 0), "Voices[2].DecodeWritePos" },
  { &((Voices[2].DecodeReadPos)), SF_IS_BOOL(&((Voices[2].DecodeReadPos))) ? 1 : sizeof((Voices[2].DecodeReadPos)), 0x80000000 | (SF_IS_BOOL(&((Voices[2].DecodeReadPos))) ? 0x08000000 : 0), "Voices[2].DecodeReadPos" },
  { &((Voices[2].DecodeAvail)), SF_IS_BOOL(&((Voices[2].DecodeAvail))) ? 1 : sizeof((Voices[2].DecodeAvail)), 0x80000000 | (SF_IS_BOOL(&((Voices[2].DecodeAvail))) ? 0x08000000 : 0), "Voices[2].DecodeAvail" },
  { &((Voices[2].DecodeShift)), SF_IS_BOOL(&((Voices[2].DecodeShift))) ? 1 : sizeof((Voices[2].DecodeShift)), 0x80000000 | (SF_IS_BOOL(&((Voices[2].DecodeShift))) ? 0x08000000 : 0), "Voices[2].DecodeShift" },
  { &((Voices[2].DecodeWeight)), SF_IS_BOOL(&((Voices[2].DecodeWeight))) ? 1 : sizeof((Voices[2].DecodeWeight)), 0x80000000 | (SF_IS_BOOL(&((Voices[2].DecodeWeight))) ? 0x08000000 : 0), "Voices[2].DecodeWeight" },
  { &((Voices[2].DecodeFlags)), SF_IS_BOOL(&((Voices[2].DecodeFlags))) ? 1 : sizeof((Voices[2].DecodeFlags)), 0x80000000 | (SF_IS_BOOL(&((Voices[2].DecodeFlags))) ? 0x08000000 : 0), "Voices[2].DecodeFlags" },
  { &((Voices[2].IgnoreSampLA)), SF_IS_BOOL(&((Voices[2].IgnoreSampLA))) ? 1 : sizeof((Voices[2].IgnoreSampLA)), 0x80000000 | (SF_IS_BOOL(&((Voices[2].IgnoreSampLA))) ? 0x08000000 : 0), "Voices[2].IgnoreSampLA" },
  { &(((Voices[2].Sweep[0]).Control)), SF_IS_BOOL(&(((Voices[2].Sweep[0]).Control))) ? 1 : sizeof(((Voices[2].Sweep[0]).Control)), 0x80000000 | (SF_IS_BOOL(&(((Voices[2].Sweep[0]).Control))) ? 0x08000000 : 0), "(Voices[2].Sweep[0]).Control" },
  { &(((Voices[2].Sweep[0]).Current)), SF_IS_BOOL(&(((Voices[2].Sweep[0]).Current))) ? 1 : sizeof(((Voices[2].Sweep[0]).Current)), 0x80000000 | (SF_IS_BOOL(&(((Voices[2].Sweep[0]).Current))) ? 0x08000000 : 0), "(Voices[2].Sweep[0]).Current" },
  { &(((Voices[2].Sweep[0]).Divider)), SF_IS_BOOL(&(((Voices[2].Sweep[0]).Divider))) ? 1 : sizeof(((Voices[2].Sweep[0]).Divider)), 0x80000000 | (SF_IS_BOOL(&(((Voices[2].Sweep[0]).Divider))) ? 0x08000000 : 0), "(Voices[2].Sweep[0]).Divider" },
  { &(((Voices[2].Sweep[1]).Control)), SF_IS_BOOL(&(((Voices[2].Sweep[1]).Control))) ? 1 : sizeof(((Voices[2].Sweep[1]).Control)), 0x80000000 | (SF_IS_BOOL(&(((Voices[2].Sweep[1]).Control))) ? 0x08000000 : 0), "(Voices[2].Sweep[1]).Control" },
  { &(((Voices[2].Sweep[1]).Current)), SF_IS_BOOL(&(((Voices[2].Sweep[1]).Current))) ? 1 : sizeof(((Voices[2].Sweep[1]).Current)), 0x80000000 | (SF_IS_BOOL(&(((Voices[2].Sweep[1]).Current))) ? 0x08000000 : 0), "(Voices[2].Sweep[1]).Current" },
  { &(((Voices[2].Sweep[1]).Divider)), SF_IS_BOOL(&(((Voices[2].Sweep[1]).Divider))) ? 1 : sizeof(((Voices[2].Sweep[1]).Divider)), 0x80000000 | (SF_IS_BOOL(&(((Voices[2].Sweep[1]).Divider))) ? 0x08000000 : 0), "(Voices[2].Sweep[1]).Divider" },
  { &((Voices[2].Pitch)), SF_IS_BOOL(&((Voices[2].Pitch))) ? 1 : sizeof((Voices[2].Pitch)), 0x80000000 | (SF_IS_BOOL(&((Voices[2].Pitch))) ? 0x08000000 : 0), "Voices[2].Pitch" },
  { &((Voices[2].CurPhase)), SF_IS_BOOL(&((Voices[2].CurPhase))) ? 1 : sizeof((Voices[2].CurPhase)), 0x80000000 | (SF_IS_BOOL(&((Voices[2].CurPhase))) ? 0x08000000 : 0), "Voices[2].CurPhase" },
  { &((Voices[2].StartAddr)), SF_IS_BOOL(&((Voices[2].StartAddr))) ? 1 : sizeof((Voices[2].StartAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[2].StartAddr))) ? 0x08000000 : 0), "Voices[2].StartAddr" },
  { &((Voices[2].CurAddr)), SF_IS_BOOL(&((Voices[2].CurAddr))) ? 1 : sizeof((Voices[2].CurAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[2].CurAddr))) ? 0x08000000 : 0), "Voices[2].CurAddr" },
  { &((Voices[2].ADSRControl)), SF_IS_BOOL(&((Voices[2].ADSRControl))) ? 1 : sizeof((Voices[2].ADSRControl)), 0x80000000 | (SF_IS_BOOL(&((Voices[2].ADSRControl))) ? 0x08000000 : 0), "Voices[2].ADSRControl" },
  { &((Voices[2].LoopAddr)), SF_IS_BOOL(&((Voices[2].LoopAddr))) ? 1 : sizeof((Voices[2].LoopAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[2].LoopAddr))) ? 0x08000000 : 0), "Voices[2].LoopAddr" },
  { &((Voices[2].PreLRSample)), SF_IS_BOOL(&((Voices[2].PreLRSample))) ? 1 : sizeof((Voices[2].PreLRSample)), 0x80000000 | (SF_IS_BOOL(&((Voices[2].PreLRSample))) ? 0x08000000 : 0), "Voices[2].PreLRSample" },
  { &((Voices[2].ADSR.EnvLevel)), SF_IS_BOOL(&((Voices[2].ADSR.EnvLevel))) ? 1 : sizeof((Voices[2].ADSR.EnvLevel)), 0x80000000 | (SF_IS_BOOL(&((Voices[2].ADSR.EnvLevel))) ? 0x08000000 : 0), "Voices[2].ADSR.EnvLevel" },
  { &((Voices[2].ADSR.Divider)), SF_IS_BOOL(&((Voices[2].ADSR.Divider))) ? 1 : sizeof((Voices[2].ADSR.Divider)), 0x80000000 | (SF_IS_BOOL(&((Voices[2].ADSR.Divider))) ? 0x08000000 : 0), "Voices[2].ADSR.Divider" },
  { &((Voices[2].ADSR.Phase)), SF_IS_BOOL(&((Voices[2].ADSR.Phase))) ? 1 : sizeof((Voices[2].ADSR.Phase)), 0x80000000 | (SF_IS_BOOL(&((Voices[2].ADSR.Phase))) ? 0x08000000 : 0), "Voices[2].ADSR.Phase" },
  { &((Voices[2].ADSR.AttackExp)), SF_IS_BOOL(&((Voices[2].ADSR.AttackExp))) ? 1 : sizeof((Voices[2].ADSR.AttackExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[2].ADSR.AttackExp))) ? 0x08000000 : 0), "Voices[2].ADSR.AttackExp" },
  { &((Voices[2].ADSR.SustainExp)), SF_IS_BOOL(&((Voices[2].ADSR.SustainExp))) ? 1 : sizeof((Voices[2].ADSR.SustainExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[2].ADSR.SustainExp))) ? 0x08000000 : 0), "Voices[2].ADSR.SustainExp" },
  { &((Voices[2].ADSR.SustainDec)), SF_IS_BOOL(&((Voices[2].ADSR.SustainDec))) ? 1 : sizeof((Voices[2].ADSR.SustainDec)), 0x80000000 | (SF_IS_BOOL(&((Voices[2].ADSR.SustainDec))) ? 0x08000000 : 0), "Voices[2].ADSR.SustainDec" },
  { &((Voices[2].ADSR.ReleaseExp)), SF_IS_BOOL(&((Voices[2].ADSR.ReleaseExp))) ? 1 : sizeof((Voices[2].ADSR.ReleaseExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[2].ADSR.ReleaseExp))) ? 0x08000000 : 0), "Voices[2].ADSR.ReleaseExp" },
  { &((Voices[2].ADSR.AttackRate)), SF_IS_BOOL(&((Voices[2].ADSR.AttackRate))) ? 1 : sizeof((Voices[2].ADSR.AttackRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[2].ADSR.AttackRate))) ? 0x08000000 : 0), "Voices[2].ADSR.AttackRate" },
  { &((Voices[2].ADSR.DecayRate)), SF_IS_BOOL(&((Voices[2].ADSR.DecayRate))) ? 1 : sizeof((Voices[2].ADSR.DecayRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[2].ADSR.DecayRate))) ? 0x08000000 : 0), "Voices[2].ADSR.DecayRate" },
  { &((Voices[2].ADSR.SustainRate)), SF_IS_BOOL(&((Voices[2].ADSR.SustainRate))) ? 1 : sizeof((Voices[2].ADSR.SustainRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[2].ADSR.SustainRate))) ? 0x08000000 : 0), "Voices[2].ADSR.SustainRate" },
  { &((Voices[2].ADSR.ReleaseRate)), SF_IS_BOOL(&((Voices[2].ADSR.ReleaseRate))) ? 1 : sizeof((Voices[2].ADSR.ReleaseRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[2].ADSR.ReleaseRate))) ? 0x08000000 : 0), "Voices[2].ADSR.ReleaseRate" },
  { &((Voices[2].ADSR.SustainLevel)), SF_IS_BOOL(&((Voices[2].ADSR.SustainLevel))) ? 1 : sizeof((Voices[2].ADSR.SustainLevel)), 0x80000000 | (SF_IS_BOOL(&((Voices[2].ADSR.SustainLevel))) ? 0x08000000 : 0), "Voices[2].ADSR.SustainLevel" },
  { ((&Voices[3].DecodeBuffer[0])), (uint32)(((sizeof(Voices[3].DecodeBuffer) / sizeof(Voices[3].DecodeBuffer[0]))) * sizeof(uint16)), 0x20000000 | SF_FORCE_A16((&Voices[3].DecodeBuffer[0])), "&Voices[3].DecodeBuffer[0]" },
  { &((Voices[3].DecodeM2)), SF_IS_BOOL(&((Voices[3].DecodeM2))) ? 1 : sizeof((Voices[3].DecodeM2)), 0x80000000 | (SF_IS_BOOL(&((Voices[3].DecodeM2))) ? 0x08000000 : 0), "Voices[3].DecodeM2" },
  { &((Voices[3].DecodeM1)), SF_IS_BOOL(&((Voices[3].DecodeM1))) ? 1 : sizeof((Voices[3].DecodeM1)), 0x80000000 | (SF_IS_BOOL(&((Voices[3].DecodeM1))) ? 0x08000000 : 0), "Voices[3].DecodeM1" },
  { &((Voices[3].DecodePlayDelay)), SF_IS_BOOL(&((Voices[3].DecodePlayDelay))) ? 1 : sizeof((Voices[3].DecodePlayDelay)), 0x80000000 | (SF_IS_BOOL(&((Voices[3].DecodePlayDelay))) ? 0x08000000 : 0), "Voices[3].DecodePlayDelay" },
  { &((Voices[3].DecodeWritePos)), SF_IS_BOOL(&((Voices[3].DecodeWritePos))) ? 1 : sizeof((Voices[3].DecodeWritePos)), 0x80000000 | (SF_IS_BOOL(&((Voices[3].DecodeWritePos))) ? 0x08000000 : 0), "Voices[3].DecodeWritePos" },
  { &((Voices[3].DecodeReadPos)), SF_IS_BOOL(&((Voices[3].DecodeReadPos))) ? 1 : sizeof((Voices[3].DecodeReadPos)), 0x80000000 | (SF_IS_BOOL(&((Voices[3].DecodeReadPos))) ? 0x08000000 : 0), "Voices[3].DecodeReadPos" },
  { &((Voices[3].DecodeAvail)), SF_IS_BOOL(&((Voices[3].DecodeAvail))) ? 1 : sizeof((Voices[3].DecodeAvail)), 0x80000000 | (SF_IS_BOOL(&((Voices[3].DecodeAvail))) ? 0x08000000 : 0), "Voices[3].DecodeAvail" },
  { &((Voices[3].DecodeShift)), SF_IS_BOOL(&((Voices[3].DecodeShift))) ? 1 : sizeof((Voices[3].DecodeShift)), 0x80000000 | (SF_IS_BOOL(&((Voices[3].DecodeShift))) ? 0x08000000 : 0), "Voices[3].DecodeShift" },
  { &((Voices[3].DecodeWeight)), SF_IS_BOOL(&((Voices[3].DecodeWeight))) ? 1 : sizeof((Voices[3].DecodeWeight)), 0x80000000 | (SF_IS_BOOL(&((Voices[3].DecodeWeight))) ? 0x08000000 : 0), "Voices[3].DecodeWeight" },
  { &((Voices[3].DecodeFlags)), SF_IS_BOOL(&((Voices[3].DecodeFlags))) ? 1 : sizeof((Voices[3].DecodeFlags)), 0x80000000 | (SF_IS_BOOL(&((Voices[3].DecodeFlags))) ? 0x08000000 : 0), "Voices[3].DecodeFlags" },
  { &((Voices[3].IgnoreSampLA)), SF_IS_BOOL(&((Voices[3].IgnoreSampLA))) ? 1 : sizeof((Voices[3].IgnoreSampLA)), 0x80000000 | (SF_IS_BOOL(&((Voices[3].IgnoreSampLA))) ? 0x08000000 : 0), "Voices[3].IgnoreSampLA" },
  { &(((Voices[3].Sweep[0]).Control)), SF_IS_BOOL(&(((Voices[3].Sweep[0]).Control))) ? 1 : sizeof(((Voices[3].Sweep[0]).Control)), 0x80000000 | (SF_IS_BOOL(&(((Voices[3].Sweep[0]).Control))) ? 0x08000000 : 0), "(Voices[3].Sweep[0]).Control" },
  { &(((Voices[3].Sweep[0]).Current)), SF_IS_BOOL(&(((Voices[3].Sweep[0]).Current))) ? 1 : sizeof(((Voices[3].Sweep[0]).Current)), 0x80000000 | (SF_IS_BOOL(&(((Voices[3].Sweep[0]).Current))) ? 0x08000000 : 0), "(Voices[3].Sweep[0]).Current" },
  { &(((Voices[3].Sweep[0]).Divider)), SF_IS_BOOL(&(((Voices[3].Sweep[0]).Divider))) ? 1 : sizeof(((Voices[3].Sweep[0]).Divider)), 0x80000000 | (SF_IS_BOOL(&(((Voices[3].Sweep[0]).Divider))) ? 0x08000000 : 0), "(Voices[3].Sweep[0]).Divider" },
  { &(((Voices[3].Sweep[1]).Control)), SF_IS_BOOL(&(((Voices[3].Sweep[1]).Control))) ? 1 : sizeof(((Voices[3].Sweep[1]).Control)), 0x80000000 | (SF_IS_BOOL(&(((Voices[3].Sweep[1]).Control))) ? 0x08000000 : 0), "(Voices[3].Sweep[1]).Control" },
  { &(((Voices[3].Sweep[1]).Current)), SF_IS_BOOL(&(((Voices[3].Sweep[1]).Current))) ? 1 : sizeof(((Voices[3].Sweep[1]).Current)), 0x80000000 | (SF_IS_BOOL(&(((Voices[3].Sweep[1]).Current))) ? 0x08000000 : 0), "(Voices[3].Sweep[1]).Current" },
  { &(((Voices[3].Sweep[1]).Divider)), SF_IS_BOOL(&(((Voices[3].Sweep[1]).Divider))) ? 1 : sizeof(((Voices[3].Sweep[1]).Divider)), 0x80000000 | (SF_IS_BOOL(&(((Voices[3].Sweep[1]).Divider))) ? 0x08000000 : 0), "(Voices[3].Sweep[1]).Divider" },
  { &((Voices[3].Pitch)), SF_IS_BOOL(&((Voices[3].Pitch))) ? 1 : sizeof((Voices[3].Pitch)), 0x80000000 | (SF_IS_BOOL(&((Voices[3].Pitch))) ? 0x08000000 : 0), "Voices[3].Pitch" },
  { &((Voices[3].CurPhase)), SF_IS_BOOL(&((Voices[3].CurPhase))) ? 1 : sizeof((Voices[3].CurPhase)), 0x80000000 | (SF_IS_BOOL(&((Voices[3].CurPhase))) ? 0x08000000 : 0), "Voices[3].CurPhase" },
  { &((Voices[3].StartAddr)), SF_IS_BOOL(&((Voices[3].StartAddr))) ? 1 : sizeof((Voices[3].StartAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[3].StartAddr))) ? 0x08000000 : 0), "Voices[3].StartAddr" },
  { &((Voices[3].CurAddr)), SF_IS_BOOL(&((Voices[3].CurAddr))) ? 1 : sizeof((Voices[3].CurAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[3].CurAddr))) ? 0x08000000 : 0), "Voices[3].CurAddr" },
  { &((Voices[3].ADSRControl)), SF_IS_BOOL(&((Voices[3].ADSRControl))) ? 1 : sizeof((Voices[3].ADSRControl)), 0x80000000 | (SF_IS_BOOL(&((Voices[3].ADSRControl))) ? 0x08000000 : 0), "Voices[3].ADSRControl" },
  { &((Voices[3].LoopAddr)), SF_IS_BOOL(&((Voices[3].LoopAddr))) ? 1 : sizeof((Voices[3].LoopAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[3].LoopAddr))) ? 0x08000000 : 0), "Voices[3].LoopAddr" },
  { &((Voices[3].PreLRSample)), SF_IS_BOOL(&((Voices[3].PreLRSample))) ? 1 : sizeof((Voices[3].PreLRSample)), 0x80000000 | (SF_IS_BOOL(&((Voices[3].PreLRSample))) ? 0x08000000 : 0), "Voices[3].PreLRSample" },
  { &((Voices[3].ADSR.EnvLevel)), SF_IS_BOOL(&((Voices[3].ADSR.EnvLevel))) ? 1 : sizeof((Voices[3].ADSR.EnvLevel)), 0x80000000 | (SF_IS_BOOL(&((Voices[3].ADSR.EnvLevel))) ? 0x08000000 : 0), "Voices[3].ADSR.EnvLevel" },
  { &((Voices[3].ADSR.Divider)), SF_IS_BOOL(&((Voices[3].ADSR.Divider))) ? 1 : sizeof((Voices[3].ADSR.Divider)), 0x80000000 | (SF_IS_BOOL(&((Voices[3].ADSR.Divider))) ? 0x08000000 : 0), "Voices[3].ADSR.Divider" },
  { &((Voices[3].ADSR.Phase)), SF_IS_BOOL(&((Voices[3].ADSR.Phase))) ? 1 : sizeof((Voices[3].ADSR.Phase)), 0x80000000 | (SF_IS_BOOL(&((Voices[3].ADSR.Phase))) ? 0x08000000 : 0), "Voices[3].ADSR.Phase" },
  { &((Voices[3].ADSR.AttackExp)), SF_IS_BOOL(&((Voices[3].ADSR.AttackExp))) ? 1 : sizeof((Voices[3].ADSR.AttackExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[3].ADSR.AttackExp))) ? 0x08000000 : 0), "Voices[3].ADSR.AttackExp" },
  { &((Voices[3].ADSR.SustainExp)), SF_IS_BOOL(&((Voices[3].ADSR.SustainExp))) ? 1 : sizeof((Voices[3].ADSR.SustainExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[3].ADSR.SustainExp))) ? 0x08000000 : 0), "Voices[3].ADSR.SustainExp" },
  { &((Voices[3].ADSR.SustainDec)), SF_IS_BOOL(&((Voices[3].ADSR.SustainDec))) ? 1 : sizeof((Voices[3].ADSR.SustainDec)), 0x80000000 | (SF_IS_BOOL(&((Voices[3].ADSR.SustainDec))) ? 0x08000000 : 0), "Voices[3].ADSR.SustainDec" },
  { &((Voices[3].ADSR.ReleaseExp)), SF_IS_BOOL(&((Voices[3].ADSR.ReleaseExp))) ? 1 : sizeof((Voices[3].ADSR.ReleaseExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[3].ADSR.ReleaseExp))) ? 0x08000000 : 0), "Voices[3].ADSR.ReleaseExp" },
  { &((Voices[3].ADSR.AttackRate)), SF_IS_BOOL(&((Voices[3].ADSR.AttackRate))) ? 1 : sizeof((Voices[3].ADSR.AttackRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[3].ADSR.AttackRate))) ? 0x08000000 : 0), "Voices[3].ADSR.AttackRate" },
  { &((Voices[3].ADSR.DecayRate)), SF_IS_BOOL(&((Voices[3].ADSR.DecayRate))) ? 1 : sizeof((Voices[3].ADSR.DecayRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[3].ADSR.DecayRate))) ? 0x08000000 : 0), "Voices[3].ADSR.DecayRate" },
  { &((Voices[3].ADSR.SustainRate)), SF_IS_BOOL(&((Voices[3].ADSR.SustainRate))) ? 1 : sizeof((Voices[3].ADSR.SustainRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[3].ADSR.SustainRate))) ? 0x08000000 : 0), "Voices[3].ADSR.SustainRate" },
  { &((Voices[3].ADSR.ReleaseRate)), SF_IS_BOOL(&((Voices[3].ADSR.ReleaseRate))) ? 1 : sizeof((Voices[3].ADSR.ReleaseRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[3].ADSR.ReleaseRate))) ? 0x08000000 : 0), "Voices[3].ADSR.ReleaseRate" },
  { &((Voices[3].ADSR.SustainLevel)), SF_IS_BOOL(&((Voices[3].ADSR.SustainLevel))) ? 1 : sizeof((Voices[3].ADSR.SustainLevel)), 0x80000000 | (SF_IS_BOOL(&((Voices[3].ADSR.SustainLevel))) ? 0x08000000 : 0), "Voices[3].ADSR.SustainLevel" },
  { ((&Voices[4].DecodeBuffer[0])), (uint32)(((sizeof(Voices[4].DecodeBuffer) / sizeof(Voices[4].DecodeBuffer[0]))) * sizeof(uint16)), 0x20000000 | SF_FORCE_A16((&Voices[4].DecodeBuffer[0])), "&Voices[4].DecodeBuffer[0]" },
  { &((Voices[4].DecodeM2)), SF_IS_BOOL(&((Voices[4].DecodeM2))) ? 1 : sizeof((Voices[4].DecodeM2)), 0x80000000 | (SF_IS_BOOL(&((Voices[4].DecodeM2))) ? 0x08000000 : 0), "Voices[4].DecodeM2" },
  { &((Voices[4].DecodeM1)), SF_IS_BOOL(&((Voices[4].DecodeM1))) ? 1 : sizeof((Voices[4].DecodeM1)), 0x80000000 | (SF_IS_BOOL(&((Voices[4].DecodeM1))) ? 0x08000000 : 0), "Voices[4].DecodeM1" },
  { &((Voices[4].DecodePlayDelay)), SF_IS_BOOL(&((Voices[4].DecodePlayDelay))) ? 1 : sizeof((Voices[4].DecodePlayDelay)), 0x80000000 | (SF_IS_BOOL(&((Voices[4].DecodePlayDelay))) ? 0x08000000 : 0), "Voices[4].DecodePlayDelay" },
  { &((Voices[4].DecodeWritePos)), SF_IS_BOOL(&((Voices[4].DecodeWritePos))) ? 1 : sizeof((Voices[4].DecodeWritePos)), 0x80000000 | (SF_IS_BOOL(&((Voices[4].DecodeWritePos))) ? 0x08000000 : 0), "Voices[4].DecodeWritePos" },
  { &((Voices[4].DecodeReadPos)), SF_IS_BOOL(&((Voices[4].DecodeReadPos))) ? 1 : sizeof((Voices[4].DecodeReadPos)), 0x80000000 | (SF_IS_BOOL(&((Voices[4].DecodeReadPos))) ? 0x08000000 : 0), "Voices[4].DecodeReadPos" },
  { &((Voices[4].DecodeAvail)), SF_IS_BOOL(&((Voices[4].DecodeAvail))) ? 1 : sizeof((Voices[4].DecodeAvail)), 0x80000000 | (SF_IS_BOOL(&((Voices[4].DecodeAvail))) ? 0x08000000 : 0), "Voices[4].DecodeAvail" },
  { &((Voices[4].DecodeShift)), SF_IS_BOOL(&((Voices[4].DecodeShift))) ? 1 : sizeof((Voices[4].DecodeShift)), 0x80000000 | (SF_IS_BOOL(&((Voices[4].DecodeShift))) ? 0x08000000 : 0), "Voices[4].DecodeShift" },
  { &((Voices[4].DecodeWeight)), SF_IS_BOOL(&((Voices[4].DecodeWeight))) ? 1 : sizeof((Voices[4].DecodeWeight)), 0x80000000 | (SF_IS_BOOL(&((Voices[4].DecodeWeight))) ? 0x08000000 : 0), "Voices[4].DecodeWeight" },
  { &((Voices[4].DecodeFlags)), SF_IS_BOOL(&((Voices[4].DecodeFlags))) ? 1 : sizeof((Voices[4].DecodeFlags)), 0x80000000 | (SF_IS_BOOL(&((Voices[4].DecodeFlags))) ? 0x08000000 : 0), "Voices[4].DecodeFlags" },
  { &((Voices[4].IgnoreSampLA)), SF_IS_BOOL(&((Voices[4].IgnoreSampLA))) ? 1 : sizeof((Voices[4].IgnoreSampLA)), 0x80000000 | (SF_IS_BOOL(&((Voices[4].IgnoreSampLA))) ? 0x08000000 : 0), "Voices[4].IgnoreSampLA" },
  { &(((Voices[4].Sweep[0]).Control)), SF_IS_BOOL(&(((Voices[4].Sweep[0]).Control))) ? 1 : sizeof(((Voices[4].Sweep[0]).Control)), 0x80000000 | (SF_IS_BOOL(&(((Voices[4].Sweep[0]).Control))) ? 0x08000000 : 0), "(Voices[4].Sweep[0]).Control" },
  { &(((Voices[4].Sweep[0]).Current)), SF_IS_BOOL(&(((Voices[4].Sweep[0]).Current))) ? 1 : sizeof(((Voices[4].Sweep[0]).Current)), 0x80000000 | (SF_IS_BOOL(&(((Voices[4].Sweep[0]).Current))) ? 0x08000000 : 0), "(Voices[4].Sweep[0]).Current" },
  { &(((Voices[4].Sweep[0]).Divider)), SF_IS_BOOL(&(((Voices[4].Sweep[0]).Divider))) ? 1 : sizeof(((Voices[4].Sweep[0]).Divider)), 0x80000000 | (SF_IS_BOOL(&(((Voices[4].Sweep[0]).Divider))) ? 0x08000000 : 0), "(Voices[4].Sweep[0]).Divider" },
  { &(((Voices[4].Sweep[1]).Control)), SF_IS_BOOL(&(((Voices[4].Sweep[1]).Control))) ? 1 : sizeof(((Voices[4].Sweep[1]).Control)), 0x80000000 | (SF_IS_BOOL(&(((Voices[4].Sweep[1]).Control))) ? 0x08000000 : 0), "(Voices[4].Sweep[1]).Control" },
  { &(((Voices[4].Sweep[1]).Current)), SF_IS_BOOL(&(((Voices[4].Sweep[1]).Current))) ? 1 : sizeof(((Voices[4].Sweep[1]).Current)), 0x80000000 | (SF_IS_BOOL(&(((Voices[4].Sweep[1]).Current))) ? 0x08000000 : 0), "(Voices[4].Sweep[1]).Current" },
  { &(((Voices[4].Sweep[1]).Divider)), SF_IS_BOOL(&(((Voices[4].Sweep[1]).Divider))) ? 1 : sizeof(((Voices[4].Sweep[1]).Divider)), 0x80000000 | (SF_IS_BOOL(&(((Voices[4].Sweep[1]).Divider))) ? 0x08000000 : 0), "(Voices[4].Sweep[1]).Divider" },
  { &((Voices[4].Pitch)), SF_IS_BOOL(&((Voices[4].Pitch))) ? 1 : sizeof((Voices[4].Pitch)), 0x80000000 | (SF_IS_BOOL(&((Voices[4].Pitch))) ? 0x08000000 : 0), "Voices[4].Pitch" },
  { &((Voices[4].CurPhase)), SF_IS_BOOL(&((Voices[4].CurPhase))) ? 1 : sizeof((Voices[4].CurPhase)), 0x80000000 | (SF_IS_BOOL(&((Voices[4].CurPhase))) ? 0x08000000 : 0), "Voices[4].CurPhase" },
  { &((Voices[4].StartAddr)), SF_IS_BOOL(&((Voices[4].StartAddr))) ? 1 : sizeof((Voices[4].StartAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[4].StartAddr))) ? 0x08000000 : 0), "Voices[4].StartAddr" },
  { &((Voices[4].CurAddr)), SF_IS_BOOL(&((Voices[4].CurAddr))) ? 1 : sizeof((Voices[4].CurAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[4].CurAddr))) ? 0x08000000 : 0), "Voices[4].CurAddr" },
  { &((Voices[4].ADSRControl)), SF_IS_BOOL(&((Voices[4].ADSRControl))) ? 1 : sizeof((Voices[4].ADSRControl)), 0x80000000 | (SF_IS_BOOL(&((Voices[4].ADSRControl))) ? 0x08000000 : 0), "Voices[4].ADSRControl" },
  { &((Voices[4].LoopAddr)), SF_IS_BOOL(&((Voices[4].LoopAddr))) ? 1 : sizeof((Voices[4].LoopAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[4].LoopAddr))) ? 0x08000000 : 0), "Voices[4].LoopAddr" },
  { &((Voices[4].PreLRSample)), SF_IS_BOOL(&((Voices[4].PreLRSample))) ? 1 : sizeof((Voices[4].PreLRSample)), 0x80000000 | (SF_IS_BOOL(&((Voices[4].PreLRSample))) ? 0x08000000 : 0), "Voices[4].PreLRSample" },
  { &((Voices[4].ADSR.EnvLevel)), SF_IS_BOOL(&((Voices[4].ADSR.EnvLevel))) ? 1 : sizeof((Voices[4].ADSR.EnvLevel)), 0x80000000 | (SF_IS_BOOL(&((Voices[4].ADSR.EnvLevel))) ? 0x08000000 : 0), "Voices[4].ADSR.EnvLevel" },
  { &((Voices[4].ADSR.Divider)), SF_IS_BOOL(&((Voices[4].ADSR.Divider))) ? 1 : sizeof((Voices[4].ADSR.Divider)), 0x80000000 | (SF_IS_BOOL(&((Voices[4].ADSR.Divider))) ? 0x08000000 : 0), "Voices[4].ADSR.Divider" },
  { &((Voices[4].ADSR.Phase)), SF_IS_BOOL(&((Voices[4].ADSR.Phase))) ? 1 : sizeof((Voices[4].ADSR.Phase)), 0x80000000 | (SF_IS_BOOL(&((Voices[4].ADSR.Phase))) ? 0x08000000 : 0), "Voices[4].ADSR.Phase" },
  { &((Voices[4].ADSR.AttackExp)), SF_IS_BOOL(&((Voices[4].ADSR.AttackExp))) ? 1 : sizeof((Voices[4].ADSR.AttackExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[4].ADSR.AttackExp))) ? 0x08000000 : 0), "Voices[4].ADSR.AttackExp" },
  { &((Voices[4].ADSR.SustainExp)), SF_IS_BOOL(&((Voices[4].ADSR.SustainExp))) ? 1 : sizeof((Voices[4].ADSR.SustainExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[4].ADSR.SustainExp))) ? 0x08000000 : 0), "Voices[4].ADSR.SustainExp" },
  { &((Voices[4].ADSR.SustainDec)), SF_IS_BOOL(&((Voices[4].ADSR.SustainDec))) ? 1 : sizeof((Voices[4].ADSR.SustainDec)), 0x80000000 | (SF_IS_BOOL(&((Voices[4].ADSR.SustainDec))) ? 0x08000000 : 0), "Voices[4].ADSR.SustainDec" },
  { &((Voices[4].ADSR.ReleaseExp)), SF_IS_BOOL(&((Voices[4].ADSR.ReleaseExp))) ? 1 : sizeof((Voices[4].ADSR.ReleaseExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[4].ADSR.ReleaseExp))) ? 0x08000000 : 0), "Voices[4].ADSR.ReleaseExp" },
  { &((Voices[4].ADSR.AttackRate)), SF_IS_BOOL(&((Voices[4].ADSR.AttackRate))) ? 1 : sizeof((Voices[4].ADSR.AttackRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[4].ADSR.AttackRate))) ? 0x08000000 : 0), "Voices[4].ADSR.AttackRate" },
  { &((Voices[4].ADSR.DecayRate)), SF_IS_BOOL(&((Voices[4].ADSR.DecayRate))) ? 1 : sizeof((Voices[4].ADSR.DecayRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[4].ADSR.DecayRate))) ? 0x08000000 : 0), "Voices[4].ADSR.DecayRate" },
  { &((Voices[4].ADSR.SustainRate)), SF_IS_BOOL(&((Voices[4].ADSR.SustainRate))) ? 1 : sizeof((Voices[4].ADSR.SustainRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[4].ADSR.SustainRate))) ? 0x08000000 : 0), "Voices[4].ADSR.SustainRate" },
  { &((Voices[4].ADSR.ReleaseRate)), SF_IS_BOOL(&((Voices[4].ADSR.ReleaseRate))) ? 1 : sizeof((Voices[4].ADSR.ReleaseRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[4].ADSR.ReleaseRate))) ? 0x08000000 : 0), "Voices[4].ADSR.ReleaseRate" },
  { &((Voices[4].ADSR.SustainLevel)), SF_IS_BOOL(&((Voices[4].ADSR.SustainLevel))) ? 1 : sizeof((Voices[4].ADSR.SustainLevel)), 0x80000000 | (SF_IS_BOOL(&((Voices[4].ADSR.SustainLevel))) ? 0x08000000 : 0), "Voices[4].ADSR.SustainLevel" },
  { ((&Voices[5].DecodeBuffer[0])), (uint32)(((sizeof(Voices[5].DecodeBuffer) / sizeof(Voices[5].DecodeBuffer[0]))) * sizeof(uint16)), 0x20000000 | SF_FORCE_A16((&Voices[5].DecodeBuffer[0])), "&Voices[5].DecodeBuffer[0]" },
  { &((Voices[5].DecodeM2)), SF_IS_BOOL(&((Voices[5].DecodeM2))) ? 1 : sizeof((Voices[5].DecodeM2)), 0x80000000 | (SF_IS_BOOL(&((Voices[5].DecodeM2))) ? 0x08000000 : 0), "Voices[5].DecodeM2" },
  { &((Voices[5].DecodeM1)), SF_IS_BOOL(&((Voices[5].DecodeM1))) ? 1 : sizeof((Voices[5].DecodeM1)), 0x80000000 | (SF_IS_BOOL(&((Voices[5].DecodeM1))) ? 0x08000000 : 0), "Voices[5].DecodeM1" },
  { &((Voices[5].DecodePlayDelay)), SF_IS_BOOL(&((Voices[5].DecodePlayDelay))) ? 1 : sizeof((Voices[5].DecodePlayDelay)), 0x80000000 | (SF_IS_BOOL(&((Voices[5].DecodePlayDelay))) ? 0x08000000 : 0), "Voices[5].DecodePlayDelay" },
  { &((Voices[5].DecodeWritePos)), SF_IS_BOOL(&((Voices[5].DecodeWritePos))) ? 1 : sizeof((Voices[5].DecodeWritePos)), 0x80000000 | (SF_IS_BOOL(&((Voices[5].DecodeWritePos))) ? 0x08000000 : 0), "Voices[5].DecodeWritePos" },
  { &((Voices[5].DecodeReadPos)), SF_IS_BOOL(&((Voices[5].DecodeReadPos))) ? 1 : sizeof((Voices[5].DecodeReadPos)), 0x80000000 | (SF_IS_BOOL(&((Voices[5].DecodeReadPos))) ? 0x08000000 : 0), "Voices[5].DecodeReadPos" },
  { &((Voices[5].DecodeAvail)), SF_IS_BOOL(&((Voices[5].DecodeAvail))) ? 1 : sizeof((Voices[5].DecodeAvail)), 0x80000000 | (SF_IS_BOOL(&((Voices[5].DecodeAvail))) ? 0x08000000 : 0), "Voices[5].DecodeAvail" },
  { &((Voices[5].DecodeShift)), SF_IS_BOOL(&((Voices[5].DecodeShift))) ? 1 : sizeof((Voices[5].DecodeShift)), 0x80000000 | (SF_IS_BOOL(&((Voices[5].DecodeShift))) ? 0x08000000 : 0), "Voices[5].DecodeShift" },
  { &((Voices[5].DecodeWeight)), SF_IS_BOOL(&((Voices[5].DecodeWeight))) ? 1 : sizeof((Voices[5].DecodeWeight)), 0x80000000 | (SF_IS_BOOL(&((Voices[5].DecodeWeight))) ? 0x08000000 : 0), "Voices[5].DecodeWeight" },
  { &((Voices[5].DecodeFlags)), SF_IS_BOOL(&((Voices[5].DecodeFlags))) ? 1 : sizeof((Voices[5].DecodeFlags)), 0x80000000 | (SF_IS_BOOL(&((Voices[5].DecodeFlags))) ? 0x08000000 : 0), "Voices[5].DecodeFlags" },
  { &((Voices[5].IgnoreSampLA)), SF_IS_BOOL(&((Voices[5].IgnoreSampLA))) ? 1 : sizeof((Voices[5].IgnoreSampLA)), 0x80000000 | (SF_IS_BOOL(&((Voices[5].IgnoreSampLA))) ? 0x08000000 : 0), "Voices[5].IgnoreSampLA" },
  { &(((Voices[5].Sweep[0]).Control)), SF_IS_BOOL(&(((Voices[5].Sweep[0]).Control))) ? 1 : sizeof(((Voices[5].Sweep[0]).Control)), 0x80000000 | (SF_IS_BOOL(&(((Voices[5].Sweep[0]).Control))) ? 0x08000000 : 0), "(Voices[5].Sweep[0]).Control" },
  { &(((Voices[5].Sweep[0]).Current)), SF_IS_BOOL(&(((Voices[5].Sweep[0]).Current))) ? 1 : sizeof(((Voices[5].Sweep[0]).Current)), 0x80000000 | (SF_IS_BOOL(&(((Voices[5].Sweep[0]).Current))) ? 0x08000000 : 0), "(Voices[5].Sweep[0]).Current" },
  { &(((Voices[5].Sweep[0]).Divider)), SF_IS_BOOL(&(((Voices[5].Sweep[0]).Divider))) ? 1 : sizeof(((Voices[5].Sweep[0]).Divider)), 0x80000000 | (SF_IS_BOOL(&(((Voices[5].Sweep[0]).Divider))) ? 0x08000000 : 0), "(Voices[5].Sweep[0]).Divider" },
  { &(((Voices[5].Sweep[1]).Control)), SF_IS_BOOL(&(((Voices[5].Sweep[1]).Control))) ? 1 : sizeof(((Voices[5].Sweep[1]).Control)), 0x80000000 | (SF_IS_BOOL(&(((Voices[5].Sweep[1]).Control))) ? 0x08000000 : 0), "(Voices[5].Sweep[1]).Control" },
  { &(((Voices[5].Sweep[1]).Current)), SF_IS_BOOL(&(((Voices[5].Sweep[1]).Current))) ? 1 : sizeof(((Voices[5].Sweep[1]).Current)), 0x80000000 | (SF_IS_BOOL(&(((Voices[5].Sweep[1]).Current))) ? 0x08000000 : 0), "(Voices[5].Sweep[1]).Current" },
  { &(((Voices[5].Sweep[1]).Divider)), SF_IS_BOOL(&(((Voices[5].Sweep[1]).Divider))) ? 1 : sizeof(((Voices[5].Sweep[1]).Divider)), 0x80000000 | (SF_IS_BOOL(&(((Voices[5].Sweep[1]).Divider))) ? 0x08000000 : 0), "(Voices[5].Sweep[1]).Divider" },
  { &((Voices[5].Pitch)), SF_IS_BOOL(&((Voices[5].Pitch))) ? 1 : sizeof((Voices[5].Pitch)), 0x80000000 | (SF_IS_BOOL(&((Voices[5].Pitch))) ? 0x08000000 : 0), "Voices[5].Pitch" },
  { &((Voices[5].CurPhase)), SF_IS_BOOL(&((Voices[5].CurPhase))) ? 1 : sizeof((Voices[5].CurPhase)), 0x80000000 | (SF_IS_BOOL(&((Voices[5].CurPhase))) ? 0x08000000 : 0), "Voices[5].CurPhase" },
  { &((Voices[5].StartAddr)), SF_IS_BOOL(&((Voices[5].StartAddr))) ? 1 : sizeof((Voices[5].StartAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[5].StartAddr))) ? 0x08000000 : 0), "Voices[5].StartAddr" },
  { &((Voices[5].CurAddr)), SF_IS_BOOL(&((Voices[5].CurAddr))) ? 1 : sizeof((Voices[5].CurAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[5].CurAddr))) ? 0x08000000 : 0), "Voices[5].CurAddr" },
  { &((Voices[5].ADSRControl)), SF_IS_BOOL(&((Voices[5].ADSRControl))) ? 1 : sizeof((Voices[5].ADSRControl)), 0x80000000 | (SF_IS_BOOL(&((Voices[5].ADSRControl))) ? 0x08000000 : 0), "Voices[5].ADSRControl" },
  { &((Voices[5].LoopAddr)), SF_IS_BOOL(&((Voices[5].LoopAddr))) ? 1 : sizeof((Voices[5].LoopAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[5].LoopAddr))) ? 0x08000000 : 0), "Voices[5].LoopAddr" },
  { &((Voices[5].PreLRSample)), SF_IS_BOOL(&((Voices[5].PreLRSample))) ? 1 : sizeof((Voices[5].PreLRSample)), 0x80000000 | (SF_IS_BOOL(&((Voices[5].PreLRSample))) ? 0x08000000 : 0), "Voices[5].PreLRSample" },
  { &((Voices[5].ADSR.EnvLevel)), SF_IS_BOOL(&((Voices[5].ADSR.EnvLevel))) ? 1 : sizeof((Voices[5].ADSR.EnvLevel)), 0x80000000 | (SF_IS_BOOL(&((Voices[5].ADSR.EnvLevel))) ? 0x08000000 : 0), "Voices[5].ADSR.EnvLevel" },
  { &((Voices[5].ADSR.Divider)), SF_IS_BOOL(&((Voices[5].ADSR.Divider))) ? 1 : sizeof((Voices[5].ADSR.Divider)), 0x80000000 | (SF_IS_BOOL(&((Voices[5].ADSR.Divider))) ? 0x08000000 : 0), "Voices[5].ADSR.Divider" },
  { &((Voices[5].ADSR.Phase)), SF_IS_BOOL(&((Voices[5].ADSR.Phase))) ? 1 : sizeof((Voices[5].ADSR.Phase)), 0x80000000 | (SF_IS_BOOL(&((Voices[5].ADSR.Phase))) ? 0x08000000 : 0), "Voices[5].ADSR.Phase" },
  { &((Voices[5].ADSR.AttackExp)), SF_IS_BOOL(&((Voices[5].ADSR.AttackExp))) ? 1 : sizeof((Voices[5].ADSR.AttackExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[5].ADSR.AttackExp))) ? 0x08000000 : 0), "Voices[5].ADSR.AttackExp" },
  { &((Voices[5].ADSR.SustainExp)), SF_IS_BOOL(&((Voices[5].ADSR.SustainExp))) ? 1 : sizeof((Voices[5].ADSR.SustainExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[5].ADSR.SustainExp))) ? 0x08000000 : 0), "Voices[5].ADSR.SustainExp" },
  { &((Voices[5].ADSR.SustainDec)), SF_IS_BOOL(&((Voices[5].ADSR.SustainDec))) ? 1 : sizeof((Voices[5].ADSR.SustainDec)), 0x80000000 | (SF_IS_BOOL(&((Voices[5].ADSR.SustainDec))) ? 0x08000000 : 0), "Voices[5].ADSR.SustainDec" },
  { &((Voices[5].ADSR.ReleaseExp)), SF_IS_BOOL(&((Voices[5].ADSR.ReleaseExp))) ? 1 : sizeof((Voices[5].ADSR.ReleaseExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[5].ADSR.ReleaseExp))) ? 0x08000000 : 0), "Voices[5].ADSR.ReleaseExp" },
  { &((Voices[5].ADSR.AttackRate)), SF_IS_BOOL(&((Voices[5].ADSR.AttackRate))) ? 1 : sizeof((Voices[5].ADSR.AttackRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[5].ADSR.AttackRate))) ? 0x08000000 : 0), "Voices[5].ADSR.AttackRate" },
  { &((Voices[5].ADSR.DecayRate)), SF_IS_BOOL(&((Voices[5].ADSR.DecayRate))) ? 1 : sizeof((Voices[5].ADSR.DecayRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[5].ADSR.DecayRate))) ? 0x08000000 : 0), "Voices[5].ADSR.DecayRate" },
  { &((Voices[5].ADSR.SustainRate)), SF_IS_BOOL(&((Voices[5].ADSR.SustainRate))) ? 1 : sizeof((Voices[5].ADSR.SustainRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[5].ADSR.SustainRate))) ? 0x08000000 : 0), "Voices[5].ADSR.SustainRate" },
  { &((Voices[5].ADSR.ReleaseRate)), SF_IS_BOOL(&((Voices[5].ADSR.ReleaseRate))) ? 1 : sizeof((Voices[5].ADSR.ReleaseRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[5].ADSR.ReleaseRate))) ? 0x08000000 : 0), "Voices[5].ADSR.ReleaseRate" },
  { &((Voices[5].ADSR.SustainLevel)), SF_IS_BOOL(&((Voices[5].ADSR.SustainLevel))) ? 1 : sizeof((Voices[5].ADSR.SustainLevel)), 0x80000000 | (SF_IS_BOOL(&((Voices[5].ADSR.SustainLevel))) ? 0x08000000 : 0), "Voices[5].ADSR.SustainLevel" },
  { ((&Voices[6].DecodeBuffer[0])), (uint32)(((sizeof(Voices[6].DecodeBuffer) / sizeof(Voices[6].DecodeBuffer[0]))) * sizeof(uint16)), 0x20000000 | SF_FORCE_A16((&Voices[6].DecodeBuffer[0])), "&Voices[6].DecodeBuffer[0]" },
  { &((Voices[6].DecodeM2)), SF_IS_BOOL(&((Voices[6].DecodeM2))) ? 1 : sizeof((Voices[6].DecodeM2)), 0x80000000 | (SF_IS_BOOL(&((Voices[6].DecodeM2))) ? 0x08000000 : 0), "Voices[6].DecodeM2" },
  { &((Voices[6].DecodeM1)), SF_IS_BOOL(&((Voices[6].DecodeM1))) ? 1 : sizeof((Voices[6].DecodeM1)), 0x80000000 | (SF_IS_BOOL(&((Voices[6].DecodeM1))) ? 0x08000000 : 0), "Voices[6].DecodeM1" },
  { &((Voices[6].DecodePlayDelay)), SF_IS_BOOL(&((Voices[6].DecodePlayDelay))) ? 1 : sizeof((Voices[6].DecodePlayDelay)), 0x80000000 | (SF_IS_BOOL(&((Voices[6].DecodePlayDelay))) ? 0x08000000 : 0), "Voices[6].DecodePlayDelay" },
  { &((Voices[6].DecodeWritePos)), SF_IS_BOOL(&((Voices[6].DecodeWritePos))) ? 1 : sizeof((Voices[6].DecodeWritePos)), 0x80000000 | (SF_IS_BOOL(&((Voices[6].DecodeWritePos))) ? 0x08000000 : 0), "Voices[6].DecodeWritePos" },
  { &((Voices[6].DecodeReadPos)), SF_IS_BOOL(&((Voices[6].DecodeReadPos))) ? 1 : sizeof((Voices[6].DecodeReadPos)), 0x80000000 | (SF_IS_BOOL(&((Voices[6].DecodeReadPos))) ? 0x08000000 : 0), "Voices[6].DecodeReadPos" },
  { &((Voices[6].DecodeAvail)), SF_IS_BOOL(&((Voices[6].DecodeAvail))) ? 1 : sizeof((Voices[6].DecodeAvail)), 0x80000000 | (SF_IS_BOOL(&((Voices[6].DecodeAvail))) ? 0x08000000 : 0), "Voices[6].DecodeAvail" },
  { &((Voices[6].DecodeShift)), SF_IS_BOOL(&((Voices[6].DecodeShift))) ? 1 : sizeof((Voices[6].DecodeShift)), 0x80000000 | (SF_IS_BOOL(&((Voices[6].DecodeShift))) ? 0x08000000 : 0), "Voices[6].DecodeShift" },
  { &((Voices[6].DecodeWeight)), SF_IS_BOOL(&((Voices[6].DecodeWeight))) ? 1 : sizeof((Voices[6].DecodeWeight)), 0x80000000 | (SF_IS_BOOL(&((Voices[6].DecodeWeight))) ? 0x08000000 : 0), "Voices[6].DecodeWeight" },
  { &((Voices[6].DecodeFlags)), SF_IS_BOOL(&((Voices[6].DecodeFlags))) ? 1 : sizeof((Voices[6].DecodeFlags)), 0x80000000 | (SF_IS_BOOL(&((Voices[6].DecodeFlags))) ? 0x08000000 : 0), "Voices[6].DecodeFlags" },
  { &((Voices[6].IgnoreSampLA)), SF_IS_BOOL(&((Voices[6].IgnoreSampLA))) ? 1 : sizeof((Voices[6].IgnoreSampLA)), 0x80000000 | (SF_IS_BOOL(&((Voices[6].IgnoreSampLA))) ? 0x08000000 : 0), "Voices[6].IgnoreSampLA" },
  { &(((Voices[6].Sweep[0]).Control)), SF_IS_BOOL(&(((Voices[6].Sweep[0]).Control))) ? 1 : sizeof(((Voices[6].Sweep[0]).Control)), 0x80000000 | (SF_IS_BOOL(&(((Voices[6].Sweep[0]).Control))) ? 0x08000000 : 0), "(Voices[6].Sweep[0]).Control" },
  { &(((Voices[6].Sweep[0]).Current)), SF_IS_BOOL(&(((Voices[6].Sweep[0]).Current))) ? 1 : sizeof(((Voices[6].Sweep[0]).Current)), 0x80000000 | (SF_IS_BOOL(&(((Voices[6].Sweep[0]).Current))) ? 0x08000000 : 0), "(Voices[6].Sweep[0]).Current" },
  { &(((Voices[6].Sweep[0]).Divider)), SF_IS_BOOL(&(((Voices[6].Sweep[0]).Divider))) ? 1 : sizeof(((Voices[6].Sweep[0]).Divider)), 0x80000000 | (SF_IS_BOOL(&(((Voices[6].Sweep[0]).Divider))) ? 0x08000000 : 0), "(Voices[6].Sweep[0]).Divider" },
  { &(((Voices[6].Sweep[1]).Control)), SF_IS_BOOL(&(((Voices[6].Sweep[1]).Control))) ? 1 : sizeof(((Voices[6].Sweep[1]).Control)), 0x80000000 | (SF_IS_BOOL(&(((Voices[6].Sweep[1]).Control))) ? 0x08000000 : 0), "(Voices[6].Sweep[1]).Control" },
  { &(((Voices[6].Sweep[1]).Current)), SF_IS_BOOL(&(((Voices[6].Sweep[1]).Current))) ? 1 : sizeof(((Voices[6].Sweep[1]).Current)), 0x80000000 | (SF_IS_BOOL(&(((Voices[6].Sweep[1]).Current))) ? 0x08000000 : 0), "(Voices[6].Sweep[1]).Current" },
  { &(((Voices[6].Sweep[1]).Divider)), SF_IS_BOOL(&(((Voices[6].Sweep[1]).Divider))) ? 1 : sizeof(((Voices[6].Sweep[1]).Divider)), 0x80000000 | (SF_IS_BOOL(&(((Voices[6].Sweep[1]).Divider))) ? 0x08000000 : 0), "(Voices[6].Sweep[1]).Divider" },
  { &((Voices[6].Pitch)), SF_IS_BOOL(&((Voices[6].Pitch))) ? 1 : sizeof((Voices[6].Pitch)), 0x80000000 | (SF_IS_BOOL(&((Voices[6].Pitch))) ? 0x08000000 : 0), "Voices[6].Pitch" },
  { &((Voices[6].CurPhase)), SF_IS_BOOL(&((Voices[6].CurPhase))) ? 1 : sizeof((Voices[6].CurPhase)), 0x80000000 | (SF_IS_BOOL(&((Voices[6].CurPhase))) ? 0x08000000 : 0), "Voices[6].CurPhase" },
  { &((Voices[6].StartAddr)), SF_IS_BOOL(&((Voices[6].StartAddr))) ? 1 : sizeof((Voices[6].StartAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[6].StartAddr))) ? 0x08000000 : 0), "Voices[6].StartAddr" },
  { &((Voices[6].CurAddr)), SF_IS_BOOL(&((Voices[6].CurAddr))) ? 1 : sizeof((Voices[6].CurAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[6].CurAddr))) ? 0x08000000 : 0), "Voices[6].CurAddr" },
  { &((Voices[6].ADSRControl)), SF_IS_BOOL(&((Voices[6].ADSRControl))) ? 1 : sizeof((Voices[6].ADSRControl)), 0x80000000 | (SF_IS_BOOL(&((Voices[6].ADSRControl))) ? 0x08000000 : 0), "Voices[6].ADSRControl" },
  { &((Voices[6].LoopAddr)), SF_IS_BOOL(&((Voices[6].LoopAddr))) ? 1 : sizeof((Voices[6].LoopAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[6].LoopAddr))) ? 0x08000000 : 0), "Voices[6].LoopAddr" },
  { &((Voices[6].PreLRSample)), SF_IS_BOOL(&((Voices[6].PreLRSample))) ? 1 : sizeof((Voices[6].PreLRSample)), 0x80000000 | (SF_IS_BOOL(&((Voices[6].PreLRSample))) ? 0x08000000 : 0), "Voices[6].PreLRSample" },
  { &((Voices[6].ADSR.EnvLevel)), SF_IS_BOOL(&((Voices[6].ADSR.EnvLevel))) ? 1 : sizeof((Voices[6].ADSR.EnvLevel)), 0x80000000 | (SF_IS_BOOL(&((Voices[6].ADSR.EnvLevel))) ? 0x08000000 : 0), "Voices[6].ADSR.EnvLevel" },
  { &((Voices[6].ADSR.Divider)), SF_IS_BOOL(&((Voices[6].ADSR.Divider))) ? 1 : sizeof((Voices[6].ADSR.Divider)), 0x80000000 | (SF_IS_BOOL(&((Voices[6].ADSR.Divider))) ? 0x08000000 : 0), "Voices[6].ADSR.Divider" },
  { &((Voices[6].ADSR.Phase)), SF_IS_BOOL(&((Voices[6].ADSR.Phase))) ? 1 : sizeof((Voices[6].ADSR.Phase)), 0x80000000 | (SF_IS_BOOL(&((Voices[6].ADSR.Phase))) ? 0x08000000 : 0), "Voices[6].ADSR.Phase" },
  { &((Voices[6].ADSR.AttackExp)), SF_IS_BOOL(&((Voices[6].ADSR.AttackExp))) ? 1 : sizeof((Voices[6].ADSR.AttackExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[6].ADSR.AttackExp))) ? 0x08000000 : 0), "Voices[6].ADSR.AttackExp" },
  { &((Voices[6].ADSR.SustainExp)), SF_IS_BOOL(&((Voices[6].ADSR.SustainExp))) ? 1 : sizeof((Voices[6].ADSR.SustainExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[6].ADSR.SustainExp))) ? 0x08000000 : 0), "Voices[6].ADSR.SustainExp" },
  { &((Voices[6].ADSR.SustainDec)), SF_IS_BOOL(&((Voices[6].ADSR.SustainDec))) ? 1 : sizeof((Voices[6].ADSR.SustainDec)), 0x80000000 | (SF_IS_BOOL(&((Voices[6].ADSR.SustainDec))) ? 0x08000000 : 0), "Voices[6].ADSR.SustainDec" },
  { &((Voices[6].ADSR.ReleaseExp)), SF_IS_BOOL(&((Voices[6].ADSR.ReleaseExp))) ? 1 : sizeof((Voices[6].ADSR.ReleaseExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[6].ADSR.ReleaseExp))) ? 0x08000000 : 0), "Voices[6].ADSR.ReleaseExp" },
  {&((Voices[6].ADSR.AttackRate)), SF_IS_BOOL(&((Voices[6].ADSR.AttackRate))) ? 1 : sizeof((Voices[6].ADSR.AttackRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[6].ADSR.AttackRate))) ? 0x08000000 : 0), "Voices[6].ADSR.AttackRate" },
  { &((Voices[6].ADSR.DecayRate)), SF_IS_BOOL(&((Voices[6].ADSR.DecayRate))) ? 1 : sizeof((Voices[6].ADSR.DecayRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[6].ADSR.DecayRate))) ? 0x08000000 : 0), "Voices[6].ADSR.DecayRate" },
  { &((Voices[6].ADSR.SustainRate)), SF_IS_BOOL(&((Voices[6].ADSR.SustainRate))) ? 1 : sizeof((Voices[6].ADSR.SustainRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[6].ADSR.SustainRate))) ? 0x08000000 : 0), "Voices[6].ADSR.SustainRate" },
  { &((Voices[6].ADSR.ReleaseRate)), SF_IS_BOOL(&((Voices[6].ADSR.ReleaseRate))) ? 1 : sizeof((Voices[6].ADSR.ReleaseRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[6].ADSR.ReleaseRate))) ? 0x08000000 : 0), "Voices[6].ADSR.ReleaseRate" },
  { &((Voices[6].ADSR.SustainLevel)), SF_IS_BOOL(&((Voices[6].ADSR.SustainLevel))) ? 1 : sizeof((Voices[6].ADSR.SustainLevel)), 0x80000000 | (SF_IS_BOOL(&((Voices[6].ADSR.SustainLevel))) ? 0x08000000 : 0), "Voices[6].ADSR.SustainLevel" },
  { ((&Voices[7].DecodeBuffer[0])), (uint32)(((sizeof(Voices[7].DecodeBuffer) / sizeof(Voices[7].DecodeBuffer[0]))) * sizeof(uint16)), 0x20000000 | SF_FORCE_A16((&Voices[7].DecodeBuffer[0])), "&Voices[7].DecodeBuffer[0]" },
  { &((Voices[7].DecodeM2)), SF_IS_BOOL(&((Voices[7].DecodeM2))) ? 1 : sizeof((Voices[7].DecodeM2)), 0x80000000 | (SF_IS_BOOL(&((Voices[7].DecodeM2))) ? 0x08000000 : 0), "Voices[7].DecodeM2" },
  { &((Voices[7].DecodeM1)), SF_IS_BOOL(&((Voices[7].DecodeM1))) ? 1 : sizeof((Voices[7].DecodeM1)), 0x80000000 | (SF_IS_BOOL(&((Voices[7].DecodeM1))) ? 0x08000000 : 0), "Voices[7].DecodeM1" },
  { &((Voices[7].DecodePlayDelay)), SF_IS_BOOL(&((Voices[7].DecodePlayDelay))) ? 1 : sizeof((Voices[7].DecodePlayDelay)), 0x80000000 | (SF_IS_BOOL(&((Voices[7].DecodePlayDelay))) ? 0x08000000 : 0), "Voices[7].DecodePlayDelay" },
  { &((Voices[7].DecodeWritePos)), SF_IS_BOOL(&((Voices[7].DecodeWritePos))) ? 1 : sizeof((Voices[7].DecodeWritePos)), 0x80000000 | (SF_IS_BOOL(&((Voices[7].DecodeWritePos))) ? 0x08000000 : 0), "Voices[7].DecodeWritePos" },
  { &((Voices[7].DecodeReadPos)), SF_IS_BOOL(&((Voices[7].DecodeReadPos))) ? 1 : sizeof((Voices[7].DecodeReadPos)), 0x80000000 | (SF_IS_BOOL(&((Voices[7].DecodeReadPos))) ? 0x08000000 : 0), "Voices[7].DecodeReadPos" },
  { &((Voices[7].DecodeAvail)), SF_IS_BOOL(&((Voices[7].DecodeAvail))) ? 1 : sizeof((Voices[7].DecodeAvail)), 0x80000000 | (SF_IS_BOOL(&((Voices[7].DecodeAvail))) ? 0x08000000 : 0), "Voices[7].DecodeAvail" },
  { &((Voices[7].DecodeShift)), SF_IS_BOOL(&((Voices[7].DecodeShift))) ? 1 : sizeof((Voices[7].DecodeShift)), 0x80000000 | (SF_IS_BOOL(&((Voices[7].DecodeShift))) ? 0x08000000 : 0), "Voices[7].DecodeShift" },
  { &((Voices[7].DecodeWeight)), SF_IS_BOOL(&((Voices[7].DecodeWeight))) ? 1 : sizeof((Voices[7].DecodeWeight)), 0x80000000 | (SF_IS_BOOL(&((Voices[7].DecodeWeight))) ? 0x08000000 : 0), "Voices[7].DecodeWeight" },
  { &((Voices[7].DecodeFlags)), SF_IS_BOOL(&((Voices[7].DecodeFlags))) ? 1 : sizeof((Voices[7].DecodeFlags)), 0x80000000 | (SF_IS_BOOL(&((Voices[7].DecodeFlags))) ? 0x08000000 : 0), "Voices[7].DecodeFlags" },
  { &((Voices[7].IgnoreSampLA)), SF_IS_BOOL(&((Voices[7].IgnoreSampLA))) ? 1 : sizeof((Voices[7].IgnoreSampLA)), 0x80000000 | (SF_IS_BOOL(&((Voices[7].IgnoreSampLA))) ? 0x08000000 : 0), "Voices[7].IgnoreSampLA" },
  { &(((Voices[7].Sweep[0]).Control)), SF_IS_BOOL(&(((Voices[7].Sweep[0]).Control))) ? 1 : sizeof(((Voices[7].Sweep[0]).Control)), 0x80000000 | (SF_IS_BOOL(&(((Voices[7].Sweep[0]).Control))) ? 0x08000000 : 0), "(Voices[7].Sweep[0]).Control" },
  { &(((Voices[7].Sweep[0]).Current)), SF_IS_BOOL(&(((Voices[7].Sweep[0]).Current))) ? 1 : sizeof(((Voices[7].Sweep[0]).Current)), 0x80000000 | (SF_IS_BOOL(&(((Voices[7].Sweep[0]).Current))) ? 0x08000000 : 0), "(Voices[7].Sweep[0]).Current" },
  { &(((Voices[7].Sweep[0]).Divider)), SF_IS_BOOL(&(((Voices[7].Sweep[0]).Divider))) ? 1 : sizeof(((Voices[7].Sweep[0]).Divider)), 0x80000000 | (SF_IS_BOOL(&(((Voices[7].Sweep[0]).Divider))) ? 0x08000000 : 0), "(Voices[7].Sweep[0]).Divider" },
  { &(((Voices[7].Sweep[1]).Control)), SF_IS_BOOL(&(((Voices[7].Sweep[1]).Control))) ? 1 : sizeof(((Voices[7].Sweep[1]).Control)), 0x80000000 | (SF_IS_BOOL(&(((Voices[7].Sweep[1]).Control))) ? 0x08000000 : 0), "(Voices[7].Sweep[1]).Control" },
  { &(((Voices[7].Sweep[1]).Current)), SF_IS_BOOL(&(((Voices[7].Sweep[1]).Current))) ? 1 : sizeof(((Voices[7].Sweep[1]).Current)), 0x80000000 | (SF_IS_BOOL(&(((Voices[7].Sweep[1]).Current))) ? 0x08000000 : 0), "(Voices[7].Sweep[1]).Current" },
  { &(((Voices[7].Sweep[1]).Divider)), SF_IS_BOOL(&(((Voices[7].Sweep[1]).Divider))) ? 1 : sizeof(((Voices[7].Sweep[1]).Divider)), 0x80000000 | (SF_IS_BOOL(&(((Voices[7].Sweep[1]).Divider))) ? 0x08000000 : 0), "(Voices[7].Sweep[1]).Divider" },
  { &((Voices[7].Pitch)), SF_IS_BOOL(&((Voices[7].Pitch))) ? 1 : sizeof((Voices[7].Pitch)), 0x80000000 | (SF_IS_BOOL(&((Voices[7].Pitch))) ? 0x08000000 : 0), "Voices[7].Pitch" },
  { &((Voices[7].CurPhase)), SF_IS_BOOL(&((Voices[7].CurPhase))) ? 1 : sizeof((Voices[7].CurPhase)), 0x80000000 | (SF_IS_BOOL(&((Voices[7].CurPhase))) ? 0x08000000 : 0), "Voices[7].CurPhase" },
  { &((Voices[7].StartAddr)), SF_IS_BOOL(&((Voices[7].StartAddr))) ? 1 : sizeof((Voices[7].StartAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[7].StartAddr))) ? 0x08000000 : 0), "Voices[7].StartAddr" },
  { &((Voices[7].CurAddr)), SF_IS_BOOL(&((Voices[7].CurAddr))) ? 1 : sizeof((Voices[7].CurAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[7].CurAddr))) ? 0x08000000 : 0), "Voices[7].CurAddr" },
  { &((Voices[7].ADSRControl)), SF_IS_BOOL(&((Voices[7].ADSRControl))) ? 1 : sizeof((Voices[7].ADSRControl)), 0x80000000 | (SF_IS_BOOL(&((Voices[7].ADSRControl))) ? 0x08000000 : 0), "Voices[7].ADSRControl" },
  { &((Voices[7].LoopAddr)), SF_IS_BOOL(&((Voices[7].LoopAddr))) ? 1 : sizeof((Voices[7].LoopAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[7].LoopAddr))) ? 0x08000000 : 0), "Voices[7].LoopAddr" },
  { &((Voices[7].PreLRSample)), SF_IS_BOOL(&((Voices[7].PreLRSample))) ? 1 : sizeof((Voices[7].PreLRSample)), 0x80000000 | (SF_IS_BOOL(&((Voices[7].PreLRSample))) ? 0x08000000 : 0), "Voices[7].PreLRSample" },
  { &((Voices[7].ADSR.EnvLevel)), SF_IS_BOOL(&((Voices[7].ADSR.EnvLevel))) ? 1 : sizeof((Voices[7].ADSR.EnvLevel)), 0x80000000 | (SF_IS_BOOL(&((Voices[7].ADSR.EnvLevel))) ? 0x08000000 : 0), "Voices[7].ADSR.EnvLevel" },
  { &((Voices[7].ADSR.Divider)), SF_IS_BOOL(&((Voices[7].ADSR.Divider))) ? 1 : sizeof((Voices[7].ADSR.Divider)), 0x80000000 | (SF_IS_BOOL(&((Voices[7].ADSR.Divider))) ? 0x08000000 : 0), "Voices[7].ADSR.Divider" },
  { &((Voices[7].ADSR.Phase)), SF_IS_BOOL(&((Voices[7].ADSR.Phase))) ? 1 : sizeof((Voices[7].ADSR.Phase)), 0x80000000 | (SF_IS_BOOL(&((Voices[7].ADSR.Phase))) ? 0x08000000 : 0), "Voices[7].ADSR.Phase" },
  { &((Voices[7].ADSR.AttackExp)), SF_IS_BOOL(&((Voices[7].ADSR.AttackExp))) ? 1 : sizeof((Voices[7].ADSR.AttackExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[7].ADSR.AttackExp))) ? 0x08000000 : 0), "Voices[7].ADSR.AttackExp" },
  { &((Voices[7].ADSR.SustainExp)), SF_IS_BOOL(&((Voices[7].ADSR.SustainExp))) ? 1 : sizeof((Voices[7].ADSR.SustainExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[7].ADSR.SustainExp))) ? 0x08000000 : 0), "Voices[7].ADSR.SustainExp" },
  { &((Voices[7].ADSR.SustainDec)), SF_IS_BOOL(&((Voices[7].ADSR.SustainDec))) ? 1 : sizeof((Voices[7].ADSR.SustainDec)), 0x80000000 | (SF_IS_BOOL(&((Voices[7].ADSR.SustainDec))) ? 0x08000000 : 0), "Voices[7].ADSR.SustainDec" },
  { &((Voices[7].ADSR.ReleaseExp)), SF_IS_BOOL(&((Voices[7].ADSR.ReleaseExp))) ? 1 : sizeof((Voices[7].ADSR.ReleaseExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[7].ADSR.ReleaseExp))) ? 0x08000000 : 0), "Voices[7].ADSR.ReleaseExp" },
  { &((Voices[7].ADSR.AttackRate)), SF_IS_BOOL(&((Voices[7].ADSR.AttackRate))) ? 1 : sizeof((Voices[7].ADSR.AttackRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[7].ADSR.AttackRate))) ? 0x08000000 : 0), "Voices[7].ADSR.AttackRate" },
  { &((Voices[7].ADSR.DecayRate)), SF_IS_BOOL(&((Voices[7].ADSR.DecayRate))) ? 1 : sizeof((Voices[7].ADSR.DecayRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[7].ADSR.DecayRate))) ? 0x08000000 : 0), "Voices[7].ADSR.DecayRate" },
  { &((Voices[7].ADSR.SustainRate)), SF_IS_BOOL(&((Voices[7].ADSR.SustainRate))) ? 1 : sizeof((Voices[7].ADSR.SustainRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[7].ADSR.SustainRate))) ? 0x08000000 : 0), "Voices[7].ADSR.SustainRate" },
  { &((Voices[7].ADSR.ReleaseRate)), SF_IS_BOOL(&((Voices[7].ADSR.ReleaseRate))) ? 1 : sizeof((Voices[7].ADSR.ReleaseRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[7].ADSR.ReleaseRate))) ? 0x08000000 : 0), "Voices[7].ADSR.ReleaseRate" },
  { &((Voices[7].ADSR.SustainLevel)), SF_IS_BOOL(&((Voices[7].ADSR.SustainLevel))) ? 1 : sizeof((Voices[7].ADSR.SustainLevel)), 0x80000000 | (SF_IS_BOOL(&((Voices[7].ADSR.SustainLevel))) ? 0x08000000 : 0), "Voices[7].ADSR.SustainLevel" },
  { ((&Voices[8].DecodeBuffer[0])), (uint32)(((sizeof(Voices[8].DecodeBuffer) / sizeof(Voices[8].DecodeBuffer[0]))) * sizeof(uint16)), 0x20000000 | SF_FORCE_A16((&Voices[8].DecodeBuffer[0])), "&Voices[8].DecodeBuffer[0]" },
  { &((Voices[8].DecodeM2)), SF_IS_BOOL(&((Voices[8].DecodeM2))) ? 1 : sizeof((Voices[8].DecodeM2)), 0x80000000 | (SF_IS_BOOL(&((Voices[8].DecodeM2))) ? 0x08000000 : 0), "Voices[8].DecodeM2" },
  { &((Voices[8].DecodeM1)), SF_IS_BOOL(&((Voices[8].DecodeM1))) ? 1 : sizeof((Voices[8].DecodeM1)), 0x80000000 | (SF_IS_BOOL(&((Voices[8].DecodeM1))) ? 0x08000000 : 0), "Voices[8].DecodeM1" },
  { &((Voices[8].DecodePlayDelay)), SF_IS_BOOL(&((Voices[8].DecodePlayDelay))) ? 1 : sizeof((Voices[8].DecodePlayDelay)), 0x80000000 | (SF_IS_BOOL(&((Voices[8].DecodePlayDelay))) ? 0x08000000 : 0), "Voices[8].DecodePlayDelay" },
  { &((Voices[8].DecodeWritePos)), SF_IS_BOOL(&((Voices[8].DecodeWritePos))) ? 1 : sizeof((Voices[8].DecodeWritePos)), 0x80000000 | (SF_IS_BOOL(&((Voices[8].DecodeWritePos))) ? 0x08000000 : 0), "Voices[8].DecodeWritePos" },
  { &((Voices[8].DecodeReadPos)), SF_IS_BOOL(&((Voices[8].DecodeReadPos))) ? 1 : sizeof((Voices[8].DecodeReadPos)), 0x80000000 | (SF_IS_BOOL(&((Voices[8].DecodeReadPos))) ? 0x08000000 : 0), "Voices[8].DecodeReadPos" },
  { &((Voices[8].DecodeAvail)), SF_IS_BOOL(&((Voices[8].DecodeAvail))) ? 1 : sizeof((Voices[8].DecodeAvail)), 0x80000000 | (SF_IS_BOOL(&((Voices[8].DecodeAvail))) ? 0x08000000 : 0), "Voices[8].DecodeAvail" },
  { &((Voices[8].DecodeShift)), SF_IS_BOOL(&((Voices[8].DecodeShift))) ? 1 : sizeof((Voices[8].DecodeShift)), 0x80000000 | (SF_IS_BOOL(&((Voices[8].DecodeShift))) ? 0x08000000 : 0), "Voices[8].DecodeShift" },
  { &((Voices[8].DecodeWeight)), SF_IS_BOOL(&((Voices[8].DecodeWeight))) ? 1 : sizeof((Voices[8].DecodeWeight)), 0x80000000 | (SF_IS_BOOL(&((Voices[8].DecodeWeight))) ? 0x08000000 : 0), "Voices[8].DecodeWeight" },
  { &((Voices[8].DecodeFlags)), SF_IS_BOOL(&((Voices[8].DecodeFlags))) ? 1 : sizeof((Voices[8].DecodeFlags)), 0x80000000 | (SF_IS_BOOL(&((Voices[8].DecodeFlags))) ? 0x08000000 : 0), "Voices[8].DecodeFlags" },
  { &((Voices[8].IgnoreSampLA)), SF_IS_BOOL(&((Voices[8].IgnoreSampLA))) ? 1 : sizeof((Voices[8].IgnoreSampLA)), 0x80000000 | (SF_IS_BOOL(&((Voices[8].IgnoreSampLA))) ? 0x08000000 : 0), "Voices[8].IgnoreSampLA" },
  { &(((Voices[8].Sweep[0]).Control)), SF_IS_BOOL(&(((Voices[8].Sweep[0]).Control))) ? 1 : sizeof(((Voices[8].Sweep[0]).Control)), 0x80000000 | (SF_IS_BOOL(&(((Voices[8].Sweep[0]).Control))) ? 0x08000000 : 0), "(Voices[8].Sweep[0]).Control" },
  { &(((Voices[8].Sweep[0]).Current)), SF_IS_BOOL(&(((Voices[8].Sweep[0]).Current))) ? 1 : sizeof(((Voices[8].Sweep[0]).Current)), 0x80000000 | (SF_IS_BOOL(&(((Voices[8].Sweep[0]).Current))) ? 0x08000000 : 0), "(Voices[8].Sweep[0]).Current" },
  { &(((Voices[8].Sweep[0]).Divider)), SF_IS_BOOL(&(((Voices[8].Sweep[0]).Divider))) ? 1 : sizeof(((Voices[8].Sweep[0]).Divider)), 0x80000000 | (SF_IS_BOOL(&(((Voices[8].Sweep[0]).Divider))) ? 0x08000000 : 0), "(Voices[8].Sweep[0]).Divider" },
  { &(((Voices[8].Sweep[1]).Control)), SF_IS_BOOL(&(((Voices[8].Sweep[1]).Control))) ? 1 : sizeof(((Voices[8].Sweep[1]).Control)), 0x80000000 | (SF_IS_BOOL(&(((Voices[8].Sweep[1]).Control))) ? 0x08000000 : 0), "(Voices[8].Sweep[1]).Control" },
  { &(((Voices[8].Sweep[1]).Current)), SF_IS_BOOL(&(((Voices[8].Sweep[1]).Current))) ? 1 : sizeof(((Voices[8].Sweep[1]).Current)), 0x80000000 | (SF_IS_BOOL(&(((Voices[8].Sweep[1]).Current))) ? 0x08000000 : 0), "(Voices[8].Sweep[1]).Current" },
  { &(((Voices[8].Sweep[1]).Divider)), SF_IS_BOOL(&(((Voices[8].Sweep[1]).Divider))) ? 1 : sizeof(((Voices[8].Sweep[1]).Divider)), 0x80000000 | (SF_IS_BOOL(&(((Voices[8].Sweep[1]).Divider))) ? 0x08000000 : 0), "(Voices[8].Sweep[1]).Divider" },
  { &((Voices[8].Pitch)), SF_IS_BOOL(&((Voices[8].Pitch))) ? 1 : sizeof((Voices[8].Pitch)), 0x80000000 | (SF_IS_BOOL(&((Voices[8].Pitch))) ? 0x08000000 : 0), "Voices[8].Pitch" },
  { &((Voices[8].CurPhase)), SF_IS_BOOL(&((Voices[8].CurPhase))) ? 1 : sizeof((Voices[8].CurPhase)), 0x80000000 | (SF_IS_BOOL(&((Voices[8].CurPhase))) ? 0x08000000 : 0), "Voices[8].CurPhase" },
  { &((Voices[8].StartAddr)), SF_IS_BOOL(&((Voices[8].StartAddr))) ? 1 : sizeof((Voices[8].StartAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[8].StartAddr))) ? 0x08000000 : 0), "Voices[8].StartAddr" },
  { &((Voices[8].CurAddr)), SF_IS_BOOL(&((Voices[8].CurAddr))) ? 1 : sizeof((Voices[8].CurAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[8].CurAddr))) ? 0x08000000 : 0), "Voices[8].CurAddr" },
  { &((Voices[8].ADSRControl)), SF_IS_BOOL(&((Voices[8].ADSRControl))) ? 1 : sizeof((Voices[8].ADSRControl)), 0x80000000 | (SF_IS_BOOL(&((Voices[8].ADSRControl))) ? 0x08000000 : 0), "Voices[8].ADSRControl" },
  { &((Voices[8].LoopAddr)), SF_IS_BOOL(&((Voices[8].LoopAddr))) ? 1 : sizeof((Voices[8].LoopAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[8].LoopAddr))) ? 0x08000000 : 0), "Voices[8].LoopAddr" },
  { &((Voices[8].PreLRSample)), SF_IS_BOOL(&((Voices[8].PreLRSample))) ? 1 : sizeof((Voices[8].PreLRSample)), 0x80000000 | (SF_IS_BOOL(&((Voices[8].PreLRSample))) ? 0x08000000 : 0), "Voices[8].PreLRSample" },
  { &((Voices[8].ADSR.EnvLevel)), SF_IS_BOOL(&((Voices[8].ADSR.EnvLevel))) ? 1 : sizeof((Voices[8].ADSR.EnvLevel)), 0x80000000 | (SF_IS_BOOL(&((Voices[8].ADSR.EnvLevel))) ? 0x08000000 : 0), "Voices[8].ADSR.EnvLevel" },
  { &((Voices[8].ADSR.Divider)), SF_IS_BOOL(&((Voices[8].ADSR.Divider))) ? 1 : sizeof((Voices[8].ADSR.Divider)), 0x80000000 | (SF_IS_BOOL(&((Voices[8].ADSR.Divider))) ? 0x08000000 : 0), "Voices[8].ADSR.Divider" },
  { &((Voices[8].ADSR.Phase)), SF_IS_BOOL(&((Voices[8].ADSR.Phase))) ? 1 : sizeof((Voices[8].ADSR.Phase)), 0x80000000 | (SF_IS_BOOL(&((Voices[8].ADSR.Phase))) ? 0x08000000 : 0), "Voices[8].ADSR.Phase" },
  { &((Voices[8].ADSR.AttackExp)), SF_IS_BOOL(&((Voices[8].ADSR.AttackExp))) ? 1 : sizeof((Voices[8].ADSR.AttackExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[8].ADSR.AttackExp))) ? 0x08000000 : 0), "Voices[8].ADSR.AttackExp" },
  { &((Voices[8].ADSR.SustainExp)), SF_IS_BOOL(&((Voices[8].ADSR.SustainExp))) ? 1 : sizeof((Voices[8].ADSR.SustainExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[8].ADSR.SustainExp))) ? 0x08000000 : 0), "Voices[8].ADSR.SustainExp" },
  { &((Voices[8].ADSR.SustainDec)), SF_IS_BOOL(&((Voices[8].ADSR.SustainDec))) ? 1 : sizeof((Voices[8].ADSR.SustainDec)), 0x80000000 | (SF_IS_BOOL(&((Voices[8].ADSR.SustainDec))) ? 0x08000000 : 0), "Voices[8].ADSR.SustainDec" },
  { &((Voices[8].ADSR.ReleaseExp)), SF_IS_BOOL(&((Voices[8].ADSR.ReleaseExp))) ? 1 : sizeof((Voices[8].ADSR.ReleaseExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[8].ADSR.ReleaseExp))) ? 0x08000000 : 0), "Voices[8].ADSR.ReleaseExp" },
  { &((Voices[8].ADSR.AttackRate)), SF_IS_BOOL(&((Voices[8].ADSR.AttackRate))) ? 1 : sizeof((Voices[8].ADSR.AttackRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[8].ADSR.AttackRate))) ? 0x08000000 : 0), "Voices[8].ADSR.AttackRate" },
  { &((Voices[8].ADSR.DecayRate)), SF_IS_BOOL(&((Voices[8].ADSR.DecayRate))) ? 1 : sizeof((Voices[8].ADSR.DecayRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[8].ADSR.DecayRate))) ? 0x08000000 : 0), "Voices[8].ADSR.DecayRate" },
  { &((Voices[8].ADSR.SustainRate)), SF_IS_BOOL(&((Voices[8].ADSR.SustainRate))) ? 1 : sizeof((Voices[8].ADSR.SustainRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[8].ADSR.SustainRate))) ? 0x08000000 : 0), "Voices[8].ADSR.SustainRate" },
  { &((Voices[8].ADSR.ReleaseRate)), SF_IS_BOOL(&((Voices[8].ADSR.ReleaseRate))) ? 1 : sizeof((Voices[8].ADSR.ReleaseRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[8].ADSR.ReleaseRate))) ? 0x08000000 : 0), "Voices[8].ADSR.ReleaseRate" },
  { &((Voices[8].ADSR.SustainLevel)), SF_IS_BOOL(&((Voices[8].ADSR.SustainLevel))) ? 1 : sizeof((Voices[8].ADSR.SustainLevel)), 0x80000000 | (SF_IS_BOOL(&((Voices[8].ADSR.SustainLevel))) ? 0x08000000 : 0), "Voices[8].ADSR.SustainLevel" },
  { ((&Voices[9].DecodeBuffer[0])), (uint32)(((sizeof(Voices[9].DecodeBuffer) / sizeof(Voices[9].DecodeBuffer[0]))) * sizeof(uint16)), 0x20000000 | SF_FORCE_A16((&Voices[9].DecodeBuffer[0])), "&Voices[9].DecodeBuffer[0]" },
  { &((Voices[9].DecodeM2)), SF_IS_BOOL(&((Voices[9].DecodeM2))) ? 1 : sizeof((Voices[9].DecodeM2)), 0x80000000 | (SF_IS_BOOL(&((Voices[9].DecodeM2))) ? 0x08000000 : 0), "Voices[9].DecodeM2" },
  { &((Voices[9].DecodeM1)), SF_IS_BOOL(&((Voices[9].DecodeM1))) ? 1 : sizeof((Voices[9].DecodeM1)), 0x80000000 | (SF_IS_BOOL(&((Voices[9].DecodeM1))) ? 0x08000000 : 0), "Voices[9].DecodeM1" },
  { &((Voices[9].DecodePlayDelay)), SF_IS_BOOL(&((Voices[9].DecodePlayDelay))) ? 1 : sizeof((Voices[9].DecodePlayDelay)), 0x80000000 | (SF_IS_BOOL(&((Voices[9].DecodePlayDelay))) ? 0x08000000 : 0), "Voices[9].DecodePlayDelay" },
  { &((Voices[9].DecodeWritePos)), SF_IS_BOOL(&((Voices[9].DecodeWritePos))) ? 1 : sizeof((Voices[9].DecodeWritePos)), 0x80000000 | (SF_IS_BOOL(&((Voices[9].DecodeWritePos))) ? 0x08000000 : 0), "Voices[9].DecodeWritePos" },
  { &((Voices[9].DecodeReadPos)), SF_IS_BOOL(&((Voices[9].DecodeReadPos))) ? 1 : sizeof((Voices[9].DecodeReadPos)), 0x80000000 | (SF_IS_BOOL(&((Voices[9].DecodeReadPos))) ? 0x08000000 : 0), "Voices[9].DecodeReadPos" },
  { &((Voices[9].DecodeAvail)), SF_IS_BOOL(&((Voices[9].DecodeAvail))) ? 1 : sizeof((Voices[9].DecodeAvail)), 0x80000000 | (SF_IS_BOOL(&((Voices[9].DecodeAvail))) ? 0x08000000 : 0), "Voices[9].DecodeAvail" },
  { &((Voices[9].DecodeShift)), SF_IS_BOOL(&((Voices[9].DecodeShift))) ? 1 : sizeof((Voices[9].DecodeShift)), 0x80000000 | (SF_IS_BOOL(&((Voices[9].DecodeShift))) ? 0x08000000 : 0), "Voices[9].DecodeShift" },
  { &((Voices[9].DecodeWeight)), SF_IS_BOOL(&((Voices[9].DecodeWeight))) ? 1 : sizeof((Voices[9].DecodeWeight)), 0x80000000 | (SF_IS_BOOL(&((Voices[9].DecodeWeight))) ? 0x08000000 : 0), "Voices[9].DecodeWeight" },
  { &((Voices[9].DecodeFlags)), SF_IS_BOOL(&((Voices[9].DecodeFlags))) ? 1 : sizeof((Voices[9].DecodeFlags)), 0x80000000 | (SF_IS_BOOL(&((Voices[9].DecodeFlags))) ? 0x08000000 : 0), "Voices[9].DecodeFlags" },
  { &((Voices[9].IgnoreSampLA)), SF_IS_BOOL(&((Voices[9].IgnoreSampLA))) ? 1 : sizeof((Voices[9].IgnoreSampLA)), 0x80000000 | (SF_IS_BOOL(&((Voices[9].IgnoreSampLA))) ? 0x08000000 : 0), "Voices[9].IgnoreSampLA" },
  { &(((Voices[9].Sweep[0]).Control)), SF_IS_BOOL(&(((Voices[9].Sweep[0]).Control))) ? 1 : sizeof(((Voices[9].Sweep[0]).Control)), 0x80000000 | (SF_IS_BOOL(&(((Voices[9].Sweep[0]).Control))) ? 0x08000000 : 0), "(Voices[9].Sweep[0]).Control" },
  { &(((Voices[9].Sweep[0]).Current)), SF_IS_BOOL(&(((Voices[9].Sweep[0]).Current))) ? 1 : sizeof(((Voices[9].Sweep[0]).Current)), 0x80000000 | (SF_IS_BOOL(&(((Voices[9].Sweep[0]).Current))) ? 0x08000000 : 0), "(Voices[9].Sweep[0]).Current" },
  { &(((Voices[9].Sweep[0]).Divider)), SF_IS_BOOL(&(((Voices[9].Sweep[0]).Divider))) ? 1 : sizeof(((Voices[9].Sweep[0]).Divider)), 0x80000000 | (SF_IS_BOOL(&(((Voices[9].Sweep[0]).Divider))) ? 0x08000000 : 0), "(Voices[9].Sweep[0]).Divider" },
  { &(((Voices[9].Sweep[1]).Control)), SF_IS_BOOL(&(((Voices[9].Sweep[1]).Control))) ? 1 : sizeof(((Voices[9].Sweep[1]).Control)), 0x80000000 | (SF_IS_BOOL(&(((Voices[9].Sweep[1]).Control))) ? 0x08000000 : 0), "(Voices[9].Sweep[1]).Control" },
  { &(((Voices[9].Sweep[1]).Current)), SF_IS_BOOL(&(((Voices[9].Sweep[1]).Current))) ? 1 : sizeof(((Voices[9].Sweep[1]).Current)), 0x80000000 | (SF_IS_BOOL(&(((Voices[9].Sweep[1]).Current))) ? 0x08000000 : 0), "(Voices[9].Sweep[1]).Current" },
  { &(((Voices[9].Sweep[1]).Divider)), SF_IS_BOOL(&(((Voices[9].Sweep[1]).Divider))) ? 1 : sizeof(((Voices[9].Sweep[1]).Divider)), 0x80000000 | (SF_IS_BOOL(&(((Voices[9].Sweep[1]).Divider))) ? 0x08000000 : 0), "(Voices[9].Sweep[1]).Divider" },
  { &((Voices[9].Pitch)), SF_IS_BOOL(&((Voices[9].Pitch))) ? 1 : sizeof((Voices[9].Pitch)), 0x80000000 | (SF_IS_BOOL(&((Voices[9].Pitch))) ? 0x08000000 : 0), "Voices[9].Pitch" },
  { &((Voices[9].CurPhase)), SF_IS_BOOL(&((Voices[9].CurPhase))) ? 1 : sizeof((Voices[9].CurPhase)), 0x80000000 | (SF_IS_BOOL(&((Voices[9].CurPhase))) ? 0x08000000 : 0), "Voices[9].CurPhase" },
  { &((Voices[9].StartAddr)), SF_IS_BOOL(&((Voices[9].StartAddr))) ? 1 : sizeof((Voices[9].StartAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[9].StartAddr))) ? 0x08000000 : 0), "Voices[9].StartAddr" },
  { &((Voices[9].CurAddr)), SF_IS_BOOL(&((Voices[9].CurAddr))) ? 1 : sizeof((Voices[9].CurAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[9].CurAddr))) ? 0x08000000 : 0), "Voices[9].CurAddr" },
  { &((Voices[9].ADSRControl)), SF_IS_BOOL(&((Voices[9].ADSRControl))) ? 1 : sizeof((Voices[9].ADSRControl)), 0x80000000 | (SF_IS_BOOL(&((Voices[9].ADSRControl))) ? 0x08000000 : 0), "Voices[9].ADSRControl" },
  { &((Voices[9].LoopAddr)), SF_IS_BOOL(&((Voices[9].LoopAddr))) ? 1 : sizeof((Voices[9].LoopAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[9].LoopAddr))) ? 0x08000000 : 0), "Voices[9].LoopAddr" },
  { &((Voices[9].PreLRSample)), SF_IS_BOOL(&((Voices[9].PreLRSample))) ? 1 : sizeof((Voices[9].PreLRSample)), 0x80000000 | (SF_IS_BOOL(&((Voices[9].PreLRSample))) ? 0x08000000 : 0), "Voices[9].PreLRSample" },
  { &((Voices[9].ADSR.EnvLevel)), SF_IS_BOOL(&((Voices[9].ADSR.EnvLevel))) ? 1 : sizeof((Voices[9].ADSR.EnvLevel)), 0x80000000 | (SF_IS_BOOL(&((Voices[9].ADSR.EnvLevel))) ? 0x08000000 : 0), "Voices[9].ADSR.EnvLevel" },
  { &((Voices[9].ADSR.Divider)), SF_IS_BOOL(&((Voices[9].ADSR.Divider))) ? 1 : sizeof((Voices[9].ADSR.Divider)), 0x80000000 | (SF_IS_BOOL(&((Voices[9].ADSR.Divider))) ? 0x08000000 : 0), "Voices[9].ADSR.Divider" },
  { &((Voices[9].ADSR.Phase)), SF_IS_BOOL(&((Voices[9].ADSR.Phase))) ? 1 : sizeof((Voices[9].ADSR.Phase)), 0x80000000 | (SF_IS_BOOL(&((Voices[9].ADSR.Phase))) ? 0x08000000 : 0), "Voices[9].ADSR.Phase" },
  { &((Voices[9].ADSR.AttackExp)), SF_IS_BOOL(&((Voices[9].ADSR.AttackExp))) ? 1 : sizeof((Voices[9].ADSR.AttackExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[9].ADSR.AttackExp))) ? 0x08000000 : 0), "Voices[9].ADSR.AttackExp" },
  { &((Voices[9].ADSR.SustainExp)), SF_IS_BOOL(&((Voices[9].ADSR.SustainExp))) ? 1 : sizeof((Voices[9].ADSR.SustainExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[9].ADSR.SustainExp))) ? 0x08000000 : 0), "Voices[9].ADSR.SustainExp" },
  { &((Voices[9].ADSR.SustainDec)), SF_IS_BOOL(&((Voices[9].ADSR.SustainDec))) ? 1 : sizeof((Voices[9].ADSR.SustainDec)), 0x80000000 | (SF_IS_BOOL(&((Voices[9].ADSR.SustainDec))) ? 0x08000000 : 0), "Voices[9].ADSR.SustainDec" },
  { &((Voices[9].ADSR.ReleaseExp)), SF_IS_BOOL(&((Voices[9].ADSR.ReleaseExp))) ? 1 : sizeof((Voices[9].ADSR.ReleaseExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[9].ADSR.ReleaseExp))) ? 0x08000000 : 0), "Voices[9].ADSR.ReleaseExp" },
  { &((Voices[9].ADSR.AttackRate)), SF_IS_BOOL(&((Voices[9].ADSR.AttackRate))) ? 1 : sizeof((Voices[9].ADSR.AttackRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[9].ADSR.AttackRate))) ? 0x08000000 : 0), "Voices[9].ADSR.AttackRate" },
  { &((Voices[9].ADSR.DecayRate)), SF_IS_BOOL(&((Voices[9].ADSR.DecayRate))) ? 1 : sizeof((Voices[9].ADSR.DecayRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[9].ADSR.DecayRate))) ? 0x08000000 : 0), "Voices[9].ADSR.DecayRate" },
  { &((Voices[9].ADSR.SustainRate)), SF_IS_BOOL(&((Voices[9].ADSR.SustainRate))) ? 1 : sizeof((Voices[9].ADSR.SustainRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[9].ADSR.SustainRate))) ? 0x08000000 : 0), "Voices[9].ADSR.SustainRate" },
  { &((Voices[9].ADSR.ReleaseRate)), SF_IS_BOOL(&((Voices[9].ADSR.ReleaseRate))) ? 1 : sizeof((Voices[9].ADSR.ReleaseRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[9].ADSR.ReleaseRate))) ? 0x08000000 : 0), "Voices[9].ADSR.ReleaseRate" },
  { &((Voices[9].ADSR.SustainLevel)), SF_IS_BOOL(&((Voices[9].ADSR.SustainLevel))) ? 1 : sizeof((Voices[9].ADSR.SustainLevel)), 0x80000000 | (SF_IS_BOOL(&((Voices[9].ADSR.SustainLevel))) ? 0x08000000 : 0), "Voices[9].ADSR.SustainLevel" },
  { ((&Voices[10].DecodeBuffer[0])), (uint32)(((sizeof(Voices[10].DecodeBuffer) / sizeof(Voices[10].DecodeBuffer[0]))) * sizeof(uint16)), 0x20000000 | SF_FORCE_A16((&Voices[10].DecodeBuffer[0])), "&Voices[10].DecodeBuffer[0]" },
  { &((Voices[10].DecodeM2)), SF_IS_BOOL(&((Voices[10].DecodeM2))) ? 1 : sizeof((Voices[10].DecodeM2)), 0x80000000 | (SF_IS_BOOL(&((Voices[10].DecodeM2))) ? 0x08000000 : 0), "Voices[10].DecodeM2" },
  { &((Voices[10].DecodeM1)), SF_IS_BOOL(&((Voices[10].DecodeM1))) ? 1 : sizeof((Voices[10].DecodeM1)), 0x80000000 | (SF_IS_BOOL(&((Voices[10].DecodeM1))) ? 0x08000000 : 0), "Voices[10].DecodeM1" },
  { &((Voices[10].DecodePlayDelay)), SF_IS_BOOL(&((Voices[10].DecodePlayDelay))) ? 1 : sizeof((Voices[10].DecodePlayDelay)), 0x80000000 | (SF_IS_BOOL(&((Voices[10].DecodePlayDelay))) ? 0x08000000 : 0), "Voices[10].DecodePlayDelay" },
  { &((Voices[10].DecodeWritePos)), SF_IS_BOOL(&((Voices[10].DecodeWritePos))) ? 1 : sizeof((Voices[10].DecodeWritePos)), 0x80000000 | (SF_IS_BOOL(&((Voices[10].DecodeWritePos))) ? 0x08000000 : 0), "Voices[10].DecodeWritePos" },
  { &((Voices[10].DecodeReadPos)), SF_IS_BOOL(&((Voices[10].DecodeReadPos))) ? 1 : sizeof((Voices[10].DecodeReadPos)), 0x80000000 | (SF_IS_BOOL(&((Voices[10].DecodeReadPos))) ? 0x08000000 : 0), "Voices[10].DecodeReadPos" },
  { &((Voices[10].DecodeAvail)), SF_IS_BOOL(&((Voices[10].DecodeAvail))) ? 1 : sizeof((Voices[10].DecodeAvail)), 0x80000000 | (SF_IS_BOOL(&((Voices[10].DecodeAvail))) ? 0x08000000 : 0), "Voices[10].DecodeAvail" },
  { &((Voices[10].DecodeShift)), SF_IS_BOOL(&((Voices[10].DecodeShift))) ? 1 : sizeof((Voices[10].DecodeShift)), 0x80000000 | (SF_IS_BOOL(&((Voices[10].DecodeShift))) ? 0x08000000 : 0), "Voices[10].DecodeShift" },
  { &((Voices[10].DecodeWeight)), SF_IS_BOOL(&((Voices[10].DecodeWeight))) ? 1 : sizeof((Voices[10].DecodeWeight)), 0x80000000 | (SF_IS_BOOL(&((Voices[10].DecodeWeight))) ? 0x08000000 : 0), "Voices[10].DecodeWeight" },
  { &((Voices[10].DecodeFlags)), SF_IS_BOOL(&((Voices[10].DecodeFlags))) ? 1 : sizeof((Voices[10].DecodeFlags)), 0x80000000 | (SF_IS_BOOL(&((Voices[10].DecodeFlags))) ? 0x08000000 : 0), "Voices[10].DecodeFlags" },
  { &((Voices[10].IgnoreSampLA)), SF_IS_BOOL(&((Voices[10].IgnoreSampLA))) ? 1 : sizeof((Voices[10].IgnoreSampLA)), 0x80000000 | (SF_IS_BOOL(&((Voices[10].IgnoreSampLA))) ? 0x08000000 : 0), "Voices[10].IgnoreSampLA" },
  { &(((Voices[10].Sweep[0]).Control)), SF_IS_BOOL(&(((Voices[10].Sweep[0]).Control))) ? 1 : sizeof(((Voices[10].Sweep[0]).Control)), 0x80000000 | (SF_IS_BOOL(&(((Voices[10].Sweep[0]).Control))) ? 0x08000000 : 0), "(Voices[10].Sweep[0]).Control" },
  { &(((Voices[10].Sweep[0]).Current)), SF_IS_BOOL(&(((Voices[10].Sweep[0]).Current))) ? 1 : sizeof(((Voices[10].Sweep[0]).Current)), 0x80000000 | (SF_IS_BOOL(&(((Voices[10].Sweep[0]).Current))) ? 0x08000000 : 0), "(Voices[10].Sweep[0]).Current" },
  { &(((Voices[10].Sweep[0]).Divider)), SF_IS_BOOL(&(((Voices[10].Sweep[0]).Divider))) ? 1 : sizeof(((Voices[10].Sweep[0]).Divider)), 0x80000000 | (SF_IS_BOOL(&(((Voices[10].Sweep[0]).Divider))) ? 0x08000000 : 0), "(Voices[10].Sweep[0]).Divider" },
  { &(((Voices[10].Sweep[1]).Control)), SF_IS_BOOL(&(((Voices[10].Sweep[1]).Control))) ? 1 : sizeof(((Voices[10].Sweep[1]).Control)), 0x80000000 | (SF_IS_BOOL(&(((Voices[10].Sweep[1]).Control))) ? 0x08000000 : 0), "(Voices[10].Sweep[1]).Control" },
  { &(((Voices[10].Sweep[1]).Current)), SF_IS_BOOL(&(((Voices[10].Sweep[1]).Current))) ? 1 : sizeof(((Voices[10].Sweep[1]).Current)), 0x80000000 | (SF_IS_BOOL(&(((Voices[10].Sweep[1]).Current))) ? 0x08000000 : 0), "(Voices[10].Sweep[1]).Current" },
  { &(((Voices[10].Sweep[1]).Divider)), SF_IS_BOOL(&(((Voices[10].Sweep[1]).Divider))) ? 1 : sizeof(((Voices[10].Sweep[1]).Divider)), 0x80000000 | (SF_IS_BOOL(&(((Voices[10].Sweep[1]).Divider))) ? 0x08000000 : 0), "(Voices[10].Sweep[1]).Divider" },
  { &((Voices[10].Pitch)), SF_IS_BOOL(&((Voices[10].Pitch))) ? 1 : sizeof((Voices[10].Pitch)), 0x80000000 | (SF_IS_BOOL(&((Voices[10].Pitch))) ? 0x08000000 : 0), "Voices[10].Pitch" },
  { &((Voices[10].CurPhase)), SF_IS_BOOL(&((Voices[10].CurPhase))) ? 1 : sizeof((Voices[10].CurPhase)), 0x80000000 | (SF_IS_BOOL(&((Voices[10].CurPhase))) ? 0x08000000 : 0), "Voices[10].CurPhase" },
  { &((Voices[10].StartAddr)), SF_IS_BOOL(&((Voices[10].StartAddr))) ? 1 : sizeof((Voices[10].StartAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[10].StartAddr))) ? 0x08000000 : 0), "Voices[10].StartAddr" },
  { &((Voices[10].CurAddr)), SF_IS_BOOL(&((Voices[10].CurAddr))) ? 1 : sizeof((Voices[10].CurAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[10].CurAddr))) ? 0x08000000 : 0), "Voices[10].CurAddr" },
  { &((Voices[10].ADSRControl)), SF_IS_BOOL(&((Voices[10].ADSRControl))) ? 1 : sizeof((Voices[10].ADSRControl)), 0x80000000 | (SF_IS_BOOL(&((Voices[10].ADSRControl))) ? 0x08000000 : 0), "Voices[10].ADSRControl" },
  { &((Voices[10].LoopAddr)), SF_IS_BOOL(&((Voices[10].LoopAddr))) ? 1 : sizeof((Voices[10].LoopAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[10].LoopAddr))) ? 0x08000000 : 0), "Voices[10].LoopAddr" },
  { &((Voices[10].PreLRSample)), SF_IS_BOOL(&((Voices[10].PreLRSample))) ? 1 : sizeof((Voices[10].PreLRSample)), 0x80000000 | (SF_IS_BOOL(&((Voices[10].PreLRSample))) ? 0x08000000 : 0), "Voices[10].PreLRSample" },
  { &((Voices[10].ADSR.EnvLevel)), SF_IS_BOOL(&((Voices[10].ADSR.EnvLevel))) ? 1 : sizeof((Voices[10].ADSR.EnvLevel)), 0x80000000 | (SF_IS_BOOL(&((Voices[10].ADSR.EnvLevel))) ? 0x08000000 : 0), "Voices[10].ADSR.EnvLevel" },
  { &((Voices[10].ADSR.Divider)), SF_IS_BOOL(&((Voices[10].ADSR.Divider))) ? 1 : sizeof((Voices[10].ADSR.Divider)), 0x80000000 | (SF_IS_BOOL(&((Voices[10].ADSR.Divider))) ? 0x08000000 : 0), "Voices[10].ADSR.Divider" },
  { &((Voices[10].ADSR.Phase)), SF_IS_BOOL(&((Voices[10].ADSR.Phase))) ? 1 : sizeof((Voices[10].ADSR.Phase)), 0x80000000 | (SF_IS_BOOL(&((Voices[10].ADSR.Phase))) ? 0x08000000 : 0), "Voices[10].ADSR.Phase" },
  { &((Voices[10].ADSR.AttackExp)), SF_IS_BOOL(&((Voices[10].ADSR.AttackExp))) ? 1 : sizeof((Voices[10].ADSR.AttackExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[10].ADSR.AttackExp))) ? 0x08000000 : 0), "Voices[10].ADSR.AttackExp" },
  { &((Voices[10].ADSR.SustainExp)), SF_IS_BOOL(&((Voices[10].ADSR.SustainExp))) ? 1 : sizeof((Voices[10].ADSR.SustainExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[10].ADSR.SustainExp))) ? 0x08000000 : 0), "Voices[10].ADSR.SustainExp" },
  { &((Voices[10].ADSR.SustainDec)), SF_IS_BOOL(&((Voices[10].ADSR.SustainDec))) ? 1 : sizeof((Voices[10].ADSR.SustainDec)), 0x80000000 | (SF_IS_BOOL(&((Voices[10].ADSR.SustainDec))) ? 0x08000000 : 0), "Voices[10].ADSR.SustainDec" },
  { &((Voices[10].ADSR.ReleaseExp)), SF_IS_BOOL(&((Voices[10].ADSR.ReleaseExp))) ? 1 : sizeof((Voices[10].ADSR.ReleaseExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[10].ADSR.ReleaseExp))) ? 0x08000000 : 0), "Voices[10].ADSR.ReleaseExp" },
  { &((Voices[10].ADSR.AttackRate)), SF_IS_BOOL(&((Voices[10].ADSR.AttackRate))) ? 1 : sizeof((Voices[10].ADSR.AttackRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[10].ADSR.AttackRate))) ? 0x08000000 : 0), "Voices[10].ADSR.AttackRate" },
  { &((Voices[10].ADSR.DecayRate)), SF_IS_BOOL(&((Voices[10].ADSR.DecayRate))) ? 1 : sizeof((Voices[10].ADSR.DecayRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[10].ADSR.DecayRate))) ? 0x08000000 : 0), "Voices[10].ADSR.DecayRate" },
  { &((Voices[10].ADSR.SustainRate)), SF_IS_BOOL(&((Voices[10].ADSR.SustainRate))) ? 1 : sizeof((Voices[10].ADSR.SustainRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[10].ADSR.SustainRate))) ? 0x08000000 : 0), "Voices[10].ADSR.SustainRate" },
  { &((Voices[10].ADSR.ReleaseRate)), SF_IS_BOOL(&((Voices[10].ADSR.ReleaseRate))) ? 1 : sizeof((Voices[10].ADSR.ReleaseRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[10].ADSR.ReleaseRate))) ? 0x08000000 : 0), "Voices[10].ADSR.ReleaseRate" },
  { &((Voices[10].ADSR.SustainLevel)), SF_IS_BOOL(&((Voices[10].ADSR.SustainLevel))) ? 1 : sizeof((Voices[10].ADSR.SustainLevel)), 0x80000000 | (SF_IS_BOOL(&((Voices[10].ADSR.SustainLevel))) ? 0x08000000 : 0), "Voices[10].ADSR.SustainLevel" },
  { ((&Voices[11].DecodeBuffer[0])), (uint32)(((sizeof(Voices[11].DecodeBuffer) / sizeof(Voices[11].DecodeBuffer[0]))) * sizeof(uint16)), 0x20000000 | SF_FORCE_A16((&Voices[11].DecodeBuffer[0])), "&Voices[11].DecodeBuffer[0]" },
  { &((Voices[11].DecodeM2)), SF_IS_BOOL(&((Voices[11].DecodeM2))) ? 1 : sizeof((Voices[11].DecodeM2)), 0x80000000 | (SF_IS_BOOL(&((Voices[11].DecodeM2))) ? 0x08000000 : 0), "Voices[11].DecodeM2" },
  { &((Voices[11].DecodeM1)), SF_IS_BOOL(&((Voices[11].DecodeM1))) ? 1 : sizeof((Voices[11].DecodeM1)), 0x80000000 | (SF_IS_BOOL(&((Voices[11].DecodeM1))) ? 0x08000000 : 0), "Voices[11].DecodeM1" },
  { &((Voices[11].DecodePlayDelay)), SF_IS_BOOL(&((Voices[11].DecodePlayDelay))) ? 1 : sizeof((Voices[11].DecodePlayDelay)), 0x80000000 | (SF_IS_BOOL(&((Voices[11].DecodePlayDelay))) ? 0x08000000 : 0), "Voices[11].DecodePlayDelay" },
  { &((Voices[11].DecodeWritePos)), SF_IS_BOOL(&((Voices[11].DecodeWritePos))) ? 1 : sizeof((Voices[11].DecodeWritePos)), 0x80000000 | (SF_IS_BOOL(&((Voices[11].DecodeWritePos))) ? 0x08000000 : 0), "Voices[11].DecodeWritePos" },
  { &((Voices[11].DecodeReadPos)), SF_IS_BOOL(&((Voices[11].DecodeReadPos))) ? 1 : sizeof((Voices[11].DecodeReadPos)), 0x80000000 | (SF_IS_BOOL(&((Voices[11].DecodeReadPos))) ? 0x08000000 : 0), "Voices[11].DecodeReadPos" },
  { &((Voices[11].DecodeAvail)), SF_IS_BOOL(&((Voices[11].DecodeAvail))) ? 1 : sizeof((Voices[11].DecodeAvail)), 0x80000000 | (SF_IS_BOOL(&((Voices[11].DecodeAvail))) ? 0x08000000 : 0), "Voices[11].DecodeAvail" },
  { &((Voices[11].DecodeShift)), SF_IS_BOOL(&((Voices[11].DecodeShift))) ? 1 : sizeof((Voices[11].DecodeShift)), 0x80000000 | (SF_IS_BOOL(&((Voices[11].DecodeShift))) ? 0x08000000 : 0), "Voices[11].DecodeShift" },
  { &((Voices[11].DecodeWeight)), SF_IS_BOOL(&((Voices[11].DecodeWeight))) ? 1 : sizeof((Voices[11].DecodeWeight)), 0x80000000 | (SF_IS_BOOL(&((Voices[11].DecodeWeight))) ? 0x08000000 : 0), "Voices[11].DecodeWeight" },
  { &((Voices[11].DecodeFlags)), SF_IS_BOOL(&((Voices[11].DecodeFlags))) ? 1 : sizeof((Voices[11].DecodeFlags)), 0x80000000 | (SF_IS_BOOL(&((Voices[11].DecodeFlags))) ? 0x08000000 : 0), "Voices[11].DecodeFlags" },
  { &((Voices[11].IgnoreSampLA)), SF_IS_BOOL(&((Voices[11].IgnoreSampLA))) ? 1 : sizeof((Voices[11].IgnoreSampLA)), 0x80000000 | (SF_IS_BOOL(&((Voices[11].IgnoreSampLA))) ? 0x08000000 : 0), "Voices[11].IgnoreSampLA" },
  { &(((Voices[11].Sweep[0]).Control)), SF_IS_BOOL(&(((Voices[11].Sweep[0]).Control))) ? 1 : sizeof(((Voices[11].Sweep[0]).Control)), 0x80000000 | (SF_IS_BOOL(&(((Voices[11].Sweep[0]).Control))) ? 0x08000000 : 0), "(Voices[11].Sweep[0]).Control" },
  { &(((Voices[11].Sweep[0]).Current)), SF_IS_BOOL(&(((Voices[11].Sweep[0]).Current))) ? 1 : sizeof(((Voices[11].Sweep[0]).Current)), 0x80000000 | (SF_IS_BOOL(&(((Voices[11].Sweep[0]).Current))) ? 0x08000000 : 0), "(Voices[11].Sweep[0]).Current" },
  { &(((Voices[11].Sweep[0]).Divider)), SF_IS_BOOL(&(((Voices[11].Sweep[0]).Divider))) ? 1 : sizeof(((Voices[11].Sweep[0]).Divider)), 0x80000000 | (SF_IS_BOOL(&(((Voices[11].Sweep[0]).Divider))) ? 0x08000000 : 0), "(Voices[11].Sweep[0]).Divider" },
  { &(((Voices[11].Sweep[1]).Control)), SF_IS_BOOL(&(((Voices[11].Sweep[1]).Control))) ? 1 : sizeof(((Voices[11].Sweep[1]).Control)), 0x80000000 | (SF_IS_BOOL(&(((Voices[11].Sweep[1]).Control))) ? 0x08000000 : 0), "(Voices[11].Sweep[1]).Control" },
  { &(((Voices[11].Sweep[1]).Current)), SF_IS_BOOL(&(((Voices[11].Sweep[1]).Current))) ? 1 : sizeof(((Voices[11].Sweep[1]).Current)), 0x80000000 | (SF_IS_BOOL(&(((Voices[11].Sweep[1]).Current))) ? 0x08000000 : 0), "(Voices[11].Sweep[1]).Current" },
  { &(((Voices[11].Sweep[1]).Divider)), SF_IS_BOOL(&(((Voices[11].Sweep[1]).Divider))) ? 1 : sizeof(((Voices[11].Sweep[1]).Divider)), 0x80000000 | (SF_IS_BOOL(&(((Voices[11].Sweep[1]).Divider))) ? 0x08000000 : 0), "(Voices[11].Sweep[1]).Divider" },
  { &((Voices[11].Pitch)), SF_IS_BOOL(&((Voices[11].Pitch))) ? 1 : sizeof((Voices[11].Pitch)), 0x80000000 | (SF_IS_BOOL(&((Voices[11].Pitch))) ? 0x08000000 : 0), "Voices[11].Pitch" },
  { &((Voices[11].CurPhase)), SF_IS_BOOL(&((Voices[11].CurPhase))) ? 1 : sizeof((Voices[11].CurPhase)), 0x80000000 | (SF_IS_BOOL(&((Voices[11].CurPhase))) ? 0x08000000 : 0), "Voices[11].CurPhase" },
  { &((Voices[11].StartAddr)), SF_IS_BOOL(&((Voices[11].StartAddr))) ? 1 : sizeof((Voices[11].StartAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[11].StartAddr))) ? 0x08000000 : 0), "Voices[11].StartAddr" },
  { &((Voices[11].CurAddr)), SF_IS_BOOL(&((Voices[11].CurAddr))) ? 1 : sizeof((Voices[11].CurAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[11].CurAddr))) ? 0x08000000 : 0), "Voices[11].CurAddr" },
  { &((Voices[11].ADSRControl)), SF_IS_BOOL(&((Voices[11].ADSRControl))) ? 1 : sizeof((Voices[11].ADSRControl)), 0x80000000 | (SF_IS_BOOL(&((Voices[11].ADSRControl))) ? 0x08000000 : 0), "Voices[11].ADSRControl" },
  { &((Voices[11].LoopAddr)), SF_IS_BOOL(&((Voices[11].LoopAddr))) ? 1 : sizeof((Voices[11].LoopAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[11].LoopAddr))) ? 0x08000000 : 0), "Voices[11].LoopAddr" },
  { &((Voices[11].PreLRSample)), SF_IS_BOOL(&((Voices[11].PreLRSample))) ? 1 : sizeof((Voices[11].PreLRSample)), 0x80000000 | (SF_IS_BOOL(&((Voices[11].PreLRSample))) ? 0x08000000 : 0), "Voices[11].PreLRSample" },
  { &((Voices[11].ADSR.EnvLevel)), SF_IS_BOOL(&((Voices[11].ADSR.EnvLevel))) ? 1 : sizeof((Voices[11].ADSR.EnvLevel)), 0x80000000 | (SF_IS_BOOL(&((Voices[11].ADSR.EnvLevel))) ? 0x08000000 : 0), "Voices[11].ADSR.EnvLevel" },
  { &((Voices[11].ADSR.Divider)), SF_IS_BOOL(&((Voices[11].ADSR.Divider))) ? 1 : sizeof((Voices[11].ADSR.Divider)), 0x80000000 | (SF_IS_BOOL(&((Voices[11].ADSR.Divider))) ? 0x08000000 : 0), "Voices[11].ADSR.Divider" },
  { &((Voices[11].ADSR.Phase)), SF_IS_BOOL(&((Voices[11].ADSR.Phase))) ? 1 : sizeof((Voices[11].ADSR.Phase)), 0x80000000 | (SF_IS_BOOL(&((Voices[11].ADSR.Phase))) ? 0x08000000 : 0), "Voices[11].ADSR.Phase" },
  { &((Voices[11].ADSR.AttackExp)), SF_IS_BOOL(&((Voices[11].ADSR.AttackExp))) ? 1 : sizeof((Voices[11].ADSR.AttackExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[11].ADSR.AttackExp))) ? 0x08000000 : 0), "Voices[11].ADSR.AttackExp" },
  { &((Voices[11].ADSR.SustainExp)), SF_IS_BOOL(&((Voices[11].ADSR.SustainExp))) ? 1 : sizeof((Voices[11].ADSR.SustainExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[11].ADSR.SustainExp))) ? 0x08000000 : 0), "Voices[11].ADSR.SustainExp" },
  { &((Voices[11].ADSR.SustainDec)), SF_IS_BOOL(&((Voices[11].ADSR.SustainDec))) ? 1 : sizeof((Voices[11].ADSR.SustainDec)), 0x80000000 | (SF_IS_BOOL(&((Voices[11].ADSR.SustainDec))) ? 0x08000000 : 0), "Voices[11].ADSR.SustainDec" },
  { &((Voices[11].ADSR.ReleaseExp)), SF_IS_BOOL(&((Voices[11].ADSR.ReleaseExp))) ? 1 : sizeof((Voices[11].ADSR.ReleaseExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[11].ADSR.ReleaseExp))) ? 0x08000000 : 0), "Voices[11].ADSR.ReleaseExp" },
  { &((Voices[11].ADSR.AttackRate)), SF_IS_BOOL(&((Voices[11].ADSR.AttackRate))) ? 1 : sizeof((Voices[11].ADSR.AttackRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[11].ADSR.AttackRate))) ? 0x08000000 : 0), "Voices[11].ADSR.AttackRate" },
  { &((Voices[11].ADSR.DecayRate)), SF_IS_BOOL(&((Voices[11].ADSR.DecayRate))) ? 1 : sizeof((Voices[11].ADSR.DecayRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[11].ADSR.DecayRate))) ? 0x08000000 : 0), "Voices[11].ADSR.DecayRate" },
  { &((Voices[11].ADSR.SustainRate)), SF_IS_BOOL(&((Voices[11].ADSR.SustainRate))) ? 1 : sizeof((Voices[11].ADSR.SustainRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[11].ADSR.SustainRate))) ? 0x08000000 : 0), "Voices[11].ADSR.SustainRate" },
  { &((Voices[11].ADSR.ReleaseRate)), SF_IS_BOOL(&((Voices[11].ADSR.ReleaseRate))) ? 1 : sizeof((Voices[11].ADSR.ReleaseRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[11].ADSR.ReleaseRate))) ? 0x08000000 : 0), "Voices[11].ADSR.ReleaseRate" },
  { &((Voices[11].ADSR.SustainLevel)), SF_IS_BOOL(&((Voices[11].ADSR.SustainLevel))) ? 1 : sizeof((Voices[11].ADSR.SustainLevel)), 0x80000000 | (SF_IS_BOOL(&((Voices[11].ADSR.SustainLevel))) ? 0x08000000 : 0), "Voices[11].ADSR.SustainLevel" },
  { ((&Voices[12].DecodeBuffer[0])), (uint32)(((sizeof(Voices[12].DecodeBuffer) / sizeof(Voices[12].DecodeBuffer[0]))) * sizeof(uint16)), 0x20000000 | SF_FORCE_A16((&Voices[12].DecodeBuffer[0])), "&Voices[12].DecodeBuffer[0]" }, { &((Voices[12].DecodeM2)), SF_IS_BOOL(&((Voices[12].DecodeM2))) ? 1 : sizeof((Voices[12].DecodeM2)), 0x80000000 | (SF_IS_BOOL(&((Voices[12].DecodeM2))) ? 0x08000000 : 0), "Voices[12].DecodeM2" }, { &((Voices[12].DecodeM1)), SF_IS_BOOL(&((Voices[12].DecodeM1))) ? 1 : sizeof((Voices[12].DecodeM1)), 0x80000000 | (SF_IS_BOOL(&((Voices[12].DecodeM1))) ? 0x08000000 : 0), "Voices[12].DecodeM1" }, { &((Voices[12].DecodePlayDelay)), SF_IS_BOOL(&((Voices[12].DecodePlayDelay))) ? 1 : sizeof((Voices[12].DecodePlayDelay)), 0x80000000 | (SF_IS_BOOL(&((Voices[12].DecodePlayDelay))) ? 0x08000000 : 0), "Voices[12].DecodePlayDelay" }, { &((Voices[12].DecodeWritePos)), SF_IS_BOOL(&((Voices[12].DecodeWritePos))) ? 1 : sizeof((Voices[12].DecodeWritePos)), 0x80000000 | (SF_IS_BOOL(&((Voices[12].DecodeWritePos))) ? 0x08000000 : 0), "Voices[12].DecodeWritePos" }, { &((Voices[12].DecodeReadPos)), SF_IS_BOOL(&((Voices[12].DecodeReadPos))) ? 1 : sizeof((Voices[12].DecodeReadPos)), 0x80000000 | (SF_IS_BOOL(&((Voices[12].DecodeReadPos))) ? 0x08000000 : 0), "Voices[12].DecodeReadPos" }, { &((Voices[12].DecodeAvail)), SF_IS_BOOL(&((Voices[12].DecodeAvail))) ? 1 : sizeof((Voices[12].DecodeAvail)), 0x80000000 | (SF_IS_BOOL(&((Voices[12].DecodeAvail))) ? 0x08000000 : 0), "Voices[12].DecodeAvail" }, { &((Voices[12].DecodeShift)), SF_IS_BOOL(&((Voices[12].DecodeShift))) ? 1 : sizeof((Voices[12].DecodeShift)), 0x80000000 | (SF_IS_BOOL(&((Voices[12].DecodeShift))) ? 0x08000000 : 0), "Voices[12].DecodeShift" }, { &((Voices[12].DecodeWeight)), SF_IS_BOOL(&((Voices[12].DecodeWeight))) ? 1 : sizeof((Voices[12].DecodeWeight)), 0x80000000 | (SF_IS_BOOL(&((Voices[12].DecodeWeight))) ? 0x08000000 : 0), "Voices[12].DecodeWeight" }, { &((Voices[12].DecodeFlags)), SF_IS_BOOL(&((Voices[12].DecodeFlags))) ? 1 : sizeof((Voices[12].DecodeFlags)), 0x80000000 | (SF_IS_BOOL(&((Voices[12].DecodeFlags))) ? 0x08000000 : 0), "Voices[12].DecodeFlags" }, { &((Voices[12].IgnoreSampLA)), SF_IS_BOOL(&((Voices[12].IgnoreSampLA))) ? 1 : sizeof((Voices[12].IgnoreSampLA)), 0x80000000 | (SF_IS_BOOL(&((Voices[12].IgnoreSampLA))) ? 0x08000000 : 0), "Voices[12].IgnoreSampLA" }, { &(((Voices[12].Sweep[0]).Control)), SF_IS_BOOL(&(((Voices[12].Sweep[0]).Control))) ? 1 : sizeof(((Voices[12].Sweep[0]).Control)), 0x80000000 | (SF_IS_BOOL(&(((Voices[12].Sweep[0]).Control))) ? 0x08000000 : 0), "(Voices[12].Sweep[0]).Control" }, { &(((Voices[12].Sweep[0]).Current)), SF_IS_BOOL(&(((Voices[12].Sweep[0]).Current))) ? 1 : sizeof(((Voices[12].Sweep[0]).Current)), 0x80000000 | (SF_IS_BOOL(&(((Voices[12].Sweep[0]).Current))) ? 0x08000000 : 0), "(Voices[12].Sweep[0]).Current" }, { &(((Voices[12].Sweep[0]).Divider)), SF_IS_BOOL(&(((Voices[12].Sweep[0]).Divider))) ? 1 : sizeof(((Voices[12].Sweep[0]).Divider)), 0x80000000 | (SF_IS_BOOL(&(((Voices[12].Sweep[0]).Divider))) ? 0x08000000 : 0), "(Voices[12].Sweep[0]).Divider" }, { &(((Voices[12].Sweep[1]).Control)), SF_IS_BOOL(&(((Voices[12].Sweep[1]).Control))) ? 1 : sizeof(((Voices[12].Sweep[1]).Control)), 0x80000000 | (SF_IS_BOOL(&(((Voices[12].Sweep[1]).Control))) ? 0x08000000 : 0), "(Voices[12].Sweep[1]).Control" }, { &(((Voices[12].Sweep[1]).Current)), SF_IS_BOOL(&(((Voices[12].Sweep[1]).Current))) ? 1 : sizeof(((Voices[12].Sweep[1]).Current)), 0x80000000 | (SF_IS_BOOL(&(((Voices[12].Sweep[1]).Current))) ? 0x08000000 : 0), "(Voices[12].Sweep[1]).Current" }, { &(((Voices[12].Sweep[1]).Divider)), SF_IS_BOOL(&(((Voices[12].Sweep[1]).Divider))) ? 1 : sizeof(((Voices[12].Sweep[1]).Divider)), 0x80000000 | (SF_IS_BOOL(&(((Voices[12].Sweep[1]).Divider))) ? 0x08000000 : 0), "(Voices[12].Sweep[1]).Divider" }, { &((Voices[12].Pitch)), SF_IS_BOOL(&((Voices[12].Pitch))) ? 1 : sizeof((Voices[12].Pitch)), 0x80000000 | (SF_IS_BOOL(&((Voices[12].Pitch))) ? 0x08000000 : 0), "Voices[12].Pitch" }, { &((Voices[12].CurPhase)), SF_IS_BOOL(&((Voices[12].CurPhase))) ? 1 : sizeof((Voices[12].CurPhase)), 0x80000000 | (SF_IS_BOOL(&((Voices[12].CurPhase))) ? 0x08000000 : 0), "Voices[12].CurPhase" }, { &((Voices[12].StartAddr)), SF_IS_BOOL(&((Voices[12].StartAddr))) ? 1 : sizeof((Voices[12].StartAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[12].StartAddr))) ? 0x08000000 : 0), "Voices[12].StartAddr" }, { &((Voices[12].CurAddr)), SF_IS_BOOL(&((Voices[12].CurAddr))) ? 1 : sizeof((Voices[12].CurAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[12].CurAddr))) ? 0x08000000 : 0), "Voices[12].CurAddr" }, { &((Voices[12].ADSRControl)), SF_IS_BOOL(&((Voices[12].ADSRControl))) ? 1 : sizeof((Voices[12].ADSRControl)), 0x80000000 | (SF_IS_BOOL(&((Voices[12].ADSRControl))) ? 0x08000000 : 0), "Voices[12].ADSRControl" }, { &((Voices[12].LoopAddr)), SF_IS_BOOL(&((Voices[12].LoopAddr))) ? 1 : sizeof((Voices[12].LoopAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[12].LoopAddr))) ? 0x08000000 : 0), "Voices[12].LoopAddr" }, { &((Voices[12].PreLRSample)), SF_IS_BOOL(&((Voices[12].PreLRSample))) ? 1 : sizeof((Voices[12].PreLRSample)), 0x80000000 | (SF_IS_BOOL(&((Voices[12].PreLRSample))) ? 0x08000000 : 0), "Voices[12].PreLRSample" }, { &((Voices[12].ADSR.EnvLevel)), SF_IS_BOOL(&((Voices[12].ADSR.EnvLevel))) ? 1 : sizeof((Voices[12].ADSR.EnvLevel)), 0x80000000 | (SF_IS_BOOL(&((Voices[12].ADSR.EnvLevel))) ? 0x08000000 : 0), "Voices[12].ADSR.EnvLevel" }, { &((Voices[12].ADSR.Divider)), SF_IS_BOOL(&((Voices[12].ADSR.Divider))) ? 1 : sizeof((Voices[12].ADSR.Divider)), 0x80000000 | (SF_IS_BOOL(&((Voices[12].ADSR.Divider))) ? 0x08000000 : 0), "Voices[12].ADSR.Divider" }, { &((Voices[12].ADSR.Phase)), SF_IS_BOOL(&((Voices[12].ADSR.Phase))) ? 1 : sizeof((Voices[12].ADSR.Phase)), 0x80000000 | (SF_IS_BOOL(&((Voices[12].ADSR.Phase))) ? 0x08000000 : 0), "Voices[12].ADSR.Phase" }, { &((Voices[12].ADSR.AttackExp)), SF_IS_BOOL(&((Voices[12].ADSR.AttackExp))) ? 1 : sizeof((Voices[12].ADSR.AttackExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[12].ADSR.AttackExp))) ? 0x08000000 : 0), "Voices[12].ADSR.AttackExp" }, { &((Voices[12].ADSR.SustainExp)), SF_IS_BOOL(&((Voices[12].ADSR.SustainExp))) ? 1 : sizeof((Voices[12].ADSR.SustainExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[12].ADSR.SustainExp))) ? 0x08000000 : 0), "Voices[12].ADSR.SustainExp" }, { &((Voices[12].ADSR.SustainDec)), SF_IS_BOOL(&((Voices[12].ADSR.SustainDec))) ? 1 : sizeof((Voices[12].ADSR.SustainDec)), 0x80000000 | (SF_IS_BOOL(&((Voices[12].ADSR.SustainDec))) ? 0x08000000 : 0), "Voices[12].ADSR.SustainDec" }, { &((Voices[12].ADSR.ReleaseExp)), SF_IS_BOOL(&((Voices[12].ADSR.ReleaseExp))) ? 1 : sizeof((Voices[12].ADSR.ReleaseExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[12].ADSR.ReleaseExp))) ? 0x08000000 : 0), "Voices[12].ADSR.ReleaseExp" }, { &((Voices[12].ADSR.AttackRate)), SF_IS_BOOL(&((Voices[12].ADSR.AttackRate))) ? 1 : sizeof((Voices[12].ADSR.AttackRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[12].ADSR.AttackRate))) ? 0x08000000 : 0), "Voices[12].ADSR.AttackRate" }, { &((Voices[12].ADSR.DecayRate)), SF_IS_BOOL(&((Voices[12].ADSR.DecayRate))) ? 1 : sizeof((Voices[12].ADSR.DecayRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[12].ADSR.DecayRate))) ? 0x08000000 : 0), "Voices[12].ADSR.DecayRate" }, { &((Voices[12].ADSR.SustainRate)), SF_IS_BOOL(&((Voices[12].ADSR.SustainRate))) ? 1 : sizeof((Voices[12].ADSR.SustainRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[12].ADSR.SustainRate))) ? 0x08000000 : 0), "Voices[12].ADSR.SustainRate" }, { &((Voices[12].ADSR.ReleaseRate)), SF_IS_BOOL(&((Voices[12].ADSR.ReleaseRate))) ? 1 : sizeof((Voices[12].ADSR.ReleaseRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[12].ADSR.ReleaseRate))) ? 0x08000000 : 0), "Voices[12].ADSR.ReleaseRate" }, { &((Voices[12].ADSR.SustainLevel)), SF_IS_BOOL(&((Voices[12].ADSR.SustainLevel))) ? 1 : sizeof((Voices[12].ADSR.SustainLevel)), 0x80000000 | (SF_IS_BOOL(&((Voices[12].ADSR.SustainLevel))) ? 0x08000000 : 0), "Voices[12].ADSR.SustainLevel" },
  { ((&Voices[13].DecodeBuffer[0])), (uint32)(((sizeof(Voices[13].DecodeBuffer) / sizeof(Voices[13].DecodeBuffer[0]))) * sizeof(uint16)), 0x20000000 | SF_FORCE_A16((&Voices[13].DecodeBuffer[0])), "&Voices[13].DecodeBuffer[0]" }, { &((Voices[13].DecodeM2)), SF_IS_BOOL(&((Voices[13].DecodeM2))) ? 1 : sizeof((Voices[13].DecodeM2)), 0x80000000 | (SF_IS_BOOL(&((Voices[13].DecodeM2))) ? 0x08000000 : 0), "Voices[13].DecodeM2" }, { &((Voices[13].DecodeM1)), SF_IS_BOOL(&((Voices[13].DecodeM1))) ? 1 : sizeof((Voices[13].DecodeM1)), 0x80000000 | (SF_IS_BOOL(&((Voices[13].DecodeM1))) ? 0x08000000 : 0), "Voices[13].DecodeM1" }, { &((Voices[13].DecodePlayDelay)), SF_IS_BOOL(&((Voices[13].DecodePlayDelay))) ? 1 : sizeof((Voices[13].DecodePlayDelay)), 0x80000000 | (SF_IS_BOOL(&((Voices[13].DecodePlayDelay))) ? 0x08000000 : 0), "Voices[13].DecodePlayDelay" }, { &((Voices[13].DecodeWritePos)), SF_IS_BOOL(&((Voices[13].DecodeWritePos))) ? 1 : sizeof((Voices[13].DecodeWritePos)), 0x80000000 | (SF_IS_BOOL(&((Voices[13].DecodeWritePos))) ? 0x08000000 : 0), "Voices[13].DecodeWritePos" }, { &((Voices[13].DecodeReadPos)), SF_IS_BOOL(&((Voices[13].DecodeReadPos))) ? 1 : sizeof((Voices[13].DecodeReadPos)), 0x80000000 | (SF_IS_BOOL(&((Voices[13].DecodeReadPos))) ? 0x08000000 : 0), "Voices[13].DecodeReadPos" }, { &((Voices[13].DecodeAvail)), SF_IS_BOOL(&((Voices[13].DecodeAvail))) ? 1 : sizeof((Voices[13].DecodeAvail)), 0x80000000 | (SF_IS_BOOL(&((Voices[13].DecodeAvail))) ? 0x08000000 : 0), "Voices[13].DecodeAvail" }, { &((Voices[13].DecodeShift)), SF_IS_BOOL(&((Voices[13].DecodeShift))) ? 1 : sizeof((Voices[13].DecodeShift)), 0x80000000 | (SF_IS_BOOL(&((Voices[13].DecodeShift))) ? 0x08000000 : 0), "Voices[13].DecodeShift" }, { &((Voices[13].DecodeWeight)), SF_IS_BOOL(&((Voices[13].DecodeWeight))) ? 1 : sizeof((Voices[13].DecodeWeight)), 0x80000000 | (SF_IS_BOOL(&((Voices[13].DecodeWeight))) ? 0x08000000 : 0), "Voices[13].DecodeWeight" }, { &((Voices[13].DecodeFlags)), SF_IS_BOOL(&((Voices[13].DecodeFlags))) ? 1 : sizeof((Voices[13].DecodeFlags)), 0x80000000 | (SF_IS_BOOL(&((Voices[13].DecodeFlags))) ? 0x08000000 : 0), "Voices[13].DecodeFlags" }, { &((Voices[13].IgnoreSampLA)), SF_IS_BOOL(&((Voices[13].IgnoreSampLA))) ? 1 : sizeof((Voices[13].IgnoreSampLA)), 0x80000000 | (SF_IS_BOOL(&((Voices[13].IgnoreSampLA))) ? 0x08000000 : 0), "Voices[13].IgnoreSampLA" }, { &(((Voices[13].Sweep[0]).Control)), SF_IS_BOOL(&(((Voices[13].Sweep[0]).Control))) ? 1 : sizeof(((Voices[13].Sweep[0]).Control)), 0x80000000 | (SF_IS_BOOL(&(((Voices[13].Sweep[0]).Control))) ? 0x08000000 : 0), "(Voices[13].Sweep[0]).Control" }, { &(((Voices[13].Sweep[0]).Current)), SF_IS_BOOL(&(((Voices[13].Sweep[0]).Current))) ? 1 : sizeof(((Voices[13].Sweep[0]).Current)), 0x80000000 | (SF_IS_BOOL(&(((Voices[13].Sweep[0]).Current))) ? 0x08000000 : 0), "(Voices[13].Sweep[0]).Current" }, { &(((Voices[13].Sweep[0]).Divider)), SF_IS_BOOL(&(((Voices[13].Sweep[0]).Divider))) ? 1 : sizeof(((Voices[13].Sweep[0]).Divider)), 0x80000000 | (SF_IS_BOOL(&(((Voices[13].Sweep[0]).Divider))) ? 0x08000000 : 0), "(Voices[13].Sweep[0]).Divider" }, { &(((Voices[13].Sweep[1]).Control)), SF_IS_BOOL(&(((Voices[13].Sweep[1]).Control))) ? 1 : sizeof(((Voices[13].Sweep[1]).Control)), 0x80000000 | (SF_IS_BOOL(&(((Voices[13].Sweep[1]).Control))) ? 0x08000000 : 0), "(Voices[13].Sweep[1]).Control" }, { &(((Voices[13].Sweep[1]).Current)), SF_IS_BOOL(&(((Voices[13].Sweep[1]).Current))) ? 1 : sizeof(((Voices[13].Sweep[1]).Current)), 0x80000000 | (SF_IS_BOOL(&(((Voices[13].Sweep[1]).Current))) ? 0x08000000 : 0), "(Voices[13].Sweep[1]).Current" }, { &(((Voices[13].Sweep[1]).Divider)), SF_IS_BOOL(&(((Voices[13].Sweep[1]).Divider))) ? 1 : sizeof(((Voices[13].Sweep[1]).Divider)), 0x80000000 | (SF_IS_BOOL(&(((Voices[13].Sweep[1]).Divider))) ? 0x08000000 : 0), "(Voices[13].Sweep[1]).Divider" }, { &((Voices[13].Pitch)), SF_IS_BOOL(&((Voices[13].Pitch))) ? 1 : sizeof((Voices[13].Pitch)), 0x80000000 | (SF_IS_BOOL(&((Voices[13].Pitch))) ? 0x08000000 : 0), "Voices[13].Pitch" }, { &((Voices[13].CurPhase)), SF_IS_BOOL(&((Voices[13].CurPhase))) ? 1 : sizeof((Voices[13].CurPhase)), 0x80000000 | (SF_IS_BOOL(&((Voices[13].CurPhase))) ? 0x08000000 : 0), "Voices[13].CurPhase" }, { &((Voices[13].StartAddr)), SF_IS_BOOL(&((Voices[13].StartAddr))) ? 1 : sizeof((Voices[13].StartAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[13].StartAddr))) ? 0x08000000 : 0), "Voices[13].StartAddr" }, { &((Voices[13].CurAddr)), SF_IS_BOOL(&((Voices[13].CurAddr))) ? 1 : sizeof((Voices[13].CurAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[13].CurAddr))) ? 0x08000000 : 0), "Voices[13].CurAddr" }, { &((Voices[13].ADSRControl)), SF_IS_BOOL(&((Voices[13].ADSRControl))) ? 1 : sizeof((Voices[13].ADSRControl)), 0x80000000 | (SF_IS_BOOL(&((Voices[13].ADSRControl))) ? 0x08000000 : 0), "Voices[13].ADSRControl" }, { &((Voices[13].LoopAddr)), SF_IS_BOOL(&((Voices[13].LoopAddr))) ? 1 : sizeof((Voices[13].LoopAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[13].LoopAddr))) ? 0x08000000 : 0), "Voices[13].LoopAddr" }, { &((Voices[13].PreLRSample)), SF_IS_BOOL(&((Voices[13].PreLRSample))) ? 1 : sizeof((Voices[13].PreLRSample)), 0x80000000 | (SF_IS_BOOL(&((Voices[13].PreLRSample))) ? 0x08000000 : 0), "Voices[13].PreLRSample" }, { &((Voices[13].ADSR.EnvLevel)), SF_IS_BOOL(&((Voices[13].ADSR.EnvLevel))) ? 1 : sizeof((Voices[13].ADSR.EnvLevel)), 0x80000000 | (SF_IS_BOOL(&((Voices[13].ADSR.EnvLevel))) ? 0x08000000 : 0), "Voices[13].ADSR.EnvLevel" }, { &((Voices[13].ADSR.Divider)), SF_IS_BOOL(&((Voices[13].ADSR.Divider))) ? 1 : sizeof((Voices[13].ADSR.Divider)), 0x80000000 | (SF_IS_BOOL(&((Voices[13].ADSR.Divider))) ? 0x08000000 : 0), "Voices[13].ADSR.Divider" }, { &((Voices[13].ADSR.Phase)), SF_IS_BOOL(&((Voices[13].ADSR.Phase))) ? 1 : sizeof((Voices[13].ADSR.Phase)), 0x80000000 | (SF_IS_BOOL(&((Voices[13].ADSR.Phase))) ? 0x08000000 : 0), "Voices[13].ADSR.Phase" }, { &((Voices[13].ADSR.AttackExp)), SF_IS_BOOL(&((Voices[13].ADSR.AttackExp))) ? 1 : sizeof((Voices[13].ADSR.AttackExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[13].ADSR.AttackExp))) ? 0x08000000 : 0), "Voices[13].ADSR.AttackExp" }, { &((Voices[13].ADSR.SustainExp)), SF_IS_BOOL(&((Voices[13].ADSR.SustainExp))) ? 1 : sizeof((Voices[13].ADSR.SustainExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[13].ADSR.SustainExp))) ? 0x08000000 : 0), "Voices[13].ADSR.SustainExp" }, { &((Voices[13].ADSR.SustainDec)), SF_IS_BOOL(&((Voices[13].ADSR.SustainDec))) ? 1 : sizeof((Voices[13].ADSR.SustainDec)), 0x80000000 | (SF_IS_BOOL(&((Voices[13].ADSR.SustainDec))) ? 0x08000000 : 0), "Voices[13].ADSR.SustainDec" }, { &((Voices[13].ADSR.ReleaseExp)), SF_IS_BOOL(&((Voices[13].ADSR.ReleaseExp))) ? 1 : sizeof((Voices[13].ADSR.ReleaseExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[13].ADSR.ReleaseExp))) ? 0x08000000 : 0), "Voices[13].ADSR.ReleaseExp" }, { &((Voices[13].ADSR.AttackRate)), SF_IS_BOOL(&((Voices[13].ADSR.AttackRate))) ? 1 : sizeof((Voices[13].ADSR.AttackRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[13].ADSR.AttackRate))) ? 0x08000000 : 0), "Voices[13].ADSR.AttackRate" }, { &((Voices[13].ADSR.DecayRate)), SF_IS_BOOL(&((Voices[13].ADSR.DecayRate))) ? 1 : sizeof((Voices[13].ADSR.DecayRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[13].ADSR.DecayRate))) ? 0x08000000 : 0), "Voices[13].ADSR.DecayRate" }, { &((Voices[13].ADSR.SustainRate)), SF_IS_BOOL(&((Voices[13].ADSR.SustainRate))) ? 1 : sizeof((Voices[13].ADSR.SustainRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[13].ADSR.SustainRate))) ? 0x08000000 : 0), "Voices[13].ADSR.SustainRate" }, { &((Voices[13].ADSR.ReleaseRate)), SF_IS_BOOL(&((Voices[13].ADSR.ReleaseRate))) ? 1 : sizeof((Voices[13].ADSR.ReleaseRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[13].ADSR.ReleaseRate))) ? 0x08000000 : 0), "Voices[13].ADSR.ReleaseRate" }, { &((Voices[13].ADSR.SustainLevel)), SF_IS_BOOL(&((Voices[13].ADSR.SustainLevel))) ? 1 : sizeof((Voices[13].ADSR.SustainLevel)), 0x80000000 | (SF_IS_BOOL(&((Voices[13].ADSR.SustainLevel))) ? 0x08000000 : 0), "Voices[13].ADSR.SustainLevel" },
  { ((&Voices[14].DecodeBuffer[0])), (uint32)(((sizeof(Voices[14].DecodeBuffer) / sizeof(Voices[14].DecodeBuffer[0]))) * sizeof(uint16)), 0x20000000 | SF_FORCE_A16((&Voices[14].DecodeBuffer[0])), "&Voices[14].DecodeBuffer[0]" }, { &((Voices[14].DecodeM2)), SF_IS_BOOL(&((Voices[14].DecodeM2))) ? 1 : sizeof((Voices[14].DecodeM2)), 0x80000000 | (SF_IS_BOOL(&((Voices[14].DecodeM2))) ? 0x08000000 : 0), "Voices[14].DecodeM2" }, { &((Voices[14].DecodeM1)), SF_IS_BOOL(&((Voices[14].DecodeM1))) ? 1 : sizeof((Voices[14].DecodeM1)), 0x80000000 | (SF_IS_BOOL(&((Voices[14].DecodeM1))) ? 0x08000000 : 0), "Voices[14].DecodeM1" }, { &((Voices[14].DecodePlayDelay)), SF_IS_BOOL(&((Voices[14].DecodePlayDelay))) ? 1 : sizeof((Voices[14].DecodePlayDelay)), 0x80000000 | (SF_IS_BOOL(&((Voices[14].DecodePlayDelay))) ? 0x08000000 : 0), "Voices[14].DecodePlayDelay" }, { &((Voices[14].DecodeWritePos)), SF_IS_BOOL(&((Voices[14].DecodeWritePos))) ? 1 : sizeof((Voices[14].DecodeWritePos)), 0x80000000 | (SF_IS_BOOL(&((Voices[14].DecodeWritePos))) ? 0x08000000 : 0), "Voices[14].DecodeWritePos" }, { &((Voices[14].DecodeReadPos)), SF_IS_BOOL(&((Voices[14].DecodeReadPos))) ? 1 : sizeof((Voices[14].DecodeReadPos)), 0x80000000 | (SF_IS_BOOL(&((Voices[14].DecodeReadPos))) ? 0x08000000 : 0), "Voices[14].DecodeReadPos" }, { &((Voices[14].DecodeAvail)), SF_IS_BOOL(&((Voices[14].DecodeAvail))) ? 1 : sizeof((Voices[14].DecodeAvail)), 0x80000000 | (SF_IS_BOOL(&((Voices[14].DecodeAvail))) ? 0x08000000 : 0), "Voices[14].DecodeAvail" }, { &((Voices[14].DecodeShift)), SF_IS_BOOL(&((Voices[14].DecodeShift))) ? 1 : sizeof((Voices[14].DecodeShift)), 0x80000000 | (SF_IS_BOOL(&((Voices[14].DecodeShift))) ? 0x08000000 : 0), "Voices[14].DecodeShift" }, { &((Voices[14].DecodeWeight)), SF_IS_BOOL(&((Voices[14].DecodeWeight))) ? 1 : sizeof((Voices[14].DecodeWeight)), 0x80000000 | (SF_IS_BOOL(&((Voices[14].DecodeWeight))) ? 0x08000000 : 0), "Voices[14].DecodeWeight" }, { &((Voices[14].DecodeFlags)), SF_IS_BOOL(&((Voices[14].DecodeFlags))) ? 1 : sizeof((Voices[14].DecodeFlags)), 0x80000000 | (SF_IS_BOOL(&((Voices[14].DecodeFlags))) ? 0x08000000 : 0), "Voices[14].DecodeFlags" }, { &((Voices[14].IgnoreSampLA)), SF_IS_BOOL(&((Voices[14].IgnoreSampLA))) ? 1 : sizeof((Voices[14].IgnoreSampLA)), 0x80000000 | (SF_IS_BOOL(&((Voices[14].IgnoreSampLA))) ? 0x08000000 : 0), "Voices[14].IgnoreSampLA" }, { &(((Voices[14].Sweep[0]).Control)), SF_IS_BOOL(&(((Voices[14].Sweep[0]).Control))) ? 1 : sizeof(((Voices[14].Sweep[0]).Control)), 0x80000000 | (SF_IS_BOOL(&(((Voices[14].Sweep[0]).Control))) ? 0x08000000 : 0), "(Voices[14].Sweep[0]).Control" }, { &(((Voices[14].Sweep[0]).Current)), SF_IS_BOOL(&(((Voices[14].Sweep[0]).Current))) ? 1 : sizeof(((Voices[14].Sweep[0]).Current)), 0x80000000 | (SF_IS_BOOL(&(((Voices[14].Sweep[0]).Current))) ? 0x08000000 : 0), "(Voices[14].Sweep[0]).Current" }, { &(((Voices[14].Sweep[0]).Divider)), SF_IS_BOOL(&(((Voices[14].Sweep[0]).Divider))) ? 1 : sizeof(((Voices[14].Sweep[0]).Divider)), 0x80000000 | (SF_IS_BOOL(&(((Voices[14].Sweep[0]).Divider))) ? 0x08000000 : 0), "(Voices[14].Sweep[0]).Divider" }, { &(((Voices[14].Sweep[1]).Control)), SF_IS_BOOL(&(((Voices[14].Sweep[1]).Control))) ? 1 : sizeof(((Voices[14].Sweep[1]).Control)), 0x80000000 | (SF_IS_BOOL(&(((Voices[14].Sweep[1]).Control))) ? 0x08000000 : 0), "(Voices[14].Sweep[1]).Control" }, { &(((Voices[14].Sweep[1]).Current)), SF_IS_BOOL(&(((Voices[14].Sweep[1]).Current))) ? 1 : sizeof(((Voices[14].Sweep[1]).Current)), 0x80000000 | (SF_IS_BOOL(&(((Voices[14].Sweep[1]).Current))) ? 0x08000000 : 0), "(Voices[14].Sweep[1]).Current" }, { &(((Voices[14].Sweep[1]).Divider)), SF_IS_BOOL(&(((Voices[14].Sweep[1]).Divider))) ? 1 : sizeof(((Voices[14].Sweep[1]).Divider)), 0x80000000 | (SF_IS_BOOL(&(((Voices[14].Sweep[1]).Divider))) ? 0x08000000 : 0), "(Voices[14].Sweep[1]).Divider" }, { &((Voices[14].Pitch)), SF_IS_BOOL(&((Voices[14].Pitch))) ? 1 : sizeof((Voices[14].Pitch)), 0x80000000 | (SF_IS_BOOL(&((Voices[14].Pitch))) ? 0x08000000 : 0), "Voices[14].Pitch" }, { &((Voices[14].CurPhase)), SF_IS_BOOL(&((Voices[14].CurPhase))) ? 1 : sizeof((Voices[14].CurPhase)), 0x80000000 | (SF_IS_BOOL(&((Voices[14].CurPhase))) ? 0x08000000 : 0), "Voices[14].CurPhase" }, { &((Voices[14].StartAddr)), SF_IS_BOOL(&((Voices[14].StartAddr))) ? 1 : sizeof((Voices[14].StartAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[14].StartAddr))) ? 0x08000000 : 0), "Voices[14].StartAddr" }, { &((Voices[14].CurAddr)), SF_IS_BOOL(&((Voices[14].CurAddr))) ? 1 : sizeof((Voices[14].CurAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[14].CurAddr))) ? 0x08000000 : 0), "Voices[14].CurAddr" }, { &((Voices[14].ADSRControl)), SF_IS_BOOL(&((Voices[14].ADSRControl))) ? 1 : sizeof((Voices[14].ADSRControl)), 0x80000000 | (SF_IS_BOOL(&((Voices[14].ADSRControl))) ? 0x08000000 : 0), "Voices[14].ADSRControl" }, { &((Voices[14].LoopAddr)), SF_IS_BOOL(&((Voices[14].LoopAddr))) ? 1 : sizeof((Voices[14].LoopAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[14].LoopAddr))) ? 0x08000000 : 0), "Voices[14].LoopAddr" }, { &((Voices[14].PreLRSample)), SF_IS_BOOL(&((Voices[14].PreLRSample))) ? 1 : sizeof((Voices[14].PreLRSample)), 0x80000000 | (SF_IS_BOOL(&((Voices[14].PreLRSample))) ? 0x08000000 : 0), "Voices[14].PreLRSample" }, { &((Voices[14].ADSR.EnvLevel)), SF_IS_BOOL(&((Voices[14].ADSR.EnvLevel))) ? 1 : sizeof((Voices[14].ADSR.EnvLevel)), 0x80000000 | (SF_IS_BOOL(&((Voices[14].ADSR.EnvLevel))) ? 0x08000000 : 0), "Voices[14].ADSR.EnvLevel" }, { &((Voices[14].ADSR.Divider)), SF_IS_BOOL(&((Voices[14].ADSR.Divider))) ? 1 : sizeof((Voices[14].ADSR.Divider)), 0x80000000 | (SF_IS_BOOL(&((Voices[14].ADSR.Divider))) ? 0x08000000 : 0), "Voices[14].ADSR.Divider" }, { &((Voices[14].ADSR.Phase)), SF_IS_BOOL(&((Voices[14].ADSR.Phase))) ? 1 : sizeof((Voices[14].ADSR.Phase)), 0x80000000 | (SF_IS_BOOL(&((Voices[14].ADSR.Phase))) ? 0x08000000 : 0), "Voices[14].ADSR.Phase" }, { &((Voices[14].ADSR.AttackExp)), SF_IS_BOOL(&((Voices[14].ADSR.AttackExp))) ? 1 : sizeof((Voices[14].ADSR.AttackExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[14].ADSR.AttackExp))) ? 0x08000000 : 0), "Voices[14].ADSR.AttackExp" }, { &((Voices[14].ADSR.SustainExp)), SF_IS_BOOL(&((Voices[14].ADSR.SustainExp))) ? 1 : sizeof((Voices[14].ADSR.SustainExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[14].ADSR.SustainExp))) ? 0x08000000 : 0), "Voices[14].ADSR.SustainExp" }, { &((Voices[14].ADSR.SustainDec)), SF_IS_BOOL(&((Voices[14].ADSR.SustainDec))) ? 1 : sizeof((Voices[14].ADSR.SustainDec)), 0x80000000 | (SF_IS_BOOL(&((Voices[14].ADSR.SustainDec))) ? 0x08000000 : 0), "Voices[14].ADSR.SustainDec" }, { &((Voices[14].ADSR.ReleaseExp)), SF_IS_BOOL(&((Voices[14].ADSR.ReleaseExp))) ? 1 : sizeof((Voices[14].ADSR.ReleaseExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[14].ADSR.ReleaseExp))) ? 0x08000000 : 0), "Voices[14].ADSR.ReleaseExp" }, { &((Voices[14].ADSR.AttackRate)), SF_IS_BOOL(&((Voices[14].ADSR.AttackRate))) ? 1 : sizeof((Voices[14].ADSR.AttackRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[14].ADSR.AttackRate))) ? 0x08000000 : 0), "Voices[14].ADSR.AttackRate" }, { &((Voices[14].ADSR.DecayRate)), SF_IS_BOOL(&((Voices[14].ADSR.DecayRate))) ? 1 : sizeof((Voices[14].ADSR.DecayRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[14].ADSR.DecayRate))) ? 0x08000000 : 0), "Voices[14].ADSR.DecayRate" }, { &((Voices[14].ADSR.SustainRate)), SF_IS_BOOL(&((Voices[14].ADSR.SustainRate))) ? 1 : sizeof((Voices[14].ADSR.SustainRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[14].ADSR.SustainRate))) ? 0x08000000 : 0), "Voices[14].ADSR.SustainRate" }, { &((Voices[14].ADSR.ReleaseRate)), SF_IS_BOOL(&((Voices[14].ADSR.ReleaseRate))) ? 1 : sizeof((Voices[14].ADSR.ReleaseRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[14].ADSR.ReleaseRate))) ? 0x08000000 : 0), "Voices[14].ADSR.ReleaseRate" }, { &((Voices[14].ADSR.SustainLevel)), SF_IS_BOOL(&((Voices[14].ADSR.SustainLevel))) ? 1 : sizeof((Voices[14].ADSR.SustainLevel)), 0x80000000 | (SF_IS_BOOL(&((Voices[14].ADSR.SustainLevel))) ? 0x08000000 : 0), "Voices[14].ADSR.SustainLevel" },
  { ((&Voices[15].DecodeBuffer[0])), (uint32)(((sizeof(Voices[15].DecodeBuffer) / sizeof(Voices[15].DecodeBuffer[0]))) * sizeof(uint16)), 0x20000000 | SF_FORCE_A16((&Voices[15].DecodeBuffer[0])), "&Voices[15].DecodeBuffer[0]" }, { &((Voices[15].DecodeM2)), SF_IS_BOOL(&((Voices[15].DecodeM2))) ? 1 : sizeof((Voices[15].DecodeM2)), 0x80000000 | (SF_IS_BOOL(&((Voices[15].DecodeM2))) ? 0x08000000 : 0), "Voices[15].DecodeM2" }, { &((Voices[15].DecodeM1)), SF_IS_BOOL(&((Voices[15].DecodeM1))) ? 1 : sizeof((Voices[15].DecodeM1)), 0x80000000 | (SF_IS_BOOL(&((Voices[15].DecodeM1))) ? 0x08000000 : 0), "Voices[15].DecodeM1" }, { &((Voices[15].DecodePlayDelay)), SF_IS_BOOL(&((Voices[15].DecodePlayDelay))) ? 1 : sizeof((Voices[15].DecodePlayDelay)), 0x80000000 | (SF_IS_BOOL(&((Voices[15].DecodePlayDelay))) ? 0x08000000 : 0), "Voices[15].DecodePlayDelay" }, { &((Voices[15].DecodeWritePos)), SF_IS_BOOL(&((Voices[15].DecodeWritePos))) ? 1 : sizeof((Voices[15].DecodeWritePos)), 0x80000000 | (SF_IS_BOOL(&((Voices[15].DecodeWritePos))) ? 0x08000000 : 0), "Voices[15].DecodeWritePos" }, { &((Voices[15].DecodeReadPos)), SF_IS_BOOL(&((Voices[15].DecodeReadPos))) ? 1 : sizeof((Voices[15].DecodeReadPos)), 0x80000000 | (SF_IS_BOOL(&((Voices[15].DecodeReadPos))) ? 0x08000000 : 0), "Voices[15].DecodeReadPos" }, { &((Voices[15].DecodeAvail)), SF_IS_BOOL(&((Voices[15].DecodeAvail))) ? 1 : sizeof((Voices[15].DecodeAvail)), 0x80000000 | (SF_IS_BOOL(&((Voices[15].DecodeAvail))) ? 0x08000000 : 0), "Voices[15].DecodeAvail" }, { &((Voices[15].DecodeShift)), SF_IS_BOOL(&((Voices[15].DecodeShift))) ? 1 : sizeof((Voices[15].DecodeShift)), 0x80000000 | (SF_IS_BOOL(&((Voices[15].DecodeShift))) ? 0x08000000 : 0), "Voices[15].DecodeShift" }, { &((Voices[15].DecodeWeight)), SF_IS_BOOL(&((Voices[15].DecodeWeight))) ? 1 : sizeof((Voices[15].DecodeWeight)), 0x80000000 | (SF_IS_BOOL(&((Voices[15].DecodeWeight))) ? 0x08000000 : 0), "Voices[15].DecodeWeight" }, { &((Voices[15].DecodeFlags)), SF_IS_BOOL(&((Voices[15].DecodeFlags))) ? 1 : sizeof((Voices[15].DecodeFlags)), 0x80000000 | (SF_IS_BOOL(&((Voices[15].DecodeFlags))) ? 0x08000000 : 0), "Voices[15].DecodeFlags" }, { &((Voices[15].IgnoreSampLA)), SF_IS_BOOL(&((Voices[15].IgnoreSampLA))) ? 1 : sizeof((Voices[15].IgnoreSampLA)), 0x80000000 | (SF_IS_BOOL(&((Voices[15].IgnoreSampLA))) ? 0x08000000 : 0), "Voices[15].IgnoreSampLA" }, { &(((Voices[15].Sweep[0]).Control)), SF_IS_BOOL(&(((Voices[15].Sweep[0]).Control))) ? 1 : sizeof(((Voices[15].Sweep[0]).Control)), 0x80000000 | (SF_IS_BOOL(&(((Voices[15].Sweep[0]).Control))) ? 0x08000000 : 0), "(Voices[15].Sweep[0]).Control" }, { &(((Voices[15].Sweep[0]).Current)), SF_IS_BOOL(&(((Voices[15].Sweep[0]).Current))) ? 1 : sizeof(((Voices[15].Sweep[0]).Current)), 0x80000000 | (SF_IS_BOOL(&(((Voices[15].Sweep[0]).Current))) ? 0x08000000 : 0), "(Voices[15].Sweep[0]).Current" }, { &(((Voices[15].Sweep[0]).Divider)), SF_IS_BOOL(&(((Voices[15].Sweep[0]).Divider))) ? 1 : sizeof(((Voices[15].Sweep[0]).Divider)), 0x80000000 | (SF_IS_BOOL(&(((Voices[15].Sweep[0]).Divider))) ? 0x08000000 : 0), "(Voices[15].Sweep[0]).Divider" }, { &(((Voices[15].Sweep[1]).Control)), SF_IS_BOOL(&(((Voices[15].Sweep[1]).Control))) ? 1 : sizeof(((Voices[15].Sweep[1]).Control)), 0x80000000 | (SF_IS_BOOL(&(((Voices[15].Sweep[1]).Control))) ? 0x08000000 : 0), "(Voices[15].Sweep[1]).Control" }, { &(((Voices[15].Sweep[1]).Current)), SF_IS_BOOL(&(((Voices[15].Sweep[1]).Current))) ? 1 : sizeof(((Voices[15].Sweep[1]).Current)), 0x80000000 | (SF_IS_BOOL(&(((Voices[15].Sweep[1]).Current))) ? 0x08000000 : 0), "(Voices[15].Sweep[1]).Current" }, { &(((Voices[15].Sweep[1]).Divider)), SF_IS_BOOL(&(((Voices[15].Sweep[1]).Divider))) ? 1 : sizeof(((Voices[15].Sweep[1]).Divider)), 0x80000000 | (SF_IS_BOOL(&(((Voices[15].Sweep[1]).Divider))) ? 0x08000000 : 0), "(Voices[15].Sweep[1]).Divider" }, { &((Voices[15].Pitch)), SF_IS_BOOL(&((Voices[15].Pitch))) ? 1 : sizeof((Voices[15].Pitch)), 0x80000000 | (SF_IS_BOOL(&((Voices[15].Pitch))) ? 0x08000000 : 0), "Voices[15].Pitch" }, { &((Voices[15].CurPhase)), SF_IS_BOOL(&((Voices[15].CurPhase))) ? 1 : sizeof((Voices[15].CurPhase)), 0x80000000 | (SF_IS_BOOL(&((Voices[15].CurPhase))) ? 0x08000000 : 0), "Voices[15].CurPhase" }, { &((Voices[15].StartAddr)), SF_IS_BOOL(&((Voices[15].StartAddr))) ? 1 : sizeof((Voices[15].StartAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[15].StartAddr))) ? 0x08000000 : 0), "Voices[15].StartAddr" }, { &((Voices[15].CurAddr)), SF_IS_BOOL(&((Voices[15].CurAddr))) ? 1 : sizeof((Voices[15].CurAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[15].CurAddr))) ? 0x08000000 : 0), "Voices[15].CurAddr" }, { &((Voices[15].ADSRControl)), SF_IS_BOOL(&((Voices[15].ADSRControl))) ? 1 : sizeof((Voices[15].ADSRControl)), 0x80000000 | (SF_IS_BOOL(&((Voices[15].ADSRControl))) ? 0x08000000 : 0), "Voices[15].ADSRControl" }, { &((Voices[15].LoopAddr)), SF_IS_BOOL(&((Voices[15].LoopAddr))) ? 1 : sizeof((Voices[15].LoopAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[15].LoopAddr))) ? 0x08000000 : 0), "Voices[15].LoopAddr" }, { &((Voices[15].PreLRSample)), SF_IS_BOOL(&((Voices[15].PreLRSample))) ? 1 : sizeof((Voices[15].PreLRSample)), 0x80000000 | (SF_IS_BOOL(&((Voices[15].PreLRSample))) ? 0x08000000 : 0), "Voices[15].PreLRSample" }, { &((Voices[15].ADSR.EnvLevel)), SF_IS_BOOL(&((Voices[15].ADSR.EnvLevel))) ? 1 : sizeof((Voices[15].ADSR.EnvLevel)), 0x80000000 | (SF_IS_BOOL(&((Voices[15].ADSR.EnvLevel))) ? 0x08000000 : 0), "Voices[15].ADSR.EnvLevel" }, { &((Voices[15].ADSR.Divider)), SF_IS_BOOL(&((Voices[15].ADSR.Divider))) ? 1 : sizeof((Voices[15].ADSR.Divider)), 0x80000000 | (SF_IS_BOOL(&((Voices[15].ADSR.Divider))) ? 0x08000000 : 0), "Voices[15].ADSR.Divider" }, { &((Voices[15].ADSR.Phase)), SF_IS_BOOL(&((Voices[15].ADSR.Phase))) ? 1 : sizeof((Voices[15].ADSR.Phase)), 0x80000000 | (SF_IS_BOOL(&((Voices[15].ADSR.Phase))) ? 0x08000000 : 0), "Voices[15].ADSR.Phase" }, { &((Voices[15].ADSR.AttackExp)), SF_IS_BOOL(&((Voices[15].ADSR.AttackExp))) ? 1 : sizeof((Voices[15].ADSR.AttackExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[15].ADSR.AttackExp))) ? 0x08000000 : 0), "Voices[15].ADSR.AttackExp" }, { &((Voices[15].ADSR.SustainExp)), SF_IS_BOOL(&((Voices[15].ADSR.SustainExp))) ? 1 : sizeof((Voices[15].ADSR.SustainExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[15].ADSR.SustainExp))) ? 0x08000000 : 0), "Voices[15].ADSR.SustainExp" }, { &((Voices[15].ADSR.SustainDec)), SF_IS_BOOL(&((Voices[15].ADSR.SustainDec))) ? 1 : sizeof((Voices[15].ADSR.SustainDec)), 0x80000000 | (SF_IS_BOOL(&((Voices[15].ADSR.SustainDec))) ? 0x08000000 : 0), "Voices[15].ADSR.SustainDec" }, { &((Voices[15].ADSR.ReleaseExp)), SF_IS_BOOL(&((Voices[15].ADSR.ReleaseExp))) ? 1 : sizeof((Voices[15].ADSR.ReleaseExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[15].ADSR.ReleaseExp))) ? 0x08000000 : 0), "Voices[15].ADSR.ReleaseExp" }, { &((Voices[15].ADSR.AttackRate)), SF_IS_BOOL(&((Voices[15].ADSR.AttackRate))) ? 1 : sizeof((Voices[15].ADSR.AttackRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[15].ADSR.AttackRate))) ? 0x08000000 : 0), "Voices[15].ADSR.AttackRate" }, { &((Voices[15].ADSR.DecayRate)), SF_IS_BOOL(&((Voices[15].ADSR.DecayRate))) ? 1 : sizeof((Voices[15].ADSR.DecayRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[15].ADSR.DecayRate))) ? 0x08000000 : 0), "Voices[15].ADSR.DecayRate" }, { &((Voices[15].ADSR.SustainRate)), SF_IS_BOOL(&((Voices[15].ADSR.SustainRate))) ? 1 : sizeof((Voices[15].ADSR.SustainRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[15].ADSR.SustainRate))) ? 0x08000000 : 0), "Voices[15].ADSR.SustainRate" }, { &((Voices[15].ADSR.ReleaseRate)), SF_IS_BOOL(&((Voices[15].ADSR.ReleaseRate))) ? 1 : sizeof((Voices[15].ADSR.ReleaseRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[15].ADSR.ReleaseRate))) ? 0x08000000 : 0), "Voices[15].ADSR.ReleaseRate" }, { &((Voices[15].ADSR.SustainLevel)), SF_IS_BOOL(&((Voices[15].ADSR.SustainLevel))) ? 1 : sizeof((Voices[15].ADSR.SustainLevel)), 0x80000000 | (SF_IS_BOOL(&((Voices[15].ADSR.SustainLevel))) ? 0x08000000 : 0), "Voices[15].ADSR.SustainLevel" },
  { ((&Voices[16].DecodeBuffer[0])), (uint32)(((sizeof(Voices[16].DecodeBuffer) / sizeof(Voices[16].DecodeBuffer[0]))) * sizeof(uint16)), 0x20000000 | SF_FORCE_A16((&Voices[16].DecodeBuffer[0])), "&Voices[16].DecodeBuffer[0]" }, { &((Voices[16].DecodeM2)), SF_IS_BOOL(&((Voices[16].DecodeM2))) ? 1 : sizeof((Voices[16].DecodeM2)), 0x80000000 | (SF_IS_BOOL(&((Voices[16].DecodeM2))) ? 0x08000000 : 0), "Voices[16].DecodeM2" }, { &((Voices[16].DecodeM1)), SF_IS_BOOL(&((Voices[16].DecodeM1))) ? 1 : sizeof((Voices[16].DecodeM1)), 0x80000000 | (SF_IS_BOOL(&((Voices[16].DecodeM1))) ? 0x08000000 : 0), "Voices[16].DecodeM1" }, { &((Voices[16].DecodePlayDelay)), SF_IS_BOOL(&((Voices[16].DecodePlayDelay))) ? 1 : sizeof((Voices[16].DecodePlayDelay)), 0x80000000 | (SF_IS_BOOL(&((Voices[16].DecodePlayDelay))) ? 0x08000000 : 0), "Voices[16].DecodePlayDelay" }, { &((Voices[16].DecodeWritePos)), SF_IS_BOOL(&((Voices[16].DecodeWritePos))) ? 1 : sizeof((Voices[16].DecodeWritePos)), 0x80000000 | (SF_IS_BOOL(&((Voices[16].DecodeWritePos))) ? 0x08000000 : 0), "Voices[16].DecodeWritePos" }, { &((Voices[16].DecodeReadPos)), SF_IS_BOOL(&((Voices[16].DecodeReadPos))) ? 1 : sizeof((Voices[16].DecodeReadPos)), 0x80000000 | (SF_IS_BOOL(&((Voices[16].DecodeReadPos))) ? 0x08000000 : 0), "Voices[16].DecodeReadPos" }, { &((Voices[16].DecodeAvail)), SF_IS_BOOL(&((Voices[16].DecodeAvail))) ? 1 : sizeof((Voices[16].DecodeAvail)), 0x80000000 | (SF_IS_BOOL(&((Voices[16].DecodeAvail))) ? 0x08000000 : 0), "Voices[16].DecodeAvail" }, { &((Voices[16].DecodeShift)), SF_IS_BOOL(&((Voices[16].DecodeShift))) ? 1 : sizeof((Voices[16].DecodeShift)), 0x80000000 | (SF_IS_BOOL(&((Voices[16].DecodeShift))) ? 0x08000000 : 0), "Voices[16].DecodeShift" }, { &((Voices[16].DecodeWeight)), SF_IS_BOOL(&((Voices[16].DecodeWeight))) ? 1 : sizeof((Voices[16].DecodeWeight)), 0x80000000 | (SF_IS_BOOL(&((Voices[16].DecodeWeight))) ? 0x08000000 : 0), "Voices[16].DecodeWeight" }, { &((Voices[16].DecodeFlags)), SF_IS_BOOL(&((Voices[16].DecodeFlags))) ? 1 : sizeof((Voices[16].DecodeFlags)), 0x80000000 | (SF_IS_BOOL(&((Voices[16].DecodeFlags))) ? 0x08000000 : 0), "Voices[16].DecodeFlags" }, { &((Voices[16].IgnoreSampLA)), SF_IS_BOOL(&((Voices[16].IgnoreSampLA))) ? 1 : sizeof((Voices[16].IgnoreSampLA)), 0x80000000 | (SF_IS_BOOL(&((Voices[16].IgnoreSampLA))) ? 0x08000000 : 0), "Voices[16].IgnoreSampLA" }, { &(((Voices[16].Sweep[0]).Control)), SF_IS_BOOL(&(((Voices[16].Sweep[0]).Control))) ? 1 : sizeof(((Voices[16].Sweep[0]).Control)), 0x80000000 | (SF_IS_BOOL(&(((Voices[16].Sweep[0]).Control))) ? 0x08000000 : 0), "(Voices[16].Sweep[0]).Control" }, { &(((Voices[16].Sweep[0]).Current)), SF_IS_BOOL(&(((Voices[16].Sweep[0]).Current))) ? 1 : sizeof(((Voices[16].Sweep[0]).Current)), 0x80000000 | (SF_IS_BOOL(&(((Voices[16].Sweep[0]).Current))) ? 0x08000000 : 0), "(Voices[16].Sweep[0]).Current" }, { &(((Voices[16].Sweep[0]).Divider)), SF_IS_BOOL(&(((Voices[16].Sweep[0]).Divider))) ? 1 : sizeof(((Voices[16].Sweep[0]).Divider)), 0x80000000 | (SF_IS_BOOL(&(((Voices[16].Sweep[0]).Divider))) ? 0x08000000 : 0), "(Voices[16].Sweep[0]).Divider" }, { &(((Voices[16].Sweep[1]).Control)), SF_IS_BOOL(&(((Voices[16].Sweep[1]).Control))) ? 1 : sizeof(((Voices[16].Sweep[1]).Control)), 0x80000000 | (SF_IS_BOOL(&(((Voices[16].Sweep[1]).Control))) ? 0x08000000 : 0), "(Voices[16].Sweep[1]).Control" }, { &(((Voices[16].Sweep[1]).Current)), SF_IS_BOOL(&(((Voices[16].Sweep[1]).Current))) ? 1 : sizeof(((Voices[16].Sweep[1]).Current)), 0x80000000 | (SF_IS_BOOL(&(((Voices[16].Sweep[1]).Current))) ? 0x08000000 : 0), "(Voices[16].Sweep[1]).Current" }, { &(((Voices[16].Sweep[1]).Divider)), SF_IS_BOOL(&(((Voices[16].Sweep[1]).Divider))) ? 1 : sizeof(((Voices[16].Sweep[1]).Divider)), 0x80000000 | (SF_IS_BOOL(&(((Voices[16].Sweep[1]).Divider))) ? 0x08000000 : 0), "(Voices[16].Sweep[1]).Divider" }, { &((Voices[16].Pitch)), SF_IS_BOOL(&((Voices[16].Pitch))) ? 1 : sizeof((Voices[16].Pitch)), 0x80000000 | (SF_IS_BOOL(&((Voices[16].Pitch))) ? 0x08000000 : 0), "Voices[16].Pitch" }, { &((Voices[16].CurPhase)), SF_IS_BOOL(&((Voices[16].CurPhase))) ? 1 : sizeof((Voices[16].CurPhase)), 0x80000000 | (SF_IS_BOOL(&((Voices[16].CurPhase))) ? 0x08000000 : 0), "Voices[16].CurPhase" }, { &((Voices[16].StartAddr)), SF_IS_BOOL(&((Voices[16].StartAddr))) ? 1 : sizeof((Voices[16].StartAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[16].StartAddr))) ? 0x08000000 : 0), "Voices[16].StartAddr" }, { &((Voices[16].CurAddr)), SF_IS_BOOL(&((Voices[16].CurAddr))) ? 1 : sizeof((Voices[16].CurAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[16].CurAddr))) ? 0x08000000 : 0), "Voices[16].CurAddr" }, { &((Voices[16].ADSRControl)), SF_IS_BOOL(&((Voices[16].ADSRControl))) ? 1 : sizeof((Voices[16].ADSRControl)), 0x80000000 | (SF_IS_BOOL(&((Voices[16].ADSRControl))) ? 0x08000000 : 0), "Voices[16].ADSRControl" }, { &((Voices[16].LoopAddr)), SF_IS_BOOL(&((Voices[16].LoopAddr))) ? 1 : sizeof((Voices[16].LoopAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[16].LoopAddr))) ? 0x08000000 : 0), "Voices[16].LoopAddr" }, { &((Voices[16].PreLRSample)), SF_IS_BOOL(&((Voices[16].PreLRSample))) ? 1 : sizeof((Voices[16].PreLRSample)), 0x80000000 | (SF_IS_BOOL(&((Voices[16].PreLRSample))) ? 0x08000000 : 0), "Voices[16].PreLRSample" }, { &((Voices[16].ADSR.EnvLevel)), SF_IS_BOOL(&((Voices[16].ADSR.EnvLevel))) ? 1 : sizeof((Voices[16].ADSR.EnvLevel)), 0x80000000 | (SF_IS_BOOL(&((Voices[16].ADSR.EnvLevel))) ? 0x08000000 : 0), "Voices[16].ADSR.EnvLevel" }, { &((Voices[16].ADSR.Divider)), SF_IS_BOOL(&((Voices[16].ADSR.Divider))) ? 1 : sizeof((Voices[16].ADSR.Divider)), 0x80000000 | (SF_IS_BOOL(&((Voices[16].ADSR.Divider))) ? 0x08000000 : 0), "Voices[16].ADSR.Divider" }, { &((Voices[16].ADSR.Phase)), SF_IS_BOOL(&((Voices[16].ADSR.Phase))) ? 1 : sizeof((Voices[16].ADSR.Phase)), 0x80000000 | (SF_IS_BOOL(&((Voices[16].ADSR.Phase))) ? 0x08000000 : 0), "Voices[16].ADSR.Phase" }, { &((Voices[16].ADSR.AttackExp)), SF_IS_BOOL(&((Voices[16].ADSR.AttackExp))) ? 1 : sizeof((Voices[16].ADSR.AttackExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[16].ADSR.AttackExp))) ? 0x08000000 : 0), "Voices[16].ADSR.AttackExp" }, { &((Voices[16].ADSR.SustainExp)), SF_IS_BOOL(&((Voices[16].ADSR.SustainExp))) ? 1 : sizeof((Voices[16].ADSR.SustainExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[16].ADSR.SustainExp))) ? 0x08000000 : 0), "Voices[16].ADSR.SustainExp" }, { &((Voices[16].ADSR.SustainDec)), SF_IS_BOOL(&((Voices[16].ADSR.SustainDec))) ? 1 : sizeof((Voices[16].ADSR.SustainDec)), 0x80000000 | (SF_IS_BOOL(&((Voices[16].ADSR.SustainDec))) ? 0x08000000 : 0), "Voices[16].ADSR.SustainDec" }, { &((Voices[16].ADSR.ReleaseExp)), SF_IS_BOOL(&((Voices[16].ADSR.ReleaseExp))) ? 1 : sizeof((Voices[16].ADSR.ReleaseExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[16].ADSR.ReleaseExp))) ? 0x08000000 : 0), "Voices[16].ADSR.ReleaseExp" }, { &((Voices[16].ADSR.AttackRate)), SF_IS_BOOL(&((Voices[16].ADSR.AttackRate))) ? 1 : sizeof((Voices[16].ADSR.AttackRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[16].ADSR.AttackRate))) ? 0x08000000 : 0), "Voices[16].ADSR.AttackRate" }, { &((Voices[16].ADSR.DecayRate)), SF_IS_BOOL(&((Voices[16].ADSR.DecayRate))) ? 1 : sizeof((Voices[16].ADSR.DecayRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[16].ADSR.DecayRate))) ? 0x08000000 : 0), "Voices[16].ADSR.DecayRate" }, { &((Voices[16].ADSR.SustainRate)), SF_IS_BOOL(&((Voices[16].ADSR.SustainRate))) ? 1 : sizeof((Voices[16].ADSR.SustainRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[16].ADSR.SustainRate))) ? 0x08000000 : 0), "Voices[16].ADSR.SustainRate" }, { &((Voices[16].ADSR.ReleaseRate)), SF_IS_BOOL(&((Voices[16].ADSR.ReleaseRate))) ? 1 : sizeof((Voices[16].ADSR.ReleaseRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[16].ADSR.ReleaseRate))) ? 0x08000000 : 0), "Voices[16].ADSR.ReleaseRate" }, { &((Voices[16].ADSR.SustainLevel)), SF_IS_BOOL(&((Voices[16].ADSR.SustainLevel))) ? 1 : sizeof((Voices[16].ADSR.SustainLevel)), 0x80000000 | (SF_IS_BOOL(&((Voices[16].ADSR.SustainLevel))) ? 0x08000000 : 0), "Voices[16].ADSR.SustainLevel" },
  { ((&Voices[17].DecodeBuffer[0])), (uint32)(((sizeof(Voices[17].DecodeBuffer) / sizeof(Voices[17].DecodeBuffer[0]))) * sizeof(uint16)), 0x20000000 | SF_FORCE_A16((&Voices[17].DecodeBuffer[0])), "&Voices[17].DecodeBuffer[0]" }, { &((Voices[17].DecodeM2)), SF_IS_BOOL(&((Voices[17].DecodeM2))) ? 1 : sizeof((Voices[17].DecodeM2)), 0x80000000 | (SF_IS_BOOL(&((Voices[17].DecodeM2))) ? 0x08000000 : 0), "Voices[17].DecodeM2" }, { &((Voices[17].DecodeM1)), SF_IS_BOOL(&((Voices[17].DecodeM1))) ? 1 : sizeof((Voices[17].DecodeM1)), 0x80000000 | (SF_IS_BOOL(&((Voices[17].DecodeM1))) ? 0x08000000 : 0), "Voices[17].DecodeM1" }, { &((Voices[17].DecodePlayDelay)), SF_IS_BOOL(&((Voices[17].DecodePlayDelay))) ? 1 : sizeof((Voices[17].DecodePlayDelay)), 0x80000000 | (SF_IS_BOOL(&((Voices[17].DecodePlayDelay))) ? 0x08000000 : 0), "Voices[17].DecodePlayDelay" }, { &((Voices[17].DecodeWritePos)), SF_IS_BOOL(&((Voices[17].DecodeWritePos))) ? 1 : sizeof((Voices[17].DecodeWritePos)), 0x80000000 | (SF_IS_BOOL(&((Voices[17].DecodeWritePos))) ? 0x08000000 : 0), "Voices[17].DecodeWritePos" }, { &((Voices[17].DecodeReadPos)), SF_IS_BOOL(&((Voices[17].DecodeReadPos))) ? 1 : sizeof((Voices[17].DecodeReadPos)), 0x80000000 | (SF_IS_BOOL(&((Voices[17].DecodeReadPos))) ? 0x08000000 : 0), "Voices[17].DecodeReadPos" }, { &((Voices[17].DecodeAvail)), SF_IS_BOOL(&((Voices[17].DecodeAvail))) ? 1 : sizeof((Voices[17].DecodeAvail)), 0x80000000 | (SF_IS_BOOL(&((Voices[17].DecodeAvail))) ? 0x08000000 : 0), "Voices[17].DecodeAvail" }, { &((Voices[17].DecodeShift)), SF_IS_BOOL(&((Voices[17].DecodeShift))) ? 1 : sizeof((Voices[17].DecodeShift)), 0x80000000 | (SF_IS_BOOL(&((Voices[17].DecodeShift))) ? 0x08000000 : 0), "Voices[17].DecodeShift" }, { &((Voices[17].DecodeWeight)), SF_IS_BOOL(&((Voices[17].DecodeWeight))) ? 1 : sizeof((Voices[17].DecodeWeight)), 0x80000000 | (SF_IS_BOOL(&((Voices[17].DecodeWeight))) ? 0x08000000 : 0), "Voices[17].DecodeWeight" }, { &((Voices[17].DecodeFlags)), SF_IS_BOOL(&((Voices[17].DecodeFlags))) ? 1 : sizeof((Voices[17].DecodeFlags)), 0x80000000 | (SF_IS_BOOL(&((Voices[17].DecodeFlags))) ? 0x08000000 : 0), "Voices[17].DecodeFlags" }, { &((Voices[17].IgnoreSampLA)), SF_IS_BOOL(&((Voices[17].IgnoreSampLA))) ? 1 : sizeof((Voices[17].IgnoreSampLA)), 0x80000000 | (SF_IS_BOOL(&((Voices[17].IgnoreSampLA))) ? 0x08000000 : 0), "Voices[17].IgnoreSampLA" }, { &(((Voices[17].Sweep[0]).Control)), SF_IS_BOOL(&(((Voices[17].Sweep[0]).Control))) ? 1 : sizeof(((Voices[17].Sweep[0]).Control)), 0x80000000 | (SF_IS_BOOL(&(((Voices[17].Sweep[0]).Control))) ? 0x08000000 : 0), "(Voices[17].Sweep[0]).Control" }, { &(((Voices[17].Sweep[0]).Current)), SF_IS_BOOL(&(((Voices[17].Sweep[0]).Current))) ? 1 : sizeof(((Voices[17].Sweep[0]).Current)), 0x80000000 | (SF_IS_BOOL(&(((Voices[17].Sweep[0]).Current))) ? 0x08000000 : 0), "(Voices[17].Sweep[0]).Current" }, { &(((Voices[17].Sweep[0]).Divider)), SF_IS_BOOL(&(((Voices[17].Sweep[0]).Divider))) ? 1 : sizeof(((Voices[17].Sweep[0]).Divider)), 0x80000000 | (SF_IS_BOOL(&(((Voices[17].Sweep[0]).Divider))) ? 0x08000000 : 0), "(Voices[17].Sweep[0]).Divider" }, { &(((Voices[17].Sweep[1]).Control)), SF_IS_BOOL(&(((Voices[17].Sweep[1]).Control))) ? 1 : sizeof(((Voices[17].Sweep[1]).Control)), 0x80000000 | (SF_IS_BOOL(&(((Voices[17].Sweep[1]).Control))) ? 0x08000000 : 0), "(Voices[17].Sweep[1]).Control" }, { &(((Voices[17].Sweep[1]).Current)), SF_IS_BOOL(&(((Voices[17].Sweep[1]).Current))) ? 1 : sizeof(((Voices[17].Sweep[1]).Current)), 0x80000000 | (SF_IS_BOOL(&(((Voices[17].Sweep[1]).Current))) ? 0x08000000 : 0), "(Voices[17].Sweep[1]).Current" }, { &(((Voices[17].Sweep[1]).Divider)), SF_IS_BOOL(&(((Voices[17].Sweep[1]).Divider))) ? 1 : sizeof(((Voices[17].Sweep[1]).Divider)), 0x80000000 | (SF_IS_BOOL(&(((Voices[17].Sweep[1]).Divider))) ? 0x08000000 : 0), "(Voices[17].Sweep[1]).Divider" }, { &((Voices[17].Pitch)), SF_IS_BOOL(&((Voices[17].Pitch))) ? 1 : sizeof((Voices[17].Pitch)), 0x80000000 | (SF_IS_BOOL(&((Voices[17].Pitch))) ? 0x08000000 : 0), "Voices[17].Pitch" }, { &((Voices[17].CurPhase)), SF_IS_BOOL(&((Voices[17].CurPhase))) ? 1 : sizeof((Voices[17].CurPhase)), 0x80000000 | (SF_IS_BOOL(&((Voices[17].CurPhase))) ? 0x08000000 : 0), "Voices[17].CurPhase" }, { &((Voices[17].StartAddr)), SF_IS_BOOL(&((Voices[17].StartAddr))) ? 1 : sizeof((Voices[17].StartAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[17].StartAddr))) ? 0x08000000 : 0), "Voices[17].StartAddr" }, { &((Voices[17].CurAddr)), SF_IS_BOOL(&((Voices[17].CurAddr))) ? 1 : sizeof((Voices[17].CurAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[17].CurAddr))) ? 0x08000000 : 0), "Voices[17].CurAddr" }, { &((Voices[17].ADSRControl)), SF_IS_BOOL(&((Voices[17].ADSRControl))) ? 1 : sizeof((Voices[17].ADSRControl)), 0x80000000 | (SF_IS_BOOL(&((Voices[17].ADSRControl))) ? 0x08000000 : 0), "Voices[17].ADSRControl" }, { &((Voices[17].LoopAddr)), SF_IS_BOOL(&((Voices[17].LoopAddr))) ? 1 : sizeof((Voices[17].LoopAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[17].LoopAddr))) ? 0x08000000 : 0), "Voices[17].LoopAddr" }, { &((Voices[17].PreLRSample)), SF_IS_BOOL(&((Voices[17].PreLRSample))) ? 1 : sizeof((Voices[17].PreLRSample)), 0x80000000 | (SF_IS_BOOL(&((Voices[17].PreLRSample))) ? 0x08000000 : 0), "Voices[17].PreLRSample" }, { &((Voices[17].ADSR.EnvLevel)), SF_IS_BOOL(&((Voices[17].ADSR.EnvLevel))) ? 1 : sizeof((Voices[17].ADSR.EnvLevel)), 0x80000000 | (SF_IS_BOOL(&((Voices[17].ADSR.EnvLevel))) ? 0x08000000 : 0), "Voices[17].ADSR.EnvLevel" }, { &((Voices[17].ADSR.Divider)), SF_IS_BOOL(&((Voices[17].ADSR.Divider))) ? 1 : sizeof((Voices[17].ADSR.Divider)), 0x80000000 | (SF_IS_BOOL(&((Voices[17].ADSR.Divider))) ? 0x08000000 : 0), "Voices[17].ADSR.Divider" }, { &((Voices[17].ADSR.Phase)), SF_IS_BOOL(&((Voices[17].ADSR.Phase))) ? 1 : sizeof((Voices[17].ADSR.Phase)), 0x80000000 | (SF_IS_BOOL(&((Voices[17].ADSR.Phase))) ? 0x08000000 : 0), "Voices[17].ADSR.Phase" }, { &((Voices[17].ADSR.AttackExp)), SF_IS_BOOL(&((Voices[17].ADSR.AttackExp))) ? 1 : sizeof((Voices[17].ADSR.AttackExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[17].ADSR.AttackExp))) ? 0x08000000 : 0), "Voices[17].ADSR.AttackExp" }, { &((Voices[17].ADSR.SustainExp)), SF_IS_BOOL(&((Voices[17].ADSR.SustainExp))) ? 1 : sizeof((Voices[17].ADSR.SustainExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[17].ADSR.SustainExp))) ? 0x08000000 : 0), "Voices[17].ADSR.SustainExp" }, { &((Voices[17].ADSR.SustainDec)), SF_IS_BOOL(&((Voices[17].ADSR.SustainDec))) ? 1 : sizeof((Voices[17].ADSR.SustainDec)), 0x80000000 | (SF_IS_BOOL(&((Voices[17].ADSR.SustainDec))) ? 0x08000000 : 0), "Voices[17].ADSR.SustainDec" }, { &((Voices[17].ADSR.ReleaseExp)), SF_IS_BOOL(&((Voices[17].ADSR.ReleaseExp))) ? 1 : sizeof((Voices[17].ADSR.ReleaseExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[17].ADSR.ReleaseExp))) ? 0x08000000 : 0), "Voices[17].ADSR.ReleaseExp" }, { &((Voices[17].ADSR.AttackRate)), SF_IS_BOOL(&((Voices[17].ADSR.AttackRate))) ? 1 : sizeof((Voices[17].ADSR.AttackRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[17].ADSR.AttackRate))) ? 0x08000000 : 0), "Voices[17].ADSR.AttackRate" }, { &((Voices[17].ADSR.DecayRate)), SF_IS_BOOL(&((Voices[17].ADSR.DecayRate))) ? 1 : sizeof((Voices[17].ADSR.DecayRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[17].ADSR.DecayRate))) ? 0x08000000 : 0), "Voices[17].ADSR.DecayRate" }, { &((Voices[17].ADSR.SustainRate)), SF_IS_BOOL(&((Voices[17].ADSR.SustainRate))) ? 1 : sizeof((Voices[17].ADSR.SustainRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[17].ADSR.SustainRate))) ? 0x08000000 : 0), "Voices[17].ADSR.SustainRate" }, { &((Voices[17].ADSR.ReleaseRate)), SF_IS_BOOL(&((Voices[17].ADSR.ReleaseRate))) ? 1 : sizeof((Voices[17].ADSR.ReleaseRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[17].ADSR.ReleaseRate))) ? 0x08000000 : 0), "Voices[17].ADSR.ReleaseRate" }, { &((Voices[17].ADSR.SustainLevel)), SF_IS_BOOL(&((Voices[17].ADSR.SustainLevel))) ? 1 : sizeof((Voices[17].ADSR.SustainLevel)), 0x80000000 | (SF_IS_BOOL(&((Voices[17].ADSR.SustainLevel))) ? 0x08000000 : 0), "Voices[17].ADSR.SustainLevel" },
  { ((&Voices[18].DecodeBuffer[0])), (uint32)(((sizeof(Voices[18].DecodeBuffer) / sizeof(Voices[18].DecodeBuffer[0]))) * sizeof(uint16)), 0x20000000 | SF_FORCE_A16((&Voices[18].DecodeBuffer[0])), "&Voices[18].DecodeBuffer[0]" }, { &((Voices[18].DecodeM2)), SF_IS_BOOL(&((Voices[18].DecodeM2))) ? 1 : sizeof((Voices[18].DecodeM2)), 0x80000000 | (SF_IS_BOOL(&((Voices[18].DecodeM2))) ? 0x08000000 : 0), "Voices[18].DecodeM2" }, { &((Voices[18].DecodeM1)), SF_IS_BOOL(&((Voices[18].DecodeM1))) ? 1 : sizeof((Voices[18].DecodeM1)), 0x80000000 | (SF_IS_BOOL(&((Voices[18].DecodeM1))) ? 0x08000000 : 0), "Voices[18].DecodeM1" }, { &((Voices[18].DecodePlayDelay)), SF_IS_BOOL(&((Voices[18].DecodePlayDelay))) ? 1 : sizeof((Voices[18].DecodePlayDelay)), 0x80000000 | (SF_IS_BOOL(&((Voices[18].DecodePlayDelay))) ? 0x08000000 : 0), "Voices[18].DecodePlayDelay" }, { &((Voices[18].DecodeWritePos)), SF_IS_BOOL(&((Voices[18].DecodeWritePos))) ? 1 : sizeof((Voices[18].DecodeWritePos)), 0x80000000 | (SF_IS_BOOL(&((Voices[18].DecodeWritePos))) ? 0x08000000 : 0), "Voices[18].DecodeWritePos" }, { &((Voices[18].DecodeReadPos)), SF_IS_BOOL(&((Voices[18].DecodeReadPos))) ? 1 : sizeof((Voices[18].DecodeReadPos)), 0x80000000 | (SF_IS_BOOL(&((Voices[18].DecodeReadPos))) ? 0x08000000 : 0), "Voices[18].DecodeReadPos" }, { &((Voices[18].DecodeAvail)), SF_IS_BOOL(&((Voices[18].DecodeAvail))) ? 1 : sizeof((Voices[18].DecodeAvail)), 0x80000000 | (SF_IS_BOOL(&((Voices[18].DecodeAvail))) ? 0x08000000 : 0), "Voices[18].DecodeAvail" }, { &((Voices[18].DecodeShift)), SF_IS_BOOL(&((Voices[18].DecodeShift))) ? 1 : sizeof((Voices[18].DecodeShift)), 0x80000000 | (SF_IS_BOOL(&((Voices[18].DecodeShift))) ? 0x08000000 : 0), "Voices[18].DecodeShift" }, { &((Voices[18].DecodeWeight)), SF_IS_BOOL(&((Voices[18].DecodeWeight))) ? 1 : sizeof((Voices[18].DecodeWeight)), 0x80000000 | (SF_IS_BOOL(&((Voices[18].DecodeWeight))) ? 0x08000000 : 0), "Voices[18].DecodeWeight" }, { &((Voices[18].DecodeFlags)), SF_IS_BOOL(&((Voices[18].DecodeFlags))) ? 1 : sizeof((Voices[18].DecodeFlags)), 0x80000000 | (SF_IS_BOOL(&((Voices[18].DecodeFlags))) ? 0x08000000 : 0), "Voices[18].DecodeFlags" }, { &((Voices[18].IgnoreSampLA)), SF_IS_BOOL(&((Voices[18].IgnoreSampLA))) ? 1 : sizeof((Voices[18].IgnoreSampLA)), 0x80000000 | (SF_IS_BOOL(&((Voices[18].IgnoreSampLA))) ? 0x08000000 : 0), "Voices[18].IgnoreSampLA" }, { &(((Voices[18].Sweep[0]).Control)), SF_IS_BOOL(&(((Voices[18].Sweep[0]).Control))) ? 1 : sizeof(((Voices[18].Sweep[0]).Control)), 0x80000000 | (SF_IS_BOOL(&(((Voices[18].Sweep[0]).Control))) ? 0x08000000 : 0), "(Voices[18].Sweep[0]).Control" }, { &(((Voices[18].Sweep[0]).Current)), SF_IS_BOOL(&(((Voices[18].Sweep[0]).Current))) ? 1 : sizeof(((Voices[18].Sweep[0]).Current)), 0x80000000 | (SF_IS_BOOL(&(((Voices[18].Sweep[0]).Current))) ? 0x08000000 : 0), "(Voices[18].Sweep[0]).Current" }, { &(((Voices[18].Sweep[0]).Divider)), SF_IS_BOOL(&(((Voices[18].Sweep[0]).Divider))) ? 1 : sizeof(((Voices[18].Sweep[0]).Divider)), 0x80000000 | (SF_IS_BOOL(&(((Voices[18].Sweep[0]).Divider))) ? 0x08000000 : 0), "(Voices[18].Sweep[0]).Divider" }, { &(((Voices[18].Sweep[1]).Control)), SF_IS_BOOL(&(((Voices[18].Sweep[1]).Control))) ? 1 : sizeof(((Voices[18].Sweep[1]).Control)), 0x80000000 | (SF_IS_BOOL(&(((Voices[18].Sweep[1]).Control))) ? 0x08000000 : 0), "(Voices[18].Sweep[1]).Control" }, { &(((Voices[18].Sweep[1]).Current)), SF_IS_BOOL(&(((Voices[18].Sweep[1]).Current))) ? 1 : sizeof(((Voices[18].Sweep[1]).Current)), 0x80000000 | (SF_IS_BOOL(&(((Voices[18].Sweep[1]).Current))) ? 0x08000000 : 0), "(Voices[18].Sweep[1]).Current" }, { &(((Voices[18].Sweep[1]).Divider)), SF_IS_BOOL(&(((Voices[18].Sweep[1]).Divider))) ? 1 : sizeof(((Voices[18].Sweep[1]).Divider)), 0x80000000 | (SF_IS_BOOL(&(((Voices[18].Sweep[1]).Divider))) ? 0x08000000 : 0), "(Voices[18].Sweep[1]).Divider" }, { &((Voices[18].Pitch)), SF_IS_BOOL(&((Voices[18].Pitch))) ? 1 : sizeof((Voices[18].Pitch)), 0x80000000 | (SF_IS_BOOL(&((Voices[18].Pitch))) ? 0x08000000 : 0), "Voices[18].Pitch" }, { &((Voices[18].CurPhase)), SF_IS_BOOL(&((Voices[18].CurPhase))) ? 1 : sizeof((Voices[18].CurPhase)), 0x80000000 | (SF_IS_BOOL(&((Voices[18].CurPhase))) ? 0x08000000 : 0), "Voices[18].CurPhase" }, { &((Voices[18].StartAddr)), SF_IS_BOOL(&((Voices[18].StartAddr))) ? 1 : sizeof((Voices[18].StartAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[18].StartAddr))) ? 0x08000000 : 0), "Voices[18].StartAddr" }, { &((Voices[18].CurAddr)), SF_IS_BOOL(&((Voices[18].CurAddr))) ? 1 : sizeof((Voices[18].CurAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[18].CurAddr))) ? 0x08000000 : 0), "Voices[18].CurAddr" }, { &((Voices[18].ADSRControl)), SF_IS_BOOL(&((Voices[18].ADSRControl))) ? 1 : sizeof((Voices[18].ADSRControl)), 0x80000000 | (SF_IS_BOOL(&((Voices[18].ADSRControl))) ? 0x08000000 : 0), "Voices[18].ADSRControl" }, { &((Voices[18].LoopAddr)), SF_IS_BOOL(&((Voices[18].LoopAddr))) ? 1 : sizeof((Voices[18].LoopAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[18].LoopAddr))) ? 0x08000000 : 0), "Voices[18].LoopAddr" }, { &((Voices[18].PreLRSample)), SF_IS_BOOL(&((Voices[18].PreLRSample))) ? 1 : sizeof((Voices[18].PreLRSample)), 0x80000000 | (SF_IS_BOOL(&((Voices[18].PreLRSample))) ? 0x08000000 : 0), "Voices[18].PreLRSample" }, { &((Voices[18].ADSR.EnvLevel)), SF_IS_BOOL(&((Voices[18].ADSR.EnvLevel))) ? 1 : sizeof((Voices[18].ADSR.EnvLevel)), 0x80000000 | (SF_IS_BOOL(&((Voices[18].ADSR.EnvLevel))) ? 0x08000000 : 0), "Voices[18].ADSR.EnvLevel" }, { &((Voices[18].ADSR.Divider)), SF_IS_BOOL(&((Voices[18].ADSR.Divider))) ? 1 : sizeof((Voices[18].ADSR.Divider)), 0x80000000 | (SF_IS_BOOL(&((Voices[18].ADSR.Divider))) ? 0x08000000 : 0), "Voices[18].ADSR.Divider" }, { &((Voices[18].ADSR.Phase)), SF_IS_BOOL(&((Voices[18].ADSR.Phase))) ? 1 : sizeof((Voices[18].ADSR.Phase)), 0x80000000 | (SF_IS_BOOL(&((Voices[18].ADSR.Phase))) ? 0x08000000 : 0), "Voices[18].ADSR.Phase" }, { &((Voices[18].ADSR.AttackExp)), SF_IS_BOOL(&((Voices[18].ADSR.AttackExp))) ? 1 : sizeof((Voices[18].ADSR.AttackExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[18].ADSR.AttackExp))) ? 0x08000000 : 0), "Voices[18].ADSR.AttackExp" }, { &((Voices[18].ADSR.SustainExp)), SF_IS_BOOL(&((Voices[18].ADSR.SustainExp))) ? 1 : sizeof((Voices[18].ADSR.SustainExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[18].ADSR.SustainExp))) ? 0x08000000 : 0), "Voices[18].ADSR.SustainExp" }, { &((Voices[18].ADSR.SustainDec)), SF_IS_BOOL(&((Voices[18].ADSR.SustainDec))) ? 1 : sizeof((Voices[18].ADSR.SustainDec)), 0x80000000 | (SF_IS_BOOL(&((Voices[18].ADSR.SustainDec))) ? 0x08000000 : 0), "Voices[18].ADSR.SustainDec" }, { &((Voices[18].ADSR.ReleaseExp)), SF_IS_BOOL(&((Voices[18].ADSR.ReleaseExp))) ? 1 : sizeof((Voices[18].ADSR.ReleaseExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[18].ADSR.ReleaseExp))) ? 0x08000000 : 0), "Voices[18].ADSR.ReleaseExp" }, { &((Voices[18].ADSR.AttackRate)), SF_IS_BOOL(&((Voices[18].ADSR.AttackRate))) ? 1 : sizeof((Voices[18].ADSR.AttackRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[18].ADSR.AttackRate))) ? 0x08000000 : 0), "Voices[18].ADSR.AttackRate" }, { &((Voices[18].ADSR.DecayRate)), SF_IS_BOOL(&((Voices[18].ADSR.DecayRate))) ? 1 : sizeof((Voices[18].ADSR.DecayRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[18].ADSR.DecayRate))) ? 0x08000000 : 0), "Voices[18].ADSR.DecayRate" }, { &((Voices[18].ADSR.SustainRate)), SF_IS_BOOL(&((Voices[18].ADSR.SustainRate))) ? 1 : sizeof((Voices[18].ADSR.SustainRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[18].ADSR.SustainRate))) ? 0x08000000 : 0), "Voices[18].ADSR.SustainRate" }, { &((Voices[18].ADSR.ReleaseRate)), SF_IS_BOOL(&((Voices[18].ADSR.ReleaseRate))) ? 1 : sizeof((Voices[18].ADSR.ReleaseRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[18].ADSR.ReleaseRate))) ? 0x08000000 : 0), "Voices[18].ADSR.ReleaseRate" }, { &((Voices[18].ADSR.SustainLevel)), SF_IS_BOOL(&((Voices[18].ADSR.SustainLevel))) ? 1 : sizeof((Voices[18].ADSR.SustainLevel)), 0x80000000 | (SF_IS_BOOL(&((Voices[18].ADSR.SustainLevel))) ? 0x08000000 : 0), "Voices[18].ADSR.SustainLevel" },
  { ((&Voices[19].DecodeBuffer[0])), (uint32)(((sizeof(Voices[19].DecodeBuffer) / sizeof(Voices[19].DecodeBuffer[0]))) * sizeof(uint16)), 0x20000000 | SF_FORCE_A16((&Voices[19].DecodeBuffer[0])), "&Voices[19].DecodeBuffer[0]" }, { &((Voices[19].DecodeM2)), SF_IS_BOOL(&((Voices[19].DecodeM2))) ? 1 : sizeof((Voices[19].DecodeM2)), 0x80000000 | (SF_IS_BOOL(&((Voices[19].DecodeM2))) ? 0x08000000 : 0), "Voices[19].DecodeM2" }, { &((Voices[19].DecodeM1)), SF_IS_BOOL(&((Voices[19].DecodeM1))) ? 1 : sizeof((Voices[19].DecodeM1)), 0x80000000 | (SF_IS_BOOL(&((Voices[19].DecodeM1))) ? 0x08000000 : 0), "Voices[19].DecodeM1" }, { &((Voices[19].DecodePlayDelay)), SF_IS_BOOL(&((Voices[19].DecodePlayDelay))) ? 1 : sizeof((Voices[19].DecodePlayDelay)), 0x80000000 | (SF_IS_BOOL(&((Voices[19].DecodePlayDelay))) ? 0x08000000 : 0), "Voices[19].DecodePlayDelay" }, { &((Voices[19].DecodeWritePos)), SF_IS_BOOL(&((Voices[19].DecodeWritePos))) ? 1 : sizeof((Voices[19].DecodeWritePos)), 0x80000000 | (SF_IS_BOOL(&((Voices[19].DecodeWritePos))) ? 0x08000000 : 0), "Voices[19].DecodeWritePos" }, { &((Voices[19].DecodeReadPos)), SF_IS_BOOL(&((Voices[19].DecodeReadPos))) ? 1 : sizeof((Voices[19].DecodeReadPos)), 0x80000000 | (SF_IS_BOOL(&((Voices[19].DecodeReadPos))) ? 0x08000000 : 0), "Voices[19].DecodeReadPos" }, { &((Voices[19].DecodeAvail)), SF_IS_BOOL(&((Voices[19].DecodeAvail))) ? 1 : sizeof((Voices[19].DecodeAvail)), 0x80000000 | (SF_IS_BOOL(&((Voices[19].DecodeAvail))) ? 0x08000000 : 0), "Voices[19].DecodeAvail" }, { &((Voices[19].DecodeShift)), SF_IS_BOOL(&((Voices[19].DecodeShift))) ? 1 : sizeof((Voices[19].DecodeShift)), 0x80000000 | (SF_IS_BOOL(&((Voices[19].DecodeShift))) ? 0x08000000 : 0), "Voices[19].DecodeShift" }, { &((Voices[19].DecodeWeight)), SF_IS_BOOL(&((Voices[19].DecodeWeight))) ? 1 : sizeof((Voices[19].DecodeWeight)), 0x80000000 | (SF_IS_BOOL(&((Voices[19].DecodeWeight))) ? 0x08000000 : 0), "Voices[19].DecodeWeight" }, { &((Voices[19].DecodeFlags)), SF_IS_BOOL(&((Voices[19].DecodeFlags))) ? 1 : sizeof((Voices[19].DecodeFlags)), 0x80000000 | (SF_IS_BOOL(&((Voices[19].DecodeFlags))) ? 0x08000000 : 0), "Voices[19].DecodeFlags" }, { &((Voices[19].IgnoreSampLA)), SF_IS_BOOL(&((Voices[19].IgnoreSampLA))) ? 1 : sizeof((Voices[19].IgnoreSampLA)), 0x80000000 | (SF_IS_BOOL(&((Voices[19].IgnoreSampLA))) ? 0x08000000 : 0), "Voices[19].IgnoreSampLA" }, { &(((Voices[19].Sweep[0]).Control)), SF_IS_BOOL(&(((Voices[19].Sweep[0]).Control))) ? 1 : sizeof(((Voices[19].Sweep[0]).Control)), 0x80000000 | (SF_IS_BOOL(&(((Voices[19].Sweep[0]).Control))) ? 0x08000000 : 0), "(Voices[19].Sweep[0]).Control" }, { &(((Voices[19].Sweep[0]).Current)), SF_IS_BOOL(&(((Voices[19].Sweep[0]).Current))) ? 1 : sizeof(((Voices[19].Sweep[0]).Current)), 0x80000000 | (SF_IS_BOOL(&(((Voices[19].Sweep[0]).Current))) ? 0x08000000 : 0), "(Voices[19].Sweep[0]).Current" }, { &(((Voices[19].Sweep[0]).Divider)), SF_IS_BOOL(&(((Voices[19].Sweep[0]).Divider))) ? 1 : sizeof(((Voices[19].Sweep[0]).Divider)), 0x80000000 | (SF_IS_BOOL(&(((Voices[19].Sweep[0]).Divider))) ? 0x08000000 : 0), "(Voices[19].Sweep[0]).Divider" }, { &(((Voices[19].Sweep[1]).Control)), SF_IS_BOOL(&(((Voices[19].Sweep[1]).Control))) ? 1 : sizeof(((Voices[19].Sweep[1]).Control)), 0x80000000 | (SF_IS_BOOL(&(((Voices[19].Sweep[1]).Control))) ? 0x08000000 : 0), "(Voices[19].Sweep[1]).Control" }, { &(((Voices[19].Sweep[1]).Current)), SF_IS_BOOL(&(((Voices[19].Sweep[1]).Current))) ? 1 : sizeof(((Voices[19].Sweep[1]).Current)), 0x80000000 | (SF_IS_BOOL(&(((Voices[19].Sweep[1]).Current))) ? 0x08000000 : 0), "(Voices[19].Sweep[1]).Current" }, { &(((Voices[19].Sweep[1]).Divider)), SF_IS_BOOL(&(((Voices[19].Sweep[1]).Divider))) ? 1 : sizeof(((Voices[19].Sweep[1]).Divider)), 0x80000000 | (SF_IS_BOOL(&(((Voices[19].Sweep[1]).Divider))) ? 0x08000000 : 0), "(Voices[19].Sweep[1]).Divider" }, { &((Voices[19].Pitch)), SF_IS_BOOL(&((Voices[19].Pitch))) ? 1 : sizeof((Voices[19].Pitch)), 0x80000000 | (SF_IS_BOOL(&((Voices[19].Pitch))) ? 0x08000000 : 0), "Voices[19].Pitch" }, { &((Voices[19].CurPhase)), SF_IS_BOOL(&((Voices[19].CurPhase))) ? 1 : sizeof((Voices[19].CurPhase)), 0x80000000 | (SF_IS_BOOL(&((Voices[19].CurPhase))) ? 0x08000000 : 0), "Voices[19].CurPhase" }, { &((Voices[19].StartAddr)), SF_IS_BOOL(&((Voices[19].StartAddr))) ? 1 : sizeof((Voices[19].StartAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[19].StartAddr))) ? 0x08000000 : 0), "Voices[19].StartAddr" }, { &((Voices[19].CurAddr)), SF_IS_BOOL(&((Voices[19].CurAddr))) ? 1 : sizeof((Voices[19].CurAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[19].CurAddr))) ? 0x08000000 : 0), "Voices[19].CurAddr" }, { &((Voices[19].ADSRControl)), SF_IS_BOOL(&((Voices[19].ADSRControl))) ? 1 : sizeof((Voices[19].ADSRControl)), 0x80000000 | (SF_IS_BOOL(&((Voices[19].ADSRControl))) ? 0x08000000 : 0), "Voices[19].ADSRControl" }, { &((Voices[19].LoopAddr)), SF_IS_BOOL(&((Voices[19].LoopAddr))) ? 1 : sizeof((Voices[19].LoopAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[19].LoopAddr))) ? 0x08000000 : 0), "Voices[19].LoopAddr" }, { &((Voices[19].PreLRSample)), SF_IS_BOOL(&((Voices[19].PreLRSample))) ? 1 : sizeof((Voices[19].PreLRSample)), 0x80000000 | (SF_IS_BOOL(&((Voices[19].PreLRSample))) ? 0x08000000 : 0), "Voices[19].PreLRSample" }, { &((Voices[19].ADSR.EnvLevel)), SF_IS_BOOL(&((Voices[19].ADSR.EnvLevel))) ? 1 : sizeof((Voices[19].ADSR.EnvLevel)), 0x80000000 | (SF_IS_BOOL(&((Voices[19].ADSR.EnvLevel))) ? 0x08000000 : 0), "Voices[19].ADSR.EnvLevel" }, { &((Voices[19].ADSR.Divider)), SF_IS_BOOL(&((Voices[19].ADSR.Divider))) ? 1 : sizeof((Voices[19].ADSR.Divider)), 0x80000000 | (SF_IS_BOOL(&((Voices[19].ADSR.Divider))) ? 0x08000000 : 0), "Voices[19].ADSR.Divider" }, { &((Voices[19].ADSR.Phase)), SF_IS_BOOL(&((Voices[19].ADSR.Phase))) ? 1 : sizeof((Voices[19].ADSR.Phase)), 0x80000000 | (SF_IS_BOOL(&((Voices[19].ADSR.Phase))) ? 0x08000000 : 0), "Voices[19].ADSR.Phase" }, { &((Voices[19].ADSR.AttackExp)), SF_IS_BOOL(&((Voices[19].ADSR.AttackExp))) ? 1 : sizeof((Voices[19].ADSR.AttackExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[19].ADSR.AttackExp))) ? 0x08000000 : 0), "Voices[19].ADSR.AttackExp" }, { &((Voices[19].ADSR.SustainExp)), SF_IS_BOOL(&((Voices[19].ADSR.SustainExp))) ? 1 : sizeof((Voices[19].ADSR.SustainExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[19].ADSR.SustainExp))) ? 0x08000000 : 0), "Voices[19].ADSR.SustainExp" }, { &((Voices[19].ADSR.SustainDec)), SF_IS_BOOL(&((Voices[19].ADSR.SustainDec))) ? 1 : sizeof((Voices[19].ADSR.SustainDec)), 0x80000000 | (SF_IS_BOOL(&((Voices[19].ADSR.SustainDec))) ? 0x08000000 : 0), "Voices[19].ADSR.SustainDec" }, { &((Voices[19].ADSR.ReleaseExp)), SF_IS_BOOL(&((Voices[19].ADSR.ReleaseExp))) ? 1 : sizeof((Voices[19].ADSR.ReleaseExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[19].ADSR.ReleaseExp))) ? 0x08000000 : 0), "Voices[19].ADSR.ReleaseExp" }, { &((Voices[19].ADSR.AttackRate)), SF_IS_BOOL(&((Voices[19].ADSR.AttackRate))) ? 1 : sizeof((Voices[19].ADSR.AttackRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[19].ADSR.AttackRate))) ? 0x08000000 : 0), "Voices[19].ADSR.AttackRate" }, { &((Voices[19].ADSR.DecayRate)), SF_IS_BOOL(&((Voices[19].ADSR.DecayRate))) ? 1 : sizeof((Voices[19].ADSR.DecayRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[19].ADSR.DecayRate))) ? 0x08000000 : 0), "Voices[19].ADSR.DecayRate" }, { &((Voices[19].ADSR.SustainRate)), SF_IS_BOOL(&((Voices[19].ADSR.SustainRate))) ? 1 : sizeof((Voices[19].ADSR.SustainRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[19].ADSR.SustainRate))) ? 0x08000000 : 0), "Voices[19].ADSR.SustainRate" }, { &((Voices[19].ADSR.ReleaseRate)), SF_IS_BOOL(&((Voices[19].ADSR.ReleaseRate))) ? 1 : sizeof((Voices[19].ADSR.ReleaseRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[19].ADSR.ReleaseRate))) ? 0x08000000 : 0), "Voices[19].ADSR.ReleaseRate" }, { &((Voices[19].ADSR.SustainLevel)), SF_IS_BOOL(&((Voices[19].ADSR.SustainLevel))) ? 1 : sizeof((Voices[19].ADSR.SustainLevel)), 0x80000000 | (SF_IS_BOOL(&((Voices[19].ADSR.SustainLevel))) ? 0x08000000 : 0), "Voices[19].ADSR.SustainLevel" },
  { ((&Voices[20].DecodeBuffer[0])), (uint32)(((sizeof(Voices[20].DecodeBuffer) / sizeof(Voices[20].DecodeBuffer[0]))) * sizeof(uint16)), 0x20000000 | SF_FORCE_A16((&Voices[20].DecodeBuffer[0])), "&Voices[20].DecodeBuffer[0]" },
  { &((Voices[20].DecodeM2)), SF_IS_BOOL(&((Voices[20].DecodeM2))) ? 1 : sizeof((Voices[20].DecodeM2)), 0x80000000 | (SF_IS_BOOL(&((Voices[20].DecodeM2))) ? 0x08000000 : 0), "Voices[20].DecodeM2" },
  { &((Voices[20].DecodeM1)), SF_IS_BOOL(&((Voices[20].DecodeM1))) ? 1 : sizeof((Voices[20].DecodeM1)), 0x80000000 | (SF_IS_BOOL(&((Voices[20].DecodeM1))) ? 0x08000000 : 0), "Voices[20].DecodeM1" },
  { &((Voices[20].DecodePlayDelay)), SF_IS_BOOL(&((Voices[20].DecodePlayDelay))) ? 1 : sizeof((Voices[20].DecodePlayDelay)), 0x80000000 | (SF_IS_BOOL(&((Voices[20].DecodePlayDelay))) ? 0x08000000 : 0), "Voices[20].DecodePlayDelay" },
  { &((Voices[20].DecodeWritePos)), SF_IS_BOOL(&((Voices[20].DecodeWritePos))) ? 1 : sizeof((Voices[20].DecodeWritePos)), 0x80000000 | (SF_IS_BOOL(&((Voices[20].DecodeWritePos))) ? 0x08000000 : 0), "Voices[20].DecodeWritePos" },
  { &((Voices[20].DecodeReadPos)), SF_IS_BOOL(&((Voices[20].DecodeReadPos))) ? 1 : sizeof((Voices[20].DecodeReadPos)), 0x80000000 | (SF_IS_BOOL(&((Voices[20].DecodeReadPos))) ? 0x08000000 : 0), "Voices[20].DecodeReadPos" },
  { &((Voices[20].DecodeAvail)), SF_IS_BOOL(&((Voices[20].DecodeAvail))) ? 1 : sizeof((Voices[20].DecodeAvail)), 0x80000000 | (SF_IS_BOOL(&((Voices[20].DecodeAvail))) ? 0x08000000 : 0), "Voices[20].DecodeAvail" },
  { &((Voices[20].DecodeShift)), SF_IS_BOOL(&((Voices[20].DecodeShift))) ? 1 : sizeof((Voices[20].DecodeShift)), 0x80000000 | (SF_IS_BOOL(&((Voices[20].DecodeShift))) ? 0x08000000 : 0), "Voices[20].DecodeShift" },
  { &((Voices[20].DecodeWeight)), SF_IS_BOOL(&((Voices[20].DecodeWeight))) ? 1 : sizeof((Voices[20].DecodeWeight)), 0x80000000 | (SF_IS_BOOL(&((Voices[20].DecodeWeight))) ? 0x08000000 : 0), "Voices[20].DecodeWeight" },
  { &((Voices[20].DecodeFlags)), SF_IS_BOOL(&((Voices[20].DecodeFlags))) ? 1 : sizeof((Voices[20].DecodeFlags)), 0x80000000 | (SF_IS_BOOL(&((Voices[20].DecodeFlags))) ? 0x08000000 : 0), "Voices[20].DecodeFlags" },
  { &((Voices[20].IgnoreSampLA)), SF_IS_BOOL(&((Voices[20].IgnoreSampLA))) ? 1 : sizeof((Voices[20].IgnoreSampLA)), 0x80000000 | (SF_IS_BOOL(&((Voices[20].IgnoreSampLA))) ? 0x08000000 : 0), "Voices[20].IgnoreSampLA" },
  { &(((Voices[20].Sweep[0]).Control)), SF_IS_BOOL(&(((Voices[20].Sweep[0]).Control))) ? 1 : sizeof(((Voices[20].Sweep[0]).Control)), 0x80000000 | (SF_IS_BOOL(&(((Voices[20].Sweep[0]).Control))) ? 0x08000000 : 0), "(Voices[20].Sweep[0]).Control" },
  { &(((Voices[20].Sweep[0]).Current)), SF_IS_BOOL(&(((Voices[20].Sweep[0]).Current))) ? 1 : sizeof(((Voices[20].Sweep[0]).Current)), 0x80000000 | (SF_IS_BOOL(&(((Voices[20].Sweep[0]).Current))) ? 0x08000000 : 0), "(Voices[20].Sweep[0]).Current" },
  { &(((Voices[20].Sweep[0]).Divider)), SF_IS_BOOL(&(((Voices[20].Sweep[0]).Divider))) ? 1 : sizeof(((Voices[20].Sweep[0]).Divider)), 0x80000000 | (SF_IS_BOOL(&(((Voices[20].Sweep[0]).Divider))) ? 0x08000000 : 0), "(Voices[20].Sweep[0]).Divider" },
  { &(((Voices[20].Sweep[1]).Control)), SF_IS_BOOL(&(((Voices[20].Sweep[1]).Control))) ? 1 : sizeof(((Voices[20].Sweep[1]).Control)), 0x80000000 | (SF_IS_BOOL(&(((Voices[20].Sweep[1]).Control))) ? 0x08000000 : 0), "(Voices[20].Sweep[1]).Control" },
  { &(((Voices[20].Sweep[1]).Current)), SF_IS_BOOL(&(((Voices[20].Sweep[1]).Current))) ? 1 : sizeof(((Voices[20].Sweep[1]).Current)), 0x80000000 | (SF_IS_BOOL(&(((Voices[20].Sweep[1]).Current))) ? 0x08000000 : 0), "(Voices[20].Sweep[1]).Current" },
  { &(((Voices[20].Sweep[1]).Divider)), SF_IS_BOOL(&(((Voices[20].Sweep[1]).Divider))) ? 1 : sizeof(((Voices[20].Sweep[1]).Divider)), 0x80000000 | (SF_IS_BOOL(&(((Voices[20].Sweep[1]).Divider))) ? 0x08000000 : 0), "(Voices[20].Sweep[1]).Divider" },
  { &((Voices[20].Pitch)), SF_IS_BOOL(&((Voices[20].Pitch))) ? 1 : sizeof((Voices[20].Pitch)), 0x80000000 | (SF_IS_BOOL(&((Voices[20].Pitch))) ? 0x08000000 : 0), "Voices[20].Pitch" },
  { &((Voices[20].CurPhase)), SF_IS_BOOL(&((Voices[20].CurPhase))) ? 1 : sizeof((Voices[20].CurPhase)), 0x80000000 | (SF_IS_BOOL(&((Voices[20].CurPhase))) ? 0x08000000 : 0), "Voices[20].CurPhase" },
  { &((Voices[20].StartAddr)), SF_IS_BOOL(&((Voices[20].StartAddr))) ? 1 : sizeof((Voices[20].StartAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[20].StartAddr))) ? 0x08000000 : 0), "Voices[20].StartAddr" },
  { &((Voices[20].CurAddr)), SF_IS_BOOL(&((Voices[20].CurAddr))) ? 1 : sizeof((Voices[20].CurAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[20].CurAddr))) ? 0x08000000 : 0), "Voices[20].CurAddr" },
  { &((Voices[20].ADSRControl)), SF_IS_BOOL(&((Voices[20].ADSRControl))) ? 1 : sizeof((Voices[20].ADSRControl)), 0x80000000 | (SF_IS_BOOL(&((Voices[20].ADSRControl))) ? 0x08000000 : 0), "Voices[20].ADSRControl" },
  { &((Voices[20].LoopAddr)), SF_IS_BOOL(&((Voices[20].LoopAddr))) ? 1 : sizeof((Voices[20].LoopAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[20].LoopAddr))) ? 0x08000000 : 0), "Voices[20].LoopAddr" },
  { &((Voices[20].PreLRSample)), SF_IS_BOOL(&((Voices[20].PreLRSample))) ? 1 : sizeof((Voices[20].PreLRSample)), 0x80000000 | (SF_IS_BOOL(&((Voices[20].PreLRSample))) ? 0x08000000 : 0), "Voices[20].PreLRSample" },
  { &((Voices[20].ADSR.EnvLevel)), SF_IS_BOOL(&((Voices[20].ADSR.EnvLevel))) ? 1 : sizeof((Voices[20].ADSR.EnvLevel)), 0x80000000 | (SF_IS_BOOL(&((Voices[20].ADSR.EnvLevel))) ? 0x08000000 : 0), "Voices[20].ADSR.EnvLevel" },
  { &((Voices[20].ADSR.Divider)), SF_IS_BOOL(&((Voices[20].ADSR.Divider))) ? 1 : sizeof((Voices[20].ADSR.Divider)), 0x80000000 | (SF_IS_BOOL(&((Voices[20].ADSR.Divider))) ? 0x08000000 : 0), "Voices[20].ADSR.Divider" },
  { &((Voices[20].ADSR.Phase)), SF_IS_BOOL(&((Voices[20].ADSR.Phase))) ? 1 : sizeof((Voices[20].ADSR.Phase)), 0x80000000 | (SF_IS_BOOL(&((Voices[20].ADSR.Phase))) ? 0x08000000 : 0), "Voices[20].ADSR.Phase" },
  { &((Voices[20].ADSR.AttackExp)), SF_IS_BOOL(&((Voices[20].ADSR.AttackExp))) ? 1 : sizeof((Voices[20].ADSR.AttackExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[20].ADSR.AttackExp))) ? 0x08000000 : 0), "Voices[20].ADSR.AttackExp" },
  { &((Voices[20].ADSR.SustainExp)), SF_IS_BOOL(&((Voices[20].ADSR.SustainExp))) ? 1 : sizeof((Voices[20].ADSR.SustainExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[20].ADSR.SustainExp))) ? 0x08000000 : 0), "Voices[20].ADSR.SustainExp" },
  { &((Voices[20].ADSR.SustainDec)), SF_IS_BOOL(&((Voices[20].ADSR.SustainDec))) ? 1 : sizeof((Voices[20].ADSR.SustainDec)), 0x80000000 | (SF_IS_BOOL(&((Voices[20].ADSR.SustainDec))) ? 0x08000000 : 0), "Voices[20].ADSR.SustainDec" },
  { &((Voices[20].ADSR.ReleaseExp)), SF_IS_BOOL(&((Voices[20].ADSR.ReleaseExp))) ? 1 : sizeof((Voices[20].ADSR.ReleaseExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[20].ADSR.ReleaseExp))) ? 0x08000000 : 0), "Voices[20].ADSR.ReleaseExp" },
  { &((Voices[20].ADSR.AttackRate)), SF_IS_BOOL(&((Voices[20].ADSR.AttackRate))) ? 1 : sizeof((Voices[20].ADSR.AttackRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[20].ADSR.AttackRate))) ? 0x08000000 : 0), "Voices[20].ADSR.AttackRate" },
  { &((Voices[20].ADSR.DecayRate)), SF_IS_BOOL(&((Voices[20].ADSR.DecayRate))) ? 1 : sizeof((Voices[20].ADSR.DecayRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[20].ADSR.DecayRate))) ? 0x08000000 : 0), "Voices[20].ADSR.DecayRate" },
  { &((Voices[20].ADSR.SustainRate)), SF_IS_BOOL(&((Voices[20].ADSR.SustainRate))) ? 1 : sizeof((Voices[20].ADSR.SustainRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[20].ADSR.SustainRate))) ? 0x08000000 : 0), "Voices[20].ADSR.SustainRate" },
  { &((Voices[20].ADSR.ReleaseRate)), SF_IS_BOOL(&((Voices[20].ADSR.ReleaseRate))) ? 1 : sizeof((Voices[20].ADSR.ReleaseRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[20].ADSR.ReleaseRate))) ? 0x08000000 : 0), "Voices[20].ADSR.ReleaseRate" },
  { &((Voices[20].ADSR.SustainLevel)), SF_IS_BOOL(&((Voices[20].ADSR.SustainLevel))) ? 1 : sizeof((Voices[20].ADSR.SustainLevel)), 0x80000000 | (SF_IS_BOOL(&((Voices[20].ADSR.SustainLevel))) ? 0x08000000 : 0), "Voices[20].ADSR.SustainLevel" },
  { ((&Voices[21].DecodeBuffer[0])), (uint32)(((sizeof(Voices[21].DecodeBuffer) / sizeof(Voices[21].DecodeBuffer[0]))) * sizeof(uint16)), 0x20000000 | SF_FORCE_A16((&Voices[21].DecodeBuffer[0])), "&Voices[21].DecodeBuffer[0]" },
  { &((Voices[21].DecodeM2)), SF_IS_BOOL(&((Voices[21].DecodeM2))) ? 1 : sizeof((Voices[21].DecodeM2)), 0x80000000 | (SF_IS_BOOL(&((Voices[21].DecodeM2))) ? 0x08000000 : 0), "Voices[21].DecodeM2" },
  { &((Voices[21].DecodeM1)), SF_IS_BOOL(&((Voices[21].DecodeM1))) ? 1 : sizeof((Voices[21].DecodeM1)), 0x80000000 | (SF_IS_BOOL(&((Voices[21].DecodeM1))) ? 0x08000000 : 0), "Voices[21].DecodeM1" },
  { &((Voices[21].DecodePlayDelay)), SF_IS_BOOL(&((Voices[21].DecodePlayDelay))) ? 1 : sizeof((Voices[21].DecodePlayDelay)), 0x80000000 | (SF_IS_BOOL(&((Voices[21].DecodePlayDelay))) ? 0x08000000 : 0), "Voices[21].DecodePlayDelay" },
  { &((Voices[21].DecodeWritePos)), SF_IS_BOOL(&((Voices[21].DecodeWritePos))) ? 1 : sizeof((Voices[21].DecodeWritePos)), 0x80000000 | (SF_IS_BOOL(&((Voices[21].DecodeWritePos))) ? 0x08000000 : 0), "Voices[21].DecodeWritePos" },
  { &((Voices[21].DecodeReadPos)), SF_IS_BOOL(&((Voices[21].DecodeReadPos))) ? 1 : sizeof((Voices[21].DecodeReadPos)), 0x80000000 | (SF_IS_BOOL(&((Voices[21].DecodeReadPos))) ? 0x08000000 : 0), "Voices[21].DecodeReadPos" },
  { &((Voices[21].DecodeAvail)), SF_IS_BOOL(&((Voices[21].DecodeAvail))) ? 1 : sizeof((Voices[21].DecodeAvail)), 0x80000000 | (SF_IS_BOOL(&((Voices[21].DecodeAvail))) ? 0x08000000 : 0), "Voices[21].DecodeAvail" },
  { &((Voices[21].DecodeShift)), SF_IS_BOOL(&((Voices[21].DecodeShift))) ? 1 : sizeof((Voices[21].DecodeShift)), 0x80000000 | (SF_IS_BOOL(&((Voices[21].DecodeShift))) ? 0x08000000 : 0), "Voices[21].DecodeShift" },
  { &((Voices[21].DecodeWeight)), SF_IS_BOOL(&((Voices[21].DecodeWeight))) ? 1 : sizeof((Voices[21].DecodeWeight)), 0x80000000 | (SF_IS_BOOL(&((Voices[21].DecodeWeight))) ? 0x08000000 : 0), "Voices[21].DecodeWeight" },
  { &((Voices[21].DecodeFlags)), SF_IS_BOOL(&((Voices[21].DecodeFlags))) ? 1 : sizeof((Voices[21].DecodeFlags)), 0x80000000 | (SF_IS_BOOL(&((Voices[21].DecodeFlags))) ? 0x08000000 : 0), "Voices[21].DecodeFlags" },
  { &((Voices[21].IgnoreSampLA)), SF_IS_BOOL(&((Voices[21].IgnoreSampLA))) ? 1 : sizeof((Voices[21].IgnoreSampLA)), 0x80000000 | (SF_IS_BOOL(&((Voices[21].IgnoreSampLA))) ? 0x08000000 : 0), "Voices[21].IgnoreSampLA" },
  { &(((Voices[21].Sweep[0]).Control)), SF_IS_BOOL(&(((Voices[21].Sweep[0]).Control))) ? 1 : sizeof(((Voices[21].Sweep[0]).Control)), 0x80000000 | (SF_IS_BOOL(&(((Voices[21].Sweep[0]).Control))) ? 0x08000000 : 0), "(Voices[21].Sweep[0]).Control" },
  { &(((Voices[21].Sweep[0]).Current)), SF_IS_BOOL(&(((Voices[21].Sweep[0]).Current))) ? 1 : sizeof(((Voices[21].Sweep[0]).Current)), 0x80000000 | (SF_IS_BOOL(&(((Voices[21].Sweep[0]).Current))) ? 0x08000000 : 0), "(Voices[21].Sweep[0]).Current" },
  { &(((Voices[21].Sweep[0]).Divider)), SF_IS_BOOL(&(((Voices[21].Sweep[0]).Divider))) ? 1 : sizeof(((Voices[21].Sweep[0]).Divider)), 0x80000000 | (SF_IS_BOOL(&(((Voices[21].Sweep[0]).Divider))) ? 0x08000000 : 0), "(Voices[21].Sweep[0]).Divider" },
  { &(((Voices[21].Sweep[1]).Control)), SF_IS_BOOL(&(((Voices[21].Sweep[1]).Control))) ? 1 : sizeof(((Voices[21].Sweep[1]).Control)), 0x80000000 | (SF_IS_BOOL(&(((Voices[21].Sweep[1]).Control))) ? 0x08000000 : 0), "(Voices[21].Sweep[1]).Control" },
  { &(((Voices[21].Sweep[1]).Current)), SF_IS_BOOL(&(((Voices[21].Sweep[1]).Current))) ? 1 : sizeof(((Voices[21].Sweep[1]).Current)), 0x80000000 | (SF_IS_BOOL(&(((Voices[21].Sweep[1]).Current))) ? 0x08000000 : 0), "(Voices[21].Sweep[1]).Current" },
  { &(((Voices[21].Sweep[1]).Divider)), SF_IS_BOOL(&(((Voices[21].Sweep[1]).Divider))) ? 1 : sizeof(((Voices[21].Sweep[1]).Divider)), 0x80000000 | (SF_IS_BOOL(&(((Voices[21].Sweep[1]).Divider))) ? 0x08000000 : 0), "(Voices[21].Sweep[1]).Divider" },
  { &((Voices[21].Pitch)), SF_IS_BOOL(&((Voices[21].Pitch))) ? 1 : sizeof((Voices[21].Pitch)), 0x80000000 | (SF_IS_BOOL(&((Voices[21].Pitch))) ? 0x08000000 : 0), "Voices[21].Pitch" },
  { &((Voices[21].CurPhase)), SF_IS_BOOL(&((Voices[21].CurPhase))) ? 1 : sizeof((Voices[21].CurPhase)), 0x80000000 | (SF_IS_BOOL(&((Voices[21].CurPhase))) ? 0x08000000 : 0), "Voices[21].CurPhase" },
  { &((Voices[21].StartAddr)), SF_IS_BOOL(&((Voices[21].StartAddr))) ? 1 : sizeof((Voices[21].StartAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[21].StartAddr))) ? 0x08000000 : 0), "Voices[21].StartAddr" },
  { &((Voices[21].CurAddr)), SF_IS_BOOL(&((Voices[21].CurAddr))) ? 1 : sizeof((Voices[21].CurAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[21].CurAddr))) ? 0x08000000 : 0), "Voices[21].CurAddr" },
  { &((Voices[21].ADSRControl)), SF_IS_BOOL(&((Voices[21].ADSRControl))) ? 1 : sizeof((Voices[21].ADSRControl)), 0x80000000 | (SF_IS_BOOL(&((Voices[21].ADSRControl))) ? 0x08000000 : 0), "Voices[21].ADSRControl" },
  { &((Voices[21].LoopAddr)), SF_IS_BOOL(&((Voices[21].LoopAddr))) ? 1 : sizeof((Voices[21].LoopAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[21].LoopAddr))) ? 0x08000000 : 0), "Voices[21].LoopAddr" },
  { &((Voices[21].PreLRSample)), SF_IS_BOOL(&((Voices[21].PreLRSample))) ? 1 : sizeof((Voices[21].PreLRSample)), 0x80000000 | (SF_IS_BOOL(&((Voices[21].PreLRSample))) ? 0x08000000 : 0), "Voices[21].PreLRSample" },
  { &((Voices[21].ADSR.EnvLevel)), SF_IS_BOOL(&((Voices[21].ADSR.EnvLevel))) ? 1 : sizeof((Voices[21].ADSR.EnvLevel)), 0x80000000 | (SF_IS_BOOL(&((Voices[21].ADSR.EnvLevel))) ? 0x08000000 : 0), "Voices[21].ADSR.EnvLevel" },
  { &((Voices[21].ADSR.Divider)), SF_IS_BOOL(&((Voices[21].ADSR.Divider))) ? 1 : sizeof((Voices[21].ADSR.Divider)), 0x80000000 | (SF_IS_BOOL(&((Voices[21].ADSR.Divider))) ? 0x08000000 : 0), "Voices[21].ADSR.Divider" },
  { &((Voices[21].ADSR.Phase)), SF_IS_BOOL(&((Voices[21].ADSR.Phase))) ? 1 : sizeof((Voices[21].ADSR.Phase)), 0x80000000 | (SF_IS_BOOL(&((Voices[21].ADSR.Phase))) ? 0x08000000 : 0), "Voices[21].ADSR.Phase" },
  { &((Voices[21].ADSR.AttackExp)), SF_IS_BOOL(&((Voices[21].ADSR.AttackExp))) ? 1 : sizeof((Voices[21].ADSR.AttackExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[21].ADSR.AttackExp))) ? 0x08000000 : 0), "Voices[21].ADSR.AttackExp" },
  { &((Voices[21].ADSR.SustainExp)), SF_IS_BOOL(&((Voices[21].ADSR.SustainExp))) ? 1 : sizeof((Voices[21].ADSR.SustainExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[21].ADSR.SustainExp))) ? 0x08000000 : 0), "Voices[21].ADSR.SustainExp" },
  { &((Voices[21].ADSR.SustainDec)), SF_IS_BOOL(&((Voices[21].ADSR.SustainDec))) ? 1 : sizeof((Voices[21].ADSR.SustainDec)), 0x80000000 | (SF_IS_BOOL(&((Voices[21].ADSR.SustainDec))) ? 0x08000000 : 0), "Voices[21].ADSR.SustainDec" },
  { &((Voices[21].ADSR.ReleaseExp)), SF_IS_BOOL(&((Voices[21].ADSR.ReleaseExp))) ? 1 : sizeof((Voices[21].ADSR.ReleaseExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[21].ADSR.ReleaseExp))) ? 0x08000000 : 0), "Voices[21].ADSR.ReleaseExp" },
  { &((Voices[21].ADSR.AttackRate)), SF_IS_BOOL(&((Voices[21].ADSR.AttackRate))) ? 1 : sizeof((Voices[21].ADSR.AttackRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[21].ADSR.AttackRate))) ? 0x08000000 : 0), "Voices[21].ADSR.AttackRate" },
  { &((Voices[21].ADSR.DecayRate)), SF_IS_BOOL(&((Voices[21].ADSR.DecayRate))) ? 1 : sizeof((Voices[21].ADSR.DecayRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[21].ADSR.DecayRate))) ? 0x08000000 : 0), "Voices[21].ADSR.DecayRate" },
  { &((Voices[21].ADSR.SustainRate)), SF_IS_BOOL(&((Voices[21].ADSR.SustainRate))) ? 1 : sizeof((Voices[21].ADSR.SustainRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[21].ADSR.SustainRate))) ? 0x08000000 : 0), "Voices[21].ADSR.SustainRate" },
  { &((Voices[21].ADSR.ReleaseRate)), SF_IS_BOOL(&((Voices[21].ADSR.ReleaseRate))) ? 1 : sizeof((Voices[21].ADSR.ReleaseRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[21].ADSR.ReleaseRate))) ? 0x08000000 : 0), "Voices[21].ADSR.ReleaseRate" },
  { &((Voices[21].ADSR.SustainLevel)), SF_IS_BOOL(&((Voices[21].ADSR.SustainLevel))) ? 1 : sizeof((Voices[21].ADSR.SustainLevel)), 0x80000000 | (SF_IS_BOOL(&((Voices[21].ADSR.SustainLevel))) ? 0x08000000 : 0), "Voices[21].ADSR.SustainLevel" },
  { ((&Voices[22].DecodeBuffer[0])), (uint32)(((sizeof(Voices[22].DecodeBuffer) / sizeof(Voices[22].DecodeBuffer[0]))) * sizeof(uint16)), 0x20000000 | SF_FORCE_A16((&Voices[22].DecodeBuffer[0])), "&Voices[22].DecodeBuffer[0]" },
  { &((Voices[22].DecodeM2)), SF_IS_BOOL(&((Voices[22].DecodeM2))) ? 1 : sizeof((Voices[22].DecodeM2)), 0x80000000 | (SF_IS_BOOL(&((Voices[22].DecodeM2))) ? 0x08000000 : 0), "Voices[22].DecodeM2" },
  { &((Voices[22].DecodeM1)), SF_IS_BOOL(&((Voices[22].DecodeM1))) ? 1 : sizeof((Voices[22].DecodeM1)), 0x80000000 | (SF_IS_BOOL(&((Voices[22].DecodeM1))) ? 0x08000000 : 0), "Voices[22].DecodeM1" },
  { &((Voices[22].DecodePlayDelay)), SF_IS_BOOL(&((Voices[22].DecodePlayDelay))) ? 1 : sizeof((Voices[22].DecodePlayDelay)), 0x80000000 | (SF_IS_BOOL(&((Voices[22].DecodePlayDelay))) ? 0x08000000 : 0), "Voices[22].DecodePlayDelay" },
  { &((Voices[22].DecodeWritePos)), SF_IS_BOOL(&((Voices[22].DecodeWritePos))) ? 1 : sizeof((Voices[22].DecodeWritePos)), 0x80000000 | (SF_IS_BOOL(&((Voices[22].DecodeWritePos))) ? 0x08000000 : 0), "Voices[22].DecodeWritePos" },
  { &((Voices[22].DecodeReadPos)), SF_IS_BOOL(&((Voices[22].DecodeReadPos))) ? 1 : sizeof((Voices[22].DecodeReadPos)), 0x80000000 | (SF_IS_BOOL(&((Voices[22].DecodeReadPos))) ? 0x08000000 : 0), "Voices[22].DecodeReadPos" },
  { &((Voices[22].DecodeAvail)), SF_IS_BOOL(&((Voices[22].DecodeAvail))) ? 1 : sizeof((Voices[22].DecodeAvail)), 0x80000000 | (SF_IS_BOOL(&((Voices[22].DecodeAvail))) ? 0x08000000 : 0), "Voices[22].DecodeAvail" },
  { &((Voices[22].DecodeShift)), SF_IS_BOOL(&((Voices[22].DecodeShift))) ? 1 : sizeof((Voices[22].DecodeShift)), 0x80000000 | (SF_IS_BOOL(&((Voices[22].DecodeShift))) ? 0x08000000 : 0), "Voices[22].DecodeShift" },
  { &((Voices[22].DecodeWeight)), SF_IS_BOOL(&((Voices[22].DecodeWeight))) ? 1 : sizeof((Voices[22].DecodeWeight)), 0x80000000 | (SF_IS_BOOL(&((Voices[22].DecodeWeight))) ? 0x08000000 : 0), "Voices[22].DecodeWeight" },
  { &((Voices[22].DecodeFlags)), SF_IS_BOOL(&((Voices[22].DecodeFlags))) ? 1 : sizeof((Voices[22].DecodeFlags)), 0x80000000 | (SF_IS_BOOL(&((Voices[22].DecodeFlags))) ? 0x08000000 : 0), "Voices[22].DecodeFlags" },
  { &((Voices[22].IgnoreSampLA)), SF_IS_BOOL(&((Voices[22].IgnoreSampLA))) ? 1 : sizeof((Voices[22].IgnoreSampLA)), 0x80000000 | (SF_IS_BOOL(&((Voices[22].IgnoreSampLA))) ? 0x08000000 : 0), "Voices[22].IgnoreSampLA" },
  { &(((Voices[22].Sweep[0]).Control)), SF_IS_BOOL(&(((Voices[22].Sweep[0]).Control))) ? 1 : sizeof(((Voices[22].Sweep[0]).Control)), 0x80000000 | (SF_IS_BOOL(&(((Voices[22].Sweep[0]).Control))) ? 0x08000000 : 0), "(Voices[22].Sweep[0]).Control" },
  { &(((Voices[22].Sweep[0]).Current)), SF_IS_BOOL(&(((Voices[22].Sweep[0]).Current))) ? 1 : sizeof(((Voices[22].Sweep[0]).Current)), 0x80000000 | (SF_IS_BOOL(&(((Voices[22].Sweep[0]).Current))) ? 0x08000000 : 0), "(Voices[22].Sweep[0]).Current" },
  { &(((Voices[22].Sweep[0]).Divider)), SF_IS_BOOL(&(((Voices[22].Sweep[0]).Divider))) ? 1 : sizeof(((Voices[22].Sweep[0]).Divider)), 0x80000000 | (SF_IS_BOOL(&(((Voices[22].Sweep[0]).Divider))) ? 0x08000000 : 0), "(Voices[22].Sweep[0]).Divider" },
  { &(((Voices[22].Sweep[1]).Control)), SF_IS_BOOL(&(((Voices[22].Sweep[1]).Control))) ? 1 : sizeof(((Voices[22].Sweep[1]).Control)), 0x80000000 | (SF_IS_BOOL(&(((Voices[22].Sweep[1]).Control))) ? 0x08000000 : 0), "(Voices[22].Sweep[1]).Control" },
  { &(((Voices[22].Sweep[1]).Current)), SF_IS_BOOL(&(((Voices[22].Sweep[1]).Current))) ? 1 : sizeof(((Voices[22].Sweep[1]).Current)), 0x80000000 | (SF_IS_BOOL(&(((Voices[22].Sweep[1]).Current))) ? 0x08000000 : 0), "(Voices[22].Sweep[1]).Current" },
  { &(((Voices[22].Sweep[1]).Divider)), SF_IS_BOOL(&(((Voices[22].Sweep[1]).Divider))) ? 1 : sizeof(((Voices[22].Sweep[1]).Divider)), 0x80000000 | (SF_IS_BOOL(&(((Voices[22].Sweep[1]).Divider))) ? 0x08000000 : 0), "(Voices[22].Sweep[1]).Divider" },
  { &((Voices[22].Pitch)), SF_IS_BOOL(&((Voices[22].Pitch))) ? 1 : sizeof((Voices[22].Pitch)), 0x80000000 | (SF_IS_BOOL(&((Voices[22].Pitch))) ? 0x08000000 : 0), "Voices[22].Pitch" },
  { &((Voices[22].CurPhase)), SF_IS_BOOL(&((Voices[22].CurPhase))) ? 1 : sizeof((Voices[22].CurPhase)), 0x80000000 | (SF_IS_BOOL(&((Voices[22].CurPhase))) ? 0x08000000 : 0), "Voices[22].CurPhase" },
  { &((Voices[22].StartAddr)), SF_IS_BOOL(&((Voices[22].StartAddr))) ? 1 : sizeof((Voices[22].StartAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[22].StartAddr))) ? 0x08000000 : 0), "Voices[22].StartAddr" },
  { &((Voices[22].CurAddr)), SF_IS_BOOL(&((Voices[22].CurAddr))) ? 1 : sizeof((Voices[22].CurAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[22].CurAddr))) ? 0x08000000 : 0), "Voices[22].CurAddr" },
  { &((Voices[22].ADSRControl)), SF_IS_BOOL(&((Voices[22].ADSRControl))) ? 1 : sizeof((Voices[22].ADSRControl)), 0x80000000 | (SF_IS_BOOL(&((Voices[22].ADSRControl))) ? 0x08000000 : 0), "Voices[22].ADSRControl" },
  { &((Voices[22].LoopAddr)), SF_IS_BOOL(&((Voices[22].LoopAddr))) ? 1 : sizeof((Voices[22].LoopAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[22].LoopAddr))) ? 0x08000000 : 0), "Voices[22].LoopAddr" },
  { &((Voices[22].PreLRSample)), SF_IS_BOOL(&((Voices[22].PreLRSample))) ? 1 : sizeof((Voices[22].PreLRSample)), 0x80000000 | (SF_IS_BOOL(&((Voices[22].PreLRSample))) ? 0x08000000 : 0), "Voices[22].PreLRSample" },
  { &((Voices[22].ADSR.EnvLevel)), SF_IS_BOOL(&((Voices[22].ADSR.EnvLevel))) ? 1 : sizeof((Voices[22].ADSR.EnvLevel)), 0x80000000 | (SF_IS_BOOL(&((Voices[22].ADSR.EnvLevel))) ? 0x08000000 : 0), "Voices[22].ADSR.EnvLevel" },
  { &((Voices[22].ADSR.Divider)), SF_IS_BOOL(&((Voices[22].ADSR.Divider))) ? 1 : sizeof((Voices[22].ADSR.Divider)), 0x80000000 | (SF_IS_BOOL(&((Voices[22].ADSR.Divider))) ? 0x08000000 : 0), "Voices[22].ADSR.Divider" },
  { &((Voices[22].ADSR.Phase)), SF_IS_BOOL(&((Voices[22].ADSR.Phase))) ? 1 : sizeof((Voices[22].ADSR.Phase)), 0x80000000 | (SF_IS_BOOL(&((Voices[22].ADSR.Phase))) ? 0x08000000 : 0), "Voices[22].ADSR.Phase" },
  { &((Voices[22].ADSR.AttackExp)), SF_IS_BOOL(&((Voices[22].ADSR.AttackExp))) ? 1 : sizeof((Voices[22].ADSR.AttackExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[22].ADSR.AttackExp))) ? 0x08000000 : 0), "Voices[22].ADSR.AttackExp" },
  { &((Voices[22].ADSR.SustainExp)), SF_IS_BOOL(&((Voices[22].ADSR.SustainExp))) ? 1 : sizeof((Voices[22].ADSR.SustainExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[22].ADSR.SustainExp))) ? 0x08000000 : 0), "Voices[22].ADSR.SustainExp" },
  { &((Voices[22].ADSR.SustainDec)), SF_IS_BOOL(&((Voices[22].ADSR.SustainDec))) ? 1 : sizeof((Voices[22].ADSR.SustainDec)), 0x80000000 | (SF_IS_BOOL(&((Voices[22].ADSR.SustainDec))) ? 0x08000000 : 0), "Voices[22].ADSR.SustainDec" },
  { &((Voices[22].ADSR.ReleaseExp)), SF_IS_BOOL(&((Voices[22].ADSR.ReleaseExp))) ? 1 : sizeof((Voices[22].ADSR.ReleaseExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[22].ADSR.ReleaseExp))) ? 0x08000000 : 0), "Voices[22].ADSR.ReleaseExp" },
  { &((Voices[22].ADSR.AttackRate)), SF_IS_BOOL(&((Voices[22].ADSR.AttackRate))) ? 1 : sizeof((Voices[22].ADSR.AttackRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[22].ADSR.AttackRate))) ? 0x08000000 : 0), "Voices[22].ADSR.AttackRate" },
  { &((Voices[22].ADSR.DecayRate)), SF_IS_BOOL(&((Voices[22].ADSR.DecayRate))) ? 1 : sizeof((Voices[22].ADSR.DecayRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[22].ADSR.DecayRate))) ? 0x08000000 : 0), "Voices[22].ADSR.DecayRate" },
  { &((Voices[22].ADSR.SustainRate)), SF_IS_BOOL(&((Voices[22].ADSR.SustainRate))) ? 1 : sizeof((Voices[22].ADSR.SustainRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[22].ADSR.SustainRate))) ? 0x08000000 : 0), "Voices[22].ADSR.SustainRate" },
  { &((Voices[22].ADSR.ReleaseRate)), SF_IS_BOOL(&((Voices[22].ADSR.ReleaseRate))) ? 1 : sizeof((Voices[22].ADSR.ReleaseRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[22].ADSR.ReleaseRate))) ? 0x08000000 : 0), "Voices[22].ADSR.ReleaseRate" },
  { &((Voices[22].ADSR.SustainLevel)), SF_IS_BOOL(&((Voices[22].ADSR.SustainLevel))) ? 1 : sizeof((Voices[22].ADSR.SustainLevel)), 0x80000000 | (SF_IS_BOOL(&((Voices[22].ADSR.SustainLevel))) ? 0x08000000 : 0), "Voices[22].ADSR.SustainLevel" },
  { ((&Voices[23].DecodeBuffer[0])), (uint32)(((sizeof(Voices[23].DecodeBuffer) / sizeof(Voices[23].DecodeBuffer[0]))) * sizeof(uint16)), 0x20000000 | SF_FORCE_A16((&Voices[23].DecodeBuffer[0])), "&Voices[23].DecodeBuffer[0]" },
  { &((Voices[23].DecodeM2)), SF_IS_BOOL(&((Voices[23].DecodeM2))) ? 1 : sizeof((Voices[23].DecodeM2)), 0x80000000 | (SF_IS_BOOL(&((Voices[23].DecodeM2))) ? 0x08000000 : 0), "Voices[23].DecodeM2" },
  { &((Voices[23].DecodeM1)), SF_IS_BOOL(&((Voices[23].DecodeM1))) ? 1 : sizeof((Voices[23].DecodeM1)), 0x80000000 | (SF_IS_BOOL(&((Voices[23].DecodeM1))) ? 0x08000000 : 0), "Voices[23].DecodeM1" },
  { &((Voices[23].DecodePlayDelay)), SF_IS_BOOL(&((Voices[23].DecodePlayDelay))) ? 1 : sizeof((Voices[23].DecodePlayDelay)), 0x80000000 | (SF_IS_BOOL(&((Voices[23].DecodePlayDelay))) ? 0x08000000 : 0), "Voices[23].DecodePlayDelay" },
  { &((Voices[23].DecodeWritePos)), SF_IS_BOOL(&((Voices[23].DecodeWritePos))) ? 1 : sizeof((Voices[23].DecodeWritePos)), 0x80000000 | (SF_IS_BOOL(&((Voices[23].DecodeWritePos))) ? 0x08000000 : 0), "Voices[23].DecodeWritePos" },
  { &((Voices[23].DecodeReadPos)), SF_IS_BOOL(&((Voices[23].DecodeReadPos))) ? 1 : sizeof((Voices[23].DecodeReadPos)), 0x80000000 | (SF_IS_BOOL(&((Voices[23].DecodeReadPos))) ? 0x08000000 : 0), "Voices[23].DecodeReadPos" },
  { &((Voices[23].DecodeAvail)), SF_IS_BOOL(&((Voices[23].DecodeAvail))) ? 1 : sizeof((Voices[23].DecodeAvail)), 0x80000000 | (SF_IS_BOOL(&((Voices[23].DecodeAvail))) ? 0x08000000 : 0), "Voices[23].DecodeAvail" },
  { &((Voices[23].DecodeShift)), SF_IS_BOOL(&((Voices[23].DecodeShift))) ? 1 : sizeof((Voices[23].DecodeShift)), 0x80000000 | (SF_IS_BOOL(&((Voices[23].DecodeShift))) ? 0x08000000 : 0), "Voices[23].DecodeShift" },
  { &((Voices[23].DecodeWeight)), SF_IS_BOOL(&((Voices[23].DecodeWeight))) ? 1 : sizeof((Voices[23].DecodeWeight)), 0x80000000 | (SF_IS_BOOL(&((Voices[23].DecodeWeight))) ? 0x08000000 : 0), "Voices[23].DecodeWeight" },
  { &((Voices[23].DecodeFlags)), SF_IS_BOOL(&((Voices[23].DecodeFlags))) ? 1 : sizeof((Voices[23].DecodeFlags)), 0x80000000 | (SF_IS_BOOL(&((Voices[23].DecodeFlags))) ? 0x08000000 : 0), "Voices[23].DecodeFlags" },
  { &((Voices[23].IgnoreSampLA)), SF_IS_BOOL(&((Voices[23].IgnoreSampLA))) ? 1 : sizeof((Voices[23].IgnoreSampLA)), 0x80000000 | (SF_IS_BOOL(&((Voices[23].IgnoreSampLA))) ? 0x08000000 : 0), "Voices[23].IgnoreSampLA" },
  { &(((Voices[23].Sweep[0]).Control)), SF_IS_BOOL(&(((Voices[23].Sweep[0]).Control))) ? 1 : sizeof(((Voices[23].Sweep[0]).Control)), 0x80000000 | (SF_IS_BOOL(&(((Voices[23].Sweep[0]).Control))) ? 0x08000000 : 0), "(Voices[23].Sweep[0]).Control" },
  { &(((Voices[23].Sweep[0]).Current)), SF_IS_BOOL(&(((Voices[23].Sweep[0]).Current))) ? 1 : sizeof(((Voices[23].Sweep[0]).Current)), 0x80000000 | (SF_IS_BOOL(&(((Voices[23].Sweep[0]).Current))) ? 0x08000000 : 0), "(Voices[23].Sweep[0]).Current" },
  { &(((Voices[23].Sweep[0]).Divider)), SF_IS_BOOL(&(((Voices[23].Sweep[0]).Divider))) ? 1 : sizeof(((Voices[23].Sweep[0]).Divider)), 0x80000000 | (SF_IS_BOOL(&(((Voices[23].Sweep[0]).Divider))) ? 0x08000000 : 0), "(Voices[23].Sweep[0]).Divider" },
  { &(((Voices[23].Sweep[1]).Control)), SF_IS_BOOL(&(((Voices[23].Sweep[1]).Control))) ? 1 : sizeof(((Voices[23].Sweep[1]).Control)), 0x80000000 | (SF_IS_BOOL(&(((Voices[23].Sweep[1]).Control))) ? 0x08000000 : 0), "(Voices[23].Sweep[1]).Control" },
  { &(((Voices[23].Sweep[1]).Current)), SF_IS_BOOL(&(((Voices[23].Sweep[1]).Current))) ? 1 : sizeof(((Voices[23].Sweep[1]).Current)), 0x80000000 | (SF_IS_BOOL(&(((Voices[23].Sweep[1]).Current))) ? 0x08000000 : 0), "(Voices[23].Sweep[1]).Current" },
  { &(((Voices[23].Sweep[1]).Divider)), SF_IS_BOOL(&(((Voices[23].Sweep[1]).Divider))) ? 1 : sizeof(((Voices[23].Sweep[1]).Divider)), 0x80000000 | (SF_IS_BOOL(&(((Voices[23].Sweep[1]).Divider))) ? 0x08000000 : 0), "(Voices[23].Sweep[1]).Divider" },
  { &((Voices[23].Pitch)), SF_IS_BOOL(&((Voices[23].Pitch))) ? 1 : sizeof((Voices[23].Pitch)), 0x80000000 | (SF_IS_BOOL(&((Voices[23].Pitch))) ? 0x08000000 : 0), "Voices[23].Pitch" },
  { &((Voices[23].CurPhase)), SF_IS_BOOL(&((Voices[23].CurPhase))) ? 1 : sizeof((Voices[23].CurPhase)), 0x80000000 | (SF_IS_BOOL(&((Voices[23].CurPhase))) ? 0x08000000 : 0), "Voices[23].CurPhase" },
  { &((Voices[23].StartAddr)), SF_IS_BOOL(&((Voices[23].StartAddr))) ? 1 : sizeof((Voices[23].StartAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[23].StartAddr))) ? 0x08000000 : 0), "Voices[23].StartAddr" },
  { &((Voices[23].CurAddr)), SF_IS_BOOL(&((Voices[23].CurAddr))) ? 1 : sizeof((Voices[23].CurAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[23].CurAddr))) ? 0x08000000 : 0), "Voices[23].CurAddr" },
  { &((Voices[23].ADSRControl)), SF_IS_BOOL(&((Voices[23].ADSRControl))) ? 1 : sizeof((Voices[23].ADSRControl)), 0x80000000 | (SF_IS_BOOL(&((Voices[23].ADSRControl))) ? 0x08000000 : 0), "Voices[23].ADSRControl" },
  { &((Voices[23].LoopAddr)), SF_IS_BOOL(&((Voices[23].LoopAddr))) ? 1 : sizeof((Voices[23].LoopAddr)), 0x80000000 | (SF_IS_BOOL(&((Voices[23].LoopAddr))) ? 0x08000000 : 0), "Voices[23].LoopAddr" },
  { &((Voices[23].PreLRSample)), SF_IS_BOOL(&((Voices[23].PreLRSample))) ? 1 : sizeof((Voices[23].PreLRSample)), 0x80000000 | (SF_IS_BOOL(&((Voices[23].PreLRSample))) ? 0x08000000 : 0), "Voices[23].PreLRSample" },
  { &((Voices[23].ADSR.EnvLevel)), SF_IS_BOOL(&((Voices[23].ADSR.EnvLevel))) ? 1 : sizeof((Voices[23].ADSR.EnvLevel)), 0x80000000 | (SF_IS_BOOL(&((Voices[23].ADSR.EnvLevel))) ? 0x08000000 : 0), "Voices[23].ADSR.EnvLevel" },
  { &((Voices[23].ADSR.Divider)), SF_IS_BOOL(&((Voices[23].ADSR.Divider))) ? 1 : sizeof((Voices[23].ADSR.Divider)), 0x80000000 | (SF_IS_BOOL(&((Voices[23].ADSR.Divider))) ? 0x08000000 : 0), "Voices[23].ADSR.Divider" },
  { &((Voices[23].ADSR.Phase)), SF_IS_BOOL(&((Voices[23].ADSR.Phase))) ? 1 : sizeof((Voices[23].ADSR.Phase)), 0x80000000 | (SF_IS_BOOL(&((Voices[23].ADSR.Phase))) ? 0x08000000 : 0), "Voices[23].ADSR.Phase" },
  { &((Voices[23].ADSR.AttackExp)), SF_IS_BOOL(&((Voices[23].ADSR.AttackExp))) ? 1 : sizeof((Voices[23].ADSR.AttackExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[23].ADSR.AttackExp))) ? 0x08000000 : 0), "Voices[23].ADSR.AttackExp" },
  { &((Voices[23].ADSR.SustainExp)), SF_IS_BOOL(&((Voices[23].ADSR.SustainExp))) ? 1 : sizeof((Voices[23].ADSR.SustainExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[23].ADSR.SustainExp))) ? 0x08000000 : 0), "Voices[23].ADSR.SustainExp" },
  { &((Voices[23].ADSR.SustainDec)), SF_IS_BOOL(&((Voices[23].ADSR.SustainDec))) ? 1 : sizeof((Voices[23].ADSR.SustainDec)), 0x80000000 | (SF_IS_BOOL(&((Voices[23].ADSR.SustainDec))) ? 0x08000000 : 0), "Voices[23].ADSR.SustainDec" },
  { &((Voices[23].ADSR.ReleaseExp)), SF_IS_BOOL(&((Voices[23].ADSR.ReleaseExp))) ? 1 : sizeof((Voices[23].ADSR.ReleaseExp)), 0x80000000 | (SF_IS_BOOL(&((Voices[23].ADSR.ReleaseExp))) ? 0x08000000 : 0), "Voices[23].ADSR.ReleaseExp" },
  { &((Voices[23].ADSR.AttackRate)), SF_IS_BOOL(&((Voices[23].ADSR.AttackRate))) ? 1 : sizeof((Voices[23].ADSR.AttackRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[23].ADSR.AttackRate))) ? 0x08000000 : 0), "Voices[23].ADSR.AttackRate" },
  { &((Voices[23].ADSR.DecayRate)), SF_IS_BOOL(&((Voices[23].ADSR.DecayRate))) ? 1 : sizeof((Voices[23].ADSR.DecayRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[23].ADSR.DecayRate))) ? 0x08000000 : 0), "Voices[23].ADSR.DecayRate" },
  { &((Voices[23].ADSR.SustainRate)), SF_IS_BOOL(&((Voices[23].ADSR.SustainRate))) ? 1 : sizeof((Voices[23].ADSR.SustainRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[23].ADSR.SustainRate))) ? 0x08000000 : 0), "Voices[23].ADSR.SustainRate" },
  { &((Voices[23].ADSR.ReleaseRate)), SF_IS_BOOL(&((Voices[23].ADSR.ReleaseRate))) ? 1 : sizeof((Voices[23].ADSR.ReleaseRate)), 0x80000000 | (SF_IS_BOOL(&((Voices[23].ADSR.ReleaseRate))) ? 0x08000000 : 0), "Voices[23].ADSR.ReleaseRate" },
  { &((Voices[23].ADSR.SustainLevel)), SF_IS_BOOL(&((Voices[23].ADSR.SustainLevel))) ? 1 : sizeof((Voices[23].ADSR.SustainLevel)), 0x80000000 | (SF_IS_BOOL(&((Voices[23].ADSR.SustainLevel))) ? 0x08000000 : 0), "Voices[23].ADSR.SustainLevel" },


  { &((NoiseDivider)), sizeof((NoiseDivider)), 0x80000000 | 0, "NoiseDivider" },
  { &((NoiseCounter)), sizeof((NoiseCounter)), 0x80000000 | 0, "NoiseCounter" },
  { &((LFSR)), sizeof((LFSR)), 0x80000000 | 0, "LFSR" },

  { &((FM_Mode)), sizeof((FM_Mode)), 0x80000000 | 0, "FM_Mode" },
  { &((Noise_Mode)), sizeof((Noise_Mode)), 0x80000000 | 0, "Noise_Mode" },
  { &((Reverb_Mode)), sizeof((Reverb_Mode)), 0x80000000 | 0, "Reverb_Mode" },

  { &((ReverbWA)), sizeof((ReverbWA)), 0x80000000 | 0, "ReverbWA" },

  { &(((GlobalSweep[0]).Control)), sizeof(((GlobalSweep[0]).Control)), 0x80000000 | 0, "(GlobalSweep[0]).Control" },
  { &(((GlobalSweep[0]).Current)), sizeof(((GlobalSweep[0]).Current)), 0x80000000 | 0, "(GlobalSweep[0]).Current" },
  { &(((GlobalSweep[0]).Divider)), sizeof(((GlobalSweep[0]).Divider)), 0x80000000 | 0, "(GlobalSweep[0]).Divider" },
  { &(((GlobalSweep[1]).Control)), sizeof(((GlobalSweep[1]).Control)), 0x80000000 | 0, "(GlobalSweep[1]).Control" },
  { &(((GlobalSweep[1]).Current)), sizeof(((GlobalSweep[1]).Current)), 0x80000000 | 0, "(GlobalSweep[1]).Current" },
  { &(((GlobalSweep[1]).Divider)), sizeof(((GlobalSweep[1]).Divider)), 0x80000000 | 0, "(GlobalSweep[1]).Divider" },

  { ((ReverbVol)), (uint32)(((sizeof(ReverbVol) / sizeof(ReverbVol[0]))) * sizeof(uint32)), 0x40000000 | 0, "ReverbVol" },

  { ((CDVol)), (uint32)(((sizeof(CDVol) / sizeof(CDVol[0]))) * sizeof(uint32)), 0x40000000 | 0, "CDVol" },
  { ((ExternVol)), (uint32)(((sizeof(ExternVol) / sizeof(ExternVol[0]))) * sizeof(uint32)), 0x40000000 | 0, "ExternVol" },

  { &((IRQAddr)), sizeof((IRQAddr)), 0x80000000 | 0, "IRQAddr" },

  { &((RWAddr)), sizeof((RWAddr)), 0x80000000 | 0, "RWAddr" },

  { &((SPUControl)),  sizeof((SPUControl)), 0x80000000 | 0, "SPUControl" },

  { &((VoiceOn)), sizeof((VoiceOn)), 0x80000000 | 0, "VoiceOn" },
  { &((VoiceOff)), sizeof((VoiceOff)), 0x80000000 | 0, "VoiceOff" },

  { &((BlockEnd)), sizeof((BlockEnd)), 0x80000000 | 0, "BlockEnd" },

  { &((CWA)), sizeof((CWA)), 0x80000000 | 0, "CWA" },

  { ((unionregs.Regs)), (uint32)(((sizeof(unionregs.Regs) / sizeof(unionregs.Regs[0]))) * sizeof(uint16)), 0x20000000 | 0, "Regs" },
  { ((AuxRegs)), (uint32)(((sizeof(AuxRegs) / sizeof(AuxRegs[0]))) * sizeof(uint16)), 0x20000000 | 0, "AuxRegs" },

  { ((&RDSB[0][0])), (uint32)(((sizeof(RDSB) / sizeof(RDSB[0][0]))) * sizeof(uint16)), 0x20000000 | 0, "&RDSB[0][0]" },
  { &((RDSB_WP)), sizeof((RDSB_WP)), 0x80000000 | 0, "RDSB_WP" },

  { ((&RUSB[0][0])), (uint32)(((sizeof(RUSB) / sizeof(RUSB[0][0]))) * sizeof(uint16)), 0x20000000 | 0, "&RUSB[0][0]" },
  { &((RUSB_WP)), sizeof((RUSB_WP)), 0x80000000 | 0, "RUSB_WP" },

  { &((ReverbCur)), sizeof((ReverbCur)), 0x80000000 | 0, "ReverbCur" },
  { &((IRQAsserted)), 1, 0x80000000 | 0x08000000, "IRQAsserted" },

  { &((clock_divider)), sizeof((clock_divider)), 0x80000000 | 0, "clock_divider" },

  { ((SPURAM)), (uint32)(((524288 / sizeof(uint16))) * sizeof(uint16)), 0x20000000 | 0, "SPURAM" },
  { 0, 0, 0, 0 }
 };
 int ret = 1;

 ret &= MDFNSS_StateAction(sm, load, StateRegs, "SPU");

 if(load)
 {
  for(unsigned i = 0; i < 24; i++)
  {
   Voices[i].DecodeReadPos &= 0x1F;
   Voices[i].DecodeWritePos &= 0x1F;
  }

  RDSB_WP &= 0x3F;
  RUSB_WP &= 0x3F;

  IRQ_Assert(IRQ_SPU, IRQAsserted);
 }

 return(ret);
}

uint16 SPU_PeekSPURAM(uint32 address)
{
 return(SPURAM[address & 0x3FFFF]);
}

void SPU_PokeSPURAM(uint32 address, uint16 value)
{
 SPURAM[address & 0x3FFFF] = value;
}

uint32 SPU_GetRegister(unsigned int which, char *special, const uint32 special_len)
{
 uint32 ret = 0xDEADBEEF;

 if(which >= 0x8000)
 {
  unsigned int v = (which - 0x8000) >> 8;

  switch((which & 0xFF) | 0x8000)
  {
   case GSREG_V0_VOL_CTRL_L:
	ret = unionregs.Regs[v * 8 + 0x0];
	break;

   case GSREG_V0_VOL_CTRL_R:
	ret = unionregs.Regs[v * 8 + 0x1];
	break;

   case GSREG_V0_VOL_L:
	ret = (int16)Voices[v].Sweep[0].Current & 0xFFFF;
	break;

   case GSREG_V0_VOL_R:
	ret = (int16)Voices[v].Sweep[1].Current & 0xFFFF;
	break;

   case GSREG_V0_PITCH:
	ret = Voices[v].Pitch;
	break;

   case GSREG_V0_STARTADDR:
	ret = Voices[v].StartAddr;
	break;

   case GSREG_V0_ADSR_CTRL:
	ret = Voices[v].ADSRControl;
	break;

   case GSREG_V0_ADSR_LEVEL:
	ret = Voices[v].ADSR.EnvLevel;
	break;

   case GSREG_V0_LOOP_ADDR:
	ret = Voices[v].LoopAddr;
	break;

   case GSREG_V0_READ_ADDR:
	ret = Voices[v].CurAddr;
	break;
  }
 }
 else if (which >= 18 && which <= 49)
    ret = unionregs.reverbregs.ReverbRegs[which - GSREG_FB_SRC_A] & 0xFFFF;
 else switch(which)
 {
  case GSREG_SPUCONTROL:
	ret = SPUControl;
	break;

  case GSREG_FM_ON:
	ret = FM_Mode;
	break;

  case GSREG_NOISE_ON:
	ret = Noise_Mode;
	break;

  case GSREG_REVERB_ON:
	ret = Reverb_Mode;
	break;

  case GSREG_CDVOL_L:
	ret = (uint16)CDVol[0];
	break;

  case GSREG_CDVOL_R:
	ret = (uint16)CDVol[1];
	break;

  case GSREG_DRYVOL_CTRL_L:
	ret = unionregs.Regs[0xC0];
	break;

  case GSREG_DRYVOL_CTRL_R:
	ret = unionregs.Regs[0xC1];
	break;

  case GSREG_DRYVOL_L:
	ret = (int16)GlobalSweep[0].Current & 0xFFFF;
	break;

  case GSREG_DRYVOL_R:
	ret = (int16)GlobalSweep[1].Current & 0xFFFF;
	break;

  case GSREG_WETVOL_L:
	ret = (uint16)ReverbVol[0];
	break;

  case GSREG_WETVOL_R:
	ret = (uint16)ReverbVol[1];
	break;

  case GSREG_RWADDR:
	ret = RWAddr;
	break;

  case GSREG_IRQADDR:
	ret = IRQAddr;
	break;

  case GSREG_REVERBWA:
	ret = ReverbWA >> 2;
	break;

  case GSREG_VOICEON:
	ret = VoiceOn;
	break;

  case GSREG_VOICEOFF:
	ret = VoiceOff;
	break;

  case GSREG_BLOCKEND:
	ret = BlockEnd;
	break;

 }

 return(ret);
}

void SPU_SetRegister(unsigned int which, uint32 value)
{

 switch(which)
 {
  case GSREG_SPUCONTROL:
	SPUControl = value;
	break;

  case GSREG_FM_ON:
	FM_Mode = value & 0xFFFFFF;
	break;

  case GSREG_NOISE_ON:
	Noise_Mode = value & 0xFFFFFF;
	break;

  case GSREG_REVERB_ON:
	Reverb_Mode = value & 0xFFFFFF;
	break;

  case GSREG_CDVOL_L:
	CDVol[0] = (int16)value;
	break;

  case GSREG_CDVOL_R:
	CDVol[1] = (int16)value;
	break;

  case GSREG_DRYVOL_CTRL_L:
	unionregs.Regs[0xC0] = value;
	GlobalSweep[0].Control = value;
	//GlobalSweep[0].Control = value;
	break;

  case GSREG_DRYVOL_CTRL_R:
	unionregs.Regs[0xC1] = value;
	GlobalSweep[1].Control = value;
	//GlobalSweep[1].Control = value;
	break;

  case GSREG_DRYVOL_L:
	GlobalSweep[0].Current = value;
	break;

  case GSREG_DRYVOL_R:
	GlobalSweep[1].Current = value;
	break;

  case GSREG_WETVOL_L:
	ReverbVol[0] = (int16)value;
	break;

  case GSREG_WETVOL_R:
	ReverbVol[1] = (int16)value;
	break;

  case GSREG_RWADDR:
	RWAddr = value & 0x3FFFF;
	break;

  case GSREG_IRQADDR:
	IRQAddr = value & 0x3FFFC;
	break;

  //
  // REVERB_WA
  //

  case GSREG_VOICEON:
        VoiceOn = value & 0xFFFFFF;
        break;

  case GSREG_VOICEOFF:
        VoiceOff = value & 0xFFFFFF;
        break;

  case GSREG_BLOCKEND:
        BlockEnd = value & 0xFFFFFF;
        break;


 }
}

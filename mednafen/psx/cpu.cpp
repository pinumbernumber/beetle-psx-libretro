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

#include "psx.h"
#include "cpu.h"

/* TODO
	Make sure load delays are correct.

	Consider preventing IRQs being taken while in a branch delay slot, to prevent potential problems with games that like to be too clever and perform
	un-restartable sequences of instructions.
*/

#define BIU_ENABLE_ICACHE_S1	0x00000800	// Enable I-cache, set 1
#define BIU_ENABLE_DCACHE	0x00000080	// Enable D-cache
#define BIU_TAG_TEST_MODE	0x00000004	// Enable TAG test mode(IsC must be set to 1 as well presumably?)
#define BIU_INVALIDATE_MODE	0x00000002	// Enable Invalidate mode(IsC must be set to 1 as well presumably?)
#define BIU_LOCK		0x00000001	// Enable Lock mode(IsC must be set to 1 as well presumably?)
						// Does lock mode prevent the actual data payload from being modified, while allowing tags to be modified/updated???
                  //
struct
{
   union
   {
      uint32_t Regs[32];
      struct
      {
         uint32_t Unused00;
         uint32_t Unused01;
         uint32_t Unused02;
         uint32_t BPC;		// RW
         uint32_t Unused04;
         uint32_t BDA;		// RW
         uint32_t TAR;
         uint32_t DCIC;	// RW
         uint32_t Unused08;	
         uint32_t BDAM;	// R/W
         uint32_t Unused0A;
         uint32_t BPCM;	// R/W
         uint32_t SR;		// R/W
         uint32_t CAUSE;	// R/W(partial)
         uint32_t EPC;		// R
         uint32_t PRID;	// R
         uint32_t ERREG;	// ?(may not exist, test)
      };
   };
} CP0;

static uint32_t IPCache;
static bool Halted;

namespace MDFN_IEN_PSX
{


PS_CPU::PS_CPU()
{
   Halted = false;

   memset(FastMap, 0, sizeof(FastMap));
   memset(DummyPage, 0xFF, sizeof(DummyPage));	// 0xFF to trigger an illegal instruction exception, so we'll know what's up when debugging.

   for(uint64 a = 0x00000000; a < (1ULL << 32); a += FAST_MAP_PSIZE)
      SetFastMap(DummyPage, a, FAST_MAP_PSIZE);

   CPUHook = NULL;
   ADDBT = NULL;
}

PS_CPU::~PS_CPU()
{


}

void PS_CPU::SetFastMap(void *region_mem, uint32_t region_address, uint32_t region_size)
{
   uint64_t A;
   // FAST_MAP_SHIFT
   // FAST_MAP_PSIZE

   for(A = region_address; A < (uint64)region_address + region_size; A += FAST_MAP_PSIZE)
      FastMap[A >> FAST_MAP_SHIFT] = ((uint8_t *)region_mem - region_address);
}

static INLINE void RecalcIPCache(void)
{
   IPCache = 0;

   if(((CP0.SR & CP0.CAUSE & 0xFF00) && (CP0.SR & 1)) || Halted)
      IPCache = 0x80;
}


void PS_CPU::Power(void)
{
   unsigned i;

   assert(sizeof(ICache) == sizeof(ICache_Bulk));

   memset(GPR, 0, sizeof(GPR));
   memset(&CP0, 0, sizeof(CP0));
   LO = 0;
   HI = 0;

   gte_ts_done = 0;
   muldiv_ts_done = 0;

   BACKED_PC = 0xBFC00000;
   BACKED_new_PC = 4;
   BACKED_new_PC_mask = ~0U;

   BACKED_LDWhich = 0x20;
   BACKED_LDValue = 0;
   LDAbsorb = 0;
   memset(ReadAbsorb, 0, sizeof(ReadAbsorb));
   ReadAbsorbWhich = 0;
   ReadFudge = 0;

   //WriteAbsorb = 0;
   //WriteAbsorbCount = 0;
   //WriteAbsorbMonkey = 0;

   CP0.SR |= (1 << 22);	// BEV
   CP0.SR |= (1 << 21);	// TS

   CP0.PRID = 0x2;

   RecalcIPCache();


   BIU = 0;

   memset(ScratchRAM.data8, 0, 1024);

   // Not quite sure about these poweron/reset values:
   for(i = 0; i < 1024; i++)
   {
      ICache[i].TV = 0x2 | ((BIU & 0x800) ? 0x0 : 0x1);
      ICache[i].Data = 0;
   }

   GTE_Power();
}

int PS_CPU::StateAction(StateMem *sm, int load, int data_only)
{
   SFORMAT StateRegs[] =
   {
      SFARRAY32(GPR, 32),
      SFVAR(LO),
      SFVAR(HI),
      SFVAR(BACKED_PC),
      SFVAR(BACKED_new_PC),
      SFVAR(BACKED_new_PC_mask),

      SFVAR(IPCache),
      SFVAR(Halted),

      SFVAR(BACKED_LDWhich),
      SFVAR(BACKED_LDValue),
      SFVAR(LDAbsorb),

      SFVAR(next_event_ts),
      SFVAR(gte_ts_done),
      SFVAR(muldiv_ts_done),

      SFVAR(BIU),
      SFARRAY32(ICache_Bulk, 2048),

      SFARRAY32(CP0.Regs, 32),

      SFARRAY(ReadAbsorb, 0x20),
      SFVAR(ReadAbsorbDummy),
      SFVAR(ReadAbsorbWhich),
      SFVAR(ReadFudge),

      SFARRAY(ScratchRAM.data8, 1024),

      SFEND
   };
   int ret = MDFNSS_StateAction(sm, load, StateRegs, "CPU");

   ret &= GTE_StateAction(sm, load, data_only);

   if(load)
   {

   }

   return(ret);
}

#ifdef __cplusplus
extern "C" {
#endif

void CPU_AssertIRQ(int which, bool asserted)
{
   assert(which >= 0 && which <= 5);

   CP0.CAUSE &= ~(1 << (10 + which));

   if(asserted)
      CP0.CAUSE |= 1 << (10 + which);

   RecalcIPCache();
}

void CPU_SetHalt(bool status)
{
   Halted = status;
   RecalcIPCache();
}

#ifdef __cplusplus
}
#endif

void PS_CPU::SetBIU(uint32_t val)
{
   unsigned i;
   const uint32_t old_BIU = BIU;

   BIU = val & ~(0x440);

   if((BIU ^ old_BIU) & 0x800)
   {
      if(BIU & 0x800)	// ICache enabled
      {
         for(i = 0; i < 1024; i++)
            ICache[i].TV &= ~0x1;
      }
      else			// ICache disabled
      {
         for(i = 0; i < 1024; i++)
            ICache[i].TV |= 0x1;
      }
   }

   PSX_DBG(PSX_DBG_SPARSE, "[CPU] Set BIU=0x%08x\n", BIU);
}

uint32_t PS_CPU::GetBIU(void)
{
   return BIU;
}

static const uint32_t addr_mask[8] = { 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
				     0x7FFFFFFF, 0x1FFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF };

template<typename T>
INLINE T PS_CPU::ReadMemory(int32_t &timestamp, uint32_t address, bool DS24, bool LWC_timing)
{
   T ret;

   //WriteAbsorb >>= WriteAbsorbMonkey * 8;
   //WriteAbsorbCount -= WriteAbsorbMonkey;
   //WriteAbsorbMonkey = WriteAbsorbCount;

   ReadAbsorb[ReadAbsorbWhich] = 0;
   ReadAbsorbWhich = 0;

   address &= addr_mask[address >> 29];

   if(address >= 0x1F800000 && address <= 0x1F8003FF)
   {
      LDAbsorb = 0;

      if(DS24)
         return ScratchRAM.ReadU24(address & 0x3FF);
      else
         return ScratchRAM.Read<T>(address & 0x3FF);
   }

   timestamp += (ReadFudge >> 4) & 2;

   //assert(!(CP0.SR & 0x10000));

   int32_t lts = timestamp;

   if(sizeof(T) == 1)
      ret = PSX_MemRead8(lts, address);
   else if(sizeof(T) == 2)
      ret = PSX_MemRead16(lts, address);
   else
   {
      if(DS24)
         ret = PSX_MemRead24(lts, address) & 0xFFFFFF;
      else
         ret = PSX_MemRead32(lts, address);
   }

   if(LWC_timing)
      lts += 1;
   else
      lts += 2;

   LDAbsorb = (lts - timestamp);
   timestamp = lts;

   return(ret);
}

template<typename T>
INLINE void PS_CPU::WriteMemory(int32_t &timestamp, uint32_t address, uint32_t value, bool DS24)
{
   if(MDFN_LIKELY(!(CP0.SR & 0x10000)))
   {
      address &= addr_mask[address >> 29];

      if(address >= 0x1F800000 && address <= 0x1F8003FF)
      {
         if(DS24)
            ScratchRAM.WriteU24(address & 0x3FF, value);
         else
            ScratchRAM.Write<T>(address & 0x3FF, value);

         return;
      }

      //if(WriteAbsorbCount == 4)
      //{
      // WriteAbsorb >>= 8;
      // WriteAbsorbCount--;
      //
      // if(WriteAbsorbMonkey)
      //  WriteAbsorbMonkey--;
      //}
      //timestamp += 3;
      //WriteAbsorb |= (3U << (WriteAbsorbCount * 8));
      //WriteAbsorbCount++;

      if(sizeof(T) == 1)
         PSX_MemWrite8(timestamp, address, value);
      else if(sizeof(T) == 2)
         PSX_MemWrite16(timestamp, address, value);
      else
      {
         if(DS24)
            PSX_MemWrite24(timestamp, address, value);
         else
            PSX_MemWrite32(timestamp, address, value);
      }
   }
   else
   {
      if(BIU & 0x800)	// Instruction cache is enabled/active
      {
         if(BIU & 0x4)	// TAG test mode.
         {
            // TODO: Respect written value.
            __ICache *ICI = &ICache[((address & 0xFF0) >> 2)];
            const uint8_t valid_bits = 0x00;

            ICI[0].TV = ((valid_bits & 0x01) ? 0x00 : 0x02) | ((BIU & 0x800) ? 0x0 : 0x1);
            ICI[1].TV = ((valid_bits & 0x02) ? 0x00 : 0x02) | ((BIU & 0x800) ? 0x0 : 0x1);
            ICI[2].TV = ((valid_bits & 0x04) ? 0x00 : 0x02) | ((BIU & 0x800) ? 0x0 : 0x1);
            ICI[3].TV = ((valid_bits & 0x08) ? 0x00 : 0x02) | ((BIU & 0x800) ? 0x0 : 0x1);
         }
         else if(!(BIU & 0x1))
         {
            ICache[(address & 0xFFC) >> 2].Data = value << ((address & 0x3) * 8);
         }
      }

      if((BIU & 0x081) == 0x080)	// Writes to the scratchpad(TODO test)
      {
         if(DS24)
            ScratchRAM.WriteU24(address & 0x3FF, value);
         else
            ScratchRAM.Write<T>(address & 0x3FF, value);
      }
      //printf("IsC WRITE%d 0x%08x 0x%08x -- CP0.SR=0x%08x\n", (int)sizeof(T), address, value, CP0.SR);
   }
}

uint32_t PS_CPU::Exception(uint32_t code, uint32_t PC, const uint32_t NPM)
{
   const bool InBDSlot = !(NPM & 0x3);
   uint32_t handler = 0x80000080;

   assert(code < 16);

   if(code != EXCEPTION_INT && code != EXCEPTION_BP && code != EXCEPTION_SYSCALL)
   {
      PSX_DBG(PSX_DBG_WARNING, "Exception: %08x @ PC=0x%08x(IBDS=%d) -- IPCache=0x%02x -- IPEND=0x%02x -- SR=0x%08x ; IRQC_Status=0x%04x -- IRQC_Mask=0x%04x\n", code, PC, InBDSlot, IPCache, (CP0.CAUSE >> 8) & 0xFF, CP0.SR,
            IRQ_GetRegister(IRQ_GSREG_STATUS, NULL, 0), IRQ_GetRegister(IRQ_GSREG_MASK, NULL, 0));
   }

   if(CP0.SR & (1 << 22))	// BEV
      handler = 0xBFC00180;

   CP0.EPC = PC;
   if(InBDSlot)
      CP0.EPC -= 4;

#ifdef HAVE_DEBUG
   if(ADDBT)
      ADDBT(PC, handler, true);
#endif

   // "Push" IEc and KUc(so that the new IEc and KUc are 0)
   CP0.SR = (CP0.SR & ~0x3F) | ((CP0.SR << 2) & 0x3F);

   // Setup cause register
   CP0.CAUSE &= 0x0000FF00;
   CP0.CAUSE |= code << 2;

   // If EPC was adjusted -= 4 because we were in a branch delay slot, set the bit.
   if(InBDSlot)
      CP0.CAUSE |= 0x80000000;

   RecalcIPCache();

   return(handler);
}

#define BACKING_TO_ACTIVE			\
	PC = BACKED_PC;				\
	new_PC = BACKED_new_PC;			\
	new_PC_mask = BACKED_new_PC_mask;	\
	LDWhich = BACKED_LDWhich;		\
	LDValue = BACKED_LDValue;

#define ACTIVE_TO_BACKING			\
	BACKED_PC = PC;				\
	BACKED_new_PC = new_PC;			\
	BACKED_new_PC_mask = new_PC_mask;	\
	BACKED_LDWhich = LDWhich;		\
	BACKED_LDValue = LDValue;

#define GPR_DEPRES_BEGIN { uint8_t back = ReadAbsorb[0];
#define GPR_DEP(n) { unsigned tn = (n); ReadAbsorb[tn] = 0; }
#define GPR_RES(n) { unsigned tn = (n); ReadAbsorb[tn] = 0; }
#define GPR_DEPRES_END ReadAbsorb[0] = back; }

int32_t PS_CPU::RunReal(int32_t timestamp_in)
{
   register int32_t timestamp = timestamp_in;

   register uint32_t PC;
   register uint32_t new_PC;
   register uint32_t new_PC_mask;
   register uint32_t LDWhich;
   register uint32_t LDValue;

   //printf("%d %d\n", gte_ts_done, muldiv_ts_done);

   gte_ts_done += timestamp;
   muldiv_ts_done += timestamp;

   BACKING_TO_ACTIVE;

   do
   {
      //printf("Running: %d %d\n", timestamp, next_event_ts);
      while(MDFN_LIKELY(timestamp < next_event_ts))
      {
         uint32_t instr;
         uint32_t opf;

         // Zero must be zero...until the Master Plan is enacted.
         GPR[0] = 0;

#ifdef HAVE_DEBUG
         if(CPUHook)
         {
            ACTIVE_TO_BACKING;

            // For save states in step mode.
            gte_ts_done -= timestamp;
            muldiv_ts_done -= timestamp;

            CPUHook(timestamp, PC);

            // For save states in step mode.
            gte_ts_done += timestamp;
            muldiv_ts_done += timestamp;

            BACKING_TO_ACTIVE;
         }
#endif

         instr = ICache[(PC & 0xFFC) >> 2].Data;

         if(ICache[(PC & 0xFFC) >> 2].TV != PC)
         {
            //WriteAbsorb = 0;
            //WriteAbsorbCount = 0;
            //WriteAbsorbMonkey = 0;
            ReadAbsorb[ReadAbsorbWhich] = 0;
            ReadAbsorbWhich = 0;

            // FIXME: Handle executing out of scratchpad.
            if(PC >= 0xA0000000 || !(BIU & 0x800))
            {
               instr = LoadU32_LE((uint32_t *)&FastMap[PC >> FAST_MAP_SHIFT][PC]);
               timestamp += 4;	// Approximate best-case cache-disabled time, per PS1 tests(executing out of 0xA0000000+); it can be 5 in *some* sequences of code(like a lot of sequential "nop"s, probably other simple instructions too).
            }
            else
            {
               __ICache *ICI = &ICache[((PC & 0xFF0) >> 2)];
               const uint32_t *FMP = (uint32_t *)&FastMap[(PC &~ 0xF) >> FAST_MAP_SHIFT][PC &~ 0xF];

               // | 0x2 to simulate (in)validity bits.
               ICI[0x00].TV = (PC &~ 0xF) | 0x00 | 0x2;
               ICI[0x01].TV = (PC &~ 0xF) | 0x04 | 0x2;
               ICI[0x02].TV = (PC &~ 0xF) | 0x08 | 0x2;
               ICI[0x03].TV = (PC &~ 0xF) | 0x0C | 0x2;

               timestamp += 3;

               switch(PC & 0xC)
               {
                  case 0x0:
                     timestamp++;
                     ICI[0x00].TV &= ~0x2;
                     ICI[0x00].Data = LoadU32_LE(&FMP[0]);
                  case 0x4:
                     timestamp++;
                     ICI[0x01].TV &= ~0x2;
                     ICI[0x01].Data = LoadU32_LE(&FMP[1]);
                  case 0x8:
                     timestamp++;
                     ICI[0x02].TV &= ~0x2;
                     ICI[0x02].Data = LoadU32_LE(&FMP[2]);
                  case 0xC:
                     timestamp++;
                     ICI[0x03].TV &= ~0x2;
                     ICI[0x03].Data = LoadU32_LE(&FMP[3]);
                     break;
               }
               instr = ICache[(PC & 0xFFC) >> 2].Data;
            }
         }

         //printf("PC=%08x, SP=%08x - op=0x%02x - funct=0x%02x - instr=0x%08x\n", PC, GPR[29], instr >> 26, instr & 0x3F, instr);
         //for(int i = 0; i < 32; i++)
         // printf("%02x : %08x\n", i, GPR[i]);
         //printf("\n");

         opf = instr & 0x3F;

         if(instr & (0x3F << 26))
            opf = 0x40 | (instr >> 26);

         opf |= IPCache;

#if 0
         {
            uint32_t tmp = (ReadAbsorb[ReadAbsorbWhich] + 0x7FFFFFFF) >> 31;
            ReadAbsorb[ReadAbsorbWhich] -= tmp;
            timestamp = timestamp + 1 - tmp;
         }
#else
         if(ReadAbsorb[ReadAbsorbWhich])
            ReadAbsorb[ReadAbsorbWhich]--;
         //else if((uint8)WriteAbsorb)
         //{
         // WriteAbsorb--;
         // if(!WriteAbsorb)
         // {
         //  WriteAbsorbCount--;
         //  if(WriteAbsorbMonkey)
         //   WriteAbsorbMonkey--;
         //  WriteAbsorb >>= 8;
         // }
         //}
         else
            timestamp++;
#endif

#define DO_LDS() { GPR[LDWhich] = LDValue; ReadAbsorb[LDWhich] = LDAbsorb; ReadFudge = LDWhich; ReadAbsorbWhich |= LDWhich & 0x1F; LDWhich = 0x20; }
#define BEGIN_OPF(name, arg_op, arg_funct) { op_##name: /*assert( ((arg_op) ? (0x40 | (arg_op)) : (arg_funct)) == opf); */
#define END_OPF goto OpDone; }

#ifdef HAVE_DEBUG
#define DO_BRANCH(offset, mask)			\
         {						\
            PC = (PC & new_PC_mask) + new_PC;		\
            new_PC = (offset);				\
            new_PC_mask = (mask) & ~3;			\
            /* Lower bits of new_PC_mask being clear signifies being in a branch delay slot. (overloaded behavior for performance) */	\
            \
            if(ADDBT)                 	\
            {						\
               ADDBT(PC, (PC & new_PC_mask) + new_PC, false);	\
            }						\
            goto SkipNPCStuff;				\
         }
#else
#define DO_BRANCH(offset, mask)			\
         {						\
            PC = (PC & new_PC_mask) + new_PC;		\
            new_PC = (offset);				\
            new_PC_mask = (mask) & ~3;			\
            /* Lower bits of new_PC_mask being clear signifies being in a branch delay slot. (overloaded behavior for performance) */	\
            goto SkipNPCStuff;				\
         }
#endif

#define ITYPE uint32_t rs MDFN_NOWARN_UNUSED = (instr >> 21) & 0x1F; uint32_t rt MDFN_NOWARN_UNUSED = (instr >> 16) & 0x1F; int32_t immediate = (int16)(instr & 0xFFFF); /*printf(" rs=%02x(%08x), rt=%02x(%08x), immediate=(%08x) ", rs, GPR[rs], rt, GPR[rt], immediate);*/
#define ITYPE_ZE uint32_t rs MDFN_NOWARN_UNUSED = (instr >> 21) & 0x1F; uint32_t rt MDFN_NOWARN_UNUSED = (instr >> 16) & 0x1F; uint32_t immediate = instr & 0xFFFF; /*printf(" rs=%02x(%08x), rt=%02x(%08x), immediate=(%08x) ", rs, GPR[rs], rt, GPR[rt], immediate);*/
#define JTYPE uint32_t target = instr & ((1 << 26) - 1); /*printf(" target=(%08x) ", target);*/
#define RTYPE uint32_t rs MDFN_NOWARN_UNUSED = (instr >> 21) & 0x1F; uint32_t rt MDFN_NOWARN_UNUSED = (instr >> 16) & 0x1F; uint32_t rd MDFN_NOWARN_UNUSED = (instr >> 11) & 0x1F; uint32_t shamt MDFN_NOWARN_UNUSED = (instr >> 6) & 0x1F; /*printf(" rs=%02x(%08x), rt=%02x(%08x), rd=%02x(%08x) ", rs, GPR[rs], rt, GPR[rt], rd, GPR[rd]);*/

#if !defined(__GNUC__) || defined(NO_COMPUTED_GOTO)
#include "cpu_bigswitch.c"
#else
#include "cpu_computedgoto.c"
#endif

OpDone: ;

        PC = (PC & new_PC_mask) + new_PC;
        new_PC_mask = ~0U;
        new_PC = 4;

SkipNPCStuff:	;

               //printf("\n");
      }
   } while(MDFN_LIKELY(PSX_EventHandler(timestamp)));

   if(gte_ts_done > 0)
      gte_ts_done -= timestamp;

   if(muldiv_ts_done > 0)
      muldiv_ts_done -= timestamp;

   ACTIVE_TO_BACKING;

   return(timestamp);
}

int32_t PS_CPU::Run(int32_t timestamp_in, const bool ILHMode)
{
#ifdef HAVE_DEBUG
   if(CPUHook || ADDBT)
      return(RunReal(timestamp_in));
#endif
   return(RunReal(timestamp_in));
}

void PS_CPU::SetCPUHook(void (*cpuh)(const int32_t timestamp, uint32_t pc), void (*addbt)(uint32_t from, uint32_t to, bool exception))
{
   ADDBT = addbt;
   CPUHook = cpuh;
}

uint32_t PS_CPU::GetRegister(unsigned int which, char *special, const uint32_t special_len)
{
   if(which >= GSREG_GPR && which < (GSREG_GPR + 32))
      return GPR[which];

   switch(which)
   {
      case GSREG_PC:
         return BACKED_PC;
      case GSREG_PC_NEXT:
         return BACKED_new_PC;
      case GSREG_IN_BD_SLOT:
         return !(BACKED_new_PC_mask & 3);
      case GSREG_LO:
         return LO;
      case GSREG_HI:
         return HI;
      case GSREG_SR:
         return CP0.SR;
      case GSREG_CAUSE:
         return CP0.CAUSE;
      case GSREG_EPC:
         return CP0.EPC;
   }

   return 0;
}

void PS_CPU::SetRegister(unsigned int which, uint32_t value)
{
   if(which >= GSREG_GPR && which < (GSREG_GPR + 32))
   {
      if(which != (GSREG_GPR + 0))
         GPR[which] = value;
   }
   else switch(which)
   {
      case GSREG_PC:
         BACKED_PC = value & ~0x3;	// Remove masking if we ever add proper misaligned PC exception
         break;

      case GSREG_LO:
         LO = value;
         break;

      case GSREG_HI:
         HI = value;
         break;

      case GSREG_SR:
         CP0.SR = value;		// TODO: mask
         break;

      case GSREG_CAUSE:
         CP0.CAUSE = value;
         break;

      case GSREG_EPC:
         CP0.EPC = value & ~0x3U;
         break;
   }
}

bool PS_CPU::PeekCheckICache(uint32_t PC, uint32_t *iw)
{
   if(ICache[(PC & 0xFFC) >> 2].TV == PC)
   {
      *iw = ICache[(PC & 0xFFC) >> 2].Data;
      return(true);
   }

   return(false);
}


uint8_t PS_CPU::PeekMem8(uint32_t address)
{
   address &= addr_mask[address >> 29];

   if(address >= 0x1F800000 && address <= 0x1F8003FF)
      return ScratchRAM.Read<uint8>(address & 0x3FF);

   return PSX_MemPeek8(address);
}

uint16_t PS_CPU::PeekMem16(uint32_t address)
{
   address &= addr_mask[address >> 29];

   if(address >= 0x1F800000 && address <= 0x1F8003FF)
      return ScratchRAM.Read<uint16>(address & 0x3FF);

   return PSX_MemPeek16(address);
}

uint32_t PS_CPU::PeekMem32(uint32_t address)
{
   address &= addr_mask[address >> 29];

   if(address >= 0x1F800000 && address <= 0x1F8003FF)
      return ScratchRAM.Read<uint32>(address & 0x3FF);

   return PSX_MemPeek32(address);
}


#undef BEGIN_OPF
#undef END_OPF
#undef MK_OPF

#define MK_OPF(op, funct)	((op) ? (0x40 | (op)) : (funct))
#define BEGIN_OPF(op, funct) case MK_OPF(op, funct): {
#define END_OPF } break;

// FIXME: should we breakpoint on an illegal address?  And with LWC2/SWC2 if CP2 isn't enabled?
void PS_CPU::CheckBreakpoints(void (*callback)(bool write, uint32_t address, unsigned int len), uint32_t instr)
{
 uint32_t opf = instr & 0x3F;

 if(instr & (0x3F << 26))
  opf = 0x40 | (instr >> 26);


 switch(opf)
 {
  default:
	break;

    //
    // LB - Load Byte
    //
    BEGIN_OPF(0x20, 0);
	ITYPE;
	uint32_t address = GPR[rs] + immediate;

        callback(false, address, 1);
    END_OPF;

    //
    // LBU - Load Byte Unsigned
    //
    BEGIN_OPF(0x24, 0);
        ITYPE;
        uint32_t address = GPR[rs] + immediate;

        callback(false, address, 1);
    END_OPF;

    //
    // LH - Load Halfword
    //
    BEGIN_OPF(0x21, 0);
        ITYPE;
        uint32_t address = GPR[rs] + immediate;

        callback(false, address, 2);
    END_OPF;

    //
    // LHU - Load Halfword Unsigned
    //
    BEGIN_OPF(0x25, 0);
        ITYPE;
        uint32_t address = GPR[rs] + immediate;

        callback(false, address, 2);
    END_OPF;


    //
    // LW - Load Word
    //
    BEGIN_OPF(0x23, 0);
        ITYPE;
        uint32_t address = GPR[rs] + immediate;

        callback(false, address, 4);
    END_OPF;

    //
    // SB - Store Byte
    //
    BEGIN_OPF(0x28, 0);
	ITYPE;
	uint32_t address = GPR[rs] + immediate;

        callback(true, address, 1);
    END_OPF;

    // 
    // SH - Store Halfword
    //
    BEGIN_OPF(0x29, 0);
        ITYPE;
        uint32_t address = GPR[rs] + immediate;

        callback(true, address, 2);
    END_OPF;

    // 
    // SW - Store Word
    //
    BEGIN_OPF(0x2B, 0);
        ITYPE;
        uint32_t address = GPR[rs] + immediate;

        callback(true, address, 4);
    END_OPF;

    //
    // LWL - Load Word Left
    //
    BEGIN_OPF(0x22, 0);
	ITYPE;
	uint32_t address = GPR[rs] + immediate;

	do
	{
         callback(false, address, 1);
	} while((address--) & 0x3);

    END_OPF;

    //
    // SWL - Store Word Left
    //
    BEGIN_OPF(0x2A, 0);
        ITYPE;
        uint32_t address = GPR[rs] + immediate;

        do
        {
	 callback(true, address, 1);
        } while((address--) & 0x3);

    END_OPF;

    //
    // LWR - Load Word Right
    //
    BEGIN_OPF(0x26, 0);
        ITYPE;
        uint32_t address = GPR[rs] + immediate;

        do
        {
	 callback(false, address, 1);
        } while((++address) & 0x3);

    END_OPF;

    //
    // SWR - Store Word Right
    //
    BEGIN_OPF(0x2E, 0);
        ITYPE;
        uint32_t address = GPR[rs] + immediate;

        do
        {
	 callback(true, address, 1);
        } while((++address) & 0x3);

    END_OPF;

    //
    // LWC2
    //
    BEGIN_OPF(0x32, 0);
        ITYPE;
        uint32_t address = GPR[rs] + immediate;

	callback(false, address, 4);
    END_OPF;

    //
    // SWC2
    //
    BEGIN_OPF(0x3A, 0);
        ITYPE;
        uint32_t address = GPR[rs] + immediate;

	callback(true, address, 4);
    END_OPF;

 }
}


}

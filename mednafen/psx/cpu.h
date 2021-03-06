#ifndef __MDFN_PSX_CPU_H
#define __MDFN_PSX_CPU_H

/*
 Load delay notes:

	// Takes 1 less
	".set noreorder\n\t"
	".set nomacro\n\t"
	"lw %0, 0(%2)\n\t"
	"nop\n\t"
	"nop\n\t"
	"or %0, %1, %1\n\t"

	// cycle than this:
	".set noreorder\n\t"
	".set nomacro\n\t"
	"lw %0, 0(%2)\n\t"
	"nop\n\t"
	"or %0, %1, %1\n\t"
	"nop\n\t"


	// Both of these
	".set noreorder\n\t"
	".set nomacro\n\t"
	"lw %0, 0(%2)\n\t"
	"nop\n\t"
	"nop\n\t"
	"or %1, %0, %0\n\t"

	// take same...(which is kind of odd).
	".set noreorder\n\t"
	".set nomacro\n\t"
	"lw %0, 0(%2)\n\t"
	"nop\n\t"
	"or %1, %0, %0\n\t"
	"nop\n\t"
*/

#include "gte.h"

 // FAST_MAP_* enums are in BYTES(8-bit), not in 32-bit units("words" in MIPS context), but the sizes
 // will always be multiples of 4.
#define FAST_MAP_SHIFT 16
#define FAST_MAP_PSIZE (1 << FAST_MAP_SHIFT)

enum
{
   EXCEPTION_INT = 0,
   EXCEPTION_MOD = 1,
   EXCEPTION_TLBL = 2,
   EXCEPTION_TLBS = 3,
   EXCEPTION_ADEL = 4, // Address error on load
   EXCEPTION_ADES = 5, // Address error on store
   EXCEPTION_IBE = 6, // Instruction bus error
   EXCEPTION_DBE = 7, // Data bus error
   EXCEPTION_SYSCALL = 8, // System call
   EXCEPTION_BP = 9, // Breakpoint
   EXCEPTION_RI = 10, // Reserved instruction
   EXCEPTION_COPU = 11,  // Coprocessor unusable
   EXCEPTION_OV = 12	// Arithmetic overflow
};

enum
{
   CP0REG_BPC = 3,		// PC breakpoint address.
   CP0REG_BDA = 5,		// Data load/store breakpoint address.
   CP0REG_TAR = 6,		// Target address(???)
   CP0REG_DCIC = 7,		// Cache control
   CP0REG_BDAM = 9,		// Data load/store address mask.
   CP0REG_BPCM = 11,		// PC breakpoint address mask.
   CP0REG_SR = 12,
   CP0REG_CAUSE = 13,
   CP0REG_EPC = 14,
   CP0REG_PRID = 15,		// Product ID
   CP0REG_ERREG = 16
};

enum
{
   GSREG_GPR = 0,
   GSREG_PC = 32,
   GSREG_PC_NEXT,
   GSREG_IN_BD_SLOT,
   GSREG_LO,
   GSREG_HI,
   GSREG_SR,
   GSREG_CAUSE,
   GSREG_EPC,
};

#define PS_CPU_EMULATE_ICACHE 1

class PS_CPU
{
 public:

 PS_CPU();
 ~PS_CPU();


 void SetFastMap(void *region_mem, uint32_t region_address, uint32_t region_size);

 INLINE void SetEventNT(const int32_t next_event_ts_arg)
 {
  next_event_ts = next_event_ts_arg;
 }

 int32_t Run(int32_t timestamp_in, const bool ILHMode);

 void Power(void);



 // TODO eventually: factor BIU address decoding directly in the CPU core somehow without hurting speed.
 void SetBIU(uint32_t val);
 uint32_t GetBIU(void);

 int StateAction(StateMem *sm, int load, int data_only);

 private:

 uint32_t GPR[32];
 uint32_t GPR_dummy;	// Used in load delay simulation(indexing past the end of GPR)
 uint32_t LO;
 uint32_t HI;


 uint32_t BACKED_PC;
 uint32_t BACKED_new_PC;
 uint32_t BACKED_new_PC_mask;

 uint32_t BACKED_LDWhich;
 uint32_t BACKED_LDValue;
 uint32_t LDAbsorb;

 int32_t next_event_ts;
 int32_t gte_ts_done;
 int32_t muldiv_ts_done;

 uint32_t BIU;

 struct __ICache
 {
  uint32_t TV;
  uint32_t Data;
 };

 union
 {
  __ICache ICache[1024];
  uint32 ICache_Bulk[2048];
 };

 uint8_t ReadAbsorb[0x20];
 uint8_t ReadAbsorbDummy;
 uint8_t ReadAbsorbWhich;
 uint8_t ReadFudge;

 MultiAccessSizeMem<1024, uint32, false> ScratchRAM;

 uint8_t *FastMap[1 << (32 - FAST_MAP_SHIFT)];
 uint8_t DummyPage[FAST_MAP_PSIZE];


 uint32_t Exception(uint32_t code, uint32_t PC, const uint32_t NPM) MDFN_WARN_UNUSED_RESULT;

 template<typename T> T PeekMemory(uint32_t address) MDFN_COLD;
 template<typename T> T ReadMemory(int32_t &timestamp, uint32_t address, bool DS24 = false, bool LWC_timing = false);
 template<typename T> void WriteMemory(int32_t &timestamp, uint32_t address, uint32_t value, bool DS24 = false);


 //
 // Mednafen debugger stuff follows:
 //
 public:
 void SetCPUHook(void (*cpuh)(const int32_t timestamp, uint32_t pc), void (*addbt)(uint32_t from, uint32_t to, bool exception));
 void CheckBreakpoints(void (*callback)(bool write, uint32_t address, unsigned int len), uint32_t instr);


 uint32_t GetRegister(unsigned int which, char *special, const uint32_t special_len);
 void SetRegister(unsigned int which, uint32_t value);
 bool PeekCheckICache(uint32_t PC, uint32_t *iw);
 uint8_t PeekMem8(uint32_t A);
 uint16_t PeekMem16(uint32_t A);
 uint32_t PeekMem32(uint32_t A);
 private:
 void (*CPUHook)(const int32_t timestamp, uint32_t pc);
 void (*ADDBT)(uint32_t from, uint32_t to, bool exception);
};


#ifdef __cplusplus
extern "C" {
#endif

// which ranges 0-5, inclusive
void CPU_AssertIRQ(int which, bool asserted);

void SetHalt(bool status);

#ifdef __cplusplus
}
#endif

#endif

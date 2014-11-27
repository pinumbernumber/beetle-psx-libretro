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

// TODO: async command counter and async command phase?
/*

 TODO:
	Implement missing commands.

	SPU CD-DA and CD-XA streaming semantics.
*/

/*
 After eject(doesn't appear to occur when drive is in STOP state):
	* Does not appear to occur in STOP state.
	* Does not appear to occur in PAUSE state.
	* DOES appear to occur in STANDBY state. (TODO: retest)

% Result 0: 16
% Result 1: 08
% IRQ Result: e5
% 19 e0

 Command abortion tests(NOP tested):
	Does not appear to occur when in STOP or PAUSE states(STOP or PAUSE command just executed).

	DOES occur after a ReadTOC completes, if ReadTOC is not followed by a STOP or PAUSE.  Odd.
*/

#include "psx.h"
#include "cdc.h"
#include "spu.h"

static SimpleFIFOU8 *DMABuffer;

static CDIF *Cur_CDIF;
static bool DiscChanged;
static int32 DiscStartupDelay;

static CD_Audio_Buffer AudioBuffer;

static uint8 Pending_DecodeVolume[2][2], DecodeVolume[2][2];		// [data_source][output_port]

static int16 ADPCM_ResampBuf[2][32 * 2];
static uint8 ADPCM_ResampCurPos;
static uint8 ADPCM_ResampCurPhase;

static uint8 RegSelector;
static uint8 ArgsBuf[16];
static uint8 ArgsWP;		// 5-bit(0 ... 31)
static uint8 ArgsRP;		// 5-bit(0 ... 31)

static uint8 ArgsReceiveLatch;
static uint8 ArgsReceiveBuf[32];
static uint8 ArgsReceiveIn;

static uint8 ResultsBuffer[16];
static uint8 ResultsIn;	// 5-bit(0 ... 31)
static uint8 ResultsWP;	// Write position, 4 bit(0 ... 15).
static uint8 ResultsRP;	// Read position, 4 bit(0 ... 15).

static uint8 SB[2340];
static uint32 SB_In;

static uint8 SectorPipe[SectorPipe_Count][2352];
static uint8 SectorPipe_Pos;
static uint8 SectorPipe_In;

static uint8 SubQBuf[0xC];
static uint8 SubQBuf_Safe[0xC];
static bool SubQChecksumOK;

static bool HeaderBufValid;
static uint8 HeaderBuf[12];


static uint8 IRQBuffer;
static uint8 IRQOutTestMask;
static int32 CDCReadyReceiveCounter;	// IRQBuffer being non-zero prevents new results and new IRQ from coming in and erasing the current results,
// but apparently at least one CONFOUNDED game is clearing the IRQ state BEFORE reading the results, so we need to have a delay
// between IRQBuffer being cleared to when we allow new results to come in.  (The real thing should be like this too,
// but the mechanism is probably more nuanced and complex and ugly and I like anchovy pizza)

static uint8 FilterFile;
static uint8 FilterChan;


static uint8 PendingCommand;
static int PendingCommandPhase;
static int32 PendingCommandCounter;

static int32 SPUCounter;

static uint8 Mode;

static int DriveStatus;
static int StatusAfterSeek;
static bool Forward;
static bool Backward;
static bool Muted;

static int32 PlayTrackMatch;

static int32 PSRCounter;

static int32 CurSector;

static unsigned AsyncIRQPending;
static uint8 AsyncResultsPending[16];
static uint8 AsyncResultsPendingCount;

static int32 SeekTarget;

static int32_t lastts;

static CDUtility::TOC toc;
static bool IsPSXDisc;
static uint8 DiscID[4];

static int32 CommandLoc;
static bool CommandLoc_Dirty;

static int16 xa_previous[2][2];
static bool xa_cur_set;
static uint8 xa_cur_file;
static uint8 xa_cur_chan;

static uint8 ReportLastF;

struct CDC_CTEntry
{
   uint8 args_min;
   uint8 args_max;
   const char *name;
   int32 (*func)(const int arg_count, const uint8 *args);
   int32 (*func2)(void);
};

static int32 Command_Nop(const int arg_count, const uint8 *args);
static int32 Command_Setloc(const int arg_count, const uint8 *args);
static int32 Command_Play(const int arg_count, const uint8 *args);
static int32 Command_Forward(const int arg_count, const uint8 *args);
static int32 Command_Backward(const int arg_count, const uint8 *args);
static int32 Command_ReadN(const int arg_count, const uint8 *args);
static int32 Command_Standby(const int arg_count, const uint8 *args);
static int32 Command_Standby_Part2(void);
static int32 Command_Stop(const int arg_count, const uint8 *args);
static int32 Command_Stop_Part2(void);
static int32 Command_Pause(const int arg_count, const uint8 *args);
static int32 Command_Pause_Part2(void);
static int32 Command_Reset(const int arg_count, const uint8 *args);
static int32 Command_Mute(const int arg_count, const uint8 *args);
static int32 Command_Demute(const int arg_count, const uint8 *args);
static int32 Command_Setfilter(const int arg_count, const uint8 *args);
static int32 Command_Setmode(const int arg_count, const uint8 *args);
static int32 Command_Getparam(const int arg_count, const uint8 *args);
static int32 Command_GetlocL(const int arg_count, const uint8 *args);
static int32 Command_GetlocP(const int arg_count, const uint8 *args);
static int32 Command_ReadT(const int arg_count, const uint8 *args);
static int32 Command_ReadT_Part2(void);
static int32 Command_GetTN(const int arg_count, const uint8 *args);
static int32 Command_GetTD(const int arg_count, const uint8 *args);
static int32 Command_SeekL(const int arg_count, const uint8 *args);
static int32 Command_SeekP(const int arg_count, const uint8 *args);
static int32 Command_Seek_PartN(void);
static int32 Command_Test(const int arg_count, const uint8 *args);
static int32 Command_ID(const int arg_count, const uint8 *args);
static int32 Command_ID_Part2(void);
static int32 Command_ReadS(const int arg_count, const uint8 *args);
static int32 Command_Init(const int arg_count, const uint8 *args);
static int32 Command_ReadTOC(const int arg_count, const uint8 *args);
static int32 Command_ReadTOC_Part2(void);
static int32 Command_0x1d(const int arg_count, const uint8 *args);

static CDC_CTEntry CDC_Commands[0x20] =
{
   { /* 0x00, */ 0, 0, NULL, NULL, NULL },
   { /* 0x01, */ 0, 0, "Nop", &Command_Nop, NULL },
   { /* 0x02, */ 3, 3, "Setloc", &Command_Setloc, NULL },
   { /* 0x03, */ 0, 1, "Play", &Command_Play, NULL },
   { /* 0x04, */ 0, 0, "Forward", &Command_Forward, NULL },
   { /* 0x05, */ 0, 0, "Backward", &Command_Backward, NULL },
   { /* 0x06, */ 0, 0, "ReadN", &Command_ReadN, NULL },
   { /* 0x07, */ 0, 0, "Standby", &Command_Standby, &Command_Standby_Part2 },
   { /* 0x08, */ 0, 0, "Stop", &Command_Stop, &Command_Stop_Part2 },
   { /* 0x09, */ 0, 0, "Pause", &Command_Pause, &Command_Pause_Part2 },
   { /* 0x0A, */ 0, 0, "Reset", &Command_Reset, NULL },
   { /* 0x0B, */ 0, 0, "Mute", &Command_Mute, NULL },
   { /* 0x0C, */ 0, 0, "Demute", &Command_Demute, NULL },
   { /* 0x0D, */ 2, 2, "Setfilter", &Command_Setfilter, NULL },
   { /* 0x0E, */ 1, 1, "Setmode", &Command_Setmode, NULL },
   { /* 0x0F, */ 0, 0, "Getparam", &Command_Getparam, NULL },
   { /* 0x10, */ 0, 0, "GetlocL", &Command_GetlocL, NULL },
   { /* 0x11, */ 0, 0, "GetlocP", &Command_GetlocP, NULL },
   { /* 0x12, */ 1, 1, "ReadT", &Command_ReadT, &Command_ReadT_Part2 },
   { /* 0x13, */ 0, 0, "GetTN", &Command_GetTN, NULL },
   { /* 0x14, */ 1, 1, "GetTD", &Command_GetTD, NULL },
   { /* 0x15, */ 0, 0, "SeekL", &Command_SeekL, &Command_Seek_PartN },
   { /* 0x16, */ 0, 0, "SeekP", &Command_SeekP, &Command_Seek_PartN },

   { /* 0x17, */ 0, 0, NULL, NULL, NULL },
   { /* 0x18, */ 0, 0, NULL, NULL, NULL },

   { /* 0x19, */ 1, 1/* ??? */, "Test", &Command_Test, NULL },
   { /* 0x1A, */ 0, 0, "ID", &Command_ID, &Command_ID_Part2 },
   { /* 0x1B, */ 0, 0, "ReadS", &Command_ReadS, NULL },
   { /* 0x1C, */ 0, 0, "Init", &Command_Init, NULL },
   { /* 0x1D, */ 2, 2, "Unknown 0x1D", &Command_0x1d, NULL },
   { /* 0x1E, */ 0, 0, "ReadTOC", &Command_ReadTOC, &Command_ReadTOC_Part2 },
   { /* 0x1F, */ 0, 0, NULL, NULL, NULL },
};

using namespace CDUtility;

#define CDC_ClearAIP() \
 AsyncResultsPendingCount = 0; \
 AsyncIRQPending = 0

void CDC_New()
{
   SimpleFIFO_New(DMABuffer, SimpleFIFOU8, uint8, 4096);
   IsPSXDisc = false;
   Cur_CDIF = NULL;

   DriveStatus = DS_STOPPED;
   PendingCommandPhase = 0;
}

void CDC_Free()
{
   SimpleFIFO_Free(DMABuffer);
}

void CDC_SetDisc(bool tray_open, CDIF *cdif, const char *disc_id)
{
   if(tray_open)
      cdif = NULL;

   Cur_CDIF = cdif;
   IsPSXDisc = false;
   memset(DiscID, 0, sizeof(DiscID));

   if(!Cur_CDIF)
   {
      PSRCounter = 0;

      if((DriveStatus != DS_PAUSED && DriveStatus != DS_STOPPED) || PendingCommandPhase >= 2)
      {
         PendingCommand = 0x00;
         PendingCommandCounter = 0;
         PendingCommandPhase = 0;
      }

      HeaderBufValid = false;
      DriveStatus = DS_STOPPED;
      CDC_ClearAIP();
      SectorPipe_Pos = SectorPipe_In = 0;
   }
   else
   {
      HeaderBufValid = false;
      DiscStartupDelay = (int64)1000 * 33868800 / 1000;
      DiscChanged = true;

      CDIF_ReadTOC(Cur_CDIF, &toc);

      if(disc_id)
      {
         strncpy((char *)DiscID, disc_id, 4);
         IsPSXDisc = true;
      }
   }
}

int32 CDC_CalcNextEvent(void)
{
   int32 next_event = SPUCounter;

   if(PSRCounter > 0 && next_event > PSRCounter)
      next_event = PSRCounter;

   if(PendingCommandCounter > 0 && next_event > PendingCommandCounter)
      next_event = PendingCommandCounter;

   if(!IRQBuffer)
   {
      if(CDCReadyReceiveCounter > 0 && next_event > CDCReadyReceiveCounter)
         next_event = CDCReadyReceiveCounter;
   }

   if(DiscStartupDelay > 0 && next_event > DiscStartupDelay)
      next_event = DiscStartupDelay;

   //fprintf(stderr, "%d %d %d %d --- %d\n", PSRCounter, PendingCommandCounter, CDCReadyReceiveCounter, DiscStartupDelay, next_event);

   return(next_event);
}

#define RecalcIRQ() IRQ_Assert(IRQ_CD, (bool)(IRQBuffer & (IRQOutTestMask & 0x1F)))

#ifdef NDEBUG
#define WriteIRQ(V) \
 /* PSX_WARNING("[CDC] ***IRQTHINGY: 0x%02x -- %u", V, doom_ts); */ \
 CDCReadyReceiveCounter = 2000; /* 1024; */ \
 IRQBuffer = V; \
 RecalcIRQ()
#else
#define WriteIRQ(V) \
 assert(CDCReadyReceiveCounter <= 0); \
 assert(!IRQBuffer); \
 /* PSX_WARNING("[CDC] ***IRQTHINGY: 0x%02x -- %u", V, doom_ts); */ \
 CDCReadyReceiveCounter = 2000; /* 1024; */ \
 IRQBuffer = V; \
 RecalcIRQ()
#endif

#define BeginResults() \
 /* TODO: test semantics on real thing. */ \
 ResultsIn = 0; \
 ResultsWP = 0; \
 ResultsRP = 0

#define WriteResult(V) \
 ResultsBuffer[ResultsWP] = V; \
 ResultsWP = (ResultsWP + 1) & 0xF; \
 ResultsIn = (ResultsIn + 1) & 0x1F

#define ReadResult(result) \
 result = ResultsBuffer[ResultsRP]; \
 ResultsRP = (ResultsRP + 1) & 0xF; \
 ResultsIn = (ResultsIn - 1) & 0x1F; \

static void CDC_ClearAudioBuffers(void)
{
   memset(&AudioBuffer, 0, sizeof(AudioBuffer));
   memset(xa_previous, 0, sizeof(xa_previous));

   xa_cur_set = false;
   xa_cur_file = 0;
   xa_cur_chan = 0;

   memset(ADPCM_ResampBuf, 0, sizeof(ADPCM_ResampBuf));
   ADPCM_ResampCurPhase = 0;
   ADPCM_ResampCurPos = 0;
}

void CDC_SoftReset(void)
{
   //PSX_WARNING("[CDC] Soft Reset");
   CDC_ClearAudioBuffers();

   // Not sure about initial volume state
   Pending_DecodeVolume[0][0] = 0x80;
   Pending_DecodeVolume[0][1] = 0x00;
   Pending_DecodeVolume[1][0] = 0x00;
   Pending_DecodeVolume[1][1] = 0x80;
   memcpy(DecodeVolume, Pending_DecodeVolume, sizeof(DecodeVolume));

   RegSelector = 0;
   memset(ArgsBuf, 0, sizeof(ArgsBuf));
   ArgsWP = ArgsRP = 0;

   memset(ResultsBuffer, 0, sizeof(ResultsBuffer));
   ResultsWP = 0;
   ResultsRP = 0;
   ResultsIn = 0;

   CDCReadyReceiveCounter = 0;

   IRQBuffer = 0;
   IRQOutTestMask = 0;
   RecalcIRQ();

   SimpleFIFO_Flush(DMABuffer);
   SB_In = 0;
   SectorPipe_Pos = SectorPipe_In = 0;

   memset(SubQBuf, 0, sizeof(SubQBuf));
   memset(SubQBuf_Safe, 0, sizeof(SubQBuf_Safe));
   SubQChecksumOK = false;

   memset(HeaderBuf, 0, sizeof(HeaderBuf));


   FilterFile = 0;
   FilterChan = 0;

   PendingCommand = 0;
   PendingCommandPhase = 0;
   PendingCommandCounter = 0;

   Mode = 0;

   HeaderBufValid = false;
   DriveStatus = DS_STOPPED;
   CDC_ClearAIP();
   StatusAfterSeek = DS_STOPPED;

   Forward = false;
   Backward = false;
   Muted = false;

   PlayTrackMatch = 0;

   PSRCounter = 0;

   CurSector = 0;

   CDC_ClearAIP();

   SeekTarget = 0;

   CommandLoc = 0;
   CommandLoc_Dirty = true;

   DiscChanged = true;
}

void CDC_Power(void)
{
   SPU_Power();

   CDC_SoftReset();

   DiscStartupDelay = 0;

   SPUCounter = SPU_UpdateFromCDC(0);
   lastts = 0;
}

int CDC_StateAction(StateMem *sm, int load, int data_only)
{
 SFORMAT StateRegs[] =
 {
 { &((DiscChanged)), 1, 0x80000000 | 0x08000000, "DiscChanged" },
 { &((DiscStartupDelay)), sizeof((DiscStartupDelay)), 0x80000000 | 0, "DiscStartupDelay" },

 { ((&AudioBuffer.Samples[0][0])), (uint32)(((sizeof(AudioBuffer.Samples) / sizeof(AudioBuffer.Samples[0][0]))) * sizeof(uint16)), 0x20000000 | 0, "&AudioBuffer.Samples[0][0]" },
 { &((AudioBuffer.Size)), sizeof((AudioBuffer.Size)), 0x80000000 | 0, "AudioBuffer.Size" },
 { &((AudioBuffer.Freq)), sizeof((AudioBuffer.Freq)), 0x80000000 | 0, "AudioBuffer.Freq" },
 { &((AudioBuffer.ReadPos)), sizeof((AudioBuffer.ReadPos)), 0x80000000 | 0, "AudioBuffer.ReadPos" },

 { ((&Pending_DecodeVolume[0][0])), (uint32)((2 * 2)), 0 | 0, "&Pending_DecodeVolume[0][0]" },
 { ((&DecodeVolume[0][0])), (uint32)((2 * 2)), 0 | 0, "&DecodeVolume[0][0]" },

 { ((&ADPCM_ResampBuf[0][0])), (uint32)(((sizeof(ADPCM_ResampBuf) / sizeof(ADPCM_ResampBuf[0][0]))) * sizeof(uint16)), 0x20000000 | 0, "&ADPCM_ResampBuf[0][0]" },
 { &((ADPCM_ResampCurPhase)), sizeof((ADPCM_ResampCurPhase)), 0x80000000 | 0, "ADPCM_ResampCurPhase" },
 { &((ADPCM_ResampCurPos)), sizeof((ADPCM_ResampCurPos)), 0x80000000 | 0, "ADPCM_ResampCurPos" },



 { &((RegSelector)), sizeof((RegSelector)), 0x80000000 | 0, "RegSelector" },
 { ((ArgsBuf)), (uint32)((16)), 0 | 0, "ArgsBuf" },
 { &((ArgsWP)), sizeof((ArgsWP)), 0x80000000 | 0, "ArgsWP" },
 { &((ArgsRP)), sizeof((ArgsRP)), 0x80000000 | 0, "ArgsRP" },

 { &((ArgsReceiveLatch)), sizeof((ArgsReceiveLatch)), 0x80000000 | 0, "ArgsReceiveLatch" },
 { ((ArgsReceiveBuf)), (uint32)((32)), 0 | 0, "ArgsReceiveBuf" },
 { &((ArgsReceiveIn)), sizeof((ArgsReceiveIn)), 0x80000000 | 0, "ArgsReceiveIn" },

 { ((ResultsBuffer)), (uint32)((16)), 0 | 0, "ResultsBuffer" },
 { &((ResultsIn)), sizeof((ResultsIn)), 0x80000000 | 0, "ResultsIn" },
 { &((ResultsWP)), sizeof((ResultsWP)), 0x80000000 | 0, "ResultsWP" },
 { &((ResultsRP)), sizeof((ResultsRP)), 0x80000000 | 0, "ResultsRP" },




  { ((DMABuffer->data)), (uint32)((DMABuffer->size)), 0 | 0, "&DMABuffer.data[0]" },
  { &((DMABuffer->read_pos)), sizeof((DMABuffer->read_pos)), 0x80000000 | 0, "DMABuffer.read_pos" },
  { &((DMABuffer->write_pos)), sizeof((DMABuffer->write_pos)), 0x80000000 | 0, "DMABuffer.write_pos" },
  { &((DMABuffer->in_count)), sizeof((DMABuffer->in_count)), 0x80000000 | 0, "DMABuffer.in_count" },




  { ((SB)), (uint32)((sizeof(SB) / sizeof(SB[0]))), 0 | 0, "SB" },
  { &((SB_In)), sizeof((SB_In)), 0x80000000 | 0, "SB_In" },

  { ((&SectorPipe[0][0])), (uint32)((sizeof(SectorPipe) / sizeof(SectorPipe[0][0]))), 0 | 0, "&SectorPipe[0][0]" },
  { &((SectorPipe_Pos)), sizeof((SectorPipe_Pos)), 0x80000000 | 0, "SectorPipe_Pos" },
  { &((SectorPipe_In)), sizeof((SectorPipe_In)), 0x80000000 | 0, "SectorPipe_In" },

  { ((SubQBuf)), (uint32)((sizeof(SubQBuf) / sizeof(SubQBuf[0]))), 0 | 0, "SubQBuf" },
  { ((SubQBuf_Safe)), (uint32)((sizeof(SubQBuf_Safe) / sizeof(SubQBuf_Safe[0]))), 0 | 0, "SubQBuf_Safe" },

  { &((SubQChecksumOK)), 1, 0x80000000 | 0x08000000, "SubQChecksumOK" },

  { &((HeaderBufValid)), 1, 0x80000000 | 0x08000000, "HeaderBufValid" },
  { ((HeaderBuf)), (uint32)((sizeof(HeaderBuf) / sizeof(HeaderBuf[0]))), 0 | 0, "HeaderBuf" },

  { &((IRQBuffer)), sizeof((IRQBuffer)), 0x80000000 | 0, "IRQBuffer" },
  { &((IRQOutTestMask)), sizeof((IRQOutTestMask)), 0x80000000 | 0, "IRQOutTestMask" },
  { &((CDCReadyReceiveCounter)), sizeof((CDCReadyReceiveCounter)), 0x80000000 | 0, "CDCReadyReceiveCounter" },

  { &((FilterFile)), sizeof((FilterFile)), 0x80000000 | 0, "FilterFile" },
  { &((FilterChan)), sizeof((FilterChan)), 0x80000000 | 0, "FilterChan" },

  { &((PendingCommand)), sizeof((PendingCommand)), 0x80000000 | 0, "PendingCommand" },
  { &((PendingCommandPhase)), sizeof((PendingCommandPhase)), 0x80000000 | 0, "PendingCommandPhase" },
  { &((PendingCommandCounter)), sizeof((PendingCommandCounter)), 0x80000000 | 0, "PendingCommandCounter" },

  { &((SPUCounter)), sizeof((SPUCounter)), 0x80000000 | 0, "SPUCounter" },

  { &((Mode)), sizeof((Mode)), 0x80000000 | 0, "Mode" },
  { &((DriveStatus)), sizeof((DriveStatus)), 0x80000000 | 0, "DriveStatus" },
  { &((StatusAfterSeek)), sizeof((StatusAfterSeek)), 0x80000000 | 0, "StatusAfterSeek" },
  { &((Forward)), 1, 0x80000000 | 0x08000000, "Forward" },
  { &((Backward)), 1, 0x80000000 | 0x08000000, "Backward" },
  { &((Muted)), 1, 0x80000000 | 0x08000000, "Muted" },

  { &((PlayTrackMatch)), sizeof((PlayTrackMatch)), 0x80000000 | 0, "PlayTrackMatch" },

  { &((PSRCounter)), sizeof((PSRCounter)), 0x80000000 | 0, "PSRCounter" },

  { &((CurSector)), sizeof((CurSector)), 0x80000000 | 0, "CurSector" },


  { &((AsyncIRQPending)), sizeof((AsyncIRQPending)), 0x80000000 | 0, "AsyncIRQPending" },
  { ((AsyncResultsPending)), (uint32)((sizeof(AsyncResultsPending) / sizeof(AsyncResultsPending[0]))), 0 | 0, "AsyncResultsPending" },
  { &((AsyncResultsPendingCount)), sizeof((AsyncResultsPendingCount)), 0x80000000 | 0, "AsyncResultsPendingCount" },

  { &((SeekTarget)), sizeof((SeekTarget)), 0x80000000 | 0, "SeekTarget" },







  { &((CommandLoc)), sizeof((CommandLoc)), 0x80000000 | 0, "CommandLoc" },
  { &((CommandLoc_Dirty)), 1, 0x80000000 | 0x08000000, "CommandLoc_Dirty" },
  { ((&xa_previous[0][0])), (uint32)(((sizeof(xa_previous) / sizeof(xa_previous[0][0]))) * sizeof(uint16)), 0x20000000 | 0, "&xa_previous[0][0]" },

  { &((xa_cur_set)), 1, 0x80000000 | 0x08000000, "xa_cur_set" },
  { &((xa_cur_file)), sizeof((xa_cur_file)), 0x80000000 | 0, "xa_cur_file" },
  { &((xa_cur_chan)), sizeof((xa_cur_chan)), 0x80000000 | 0, "xa_cur_chan" },

  { &((ReportLastF)), sizeof((ReportLastF)), 0x80000000 | 0, "ReportLastF" },

  { 0, 0, 0, 0 }
 };

 int ret = MDFNSS_StateAction(sm, load, StateRegs, "CDC");

 if(load)
 {
    SimpleFIFO_SaveStatePostLoad(DMABuffer);
    SectorPipe_Pos %= SectorPipe_Count;
 }
 return(ret);
}

void CDC_ResetTS(void)
{
   lastts = 0;
}

static uint8 CDC_MakeStatus(bool cmd_error)
{
   uint8 ret = 0;

   // Are these bit positions right?

   if(DriveStatus == DS_PLAYING)
      ret |= 0x80;

   if(DriveStatus == DS_SEEKING || DriveStatus == DS_SEEKING_LOGICAL)
      ret |= 0x40;

   if(DriveStatus == DS_READING)
      ret |= 0x20;

   // TODO: shell open and seek error
   if(!Cur_CDIF || DiscChanged)
      ret |= 0x10;

   if(DriveStatus != DS_STOPPED)
      ret |= 0x02;

   if(cmd_error)
      ret |= 0x01;

   DiscChanged = false;

   return(ret);
}

static bool CDC_DecodeSubQ(uint8 *subpw)
{
   int i;
   uint8 tmp_q[0xC];

   memset(tmp_q, 0, 0xC);

   for(i = 0; i < 96; i++)
      tmp_q[i >> 3] |= ((subpw[i] & 0x40) >> 6) << (7 - (i & 7));

   if((tmp_q[0] & 0xF) == 1)
   {
      memcpy(SubQBuf, tmp_q, 0xC);
      SubQChecksumOK = subq_check_checksum(tmp_q);

      if(SubQChecksumOK)
      {
         memcpy(SubQBuf_Safe, tmp_q, 0xC);
         return(true);
      }
   }

   return(false);
}

static const int16 CDADPCMImpulse[7][25] =
{
 {     0,    -5,    17,   -35,    70,   -23,   -68,   347,  -839,  2062, -4681, 15367, 21472, -5882,  2810, -1352,   635,  -235,    26,    43,   -35,    16,    -8,     2,     0,  }, /* 0 */
 {     0,    -2,    10,   -34,    65,   -84,    52,     9,  -266,  1024, -2680,  9036, 26516, -6016,  3021, -1571,   848,  -365,   107,    10,   -16,    17,    -8,     3,    -1,  }, /* 1 */
 {    -2,     0,     3,   -19,    60,   -75,   162,  -227,   306,   -67,  -615,  3229, 29883, -4532,  2488, -1471,   882,  -424,   166,   -27,     5,     6,    -8,     3,    -1,  }, /* 2 */
 {    -1,     3,    -2,    -5,    31,   -74,   179,  -402,   689,  -926,  1272, -1446, 31033, -1446,  1272,  -926,   689,  -402,   179,   -74,    31,    -5,    -2,     3,    -1,  }, /* 3 */
 {    -1,     3,    -8,     6,     5,   -27,   166,  -424,   882, -1471,  2488, -4532, 29883,  3229,  -615,   -67,   306,  -227,   162,   -75,    60,   -19,     3,     0,    -2,  }, /* 4 */
 {    -1,     3,    -8,    17,   -16,    10,   107,  -365,   848, -1571,  3021, -6016, 26516,  9036, -2680,  1024,  -266,     9,    52,   -84,    65,   -34,    10,    -2,     0,  }, /* 5 */
 {     0,     2,    -8,    16,   -35,    43,    26,  -235,   635, -1352,  2810, -5882, 21472, 15367, -4681,  2062,  -839,   347,   -68,   -23,    70,   -35,    17,    -5,     0,  }, /* 6 */
};

#define CDC_ReadAudioBuffer(samples) \
 samples[0] = AudioBuffer.Samples[0][AudioBuffer.ReadPos]; \
 samples[1] = AudioBuffer.Samples[1][AudioBuffer.ReadPos]; \
 AudioBuffer.ReadPos++

/* Algorithmically, volume is applied after resampling for CD-XA ADPCM playback, 
 * per PS1 tests(though when "mute" is applied wasn't tested). */ 

static INLINE void CDC_ApplyVolume(int32 samples[2])
{
   /* Take care not to alter samples[] before we're done calculating the new output samples! */ 
   int32 left_out = ((samples[0] * DecodeVolume[0][0]) >> 7) + ((samples[1] * DecodeVolume[1][0]) >> 7);
   int32 right_out = ((samples[0] * DecodeVolume[0][1]) >> 7) + ((samples[1] * DecodeVolume[1][1]) >> 7);

   clamp(&left_out, -32768, 32767);
   clamp(&right_out, -32768, 32767);

   if(Muted)
   {
      left_out = 0;
      right_out = 0;
   }

   samples[0] = left_out;
   samples[1] = right_out;
}

//
// This function must always set samples[0] and samples[1], even if just to 0; range of samples[n] shall be restricted to -32768 through 32767.
//
void CDC_GetCDAudio(int32 samples[2])
{
   const unsigned freq = (AudioBuffer.ReadPos < AudioBuffer.Size) ? AudioBuffer.Freq : 0;

   samples[0] = 0;
   samples[1] = 0;

   if(!freq)
      return;

   if(freq == 7 || freq == 14)
   {
      CDC_ReadAudioBuffer(samples);
      if(freq == 14)
      {
         CDC_ReadAudioBuffer(samples);
      }
   }
   else
   {
      int32 out_tmp[2] = { 0, 0 };

      for(unsigned i = 0; i < 2; i++)
      {
         const int16* imp = CDADPCMImpulse[ADPCM_ResampCurPhase];
         int16* wf = &ADPCM_ResampBuf[i][(ADPCM_ResampCurPos + 32 - 25) & 0x1F];

         for(unsigned s = 0; s < 25; s++)
         {
            out_tmp[i] += imp[s] * wf[s];
         }

         out_tmp[i] >>= 15;
         clamp(&out_tmp[i], -32768, 32767);
         samples[i] = out_tmp[i];
      }

      ADPCM_ResampCurPhase += freq;

      if(ADPCM_ResampCurPhase >= 7)
      {
         int32 raw[2] = { 0, 0 };

         ADPCM_ResampCurPhase -= 7;
         CDC_ReadAudioBuffer(raw);

         for(unsigned i = 0; i < 2; i++)
         {
            ADPCM_ResampBuf[i][ADPCM_ResampCurPos +  0] = 
               ADPCM_ResampBuf[i][ADPCM_ResampCurPos + 32] = raw[i];
         }
         ADPCM_ResampCurPos = (ADPCM_ResampCurPos + 1) & 0x1F;
      }
   }

   CDC_ApplyVolume(samples);
}


struct XA_Subheader
{
 uint8 file;
 uint8 channel;
 uint8 submode;
 uint8 coding;

 uint8 file_dup;
 uint8 channel_dup;
 uint8 submode_dup;
 uint8 coding_dup;
};

struct XA_SoundGroup
{
 uint8 params[16];
 uint8 samples[112];
};

#define XA_SUBMODE_EOF		0x80
#define XA_SUBMODE_REALTIME	0x40
#define XA_SUBMODE_FORM		0x20
#define XA_SUBMODE_TRIGGER	0x10
#define XA_SUBMODE_DATA		0x08
#define XA_SUBMODE_AUDIO	0x04
#define XA_SUBMODE_VIDEO	0x02
#define XA_SUBMODE_EOR		0x01

#define XA_CODING_EMPHASIS	0x40

//#define XA_CODING_BPS_MASK	0x30
//#define XA_CODING_BPS_4BIT	0x00
//#define XA_CODING_BPS_8BIT	0x10
//#define XA_CODING_SR_MASK	0x0C
//#define XA_CODING_SR_378	0x00
//#define XA_CODING_SR_

#define XA_CODING_8BIT		0x10
#define XA_CODING_189		0x04
#define XA_CODING_STEREO	0x01

// Special regression prevention test cases:
//	Um Jammer Lammy (start doing poorly)
//	Yarudora Series Vol.1 - Double Cast (non-FMV speech)

static bool CDC_XA_Test(const uint8 *sdata)
{
   const XA_Subheader *sh = (const XA_Subheader *)&sdata[12 + 4];

   if(!(Mode & MODE_STRSND))
      return false;

   if(!(sh->submode & XA_SUBMODE_AUDIO))
      return false;

   //printf("Test File: 0x%02x 0x%02x - Channel: 0x%02x 0x%02x - Submode: 0x%02x 0x%02x - Coding: 0x%02x 0x%02x - \n", sh->file, sh->file_dup, sh->channel, sh->channel_dup, sh->submode, sh->submode_dup, sh->coding, sh->coding_dup);

   if((Mode & MODE_SF) && (sh->file != FilterFile || sh->channel != FilterChan))
      return false;

   if(!xa_cur_set || (Mode & MODE_SF))
   {
      xa_cur_set = true;
      xa_cur_file = sh->file;
      xa_cur_chan = sh->channel;
   }
   else if(sh->file != xa_cur_file || sh->channel != xa_cur_chan)
      return false;

   if(sh->submode & XA_SUBMODE_EOF)
   {
      //puts("YAY");
      xa_cur_set = false;
      xa_cur_file = 0;
      xa_cur_chan = 0;
   }

   return true;
}


//
// output should be readable at -2 and -1
static void DecodeXAADPCM(const uint8 *input, int16 *output, const unsigned shift, const unsigned weight)
{
 // Weights copied over from SPU channel ADPCM playback code, may not be entirely the same for CD-XA ADPCM, we need to run tests.
 static const int32 Weights[16][2] =
 {
  // s-1    s-2
  {   0,    0 },
  {  60,    0 },
  { 115,  -52 },
  {  98,  -55 },
  { 122,  -60 },
 };

 for(int i = 0; i < 28; i++)
 {
  int32 sample;

  sample = (int16)(input[i] << 8);
  sample >>= shift;

  sample += ((output[i - 1] * Weights[weight][0]) >> 6) + ((output[i - 2] * Weights[weight][1]) >> 6);

  clamp(&sample, -32768, 32767);
  output[i] = sample;
 }
}

static void CDC_XA_ProcessSector(const uint8 *sdata, CD_Audio_Buffer *ab)
{
   const XA_Subheader *sh = (const XA_Subheader *)&sdata[12 + 4];
   const unsigned unit_index_shift = (sh->coding & XA_CODING_8BIT) ? 0 : 1;

   //printf("File: 0x%02x 0x%02x - Channel: 0x%02x 0x%02x - Submode: 0x%02x 0x%02x - Coding: 0x%02x 0x%02x - \n", sh->file, sh->file_dup, sh->channel, sh->channel_dup, sh->submode, sh->submode_dup, sh->coding, sh->coding_dup);
   ab->ReadPos = 0;
   ab->Size = 18 * (4 << unit_index_shift) * 28;

   if(sh->coding & XA_CODING_STEREO)
      ab->Size >>= 1;

   ab->Freq = (sh->coding & XA_CODING_189) ? 3 : 6;

   //fprintf(stderr, "Coding: %02x %02x\n", sh->coding, sh->coding_dup);

   for(unsigned group = 0; group < 18; group++)
   {
      const XA_SoundGroup *sg = (const XA_SoundGroup *)&sdata[12 + 4 + 8 + group * 128];

      for(unsigned unit = 0; unit < (4U << unit_index_shift); unit++)
      {
         const uint8 param = sg->params[(unit & 3) | ((unit & 4) << 1)];
         const uint8 param_copy = sg->params[4 | (unit & 3) | ((unit & 4) << 1)];
         uint8 ibuffer[28];
         int16 obuffer[2 + 28];

#if 0
         if(param != param_copy)
         {
            PSX_WARNING("[CDC] CD-XA param != param_copy --- %d %02x %02x\n", unit, param, param_copy);
         }
#endif

         for(unsigned i = 0; i < 28; i++)
         {
            uint8 tmp = sg->samples[i * 4 + (unit >> unit_index_shift)];

            if(unit_index_shift)
            {
               tmp <<= (unit & 1) ? 0 : 4;
               tmp &= 0xf0;
            }

            ibuffer[i] = tmp;
         }

         const bool ocn = (bool)(unit & 1) && (sh->coding & XA_CODING_STEREO);

         obuffer[0] = xa_previous[ocn][0];
         obuffer[1] = xa_previous[ocn][1];

         DecodeXAADPCM(ibuffer, &obuffer[2], param & 0x0F, param >> 4);

         xa_previous[ocn][0] = obuffer[28];
         xa_previous[ocn][1] = obuffer[29];

         if(param != param_copy)
            memset(obuffer, 0, sizeof(obuffer));

         if(sh->coding & XA_CODING_STEREO)
         {
            for(unsigned s = 0; s < 28; s++)
            {
               ab->Samples[ocn][group * (2 << unit_index_shift) * 28 + (unit >> 1) * 28 + s] = obuffer[2 + s];
            }
         }
         else
         {
            for(unsigned s = 0; s < 28; s++)
            {
               ab->Samples[0][group * (4 << unit_index_shift) * 28 + unit * 28 + s] = obuffer[2 + s];
               ab->Samples[1][group * (4 << unit_index_shift) * 28 + unit * 28 + s] = obuffer[2 + s];
            }
         }
      }
   }
}

static void CDC_CheckAIP(void)
{
   unsigned i;

   if(AsyncIRQPending && CDCReadyReceiveCounter <= 0)
   {
      BeginResults();

      for(i = 0; i < AsyncResultsPendingCount; i++)
         WriteResult(AsyncResultsPending[i]);

      WriteIRQ(AsyncIRQPending);

      CDC_ClearAIP();
   }
}

static void CDC_SetAIP(unsigned irq, unsigned result_count, uint8 *r)
{
#if 0
 if(AsyncIRQPending)
 {
  PSX_WARNING("***WARNING*** Previous notification skipped: CurSector=%d, old_notification=0x%02x", CurSector, AsyncIRQPending);
 }
#endif
 CDC_ClearAIP();

 AsyncResultsPendingCount = result_count;

 for(unsigned i = 0; i < result_count; i++)
  AsyncResultsPending[i] = r[i];

 AsyncIRQPending = irq;

 CDC_CheckAIP();
}

static void CDC_SetAIP2(unsigned irq, uint8 result0)
{
   uint8 tr[1] = { result0 };
   CDC_SetAIP(irq, 1, tr);
}

void CDC_SetAIP(unsigned irq, uint8 result0, uint8 result1)
{
   uint8 tr[2] = { result0, result1 };
   CDC_SetAIP(irq, 2, tr);
}


static void CDC_EnbufferizeCDDASector(const uint8 *buf)
{
   CD_Audio_Buffer *ab = &AudioBuffer;

   ab->Freq = 7 * ((Mode & MODE_SPEED) ? 2 : 1);
   ab->Size = 588;

   if(SubQBuf_Safe[0] & 0x40)
   {
      for(int i = 0; i < 588; i++)
      {
         ab->Samples[0][i] = 0;
         ab->Samples[1][i] = 0;
      }
   }
   else
   {
      for(int i = 0; i < 588; i++)
      {
         ab->Samples[0][i] = (int16)MDFN_de16lsb(&buf[i * sizeof(int16) * 2 + 0]);
         ab->Samples[1][i] = (int16)MDFN_de16lsb(&buf[i * sizeof(int16) * 2 + 2]);
      }
   }

   ab->ReadPos = 0;
}

// CDC_SetAIP(CDCIRQ_DISC_ERROR, CDC_MakeStatus(false) | 0x04, 0x04);
static void CDC_HandlePlayRead(void)
{
   uint8 read_buf[2352 + 96];

   //PSX_WARNING("Read sector: %d", CurSector);

   if(CurSector >= ((int32)toc.tracks[100].lba + 300) && CurSector >= (75 * 60 * 75 - 150))
   {
      //PSX_WARNING("[CDC] Read/Play position waaay too far out(%u), forcing STOP", CurSector);
      DriveStatus = DS_STOPPED;
      SectorPipe_Pos = SectorPipe_In = 0;
      return;
   }

#ifdef WANT_ECC
   if(CurSector >= (int32)toc.tracks[100].lba)
   {
      //PSX_WARNING("[CDC] In leadout area: %u", CurSector);

      //
      // Synthesis is a bit of a kludge... :/
      //
      synth_leadout_sector_lba(0x02, toc, CurSector, read_buf);
   }
   else
#endif
      CDIF_ReadRawSector(Cur_CDIF, read_buf, CurSector);	// FIXME: error out on error.
   CDC_DecodeSubQ(read_buf + 2352);


   if(SubQBuf_Safe[1] == 0xAA && (DriveStatus == DS_PLAYING || (!(SubQBuf_Safe[0] & 0x40) && (Mode & MODE_CDDA))))
   {
      HeaderBufValid = false;

      //PSX_WARNING("[CDC] CD-DA leadout reached: %u", CurSector);

      // Status in this end-of-disc context here should be generated after we're in the pause state.
      DriveStatus = DS_PAUSED;
      SectorPipe_Pos = SectorPipe_In = 0;
      CDC_SetAIP2(CDCIRQ_DATA_END, CDC_MakeStatus(false));

      return;
   }

   if(DriveStatus == DS_PLAYING)
   {
      // Note: Some game(s) start playing in the pregap of a track(so don't replace this with a simple subq index == 0 check for autopause).
      if(PlayTrackMatch == -1 && SubQChecksumOK)
         PlayTrackMatch = SubQBuf_Safe[0x1];

      if((Mode & MODE_AUTOPAUSE) && PlayTrackMatch != -1 && SubQBuf_Safe[0x1] != PlayTrackMatch)
      {
         // Status needs to be taken before we're paused(IE it should still report playing).
         CDC_SetAIP2(CDCIRQ_DATA_END, CDC_MakeStatus(false));

         DriveStatus = DS_PAUSED;
         SectorPipe_Pos = SectorPipe_In = 0;
         PSRCounter = 0;
         return;
      }

      if((Mode & MODE_REPORT) && (((SubQBuf_Safe[0x9] >> 4) != ReportLastF) || Forward || Backward) && SubQChecksumOK)
      {
         uint8 tr[8];
#if 0
         uint16 abs_lev_max = 0;
         bool abs_lev_chselect = SubQBuf_Safe[0x8] & 0x01;

         for(int i = 0; i < 588; i++)
            abs_lev_max = std::max<uint16>(abs_lev_max, std::min<int>(abs((int16)MDFN_de16lsb(&read_buf[i * 4 + (abs_lev_chselect * 2)])), 32767));
         abs_lev_max |= abs_lev_chselect << 15;
#endif

         ReportLastF = SubQBuf_Safe[0x9] >> 4;

         tr[0] = CDC_MakeStatus(false);
         tr[1] = SubQBuf_Safe[0x1];	// Track
         tr[2] = SubQBuf_Safe[0x2];	// Index

         if(SubQBuf_Safe[0x9] & 0x10)
         {
            tr[3] = SubQBuf_Safe[0x3];		// R M
            tr[4] = SubQBuf_Safe[0x4] | 0x80;	// R S
            tr[5] = SubQBuf_Safe[0x5];		// R F
         }
         else	
         {
            tr[3] = SubQBuf_Safe[0x7];	// A M
            tr[4] = SubQBuf_Safe[0x8];	// A S
            tr[5] = SubQBuf_Safe[0x9];	// A F
         }

         tr[6] = 0; //abs_lev_max >> 0;
         tr[7] = 0; //abs_lev_max >> 8;

         CDC_SetAIP(CDCIRQ_DATA_READY, 8, tr);
      }
   }

   if(SectorPipe_In >= SectorPipe_Count)
   {
      uint8* buf = SectorPipe[SectorPipe_Pos];
      SectorPipe_In--;

      if(DriveStatus == DS_READING)
      {
         if(SubQBuf_Safe[0] & 0x40) //) || !(Mode & MODE_CDDA))
         {
            memcpy(HeaderBuf, buf + 12, 12);
            HeaderBufValid = true;

            if((Mode & MODE_STRSND) && (buf[12 + 3] == 0x2) && ((buf[12 + 6] & 0x64) == 0x64))
            {
               if(CDC_XA_Test(buf))
               {
                  if(AudioBuffer.ReadPos < AudioBuffer.Size)
                  {
                     //PSX_WARNING("[CDC] CD-XA ADPCM sector skipped - readpos=0x%04x, size=0x%04x", AudioBuffer.ReadPos, AudioBuffer.Size);
                  }
                  else
                     CDC_XA_ProcessSector(buf, &AudioBuffer);
               }
            }
            else
            {
#if 0
               // maybe if(!(Mode & 0x30)) too?
               if(!(buf[12 + 6] & 0x20))
               {
                  if(!edc_lec_check_and_correct(buf, true))
                  {
                     MDFN_DispMessage("Bad sector? - %d", CurSector);
                  }
               }

               if(!(Mode & 0x30) && (buf[12 + 6] & 0x20))
                  PSX_WARNING("[CDC] BORK: %d", CurSector);
#endif

               int32 offs = (Mode & 0x20) ? 0 : 12;
               int32 size = (Mode & 0x20) ? 2340 : 2048;

               if(Mode & 0x10)
               {
                  offs = 12;
                  size = 2328;
               }

               memcpy(SB, buf + 12 + offs, size);
               SB_In = size;
               CDC_SetAIP2(CDCIRQ_DATA_READY, CDC_MakeStatus(false));
            }
         }
      }

      if(!(SubQBuf_Safe[0] & 0x40) && ((Mode & MODE_CDDA) || DriveStatus == DS_PLAYING))
      {
         if(AudioBuffer.ReadPos < AudioBuffer.Size)
         {
            //PSX_WARNING("[CDC] BUG CDDA buffer full");
         }
         else
            CDC_EnbufferizeCDDASector(buf);
      }
   }

   memcpy(SectorPipe[SectorPipe_Pos], read_buf, 2352);
   SectorPipe_Pos = (SectorPipe_Pos + 1) % SectorPipe_Count;
   SectorPipe_In++;

   PSRCounter += 33868800 / (75 * ((Mode & MODE_SPEED) ? 2 : 1));

   if(DriveStatus == DS_PLAYING)
   {
      // FIXME: What's the real fast-forward and backward speed?
      if(Forward)
         CurSector += 12;
      else if(Backward)
      {
         CurSector -= 12;

         if(CurSector < 0)	// FIXME: How does a real PS handle this condition?
            CurSector = 0;
      }
      else
         CurSector++;
   }
   else
      CurSector++;

}

int32_t CDC_Update(const int32_t timestamp)
{
   int32 clocks = timestamp - lastts;

   //doom_ts = timestamp;

   while(clocks > 0)
   {
      int32 chunk_clocks = clocks;

      if(PSRCounter > 0 && chunk_clocks > PSRCounter)
         chunk_clocks = PSRCounter;

      if(PendingCommandCounter > 0 && chunk_clocks > PendingCommandCounter)
         chunk_clocks = PendingCommandCounter;

      if(chunk_clocks > SPUCounter)
         chunk_clocks = SPUCounter;

      if(DiscStartupDelay > 0)
      {
         if(chunk_clocks > DiscStartupDelay)
            chunk_clocks = DiscStartupDelay;

         DiscStartupDelay -= chunk_clocks;

         if(DiscStartupDelay <= 0)
         {
            DriveStatus = DS_PAUSED;	// or is it supposed to be DS_STANDBY?
         }
      }

      //MDFN_DispMessage("%02x %d -- %d %d -- %02x", IRQBuffer, CDCReadyReceiveCounter, PSRCounter, PendingCommandCounter, PendingCommand);

      if(!IRQBuffer)
      {
         if(CDCReadyReceiveCounter > 0 && chunk_clocks > CDCReadyReceiveCounter)
            chunk_clocks = CDCReadyReceiveCounter;

         if(CDCReadyReceiveCounter > 0)
            CDCReadyReceiveCounter -= chunk_clocks;
      }

      CDC_CheckAIP();

      if(PSRCounter > 0)
      {
         uint8 buf[2352 + 96];

         PSRCounter -= chunk_clocks;

         if(PSRCounter <= 0) 
         {
            if(DriveStatus == DS_RESETTING)
            {
               CDC_SetAIP2(CDCIRQ_COMPLETE, CDC_MakeStatus(false));

               Muted = false; // Does it get reset here?
               CDC_ClearAudioBuffers();

               SB_In = 0;
               SectorPipe_Pos = SectorPipe_In = 0;

               Mode = 0;
               CurSector = 0;
               CommandLoc = 0;

               DriveStatus = DS_PAUSED;	// or DS_STANDBY?
               CDC_ClearAIP();
            }
            else if(DriveStatus == DS_SEEKING)
            {
               CurSector = SeekTarget;
               CDIF_ReadRawSector(Cur_CDIF, buf, CurSector);
               CDC_DecodeSubQ(buf + 2352);

               DriveStatus = StatusAfterSeek;

               if(DriveStatus != DS_PAUSED && DriveStatus != DS_STANDBY)
                  PSRCounter = 33868800 / (75 * ((Mode & MODE_SPEED) ? 2 : 1));
            }
            else if(DriveStatus == DS_SEEKING_LOGICAL)
            {
               CurSector = SeekTarget;
               CDIF_ReadRawSector(Cur_CDIF, buf, CurSector);
               CDC_DecodeSubQ(buf + 2352);
               memcpy(HeaderBuf, buf + 12, 12);

               DriveStatus = StatusAfterSeek;

               if(DriveStatus != DS_PAUSED && DriveStatus != DS_STANDBY)
               {
                  // TODO: SetAIP(CDCIRQ_DISC_ERROR, CDC_MakeStatus(false) | 0x04, 0x04);  when !(Mode & MODE_CDDA) and the sector isn't a data sector.
                  PSRCounter = 33868800 / (75 * ((Mode & MODE_SPEED) ? 2 : 1));
               }
            }
            else if(DriveStatus == DS_READING || DriveStatus == DS_PLAYING)
               CDC_HandlePlayRead();
         }
      }

      if(PendingCommandCounter > 0)
      {
         PendingCommandCounter -= chunk_clocks;

         if(PendingCommandCounter <= 0 && CDCReadyReceiveCounter > 0)
         {
            PendingCommandCounter = CDCReadyReceiveCounter; //256;
         }
         //else if(PendingCommandCounter <= 0 && PSRCounter > 0 && PSRCounter < 2000)
         //{
         // PendingCommandCounter = PSRCounter + 1;
         //}
         else if(PendingCommandCounter <= 0)
         {
            int32 next_time = 0;

            if(PendingCommandPhase == -1)
            {
               if(ArgsRP != ArgsWP)
               {
                  ArgsReceiveLatch = ArgsBuf[ArgsRP & 0x0F];
                  ArgsRP = (ArgsRP + 1) & 0x1F;
                  PendingCommandPhase += 1;
                  next_time = 1815;
               }
               else
               {
                  PendingCommandPhase += 2;
                  next_time = 8500;
               }
            }
            else if(PendingCommandPhase == 0)	// Command phase 0
            {
               if(ArgsReceiveIn < 32)
                  ArgsReceiveBuf[ArgsReceiveIn++] = ArgsReceiveLatch;

               if(ArgsRP != ArgsWP)
               {
                  ArgsReceiveLatch = ArgsBuf[ArgsRP & 0x0F];
                  ArgsRP = (ArgsRP + 1) & 0x1F;
                  next_time = 1815;
               }
               else
               {
                  PendingCommandPhase++;
                  next_time = 8500;
               }
            }
            else if(PendingCommandPhase >= 2)	// Command phase 2+
            {
               BeginResults();

               const CDC_CTEntry *command = &CDC_Commands[PendingCommand];

               next_time = (*(command->func2))();
            }
            else	// Command phase 1
            {
               if(PendingCommand >= 0x20 || !CDC_Commands[PendingCommand].func)
               {
                  BeginResults();

                  //PSX_WARNING("[CDC] Unknown command: 0x%02x", PendingCommand);

                  WriteResult(CDC_MakeStatus(true));
                  WriteResult(ERRCODE_BAD_COMMAND);
                  WriteIRQ(CDCIRQ_DISC_ERROR);
               }
               else if(ArgsReceiveIn < CDC_Commands[PendingCommand].args_min || ArgsReceiveIn > CDC_Commands[PendingCommand].args_max)
               {
                  BeginResults();

#if 0
                  PSX_DBG(PSX_DBG_WARNING, "[CDC] Bad number(%d) of args(first check) for command 0x%02x", ArgsReceiveIn, PendingCommand);
                  for(unsigned int i = 0; i < ArgsReceiveIn; i++)
                     PSX_DBG(PSX_DBG_WARNING, " 0x%02x", ArgsReceiveBuf[i]);
                  PSX_DBG(PSX_DBG_WARNING, "\n");
#endif

                  WriteResult(CDC_MakeStatus(true));
                  WriteResult(ERRCODE_BAD_NUMARGS);
                  WriteIRQ(CDCIRQ_DISC_ERROR);
               }
               else
               {
                  BeginResults();

                  const CDC_CTEntry *command = &CDC_Commands[PendingCommand];

#if 0
                  PSX_DBG(PSX_DBG_SPARSE, "[CDC] Command: %s --- ", command->name);
                  for(unsigned int i = 0; i < ArgsReceiveIn; i++)
                     PSX_DBG(PSX_DBG_SPARSE, " 0x%02x", ArgsReceiveBuf[i]);
                  PSX_DBG(PSX_DBG_SPARSE, "\n");
#endif

                  next_time = (*(command->func))(ArgsReceiveIn, ArgsReceiveBuf);
                  PendingCommandPhase = 2;
               }
               ArgsReceiveIn = 0;
            } // end command phase 1

            if(!next_time)
               PendingCommandCounter = 0;
            else
               PendingCommandCounter += next_time;
         }
      }

      SPUCounter = SPU_UpdateFromCDC(chunk_clocks);

      clocks -= chunk_clocks;
   } // end while(clocks > 0)

   lastts = timestamp;

   return(timestamp + CDC_CalcNextEvent());
}

void CDC_Write(const int32_t timestamp, uint32 A, uint8 V)
{
 A &= 0x3;

 //printf("Write: %08x %02x\n", A, V);

 if(A == 0x00)
 {
  RegSelector = V & 0x3;
 }
 else
 {
  const unsigned reg_index = ((RegSelector & 0x3) * 3) + (A - 1);

  CDC_Update(timestamp);
  //PSX_WARNING("[CDC] Write to register 0x%02x: 0x%02x @ %d --- 0x%02x 0x%02x\n", reg_index, V, timestamp, DMABuffer->in_count, IRQBuffer);

  switch(reg_index)
  {
	default:
		//PSX_WARNING("[CDC] Unknown write to register 0x%02x: 0x%02x\n", reg_index, V);
		break;

	 case 0x00:
#if 0
		if(PendingCommandCounter > 0)
		{
		 PSX_WARNING("[CDC] WARNING: Interrupting command 0x%02x, phase=%d, timeleft=%d with command=0x%02x", PendingCommand, PendingCommandPhase,
			PendingCommandCounter, V);
		}

		if(IRQBuffer)
		{
		 PSX_WARNING("[CDC] Attempting to start command(0x%02x) while IRQBuffer(0x%02x) is not clear.", V, IRQBuffer);
		}

		if(ResultsIn > 0)
		{
		 PSX_WARNING("[CDC] Attempting to start command(0x%02x) while command results(count=%d) still in buffer.", V, ResultsIn);
		}
#endif

      PendingCommandCounter = 10500 + PSX_GetRandU32(0, 3000) + 1815;
      PendingCommand = V;
      PendingCommandPhase = -1;
      ArgsReceiveIn = 0;
		break;

	 case 0x01:
		ArgsBuf[ArgsWP & 0xF] = V;
		ArgsWP = (ArgsWP + 1) & 0x1F;
		
#if 0
		if(!((ArgsWP - ArgsRP) & 0x0F))
		{
		 PSX_WARNING("[CDC] Argument buffer overflow");
		}
#endif
		break;

	 case 0x02:
 	 	if(V & 0x80)
	 	{
	  	 if(!DMABuffer->in_count)
	  	 {
          uint8 *sb_buf = (uint8*)&SB[0];
		  if(!SB_In)
		  {
           uint8 sb_size = 2340;
		   //PSX_WARNING("[CDC] Data read begin when no data to read!");

		   SimpleFIFO_Write(DMABuffer, sb_buf, sb_size);

		   while(FIFO_CAN_WRITE(DMABuffer))
         {
		    SimpleFIFO_WriteUnit(DMABuffer, 0x00);
         }
		  }
		  else
		  {
	   	   SimpleFIFO_Write(DMABuffer, sb_buf, SB_In);
		   SB_In = 0;
		  }
	  	 }
		 else
		 {
		  //PSX_WARNING("[CDC] Attempt to start data transfer via 0x80->1803 when %d bytes still in buffer", DMABuffer->in_count);
		 }
	 	}
	 	else if(V & 0x40)	// Something CD-DA related(along with & 0x20 ???)?
	 	{
		 for(unsigned i = 0; i < 4 && DMABuffer->in_count; i++)
       {
          SimpleFIFO_ReadUnit(DMABuffer);
          SimpleFIFO_ReadUnitIncrement(DMABuffer);
       }
		}
		else
		{
		 SimpleFIFO_Flush(DMABuffer);
		}

		if(V & 0x20)
		{
		 //PSX_WARNING("[CDC] Mystery IRQ trigger bit set.");
		 IRQBuffer |= 0x10;
		}
		break;

	 case 0x04:
	 	IRQOutTestMask = V;
		RecalcIRQ();
		break;

	 case 0x05:
#if 0
		if((IRQBuffer &~ V) != IRQBuffer && ResultsIn)
		{
		 // To debug icky race-condition related problems in "Psychic Detective", and to see if any games suffer from the same potential issue
		 // (to know what to test when we emulate CPU more accurately in regards to pipeline stalls and timing, which could throw off our kludge
		 //  for this issue)
		 PSX_WARNING("[CDC] Acknowledged IRQ(wrote 0x%02x, before_IRQBuffer=0x%02x) while %u bytes in results buffer.", V, IRQBuffer, ResultsIn);
		}
#endif

	 	IRQBuffer &= ~V;
	 	RecalcIRQ();

		if(V & 0x80)	// Forced CD hardware reset of some kind(interface, controller, and drive?)  Seems to take a while(relatively speaking) to complete.
		 CDC_SoftReset();

		if(V & 0x40)	// Does it clear more than arguments buffer?  Doesn't appear to clear results buffer.
		 ArgsWP = ArgsRP = 0;
		break;

	 case 0x07:
		Pending_DecodeVolume[0][0] = V;
		break;

	 case 0x08:
		Pending_DecodeVolume[0][1] = V;
		break;

	 case 0x09:
		Pending_DecodeVolume[1][1] = V;
		break;

	 case 0x0A:
		Pending_DecodeVolume[1][0] = V;
		break;

	 case 0x0B:
		if(V & 0x20)
		{
		 memcpy(DecodeVolume, Pending_DecodeVolume, sizeof(DecodeVolume));

#if 0
		 for(int i = 0; i < 2; i++)
		 {
		  for(int o = 0; o < 2; o++)
		  {
		   //fprintf(stderr, "Input Channel %d, Output Channel %d -- Volume=%d\n", i, o, DecodeVolume[i][o]);
		  }
		 }
#endif
		}
		break;
  }
  PSX_SetEventNT(PSX_EVENT_CDC, timestamp + CDC_CalcNextEvent());
 }
}

uint8 CDC_Read(const int32_t timestamp, uint32 A)
{
   uint8 ret = 0;

   A &= 0x03;

   //printf("Read %08x\n", A);

   if(A == 0x00)
   {
      ret = RegSelector & 0x3;

      if(ArgsWP == ArgsRP)
         ret |= 0x08;	// Args FIFO empty.

      if(!((ArgsWP - ArgsRP) & 0x10))
         ret |= 0x10;	// Args FIFO has room.

      if(ResultsIn)
         ret |= 0x20;

      if(DMABuffer->in_count)
         ret |= 0x40;

      if(PendingCommandCounter > 0 && PendingCommandPhase <= 1)
         ret |= 0x80;
   }
   else
   {
      switch(A & 0x3)
      {
         case 0x01:
            ReadResult(ret);
            break;

         case 0x02:
            //PSX_WARNING("[CDC] DMA Buffer manual read");
            if(DMABuffer->in_count)
            {
               ret = SimpleFIFO_ReadUnit(DMABuffer);
               SimpleFIFO_ReadUnitIncrement(DMABuffer);
            }
#if 0
            else
            {
               PSX_WARNING("[CDC] CD data transfer port read, but no data present!");
            }
#endif
            break;

         case 0x03:
            if(RegSelector & 0x1)
               ret = 0xE0 | IRQBuffer;
            else
               ret = 0xFF;
            break;
      }
   }

   return(ret);
}

bool CDC_DMACanRead(void)
{
   if (DMABuffer)
      return(DMABuffer->in_count);
   return false;
}

#ifdef __cplusplus
extern "C" {
#endif

uint32 CDC_DMARead(void)
{
   int i;
   uint32 data = 0;

   for(i = 0; i < 4; i++)
   {
      if(!DMABuffer->in_count)
         continue;

      data |= SimpleFIFO_ReadUnit(DMABuffer) << (i * 8);
      SimpleFIFO_ReadUnitIncrement(DMABuffer);
   }

   return data;
}

#ifdef __cplusplus
}
#endif

static bool CommandCheckDiscPresent(void)
{
   if(!Cur_CDIF || DiscStartupDelay > 0)
   {
      WriteResult(CDC_MakeStatus(true));
      WriteResult(ERRCODE_NOT_READY);

      WriteIRQ(CDCIRQ_DISC_ERROR);

      return(false);
   }

   return(true);
}

static int32 Command_Nop(const int arg_count, const uint8 *args)
{
   WriteResult(CDC_MakeStatus(false));
   WriteIRQ(CDCIRQ_ACKNOWLEDGE);

   return(0);
}

static int32 Command_Setloc(const int arg_count, const uint8 *args)
{
   uint8 m = BCD_to_U8(args[0] & 0x7F);
   uint8 s = BCD_to_U8(args[1]);
   uint8 f = BCD_to_U8(args[2]);

   CommandLoc = f + 75 * s + 75 * 60 * m - 150;
   CommandLoc_Dirty = true;

   WriteResult(CDC_MakeStatus(false));
   WriteIRQ(CDCIRQ_ACKNOWLEDGE);

   return(0);
}

int32 CDC_CalcSeekTime(int32 initial, int32 target, bool motor_on, bool paused)
{
   int32 ret = 0;

   if(!motor_on)
   {
      initial = 0;
      ret += 33868800;
   }

   ret += std::max<int64>((int64)abs(initial - target) * 33868800 * 1000 / (72 * 60 * 75) / 1000, 20000);

   if(abs(initial - target) >= 2250)
      ret += (int64)33868800 * 300 / 1000;
   else if(paused)
   {
      // The delay to restart from a Pause state is...very....WEIRD.  The time it takes is related to the amount of time that has passed since the pause, and
      // where on the disc the laser head is, with generally more time passed = longer to resume, except that there's a window of time where it takes a
      // ridiculous amount of time when not much time has passed.
      // 
      // What we have here will be EXTREMELY simplified.

      //
      //

      //if(time_passed >= 67737)
      //{
      //}
      //else
      {
         // Take twice as long for 1x mode.
         ret += 1237952 * ((Mode & MODE_SPEED) ? 1 : 2);
      }
   }

   ret += PSX_GetRandU32(0, 25000);

   //PSX_DBG(PSX_DBG_SPARSE, "[CDC] CalcSeekTime() = %d\n", ret);

   return(ret);
}

/* Remove this function when we have better seek emulation; it's here 
 * because the Rockman complete works games(at least 2 and 4) apparently have finicky fubared CD
 * access code.
 */
static void CDC_PreSeekHack(bool logical, uint32 target)
{
   uint8 buf[2352 + 96];
   int max_try = 32;
   bool NeedHBuf = logical;

   CurSector = target;	// If removing/changing this, take into account how it will affect ReadN/ReadS/Play/etc command calls that interrupt a seek.

   // If removing this SubQ reading bit, think about how it will interact with a Read command of data(or audio :b) sectors when Mode bit0 is 1.
   if(target < toc.tracks[100].lba)
   {
      do
      {
         CDIF_ReadRawSector(Cur_CDIF, buf, target++);

         // GetLocL related kludge, for Gran Turismo 1 music, perhaps others?
         if(NeedHBuf)
         {
            NeedHBuf = false;
            memcpy(HeaderBuf, buf + 12, 12);
            HeaderBufValid = true;
         }
      }while(!CDC_DecodeSubQ(buf + 2352) && --max_try > 0 && target < toc.tracks[100].lba);
   }
}

static int32 Command_Play(const int arg_count, const uint8 *args)
{
   if(!CommandCheckDiscPresent())
      return(0);

   CDC_ClearAIP();

   WriteResult(CDC_MakeStatus(false));
   WriteIRQ(CDCIRQ_ACKNOWLEDGE);

   Forward = Backward = false;

   if(arg_count && args[0])
   {
      int track = BCD_to_U8(args[0]);

      if(track < toc.first_track)
      {
         //PSX_WARNING("[CDC] Attempt to play track before first track.");
         track = toc.first_track;
      }
      else if(track > toc.last_track)
      {
         //PSX_WARNING("[CDC] Attempt to play track before first track.");
         track = toc.last_track;
      }

      CDC_ClearAudioBuffers();
      SectorPipe_Pos = SectorPipe_In = 0;

      PlayTrackMatch = track;

      //PSX_WARNING("[CDC] Play track: %d", track);

      SeekTarget = toc.tracks[track].lba;
      PSRCounter = CDC_CalcSeekTime(CurSector, SeekTarget, DriveStatus != DS_STOPPED, DriveStatus == DS_PAUSED);
      HeaderBufValid = false;
      CDC_PreSeekHack(false, SeekTarget);

      ReportLastF = 0xFF;

      DriveStatus = DS_SEEKING;
      StatusAfterSeek = DS_PLAYING;
   }
   else if(CommandLoc_Dirty || DriveStatus != DS_PLAYING)
   {
      CDC_ClearAudioBuffers();
      SectorPipe_Pos = SectorPipe_In = 0;

      if(CommandLoc_Dirty)
         SeekTarget = CommandLoc;
      else
         SeekTarget = CurSector;

      PlayTrackMatch = -1;

      PSRCounter = CDC_CalcSeekTime(CurSector, SeekTarget, DriveStatus != DS_STOPPED, DriveStatus == DS_PAUSED);
      HeaderBufValid = false;
      CDC_PreSeekHack(false, SeekTarget);

      ReportLastF = 0xFF;

      DriveStatus = DS_SEEKING;
      StatusAfterSeek = DS_PLAYING;
   }

   CommandLoc_Dirty = false;
   return(0);
}

static int32 Command_Forward(const int arg_count, const uint8 *args)
{
   if(!CommandCheckDiscPresent())
      return(0);

   WriteResult(CDC_MakeStatus(false));
   WriteIRQ(CDCIRQ_ACKNOWLEDGE);

   Backward = false;
   Forward = true;

   return(0);
}

static int32 Command_Backward(const int arg_count, const uint8 *args)
{
   if(!CommandCheckDiscPresent())
      return(0);

   WriteResult(CDC_MakeStatus(false));
   WriteIRQ(CDCIRQ_ACKNOWLEDGE);

   Backward = true;
   Forward = false;

   return(0);
}


static void CDC_ReadBase(void)
{
   if(!CommandCheckDiscPresent())
      return;

   WriteResult(CDC_MakeStatus(false));
   WriteIRQ(CDCIRQ_ACKNOWLEDGE);

   if(DriveStatus == DS_SEEKING_LOGICAL && SeekTarget == CommandLoc && StatusAfterSeek == DS_READING)
   {
      CommandLoc_Dirty = false;
      return;
   }

   if(CommandLoc_Dirty || DriveStatus != DS_READING)
   {
      // Don't flush the DMABuffer here; see CTR course selection screen.
      CDC_ClearAIP();
      CDC_ClearAudioBuffers();
      SB_In = 0;
      SectorPipe_Pos = SectorPipe_In = 0;

      // TODO: separate motor start from seek phase?

      if(CommandLoc_Dirty)
         SeekTarget = CommandLoc;
      else
         SeekTarget = CurSector;

      PSRCounter = /*903168 * 1.5 +*/ CDC_CalcSeekTime(CurSector, SeekTarget, DriveStatus != DS_STOPPED, DriveStatus == DS_PAUSED);
      HeaderBufValid = false;
      CDC_PreSeekHack(true, SeekTarget);

      DriveStatus = DS_SEEKING_LOGICAL;
      StatusAfterSeek = DS_READING;
   }

   CommandLoc_Dirty = false;
}

static int32 Command_ReadN(const int arg_count, const uint8 *args)
{
   CDC_ReadBase();
   return 0;
}

static int32 Command_ReadS(const int arg_count, const uint8 *args)
{
   CDC_ReadBase();
   return 0;
}

static int32 Command_Stop(const int arg_count, const uint8 *args)
{
   if(!CommandCheckDiscPresent())
      return(0);

   WriteResult(CDC_MakeStatus(false));
   WriteIRQ(CDCIRQ_ACKNOWLEDGE);

   if(DriveStatus == DS_STOPPED)
   {
      return(5000);
   }
   else
   {
      CDC_ClearAudioBuffers();
      CDC_ClearAIP();
      SectorPipe_Pos = SectorPipe_In = 0;

      DriveStatus = DS_STOPPED;
      HeaderBufValid = false;

      return(33868);	// FIXME, should be much higher.
   }
}

static int32 Command_Stop_Part2(void)
{
   PSRCounter = 0;

   WriteResult(CDC_MakeStatus(false));
   WriteIRQ(CDCIRQ_COMPLETE);

   return(0);
}

static int32 Command_Standby(const int arg_count, const uint8 *args)
{
   if(!CommandCheckDiscPresent())
      return(0);

   if(DriveStatus != DS_STOPPED)
   {
      WriteResult(CDC_MakeStatus(true));
      WriteResult(0x20);
      WriteIRQ(CDCIRQ_DISC_ERROR);
      return(0);
   }

   WriteResult(CDC_MakeStatus(false));
   WriteIRQ(CDCIRQ_ACKNOWLEDGE);

   CDC_ClearAudioBuffers();
   CDC_ClearAIP();
   SectorPipe_Pos = SectorPipe_In = 0;

   DriveStatus = DS_STANDBY;

   return((int64)33868800 * 100 / 1000);	// No idea, FIXME.
}

static int32 Command_Standby_Part2(void)
{
 PSRCounter = 0;

 WriteResult(CDC_MakeStatus(false));
 WriteIRQ(CDCIRQ_COMPLETE);

 return(0);
}

static int32 Command_Pause(const int arg_count, const uint8 *args)
{
   if(!CommandCheckDiscPresent())
      return(0);

   WriteResult(CDC_MakeStatus(false));
   WriteIRQ(CDCIRQ_ACKNOWLEDGE);

   if(DriveStatus == DS_PAUSED || DriveStatus == DS_STOPPED)
      return(5000);

   // "Viewpoint" flips out and crashes if reading isn't stopped (almost?) immediately.
   //CDC_ClearAudioBuffers();
   SectorPipe_Pos = SectorPipe_In = 0;
   CDC_ClearAIP();
   DriveStatus = DS_PAUSED;

   // An approximation.
   return((1124584 + ((int64)CurSector * 42596 / (75 * 60))) * ((Mode & MODE_SPEED) ? 1 : 2));
}

static int32 Command_Pause_Part2(void)
{
   PSRCounter = 0;

   WriteResult(CDC_MakeStatus(false));
   WriteIRQ(CDCIRQ_COMPLETE);

   return(0);
}

static int32 Command_Reset(const int arg_count, const uint8 *args)
{
   WriteResult(CDC_MakeStatus(false));
   WriteIRQ(CDCIRQ_ACKNOWLEDGE);

   if(DriveStatus != DS_RESETTING)
   {
      HeaderBufValid = false;
      DriveStatus = DS_RESETTING;
      PSRCounter = 1136000;
   }

   return(0);
}

static int32 Command_Mute(const int arg_count, const uint8 *args)
{
   Muted = true;

   WriteResult(CDC_MakeStatus(false));
   WriteIRQ(CDCIRQ_ACKNOWLEDGE);

   return(0);
}

static int32 Command_Demute(const int arg_count, const uint8 *args)
{
   Muted = false;

   WriteResult(CDC_MakeStatus(false));
   WriteIRQ(CDCIRQ_ACKNOWLEDGE);

   return(0);
}

static int32 Command_Setfilter(const int arg_count, const uint8 *args)
{
   FilterFile = args[0];
   FilterChan = args[1];

   //PSX_WARNING("[CDC] Setfilter: %02x %02x", args[0], args[1]);

   WriteResult(CDC_MakeStatus(false));
   WriteIRQ(CDCIRQ_ACKNOWLEDGE);

   return(0);
}

static int32 Command_Setmode(const int arg_count, const uint8 *args)
{
   Mode = args[0];

   WriteResult(CDC_MakeStatus(false));
   WriteIRQ(CDCIRQ_ACKNOWLEDGE);

   return(0);
}

static int32 Command_Getparam(const int arg_count, const uint8 *args)
{
   WriteResult(CDC_MakeStatus(false));
   WriteResult(Mode);
   WriteResult(0x00);
   WriteResult(FilterFile);
   WriteResult(FilterChan);

   WriteIRQ(CDCIRQ_ACKNOWLEDGE);


   return(0);
}

static int32 Command_GetlocL(const int arg_count, const uint8 *args)
{
   if(!CommandCheckDiscPresent())
      return(0);

   if(!HeaderBufValid)
   {
      WriteResult(CDC_MakeStatus(true));
      WriteResult(0x80);
      WriteIRQ(CDCIRQ_DISC_ERROR);
      return(0);
   }

   for(unsigned i = 0; i < 8; i++)
   {
      //printf("%d %d: %02x\n", DriveStatus, i, HeaderBuf[i]);
      WriteResult(HeaderBuf[i]);
   }

   WriteIRQ(CDCIRQ_ACKNOWLEDGE);

   return(0);
}

static int32 Command_GetlocP(const int arg_count, const uint8 *args)
{
   if(!CommandCheckDiscPresent())
      return(0);

   //printf("%2x:%2x %2x:%2x:%2x %2x:%2x:%2x\n", SubQBuf_Safe[0x1], SubQBuf_Safe[0x2], SubQBuf_Safe[0x3], SubQBuf_Safe[0x4], SubQBuf_Safe[0x5], SubQBuf_Safe[0x7], SubQBuf_Safe[0x8], SubQBuf_Safe[0x9]);

   WriteResult(SubQBuf_Safe[0x1]);	// Track
   WriteResult(SubQBuf_Safe[0x2]);	// Index
   WriteResult(SubQBuf_Safe[0x3]);	// R M
   WriteResult(SubQBuf_Safe[0x4]);	// R S
   WriteResult(SubQBuf_Safe[0x5]);	// R F
   WriteResult(SubQBuf_Safe[0x7]);	// A M
   WriteResult(SubQBuf_Safe[0x8]);	// A S
   WriteResult(SubQBuf_Safe[0x9]);	// A F

   WriteIRQ(CDCIRQ_ACKNOWLEDGE);

   return(0);
}

static int32 Command_ReadT(const int arg_count, const uint8 *args)
{
   WriteResult(CDC_MakeStatus(false));
   WriteIRQ(CDCIRQ_ACKNOWLEDGE);

   return(44100 * 768 / 1000);
}

static int32 Command_ReadT_Part2(void)
{
   WriteResult(CDC_MakeStatus(false));
   WriteIRQ(CDCIRQ_COMPLETE);

   return(0);
}

static int32 Command_GetTN(const int arg_count, const uint8 *args)
{
   if(!CommandCheckDiscPresent())
      return(0);

   WriteResult(CDC_MakeStatus(false));
   WriteResult(U8_to_BCD(toc.first_track));
   WriteResult(U8_to_BCD(toc.last_track));

   WriteIRQ(CDCIRQ_ACKNOWLEDGE);

   return(0);
}

static int32 Command_GetTD(const int arg_count, const uint8 *args)
{
   if(!CommandCheckDiscPresent())
      return(0);

   int track;
   uint8 m, s, f;

   if(!args[0] || args[0] == 0xAA)
      track = 100;
   else
   {
      track= BCD_to_U8(args[0]);

      if(track < toc.first_track || track > toc.last_track)	// Error
      {
         WriteResult(CDC_MakeStatus(true));
         WriteIRQ(CDCIRQ_ACKNOWLEDGE);
         return(0);
      }
   }

   LBA_to_AMSF(toc.tracks[track].lba, &m, &s, &f);

   WriteResult(CDC_MakeStatus(false));
   WriteResult(U8_to_BCD(m));
   WriteResult(U8_to_BCD(s));
   //WriteResult(U8_to_BCD(f));

   WriteIRQ(CDCIRQ_ACKNOWLEDGE);

   return(0);
}

static int32 Command_SeekL(const int arg_count, const uint8 *args)
{
   if(!CommandCheckDiscPresent())
      return(0);

   WriteResult(CDC_MakeStatus(false));
   WriteIRQ(CDCIRQ_ACKNOWLEDGE);

   SeekTarget = CommandLoc;

   PSRCounter = CDC_CalcSeekTime(CurSector, SeekTarget, DriveStatus != DS_STOPPED, DriveStatus == DS_PAUSED);
   HeaderBufValid = false;
   CDC_PreSeekHack(true, SeekTarget);
   DriveStatus = DS_SEEKING_LOGICAL;
   StatusAfterSeek = DS_STANDBY;
   CDC_ClearAIP();

   return(PSRCounter);
}

static int32 Command_SeekP(const int arg_count, const uint8 *args)
{
   if(!CommandCheckDiscPresent())
      return(0);

   WriteResult(CDC_MakeStatus(false));
   WriteIRQ(CDCIRQ_ACKNOWLEDGE);

   SeekTarget = CommandLoc;

   PSRCounter = CDC_CalcSeekTime(CurSector, SeekTarget, DriveStatus != DS_STOPPED, DriveStatus == DS_PAUSED);
   HeaderBufValid = false;
   CDC_PreSeekHack(false, SeekTarget);
   DriveStatus = DS_SEEKING;
   StatusAfterSeek = DS_STANDBY;
   CDC_ClearAIP();

   return(PSRCounter);
}

static int32 Command_Seek_PartN(void)
{
   if(DriveStatus == DS_STANDBY)
   {
      BeginResults();
      WriteResult(CDC_MakeStatus(false));
      WriteIRQ(CDCIRQ_COMPLETE);

      return(0);
   }

   return(std::max<int32>(PSRCounter, 256));
}

static int32 Command_Test(const int arg_count, const uint8 *args)
{
   //PSX_WARNING("[CDC] Test command sub-operation: 0x%02x", args[0]);

   switch(args[0])
   {
      default:
         //PSX_WARNING("[CDC] Unknown Test command sub-operation: 0x%02x", args[0]);
         WriteResult(CDC_MakeStatus(true));
         WriteResult(0x10);
         WriteIRQ(CDCIRQ_DISC_ERROR);
         break;

      case 0x00:
      case 0x01:
      case 0x02:
      case 0x03:
      case 0x10:
      case 0x11:
      case 0x12:
      case 0x13:
      case 0x14:
      case 0x15:
      case 0x16:
      case 0x17:
      case 0x18:
      case 0x19:
      case 0x1A:
         //PSX_WARNING("[CDC] Unknown Test command sub-operation: 0x%02x", args[0]);
         WriteResult(CDC_MakeStatus(false));
         WriteIRQ(CDCIRQ_ACKNOWLEDGE);
         break;

#if 0
      case 0x50:	// *Need to retest this test command, it takes additional arguments??? Or in any case, it generates a different error code(0x20) than most other Test
         // sub-commands that generate an error code(0x10).
         break;

         // Same with 0x60, 0x71-0x76

#endif

      case 0x51:	// *Need to retest this test command
         //PSX_WARNING("[CDC] Unknown Test command sub-operation: 0x%02x", args[0]);
         WriteResult(0x01);
         WriteResult(0x00);
         WriteResult(0x00);
         break;

      case 0x75:	// *Need to retest this test command
         //PSX_WARNING("[CDC] Unknown Test command sub-operation: 0x%02x", args[0]);
         WriteResult(0x00);
         WriteResult(0xC0);
         WriteResult(0x00);
         WriteResult(0x00);
         break;

         // 
         // SCEx counters not reset by command 0x0A.
         //

      case 0x04:	// Reset SCEx counters
         WriteResult(CDC_MakeStatus(false));
         WriteIRQ(CDCIRQ_ACKNOWLEDGE);
         break;

      case 0x05:	// Read SCEx counters
         WriteResult(0x00);	// Number of TOC/leadin reads? (apparently increases by 1 or 2 per ReadTOC, even on non-PSX music CD)
         WriteResult(0x00);	// Number of SCEx strings received? (Stays at zero on music CD)
         WriteIRQ(CDCIRQ_ACKNOWLEDGE);
         break;

      case 0x20:
         {
            WriteResult(0x97);
            WriteResult(0x01);
            WriteResult(0x10);
            WriteResult(0xC2);

            WriteIRQ(CDCIRQ_ACKNOWLEDGE);
         }
         break;

      case 0x21:	// *Need to retest this test command.
         {
            WriteResult(0x01);
            WriteIRQ(CDCIRQ_ACKNOWLEDGE);
         }
         break;

      case 0x22:
         {
            static const uint8 td[7] = { 0x66, 0x6f, 0x72, 0x20, 0x55, 0x2f, 0x43 };

            for(unsigned i = 0; i < 7; i++)
               WriteResult(td[i]);

            WriteIRQ(CDCIRQ_ACKNOWLEDGE);
         }
         break;

      case 0x23:
      case 0x24:
         {
            static const uint8 td[8] = { 0x43, 0x58, 0x44, 0x32, 0x35, 0x34, 0x35, 0x51 };

            for(unsigned i = 0; i < 8; i++)
               WriteResult(td[i]);

            WriteIRQ(CDCIRQ_ACKNOWLEDGE);
         }
         break;

      case 0x25:
         {
            static const uint8 td[8] = { 0x43, 0x58, 0x44, 0x31, 0x38, 0x31, 0x35, 0x51 };

            for(unsigned i = 0; i < 8; i++)
               WriteResult(td[i]);

            WriteIRQ(CDCIRQ_ACKNOWLEDGE);
         }
         break;
   }
   return(0);
}

static int32 Command_ID(const int arg_count, const uint8 *args)
{
   if(!CommandCheckDiscPresent())
      return(0);

   WriteResult(CDC_MakeStatus(false));
   WriteIRQ(CDCIRQ_ACKNOWLEDGE);

   return(33868);
}

static int32 Command_ID_Part2(void)
{
   if(IsPSXDisc)
   {
      WriteResult(CDC_MakeStatus(false));
      WriteResult(0x00);
      WriteResult(0x20);
      WriteResult(0x00);
   }
   else
   {
      WriteResult(CDC_MakeStatus(false) | 0x08);
      WriteResult(0x90);
      WriteResult(toc.disc_type);
      WriteResult(0x00);
   }

   if(IsPSXDisc)
   {
      WriteResult(DiscID[0]);
      WriteResult(DiscID[1]);
      WriteResult(DiscID[2]);
      WriteResult(DiscID[3]);
   }
   else
   {
      WriteResult(0xff);
      WriteResult(0);
      WriteResult(0);
      WriteResult(0);
   }

   if(IsPSXDisc)
   {
      WriteIRQ(CDCIRQ_COMPLETE);
   }
   else
   {
      WriteIRQ(CDCIRQ_DISC_ERROR);
   }

   return(0);
}

static int32 Command_Init(const int arg_count, const uint8 *args)
{
   return(0);
}

static int32 Command_ReadTOC(const int arg_count, const uint8 *args)
{
   int32 ret_time;

   HeaderBufValid = false;
   WriteResult(CDC_MakeStatus(false));
   WriteIRQ(CDCIRQ_ACKNOWLEDGE);

   // ReadTOC doesn't error out if the tray is open, and it completes rather quickly in that case.
   //
   if(!CommandCheckDiscPresent())
      return(26000);



   // A gross approximation.  
   // The penalty for the drive being stopped seems to be rather high(higher than what CDC_CalcSeekTime() currently introduces), although
   // that should be investigated further.
   //
   // ...and not to mention the time taken varies from disc to disc even!
   ret_time = 30000000 + CDC_CalcSeekTime(CurSector, 0, DriveStatus != DS_STOPPED, DriveStatus == DS_PAUSED);

   DriveStatus = DS_PAUSED;	// Ends up in a pause state when the command is finished.  Maybe we should add DS_READTOC or something...
   CDC_ClearAIP();

   return ret_time;
}

static int32 Command_ReadTOC_Part2(void)
{
   //if(!CommandCheckDiscPresent())
   // DriveStatus = DS_PAUSED;

   WriteResult(CDC_MakeStatus(false));
   WriteIRQ(CDCIRQ_COMPLETE);

   return(0);
}

static int32 Command_0x1d(const int arg_count, const uint8 *args)
{
   WriteResult(CDC_MakeStatus(false));
   WriteIRQ(CDCIRQ_ACKNOWLEDGE);
   return(0);
}


#ifndef __MDFN_PSX_CDC_H
#define __MDFN_PSX_CDC_H

#include "../cdrom/cdromif.h"
#include "../cdrom/SimpleFIFO.h"
#include "../clamp.h"

struct CD_Audio_Buffer
{
 int16 Samples[2][0x1000];	// [0][...] = l, [1][...] = r
 int32 Size;
 uint32 Freq;
 int32 ReadPos;
};

enum
{
   MODE_SPEED     = 0x80,
   MODE_STRSND    = 0x40,
   MODE_SIZE      = 0x20,
   MODE_SIZE2     = 0x10,
   MODE_SF        = 0x08,
   MODE_REPORT    = 0x04,
   MODE_AUTOPAUSE = 0x02,
   MODE_CDDA      = 0x01,
};

enum
{
   DS_STANDBY = -2,
   DS_PAUSED = -1,
   DS_STOPPED = 0,
   DS_SEEKING,
   DS_SEEKING_LOGICAL,
   DS_PLAY_SEEKING,
   DS_PLAYING,
   DS_READING,
   DS_RESETTING
};

enum
{
   SectorPipe_Count = 2
};

enum
{
   CDCIRQ_NONE = 0,
   CDCIRQ_DATA_READY = 1,
   CDCIRQ_COMPLETE = 2,
   CDCIRQ_ACKNOWLEDGE = 3,
   CDCIRQ_DATA_END = 4,
   CDCIRQ_DISC_ERROR = 5
};

// Names are just guessed for these based on what conditions cause them:
enum
{
   ERRCODE_BAD_ARGVAL  = 0x10,
   ERRCODE_BAD_NUMARGS = 0x20,
   ERRCODE_BAD_COMMAND = 0x40,
   ERRCODE_NOT_READY = 0x80,	// 0x80 (happens with getlocl when drive isn't reading, pause when tray is open, and MAYBE when trying to run an async
   //	 command while another async command is currently in its asynch phase being executed[pause when in readtoc, todo test more])
};

void CDC_New();
void CDC_Free();

void CDC_SetDisc(bool tray_open, CDIF *cdif, const char disc_id[4]);

void CDC_ResetTS(void);

void CDC_Power(void);
int CDC_StateAction(StateMem *sm, int load, int data_only);

int32 CDC_CalcNextEvent(void);	// Returns in master cycles to next event.

int32_t CDC_Update(const int32_t timestamp);

void CDC_Write(const int32_t timestamp, uint32 A, uint8 V);

uint8 CDC_Read(const int32_t timestamp, uint32 A);

bool CDC_DMACanRead(void);

void CDC_SoftReset(void);

void CDC_GetCDAudio(int32 samples[2]);

#ifdef __cplusplus
extern "C" {
#endif

uint32 CDC_DMARead(void);

#ifdef __cplusplus
}
#endif

#endif

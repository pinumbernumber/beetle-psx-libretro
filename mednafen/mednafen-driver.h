#ifndef __MDFN_MEDNAFEN_DRIVER_H
#define __MDFN_MEDNAFEN_DRIVER_H

#include <stdio.h>
#include <vector>
#include <string>

#include "settings-common.h"

extern std::vector<MDFNGI *>MDFNSystems;

uint32 MDFND_GetTime(void);
void MDFND_Sleep(uint32 ms);

#ifdef WANT_THREADING
/* Being threading support. */
// Mostly based off SDL's prototypes and semantics.
// Driver code should actually define MDFN_Thread and MDFN_Mutex.

struct MDFN_Thread;
struct MDFN_Mutex;
struct MDFN_Cond;

MDFN_Thread *MDFND_CreateThread(int (*fn)(void *), void *data);
void MDFND_WaitThread(MDFN_Thread *thread, int *status);
void MDFND_KillThread(MDFN_Thread *thread);

MDFN_Mutex *MDFND_CreateMutex(void);
MDFN_Cond *MDFND_CreateCond(void);
void MDFND_DestroyCond(MDFN_Cond *cond);
void MDFND_DestroyMutex(MDFN_Mutex *mutex);
int MDFND_WaitCond(MDFN_Cond *cond, MDFN_Mutex *mutex);
int MDFND_SignalCond(MDFN_Cond *cond);
int MDFND_LockMutex(MDFN_Mutex *mutex);
int MDFND_UnlockMutex(MDFN_Mutex *mutex);

/* End threading support. */
#endif

MDFNGI *MDFNI_LoadCD(const char *sysname, const char *devicename);

// Call this function as early as possible, even before MDFNI_Initialize()
bool MDFNI_InitializeModule(void);

/* Sets the base directory(save states, snapshots, etc. are saved in directories
   below this directory. */
void MDFNI_SetBaseDirectory(const char *dir);

/* Closes currently loaded game */
void MDFNI_CloseGame(void);

void MDFN_DispMessage(const char *format, ...);
#define MDFNI_DispMessage MDFN_DispMessage

uint32 MDFNI_CRC32(uint32 crc, uint8 *buf, uint32 len);

// NES hackish function.  Should abstract in the future.
int MDFNI_DatachSet(const uint8 *rcode);

#endif

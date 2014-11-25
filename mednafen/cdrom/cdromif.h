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

#ifndef __MDFN_CDROM_CDROMIF_H
#define __MDFN_CDROM_CDROMIF_H

#include "CDUtility.h"
#include "CDAccess.h"
#include "../Stream.h"

#include <queue>

typedef CDUtility::TOC CD_TOC;

typedef struct CDInterface
{
   bool UnrecoverableError;
   CDUtility::TOC disc_toc;
   bool DiscEjected;
   CDAccess *disc_cdaccess;
} CDIF;

CDIF *CDIF_New(CDAccess *cda);

void CDIF_Free(CDIF *cdif);

static inline void CDIF_ReadTOC(CDIF *cdif, CDUtility::TOC *read_target)
{
   *read_target = cdif->disc_toc;
}

bool CDIF_ReadRawSector(CDIF *cdif, uint8 *buf, uint32 lba);

// Call for mode 1 or mode 2 form 1 only.
bool CDIF_ValidateRawSector(CDIF *cdif, uint8 *buf);

// Utility/Wrapped functions
// Reads mode 1 and mode2 form 1 sectors(2048 bytes per sector returned)
// Will return the type(1, 2) of the first sector read to the buffer supplied, 0 on error
int CDIF_ReadSector(CDIF *cdif, uint8* pBuf, uint32 lba, uint32 nSectors);

// Return true if operation succeeded or it was a NOP(either due to not being implemented, or the current status matches eject_status).
// Returns false on failure(usually drive error of some kind; not completely fatal, can try again).
bool CDIF_Eject(CDIF *cdif, bool eject_status);

// For Mode 1, or Mode 2 Form 1.
// No reference counting or whatever is done, so if you destroy the CDIF object before you destroy the returned Stream, things will go BOOM.
void *CDIF_MakeStream(CDIF *cdif, uint32 lba, uint32 sector_count);

CDIF *CDIF_Open(const char *path, const bool is_device, bool image_memcache);

#endif

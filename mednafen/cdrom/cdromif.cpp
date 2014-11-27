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

#include "../mednafen.h"
#include "../error.h"
#include <string.h>
#include <sys/types.h>
#include "cdromif.h"
#include "../general.h"

#include <algorithm>
#include "../../libretro.h"

extern retro_log_printf_t log_cb;

using namespace CDUtility;

enum
{
 // Status/Error messages
 CDIF_MSG_DONE = 0,		// Read -> emu. args: No args.
 CDIF_MSG_INFO,			// Read -> emu. args: str_message
 CDIF_MSG_FATAL_ERROR,		// Read -> emu. args: *TODO ARGS*

 //
 // Command messages.
 //
 CDIF_MSG_DIEDIEDIE,		// Emu -> read

 CDIF_MSG_READ_SECTOR,		/* Emu -> read
					args[0] = lba
				*/

 CDIF_MSG_EJECT,		// Emu -> read, args[0]; 0=insert, 1=eject
};

CDIF *CDIF_New(CDAccess *cda)
{
   CDIF *cdif = (CDIF*)calloc(1, sizeof(CDIF));

   if (!cdif)
      return NULL;

   //puts("***WARNING USING SINGLE-THREADED CD READER***");

   cdif->UnrecoverableError = false;
   cdif->DiscEjected = false;

   cdif->disc_cdaccess = cda;
   cdif->disc_cdaccess->Read_TOC(&cdif->disc_toc);

   if(cdif->disc_toc.first_track < 1 || cdif->disc_toc.last_track > 99 || cdif->disc_toc.first_track > cdif->disc_toc.last_track)
   {
      log_cb(RETRO_LOG_ERROR, "TOC first(%d)/last(%d) track numbers bad.\n", cdif->disc_toc.first_track, cdif->disc_toc.last_track);
      return NULL;
   }

   return cdif;
}

void CDIF_Free(CDIF *cdif)
{
   if(cdif->disc_cdaccess)
      delete cdif->disc_cdaccess;
   cdif->disc_cdaccess = NULL;

   free(cdif);
}

bool CDIF_ValidateRawSector(CDIF *cdif, uint8 *buf)
{
   int mode = buf[12 + 3];

   if(mode != 0x1 && mode != 0x2)
      return(false);

   if(!edc_lec_check_and_correct(buf, mode == 2))
      return(false);

   return(true);
}

int CDIF_ReadSector(CDIF *cdif, uint8* pBuf, uint32 lba, uint32 nSectors)
{
   int ret = 0;

   if(cdif->UnrecoverableError)
      return(false);

   while(nSectors--)
   {
      uint8 tmpbuf[2352 + 96];

      if(!CDIF_ReadRawSector(cdif, tmpbuf, lba))
      {
         puts("CDIF Raw Read error");
         return(FALSE);
      }

      if(!CDIF_ValidateRawSector(cdif, tmpbuf))
      {
         if (log_cb)
         {
            log_cb(RETRO_LOG_ERROR, "Uncorrectable data at sector %d\n", lba);
            log_cb(RETRO_LOG_ERROR, "Uncorrectable data at sector %d\n", lba);
         }
         return(false);
      }

      const int mode = tmpbuf[12 + 3];

      if(!ret)
         ret = mode;

      if(mode == 1)
      {
         memcpy(pBuf, &tmpbuf[12 + 4], 2048);
      }
      else if(mode == 2)
      {
         memcpy(pBuf, &tmpbuf[12 + 4 + 8], 2048);
      }
      else
      {
         printf("CDIF_ReadSector() invalid sector type at LBA=%u\n", (unsigned int)lba);
         return(false);
      }

      pBuf += 2048;
      lba++;
   }

   return(ret);
}

bool CDIF_ReadRawSector(CDIF *cdif, uint8 *buf, uint32 lba)
{
   if(cdif->UnrecoverableError)
   {
      memset(buf, 0, 2352 + 96);
      return(false);
   }

   cdif->disc_cdaccess->Read_Raw_Sector(buf, lba);

   return(true);
}

bool CDIF_Eject(CDIF *cdif, bool eject_status)
{
   if(cdif->UnrecoverableError)
      return false;

   int32 old_de = cdif->DiscEjected;
   cdif->DiscEjected = eject_status;

   if(old_de != cdif->DiscEjected)
   {
      cdif->disc_cdaccess->Eject(eject_status);

      if(!eject_status)     // Re-read the TOC
      {
         cdif->disc_cdaccess->Read_TOC(&cdif->disc_toc);

         if(cdif->disc_toc.first_track < 1 || cdif->disc_toc.last_track > 99 || cdif->disc_toc.first_track > cdif->disc_toc.last_track)
         {
            log_cb(RETRO_LOG_ERROR, "TOC first(%d)/last(%d) track numbers bad.\n", cdif->disc_toc.first_track, cdif->disc_toc.last_track);
            return false;
         }
      }
   }

   return true;
}


class CDIF_Stream_Thing : public Stream
{
 public:

 CDIF_Stream_Thing(CDIF *cdintf_arg, uint32 lba_arg, uint32 sector_count_arg);
 ~CDIF_Stream_Thing();

 virtual uint64 read(void *data, uint64 count, bool error_on_eos = true);
 virtual void write(const void *data, uint64 count);

 virtual void seek(int64 offset, int whence);
 virtual int64 tell(void);
 virtual int64 size(void);
 virtual void close(void);

 private:
 CDIF *cdintf;
 const uint32 start_lba;
 const uint32 sector_count;
 int64 position;
};

CDIF_Stream_Thing::CDIF_Stream_Thing(CDIF *cdintf_arg, uint32 start_lba_arg, uint32 sector_count_arg) : cdintf(cdintf_arg), start_lba(start_lba_arg), sector_count(sector_count_arg)
{

}

CDIF_Stream_Thing::~CDIF_Stream_Thing()
{

}

uint64 CDIF_Stream_Thing::read(void *data, uint64 count, bool error_on_eos)
{
   if(count > (((uint64)sector_count * 2048) - position))
   {
      if(error_on_eos)
      {
         log_cb(RETRO_LOG_ERROR, "EOF.\n");
         return 0;
      }

      count = ((uint64)sector_count * 2048) - position;
   }

   if(!count)
      return 0;

   for(uint64 rp = position; rp < (position + count); rp = (rp &~ 2047) + 2048)
   {
      uint8 buf[2048];  

      if(!CDIF_ReadSector(cdintf, buf, start_lba + (rp / 2048), 1))
         return 0;

      //::printf("Meow: %08llx -- %08llx\n", count, (rp - position) + std::min<uint64>(2048 - (rp & 2047), count - (rp - position)));
      memcpy((uint8*)data + (rp - position), buf + (rp & 2047), std::min<uint64>(2048 - (rp & 2047), count - (rp - position)));
   }

   position += count;

   return count;
}

void CDIF_Stream_Thing::write(const void *data, uint64 count)
{
}

void CDIF_Stream_Thing::seek(int64 offset, int whence)
{
   int64 new_position;

   switch(whence)
   {
      case SEEK_SET:
         new_position = offset;
         break;

      case SEEK_CUR:
         new_position = position + offset;
         break;

      case SEEK_END:
         new_position = ((int64)sector_count * 2048) + offset;
         break;
   }

   if(new_position < 0 || new_position > ((int64)sector_count * 2048))
   {
      log_cb(RETRO_LOG_ERROR, "EINVAL.\n");
      return;
   }

   position = new_position;
}

int64 CDIF_Stream_Thing::tell(void)
{
 return position;
}

int64 CDIF_Stream_Thing::size(void)
{
 return(sector_count * 2048);
}

void CDIF_Stream_Thing::close(void)
{

}

void *CDIF_MakeStream(CDIF *cdif, uint32 lba, uint32 sector_count)
{
 return new CDIF_Stream_Thing(cdif, lba, sector_count);
}


CDIF *CDIF_Open(const char *path, const bool is_device, bool image_memcache)
{
   return CDIF_New(cdaccess_open_image(path, image_memcache)); 
}

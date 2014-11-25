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

#include "mednafen.h"
#include <stdarg.h>
#include <string.h>
#include <errno.h>

#include "file.h"
#include "general.h"

// This function should ALWAYS close the system file "descriptor"(gzip library, zip library, or FILE *) it's given,
// even if it errors out.
bool MDFNFILE::MakeMemWrapAndClose(void *fp)
{
   bool ret = FALSE;

   location = 0;

   fseek((FILE *)fp, 0, SEEK_END);
   f_size = ftell((FILE *)fp);
   fseek((FILE *)fp, 0, SEEK_SET);

   if (!(f_data = (uint8*)MDFN_malloc(f_size, "file read buffer")))
      goto fail;
   fread(f_data, 1, f_size, (FILE *)fp);

   ret = TRUE;
fail:
   fclose((FILE*)fp);
   return ret;
}

MDFNFILE::MDFNFILE()
{
   f_data = NULL;
   f_size = 0;
   f_ext = NULL;

   location = 0;
}

MDFNFILE::MDFNFILE(const char *path, const void *known_ext, const char *purpose)
{
   (void)known_ext;
   if (!Open(path, known_ext, purpose, false))
      assert(false);
}


MDFNFILE::~MDFNFILE()
{
   Close();
}


bool MDFNFILE::Open(const char *path, const void *known_ext,
      const char *purpose, const bool suppress_notfound_pe)
{
   FILE *fp;
   (void)known_ext;

   if (!(fp = fopen(path, "rb")))
      return FALSE;

   ::fseek(fp, 0, SEEK_SET);

   if (!MakeMemWrapAndClose(fp))
      return FALSE;

   const char *ld = (const char*)strrchr(path, '.');
   f_ext = strdup(ld ? ld + 1 : "");

   return(TRUE);
}

bool MDFNFILE::Close(void)
{
   if (f_ext)
      free(f_ext);
   f_ext = 0;

   if (f_data)
      free(f_data);
   f_data = 0;

   return(1);
}

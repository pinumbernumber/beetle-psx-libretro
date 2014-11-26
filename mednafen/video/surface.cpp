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
#include "surface.h"

void *MDFN_Surface_New(void *const p_pixels, const uint32 p_width, const uint32 p_height, const uint32 p_pitchinpix, const MDFN_PixelFormat &nf)
{
   MDFN_Surface *surf = (MDFN_Surface*)calloc(1, sizeof(MDFN_Surface));

   if (!surf)
      return NULL;

   memset(&surf->format, 0, sizeof(surf->format));

   surf->pixels     = NULL;
   surf->pitchinpix = 0;
   surf->w          = 0;
   surf->h          = 0;
   surf->format     = nf;

   surf->pixels = (uint32*)calloc(1, p_pitchinpix * p_height * 4);

   if (!surf->pixels)
      return NULL;

   surf->w = p_width;
   surf->h = p_height;
   surf->pitchinpix = p_pitchinpix;

   return surf;
}

void MDFN_Surface_Free(MDFN_Surface *surf)
{
   if (!surf)
      return;

   if(surf->pixels)
      free(surf->pixels);

   free(surf);
}

#include <stdlib.h>
#include <string.h>
#include "../mednafen-types.h"
#include "../video.h"

#include "Deinterlacer.h"

static MDFN_Surface *FieldBuffer;
static int32 *LWBuffer;
static bool StateValid;
static MDFN_Rect PrevDRect;

void Deinterlacer_New()
{
   FieldBuffer = NULL;
   StateValid = false;
   PrevDRect.x = 0;
   PrevDRect.y = 0;

   PrevDRect.w = 0;
   PrevDRect.h = 0;
}

void Deinterlacer_Free()
{
   if (LWBuffer)
      free(LWBuffer);
   MDFN_Surface_Free(FieldBuffer);
   FieldBuffer = NULL;
}

void Deinterlacer_Process(MDFN_Surface *surface, MDFN_Rect *DisplayRect, int32 *LineWidths, const bool field)
{
   int y;
   const MDFN_Rect DisplayRect_Original = *DisplayRect;

   if(!FieldBuffer || FieldBuffer->w < surface->w || FieldBuffer->h < (surface->h / 2))
   {
      Deinterlacer_Free();
      FieldBuffer = (MDFN_Surface*)MDFN_Surface_New(NULL, surface->w, surface->h / 2, surface->w);
      LWBuffer = (int32*)malloc(FieldBuffer->h * sizeof(int32));
   }

   //
   // We need to output with LineWidths as always being valid to handle the case of horizontal resolution change between fields
   // while in interlace mode, so clear the first LineWidths entry if it's == ~0, and
   // [...]
   const bool LineWidths_In_Valid = (LineWidths[0] != ~0);
   const bool WeaveGood = (StateValid && PrevDRect.h == DisplayRect->h);
   //
   // XReposition stuff is to prevent exceeding the dimensions of the video surface under certain conditions(weave deinterlacer, previous field has higher
   // horizontal resolution than current field, and current field's rectangle has an x offset that's too large when taking into consideration the previous field's
   // width; for simplicity, we don't check widths, but just assume that the previous field's maximum width is >= than the current field's maximum width).
   //
   const int32 XReposition = ((WeaveGood && DisplayRect->x > PrevDRect.x) ? DisplayRect->x : 0);

   //printf("%2d %2d, %d\n", DisplayRect->x, PrevDRect.x, XReposition);

   if(XReposition)
      DisplayRect->x = 0;

   if(surface->h && !LineWidths_In_Valid)
      LineWidths[0] = 0;

   for(y = 0; y < DisplayRect->h / 2; y++)
   {
      // [...]
      // set all relevant source line widths to the contents of DisplayRect(also simplifies the src_lw and related pointer calculation code
      // farther below.
      if(!LineWidths_In_Valid)
         LineWidths[(y * 2) + field + DisplayRect->y] = DisplayRect->w;

      if(XReposition)
      {
         memmove(surface->pixels + ((y * 2) + field + DisplayRect->y) * surface->pitchinpix,
               surface->pixels + ((y * 2) + field + DisplayRect->y) * surface->pitchinpix + XReposition,
               LineWidths[(y * 2) + field + DisplayRect->y] * sizeof(uint32));
      }

      if(WeaveGood)
      {
         const uint32* src = FieldBuffer->pixels + y * FieldBuffer->pitchinpix;
         uint32* dest = surface->pixels + ((y * 2) + (field ^ 1) + DisplayRect->y) * surface->pitchinpix + DisplayRect->x;
         int32 *dest_lw = &LineWidths[(y * 2) + (field ^ 1) + DisplayRect->y];

         *dest_lw = LWBuffer[y];

         memcpy(dest, src, LWBuffer[y] * sizeof(uint32));
      }
      else
      {
         const int32 *src_lw = &LineWidths[(y * 2) + field + DisplayRect->y];
         const uint32* src = surface->pixels + ((y * 2) + field + DisplayRect->y) * surface->pitchinpix + DisplayRect->x;
         const int32 dly = ((y * 2) + (field + 1) + DisplayRect->y);
         uint32* dest = surface->pixels + dly * surface->pitchinpix + DisplayRect->x;

         if(y == 0 && field)
         {
            int x;
            uint32 black = MAKECOLOR(0, 0, 0, 0);
            uint32* dm2 = surface->pixels + (dly - 2) * surface->pitchinpix;

            LineWidths[dly - 2] = *src_lw;

            for(x = 0; x < *src_lw; x++)
               dm2[x] = black;
         }

         if(dly < (DisplayRect->y + DisplayRect->h))
         {
            LineWidths[dly] = *src_lw;
            memcpy(dest, src, *src_lw * sizeof(uint32));
         }
      }

      const int32 *src_lw = &LineWidths[(y * 2) + field + DisplayRect->y];
      const uint32* src = surface->pixels + ((y * 2) + field + DisplayRect->y) * surface->pitchinpix + DisplayRect->x;
      uint32* dest = FieldBuffer->pixels + y * FieldBuffer->pitchinpix;

      memcpy(dest, src, *src_lw * sizeof(uint32));
      LWBuffer[y] = *src_lw;

      StateValid = true;
   }

   PrevDRect = DisplayRect_Original;
}

void Deinterlacer_ClearState(void)
{
   StateValid = false;

   PrevDRect.x = 0;
   PrevDRect.y = 0;

   PrevDRect.w = 0;
   PrevDRect.h = 0;
}

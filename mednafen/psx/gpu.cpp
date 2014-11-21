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
#include "timer.h"

/*
 TODO:
	Test and clean up line, particularly polyline, drawing.

	"abe" transparency testing might be not correct, the transparency in regards to mask bit setting and evaluation may not be correct.

	Not everything is returned in the status port read yet(double check).

	"dfe" bit of drawing mode is probably not implemented 100% correctly.

	Initialize more stuff in the Power() function.

	Fix triangle span rendering order(it's bottom-to-up sometimes on the real thing, to avoid negative x step/increment values).
*/

/*
 GPU display timing master clock is nominally 53.693182 MHz for NTSC PlayStations, and 53.203425 MHz for PAL PlayStations.

 Non-interlaced NTSC mode line timing notes(real-world times calculated via PS1 timer and math with nominal CPU clock value):

	263 lines per frame

	~16714.85 us per frame, average.
	~63.55456 us per line, average.

	Multiplying the results of counter 0 in pixel clock mode by the clock divider of the current dot clock mode/width gives a result that's slightly less
	than expected; the dot clock divider is probably being reset each scanline.

 Non-interlaced PAL mode(but with an NTSC source clock in an NTSC PS1; calculated same way as NTSC values):

	314 lines per frame

	~19912.27 us per frame, average.
	~63.41486 us per line, average.

 FB X and Y display positions can be changed during active display; and Y display position appears to be treated as an offset to the current Y readout
 position that gets reset around vblank time.

*/

/*
 Known problematic games to do regression testing on:

	Dukes of Hazzard: Racing For Home
		Sensitive about the GPU busy status flag being set long enough; double-check if we ever make CPU emulation more timing-accurate(
		the fix will likely just involve reducing the timing granularity for DMA and GPU updates).

	Final Fantasy 7
		WHERE DO I BEGIN?!  (Currently broken as of Jan. 1, 2012)

	Pro Pinball (series)
		Sensitive to correct interlace and draw line skipping emulation.

	Valkyrie Profile
		Battle scenes will go all kaka with no graphics updates if GPU LL DMA completes too soon.

*/

/*
 November 29, 2012 notes:

  PAL mode can be turned on, and then off again, mid-frame(creates a neat effect).

  Pixel clock can be changed mid-frame with effect(the effect is either instantaneous, or cached at some point in the scanline, not tested to see which);
  interestingly, alignment is off on a PS1 when going 5MHz->10MHz>5MHz with a grid image.

  Vertical start and end can be changed during active display, with effect(though it needs to be vs0->ve0->vs1->ve1->..., vs0->vs1->ve0 doesn't apparently do anything 
  different from vs0->ve0.
*/
static const int32_t dither_table[4][4] =
{
 { -4,  0, -3,  1 },
 {  2, -2,  3, -1 },
 { -3,  1, -4,  0 },
 {  3, -1,  2, -2 },
};

uint8_t DitherLUT[4][4][512];	// Y, X, 8-bit source value(256 extra for saturation)

void PSXDitherApply(bool enable)
{
   int x, y, v;

   for(y = 0; y < 4; y++)
      for(x = 0; x < 4; x++)
         for(v = 0; v < 512; v++)
         {
            int value = v;
            if (enable)
               value += dither_table[y][x];

            value >>= 3;

            if(value < 0)
               value = 0;

            if(value > 0x1F)
               value = 0x1F;

            DitherLUT[y][x][v] = value;
         }
}

namespace MDFN_IEN_PSX
{

   // 0x10 on actual PS1 GPU, 0x20 here(see comment at top of gpu.h)	// 0x10)
PS_GPU::PS_GPU(bool pal_clock_and_tv, int sls, int sle) : BlitterFIFO(0x20)
{
   int x, y, v;
   HardwarePALType = pal_clock_and_tv;

   for(y = 0; y < 4; y++)
      for(x = 0; x < 4; x++)
         for(v = 0; v < 512; v++)
         {
            int value = v + dither_table[y][x];

            value >>= 3;

            if(value < 0)
               value = 0;

            if(value > 0x1F)
               value = 0x1F;

            DitherLUT[y][x][v] = value;
         }

   if(HardwarePALType == false)	// NTSC clock
      GPUClockRatio = 103896; // 65536 * 53693181.818 / (44100 * 768)
   else	// PAL clock
      GPUClockRatio = 102948; // 65536 * 53203425 / (44100 * 768)

   memset(RGB8SAT_Under, 0, sizeof(RGB8SAT_Under));

   for(int i = 0; i < 256; i++)
      RGB8SAT[i] = i;

   memset(RGB8SAT_Over, 0xFF, sizeof(RGB8SAT_Over));

   LineVisFirst = sls;
   LineVisLast = sle;
}

PS_GPU::~PS_GPU()
{

}

void PS_GPU::FillVideoParams(MDFNGI* gi)
{
   if(HardwarePALType)
   {
      gi->lcm_width = 2800;
      gi->lcm_height = (LineVisLast + 1 - LineVisFirst) * 2; //576;

      gi->nominal_width = 377;	// Dunno. :(
      gi->nominal_height = LineVisLast + 1 - LineVisFirst; //288;

      gi->fb_width = 768;
      gi->fb_height = 576;

      gi->fps = 836203078; // 49.842

      gi->VideoSystem = VIDSYS_PAL;
   }
   else
   {
      gi->lcm_width = 2800;
      gi->lcm_height = (LineVisLast + 1 - LineVisFirst) * 2; //480;

      gi->nominal_width = 320;
      gi->nominal_height = LineVisLast + 1 - LineVisFirst; //240;

      gi->fb_width = 768;
      gi->fb_height = 480;

      gi->fps = 1005643085; // 59.941

      gi->VideoSystem = VIDSYS_NTSC;
   }

   //
   // For Justifier and Guncon.
   //
   gi->mouse_scale_x = (float)gi->lcm_width / gi->nominal_width;
   gi->mouse_offs_x = 0;

   gi->mouse_scale_y = 1.0;
   gi->mouse_offs_y = LineVisFirst;
}

void PS_GPU::SoftReset(void) // Control command 0x00
{
   IRQPending = false;
   IRQ_Assert(IRQ_GPU, IRQPending);
   DMAControl = 0;

   if(DrawTimeAvail < 0)
      DrawTimeAvail = 0;

   BlitterFIFO.Flush();
   InCmd = INCMD_NONE;

   DisplayOff = 1;
   DisplayFB_XStart = 0;
   DisplayFB_YStart = 0;

   if(HardwarePALType)
   {
      DisplayMode = 0x08;

      // FIXME, timing values(I need a PAL PS1 to derive them); for now, just copy the NTSC ones.
      HorizStart = 0x200;
      HorizEnd = 0xC00;

      VertStart = 0x10;
      VertEnd = 0x100;
   }
   else
   {
      DisplayMode = 0;

      HorizStart = 0x200;
      HorizEnd = 0xC00;

      VertStart = 0x10;
      VertEnd = 0x100;
   }

   //
   TexPageX = 0;
   TexPageY = 0;

   SpriteFlip = 0;

   abr = 0;
   TexMode = 0;

   dtd = 0;
   dfe = 0;

   //
   tww = 0; 
   twh = 0; 
   twx = 0;
   twy = 0;

   RecalcTexWindowLUT();

   //
   ClipX0 = 0;
   ClipY0 = 0;

   //
   ClipX1 = 0;
   ClipY1 = 0;

   //
   OffsX = 0;
   OffsY = 0;

   //
   MaskSetOR = 0;
   MaskEvalAND = 0;
}

void PS_GPU::Power(void)
{
   memset(GPURAM, 0, sizeof(GPURAM));

   DMAControl = 0;

   ClipX0 = 0;
   ClipY0 = 0;
   ClipX1 = 1023;
   ClipY1 = 1023;

   OffsX = 0;
   OffsY = 0;

   dtd = false;
   dfe = false;

   MaskSetOR = 0;
   MaskEvalAND = 0;

   tww = 0;
   twh = 0;
   twx = 0;
   twy = 0;

   RecalcTexWindowLUT();

   TexPageX = 0;
   TexPageY = 0;
   SpriteFlip = 0;

   abr = 0;
   TexMode = 0;

   BlitterFIFO.Flush();

   InCmd = INCMD_NONE;
   FBRW_X = 0;
   FBRW_Y = 0;
   FBRW_W = 0;
   FBRW_H = 0;
   FBRW_CurY = 0;
   FBRW_CurX = 0;

   DisplayMode = 0;
   DisplayOff = 1;
   DisplayFB_XStart = 0;
   DisplayFB_YStart = 0;

   HorizStart = 0;
   HorizEnd = 0;

   VertStart = 0;
   VertEnd = 0;

   //
   //
   //
   DisplayFB_CurYOffset = 0;
   DisplayFB_CurLineYReadout = 0;
   InVBlank = true;

   // TODO: factor out in a separate function.
   LinesPerField = 263;

   //
   //
   //
   scanline = 0;
   field = 0;
   field_ram_readout = 0;
   PhaseChange = 0;

   //
   //
   //
   DotClockCounter = 0;
   GPUClockCounter = 0;
   LineClockCounter = 3412 - 200;
   LinePhase = 0;

   DrawTimeAvail = 0;

   lastts = 0;

   SoftReset();

   IRQ_Assert(IRQ_VBLANK, InVBlank);
   TIMER_SetVBlank(InVBlank);
}

void PS_GPU::ResetTS(void)
{
   lastts = 0;
}

template<int BlendMode, bool MaskEval_TA, bool textured>
INLINE void PS_GPU::PlotPixel(int32 x, int32 y, uint16_t fore_pix)
{
   y &= 511;	// More Y precision bits than GPU RAM installed in (non-arcade, at least) Playstation hardware.
   uint16_t pix = fore_pix;

   if(BlendMode >= 0 && (fore_pix & 0x8000))
   {
      uint16_t bg_pix = GPURAM[y][x];	// Don't use bg_pix for mask evaluation, it's modified in blending code paths.
      pix = 0;

      /*
         static const int32 tab[4][2] =
         {
         { 2,  2 },
         { 4,  4 },
         { 4, -4 },
         { 4,  1 }
         };
         */
      // Efficient 15bpp pixel math algorithms from blargg
      switch(BlendMode)
      {
         case 0:
            bg_pix |= 0x8000;
            pix = ((fore_pix + bg_pix) - ((fore_pix ^ bg_pix) & 0x0421)) >> 1;
            break;

         case 1:
         case 3:
            {
               uint32_t sum, carry;
               bg_pix &= ~0x8000;
               if (BlendMode == 3)
                  fore_pix = ((fore_pix >> 2) & 0x1CE7) | 0x8000;

               sum = fore_pix + bg_pix;
               carry = (sum - ((fore_pix ^ bg_pix) & 0x8421)) & 0x8420;

               pix = (sum - carry) | (carry - (carry >> 5));
            }
            break;

         case 2:
            {
               uint32_t diff, borrow;
               bg_pix |= 0x8000;
               fore_pix &= ~0x8000;

               diff = bg_pix - fore_pix + 0x108420;
               borrow = (diff - ((bg_pix ^ fore_pix) & 0x108420)) & 0x108420;

               pix = (diff - borrow) & (borrow - (borrow >> 5));
            }
            break;
      }
   }

   if(!MaskEval_TA || !(GPURAM[y][x] & 0x8000))
      GPURAM[y][x] = (textured ? pix : (pix & 0x7FFF)) | MaskSetOR;
}

INLINE uint16_t PS_GPU::ModTexel(uint16_t texel, int32 r, int32 g, int32 b, const int32 dither_x, const int32 dither_y)
{
   uint16_t ret = texel & 0x8000;

   ret |= DitherLUT[dither_y][dither_x][(((texel & 0x1F) * r) >> (5 - 1))] << 0;
   ret |= DitherLUT[dither_y][dither_x][(((texel & 0x3E0) * g) >> (10 - 1))] << 5;
   ret |= DitherLUT[dither_y][dither_x][(((texel & 0x7C00) * b) >> (15 - 1))] << 10;

   return(ret);
}

template<uint32_t TexMode_TA>
INLINE uint16_t PS_GPU::GetTexel(const uint32_t clut_offset, int32 u_arg, int32 v_arg)
{
   uint16_t fbw;
   uint32_t u, v, fbtex_x, fbtex_y;

   u = TexWindowXLUT[u_arg];
   v = TexWindowYLUT[v_arg];
   fbtex_x = TexPageX + (u >> (2 - TexMode_TA));
   fbtex_y = TexPageY + v;
   fbw = GPURAM[fbtex_y][fbtex_x & 1023];

   if(TexMode_TA != 2)
   {
      if(TexMode_TA == 0)
         fbw = (fbw >> ((u & 3) * 4)) & 0xF;
      else
         fbw = (fbw >> ((u & 1) * 8)) & 0xFF;

      fbw = GPURAM[(clut_offset >> 10) & 511][(clut_offset + fbw) & 1023];
   }

   return(fbw);
}


static INLINE bool LineSkipTest(PS_GPU* g, unsigned y)
{
   //DisplayFB_XStart >= OffsX && DisplayFB_YStart >= OffsY &&
   // ((y & 1) == (DisplayFB_CurLineYReadout & 1))

   if((g->DisplayMode & 0x24) != 0x24)
      return false;

   if(!g->dfe && ((y & 1) == ((g->DisplayFB_YStart + g->field_ram_readout) & 1))/* && !DisplayOff*/) //&& (y >> 1) >= DisplayFB_YStart && (y >> 1) < (DisplayFB_YStart + (VertEnd - VertStart)))
      return true;

   return false;
}

#include "gpu_polygon.inc"
#include "gpu_sprite.inc"
#include "gpu_line.inc"

INLINE void PS_GPU::RecalcTexWindowLUT(void)
{
   unsigned x, y;
   const unsigned TexWindowX_AND = ~(tww << 3);
   const unsigned TexWindowX_OR = (twx & tww) << 3;
   const unsigned TexWindowY_AND = ~(twh << 3);
   const unsigned TexWindowY_OR = (twy & twh) << 3;
   // printf("TWX: 0x%02x, TWW: 0x%02x\n", twx, tww);
   // printf("TWY: 0x%02x, TWH: 0x%02x\n", twy, twh);
   for(x = 0; x < 256; x++)
      TexWindowXLUT[x] = (x & TexWindowX_AND) | TexWindowX_OR;
   for(y = 0; y < 256; y++)
      TexWindowYLUT[y] = (y & TexWindowY_AND) | TexWindowY_OR;
   memset(TexWindowXLUT_Pre, TexWindowXLUT[0], sizeof(TexWindowXLUT_Pre));
   memset(TexWindowXLUT_Post, TexWindowXLUT[255], sizeof(TexWindowXLUT_Post));
   memset(TexWindowYLUT_Pre, TexWindowYLUT[0], sizeof(TexWindowYLUT_Pre));
   memset(TexWindowYLUT_Post, TexWindowYLUT[255], sizeof(TexWindowYLUT_Post));
}

//
// C-style function wrappers so our command table isn't so ginormous(in memory usage).
//
template<int numvertices, bool shaded, bool textured, int BlendMode, bool TexMult, uint32 TexMode_TA, bool MaskEval_TA>
static void G_Command_DrawPolygon(PS_GPU* g, const uint32 *cb)
{
   g->Command_DrawPolygon<numvertices, shaded, textured, BlendMode, TexMult, TexMode_TA, MaskEval_TA>(cb);
}

template<uint8 raw_size, bool textured, int BlendMode, bool TexMult, uint32 TexMode_TA, bool MaskEval_TA>
static void G_Command_DrawSprite(PS_GPU* g, const uint32 *cb)
{
   g->Command_DrawSprite<raw_size, textured, BlendMode, TexMult, TexMode_TA, MaskEval_TA>(cb);
}

template<bool polyline, bool goraud, int BlendMode, bool MaskEval_TA>
static void G_Command_DrawLine(PS_GPU* g, const uint32 *cb)
{
   g->Command_DrawLine<polyline, goraud, BlendMode, MaskEval_TA>(cb);
}

static void G_Command_ClearCache(PS_GPU* g, const uint32 *cb)
{
}

static void G_Command_IRQ(PS_GPU* g, const uint32 *cb)
{
   g->IRQPending = true;
   IRQ_Assert(IRQ_GPU, g->IRQPending);
}

// Special RAM write mode(16 pixels at a time), does *not* appear to use mask drawing environment settings.
//
static void G_Command_FBFill(PS_GPU* gpu, const uint32 *cb)
{
   int32_t x, y, r, g, b, destX, destY, width, height;
   r = cb[0] & 0xFF;
   g = (cb[0] >> 8) & 0xFF;
   b = (cb[0] >> 16) & 0xFF;
   const uint16_t fill_value = ((r >> 3) << 0) | ((g >> 3) << 5) | ((b >> 3) << 10);

   destX = (cb[1] >>  0) & 0x3F0;
   destY = (cb[1] >> 16) & 0x3FF;

   width =  (((cb[2] >> 0) & 0x3FF) + 0xF) & ~0xF;
   height = (cb[2] >> 16) & 0x1FF;

   //printf("[GPU] FB Fill %d:%d w=%d, h=%d\n", destX, destY, width, height);
   gpu->DrawTimeAvail -= 46;	// Approximate
   gpu->DrawTimeAvail -= ((width * height) >> 3) + (height * 9);

   for(y = 0; y < height; y++)
   {
      const int32 d_y = (y + destY) & 511;

      if(LineSkipTest(gpu, d_y))
         continue;

      for(x = 0; x < width; x++)
      {
         const int32 d_x = (x + destX) & 1023;

         gpu->GPURAM[d_y][d_x] = fill_value;
      }
   }
}

static void G_Command_FBCopy(PS_GPU* g, const uint32 *cb)
{
   int32 sourceX = (cb[1] >> 0) & 0x3FF;
   int32 sourceY = (cb[1] >> 16) & 0x3FF;
   int32 destX = (cb[2] >> 0) & 0x3FF;
   int32 destY = (cb[2] >> 16) & 0x3FF;

   int32 width = (cb[3] >> 0) & 0x3FF;
   int32 height = (cb[3] >> 16) & 0x1FF;

   if(!width)
      width = 0x400;

   if(!height)
      height = 0x200;

   //printf("FB Copy: %d %d %d %d %d %d\n", sourceX, sourceY, destX, destY, width, height);

   g->DrawTimeAvail -= (width * height) * 2;

   for(int32 y = 0; y < height; y++)
   {
      for(int32 x = 0; x < width; x += 128)
      {
         const int32 chunk_x_max = std::min<int32>(width - x, 128);
         uint16 tmpbuf[128];	// TODO: Check and see if the GPU is actually (ab)using the CLUT or texture cache.

         for(int32 chunk_x = 0; chunk_x < chunk_x_max; chunk_x++)
         {
            int32 s_y = (y + sourceY) & 511;
            int32 s_x = (x + chunk_x + sourceX) & 1023;

            tmpbuf[chunk_x] = g->GPURAM[s_y][s_x];
         }

         for(int32 chunk_x = 0; chunk_x < chunk_x_max; chunk_x++)
         {
            int32 d_y = (y + destY) & 511;
            int32 d_x = (x + chunk_x + destX) & 1023;

            if(!(g->GPURAM[d_y][d_x] & g->MaskEvalAND))
               g->GPURAM[d_y][d_x] = tmpbuf[chunk_x] | g->MaskSetOR;
         }
      }
   }
}

static void G_Command_FBWrite(PS_GPU* g, const uint32 *cb)
{
   //assert(InCmd == INCMD_NONE);

   g->FBRW_X = (cb[1] >>  0) & 0x3FF;
   g->FBRW_Y = (cb[1] >> 16) & 0x3FF;

   g->FBRW_W = (cb[2] >>  0) & 0x7FF;
   g->FBRW_H = (cb[2] >> 16) & 0x3FF;

   if(g->FBRW_W > 0x400)
      g->FBRW_W &= 0x3FF;

   if(g->FBRW_H > 0x200)
      g->FBRW_H &= 0x1FF;

   g->FBRW_CurX = g->FBRW_X;
   g->FBRW_CurY = g->FBRW_Y;

   if(g->FBRW_W != 0 && g->FBRW_H != 0)
      g->InCmd = g->INCMD_FBWRITE;
}

static void G_Command_FBRead(PS_GPU* g, const uint32 *cb)
{
   //assert(g->InCmd == INCMD_NONE);

   g->FBRW_X = (cb[1] >>  0) & 0x3FF;
   g->FBRW_Y = (cb[1] >> 16) & 0x3FF;

   g->FBRW_W = (cb[2] >>  0) & 0x7FF;
   g->FBRW_H = (cb[2] >> 16) & 0x3FF;

   if(g->FBRW_W > 0x400)
      g->FBRW_W &= 0x3FF;

   if(g->FBRW_H > 0x200)
      g->FBRW_H &= 0x1FF;

   g->FBRW_CurX = g->FBRW_X;
   g->FBRW_CurY = g->FBRW_Y;

   if(g->FBRW_W != 0 && g->FBRW_H != 0)
      g->InCmd = g->INCMD_FBREAD;
}

static void G_Command_DrawMode(PS_GPU* g, const uint32 *cb)
{
   g->TexPageX = (*cb & 0xF) * 64;
   g->TexPageY = (*cb & 0x10) * 16;

   g->SpriteFlip = *cb & 0x3000;

   g->abr = (*cb >> 5) & 0x3;
   g->TexMode = (*cb >> 7) & 0x3;

   g->dtd = (*cb >> 9) & 1;
   g->dfe = (*cb >> 10) & 1;
   //printf("*******************DFE: %d -- scanline=%d\n", dfe, scanline);
}

static void G_Command_TexWindow(PS_GPU* g, const uint32 *cb)
{
   g->tww = (*cb & 0x1F);
   g->twh = ((*cb >> 5) & 0x1F);
   g->twx = ((*cb >> 10) & 0x1F);
   g->twy = ((*cb >> 15) & 0x1F);

   g->RecalcTexWindowLUT();
}

static void G_Command_Clip0(PS_GPU* g, const uint32 *cb)
{
   g->ClipX0 = *cb & 1023;
   g->ClipY0 = (*cb >> 10) & 1023;
}

static void G_Command_Clip1(PS_GPU* g, const uint32 *cb)
{
   g->ClipX1 = *cb & 1023;
   g->ClipY1 = (*cb >> 10) & 1023;
}

static void G_Command_DrawingOffset(PS_GPU* g, const uint32 *cb)
{
   g->OffsX = sign_x_to_s32(11, (*cb & 2047));
   g->OffsY = sign_x_to_s32(11, ((*cb >> 11) & 2047));

   //fprintf(stderr, "[GPU] Drawing offset: %d(raw=%d) %d(raw=%d) -- %d\n", OffsX, *cb, OffsY, *cb >> 11, scanline);
}

static void G_Command_MaskSetting(PS_GPU* g, const uint32 *cb)
{
   //printf("Mask setting: %08x\n", *cb);
   g->MaskSetOR = (*cb & 1) ? 0x8000 : 0x0000;
   g->MaskEvalAND = (*cb & 2) ? 0x8000 : 0x0000;
}


CTEntry PS_GPU::Commands[256] =
{
#include "gpu_command_table.inc"
};


void PS_GPU::ProcessFIFO(void)
{
   if(!BlitterFIFO.CanRead())
      return;

   switch(InCmd)
   {
      default:
         abort();
         break;

      case INCMD_NONE:
         break;

      case INCMD_FBREAD:
         puts("BOGUS SALAMANDERS, CAPTAIN!");
         return;

      case INCMD_FBWRITE:
         {
            uint32_t InData = BlitterFIFO.ReadUnit();

            for(int i = 0; i < 2; i++)
            {
               if(!(GPURAM[FBRW_CurY & 511][FBRW_CurX & 1023] & MaskEvalAND))
                  GPURAM[FBRW_CurY & 511][FBRW_CurX & 1023] = InData | MaskSetOR;

               FBRW_CurX++;
               if(FBRW_CurX == (FBRW_X + FBRW_W))
               {
                  FBRW_CurX = FBRW_X;
                  FBRW_CurY++;
                  if(FBRW_CurY == (FBRW_Y + FBRW_H))
                  {
                     InCmd = INCMD_NONE;
                     break;	// Break out of the for() loop.
                  }
               }
               InData >>= 16;
            }
            return;
         }
         break;

      case INCMD_QUAD:
         {
            if(DrawTimeAvail < 0)
               return;

            const uint32_t cc = InCmd_CC;
            const CTEntry *command = &Commands[cc];
            unsigned vl = 1 + (bool)(cc & 0x4) + (bool)(cc & 0x10);
            uint32_t CB[3];

            if(BlitterFIFO.CanRead() >= vl)
            {
               for(unsigned i = 0; i < vl; i++)
               {
                  CB[i] = BlitterFIFO.ReadUnit();
               }

               command->func[abr][TexMode | (MaskEvalAND ? 0x4 : 0x0)](this, CB);
            }
            return;
         }
         break;

      case INCMD_PLINE:
         {
            if(DrawTimeAvail < 0)
               return;

            const uint32_t cc = InCmd_CC;
            const CTEntry *command = &Commands[cc];
            unsigned vl = 1 + (bool)(InCmd_CC & 0x10);
            uint32_t CB[2];

            if((BlitterFIFO.ReadUnit(true) & 0xF000F000) == 0x50005000)
            {
               BlitterFIFO.ReadUnit();
               InCmd = INCMD_NONE;
               return;
            }

            if(BlitterFIFO.CanRead() >= vl)
            {
               for(unsigned i = 0; i < vl; i++)
               {
                  CB[i] = BlitterFIFO.ReadUnit();
               }

               command->func[abr][TexMode | (MaskEvalAND ? 0x4 : 0x0)](this, CB);
            }
            return;
         }
         break;
   }

   const uint32_t cc = BlitterFIFO.ReadUnit(true) >> 24;
   const CTEntry *command = &Commands[cc];

   if(DrawTimeAvail < 0 && !command->ss_cmd)
      return;

   if(BlitterFIFO.CanRead() >= command->len)
   {
      uint32_t CB[0x10];

      for(unsigned i = 0; i < command->len; i++)
         CB[i] = BlitterFIFO.ReadUnit();

      if(!command->ss_cmd)
         DrawTimeAvail -= 2;

#if 0
      PSX_WARNING("[GPU] Command: %08x %s %d %d %d", CB[0], command->name, command->len, scanline, DrawTimeAvail);
      if(1)
      {
         printf("[GPU]    ");
         for(unsigned i = 0; i < command->len; i++)
            printf("0x%08x ", CB[i]);
         printf("\n");
      }
#endif
      // A very very ugly kludge to support texture mode specialization. fixme/cleanup/SOMETHING in the future.
      if(cc >= 0x20 && cc <= 0x3F && (cc & 0x4))
      {
         uint32 tpage;

         tpage = CB[4 + ((cc >> 4) & 0x1)] >> 16;

         TexPageX = (tpage & 0xF) * 64;
         TexPageY = (tpage & 0x10) * 16;

         SpriteFlip = tpage & 0x3000;

         abr = (tpage >> 5) & 0x3;
         TexMode = (tpage >> 7) & 0x3;
      }
      if(!command->func[abr][TexMode])
      {
         if(CB[0])
            PSX_WARNING("[GPU] Unknown command: %08x, %d", CB[0], scanline);
      }
      else
      {
         command->func[abr][TexMode | (MaskEvalAND ? 0x4 : 0x0)](this, CB);
      }
   }
}

INLINE void PS_GPU::WriteCB(uint32_t InData)
{
   if(BlitterFIFO.CanRead() >= 0x10 && (InCmd != INCMD_NONE || (BlitterFIFO.CanRead() - 0x10) >= Commands[BlitterFIFO.ReadUnit(true) >> 24].fifo_fb_len))
   {
      PSX_DBG(PSX_DBG_WARNING, "GPU FIFO overflow!!!\n");
      return;
   }

   BlitterFIFO.WriteUnit(InData);
   ProcessFIFO();
}

void PS_GPU::Write(const pscpu_timestamp_t timestamp, uint32_t A, uint32_t V)
{
   V <<= (A & 3) * 8;

   if(A & 4)	// GP1 ("Control")
   {
      uint32_t command = V >> 24;

      V &= 0x00FFFFFF;

      //PSX_WARNING("[GPU] Control command: %02x %06x %d", command, V, scanline);

      switch(command)
      {
         /*
            0x40-0xFF do NOT appear to be mirrors, at least not on my PS1's GPU.
            */
         default:
            PSX_WARNING("[GPU] Unknown control command %02x - %06x", command, V);
            break;
         case 0x00:	// Reset GPU
            //printf("\n\n************ Soft Reset %u ********* \n\n", scanline);
            SoftReset();
            break;

         case 0x01:	// Reset command buffer
            if(DrawTimeAvail < 0)
               DrawTimeAvail = 0;
            BlitterFIFO.Flush();
            InCmd = INCMD_NONE;
            break;

         case 0x02: 	// Acknowledge IRQ
            IRQPending = false;
            IRQ_Assert(IRQ_GPU, IRQPending);            
            break;

         case 0x03:	// Display enable
            DisplayOff = V & 1;
            break;

         case 0x04:	// DMA Setup
            DMAControl = V & 0x3;
            break;

         case 0x05:	// Start of display area in framebuffer
            DisplayFB_XStart = V & 0x3FE; // Lower bit is apparently ignored.
            DisplayFB_YStart = (V >> 10) & 0x1FF;
            break;

         case 0x06:	// Horizontal display range
            HorizStart = V & 0xFFF;
            HorizEnd = (V >> 12) & 0xFFF;
            break;

         case 0x07:
            VertStart = V & 0x3FF;
            VertEnd = (V >> 10) & 0x3FF;
            break;

         case 0x08:
            //printf("\n\nDISPLAYMODE SET: 0x%02x, %u *************************\n\n\n", V & 0xFF, scanline);
            DisplayMode = V & 0xFF;
            break;

         case 0x10:	// GPU info(?)
            switch(V & 0xF)
            {
               // DataReadBuffer must remain unchanged for any unhandled GPU info index.
               default:  break;

               case 0x2: DataReadBuffer = (tww << 0) | (twh << 5) | (twx << 10) | (twy << 15);
                         break;

               case 0x3: DataReadBuffer = (ClipY0 << 10) | ClipX0;
                         break;

               case 0x4: DataReadBuffer = (ClipY1 << 10) | ClipX1;
                         break;

               case 0x5: DataReadBuffer = (OffsX & 2047) | ((OffsY & 2047) << 11);
                         break;

               case 0x7: DataReadBuffer = 2;
                         break;

               case 0x8: DataReadBuffer = 0;
                         break;
            }
            break;

      }
   }
   else		// GP0 ("Data")
   {
      //uint32_t command = V >> 24;
      //printf("Meow command: %02x\n", command);
      //assert(!(DMAControl & 2));
      WriteCB(V);
   }
}


void PS_GPU::WriteDMA(uint32_t V)
{
   WriteCB(V);
}

INLINE uint32_t PS_GPU::ReadData(void)
{
   if(InCmd == INCMD_FBREAD)
   {
      DataReadBuffer = 0;
      for(int i = 0; i < 2; i++)
      {
         DataReadBuffer |= GPURAM[FBRW_CurY & 511][FBRW_CurX & 1023] << (i * 16);

         FBRW_CurX++;
         if(FBRW_CurX == (FBRW_X + FBRW_W))
         {
            FBRW_CurX = FBRW_X;
            FBRW_CurY++;
            if(FBRW_CurY == (FBRW_Y + FBRW_H))
            {
               InCmd = INCMD_NONE;
               break;
            }
         }
      }
   }

   return DataReadBuffer;
}

uint32_t PS_GPU::ReadDMA(void)
{
 return ReadData();
}

uint32_t PS_GPU::Read(const pscpu_timestamp_t timestamp, uint32_t A)
{
 uint32_t ret = 0;

 if(A & 4)	// Status
 {
  ret = (((DisplayMode << 1) & 0x7F) | ((DisplayMode >> 6) & 1)) << 16;

  ret |= DMAControl << 29;

  ret |= (DisplayFB_CurLineYReadout & 1) << 31;

  ret |= (!field) << 13;

  if(DMAControl & 0x02)
   ret |= 1 << 25;

  ret |= IRQPending << 24;

  ret |= DisplayOff << 23;

  if(InCmd == INCMD_NONE && DrawTimeAvail >= 0 && BlitterFIFO.CanRead() == 0x00)	// GPU idle bit.
   ret |= 1 << 26;

  if(InCmd == INCMD_FBREAD)	// Might want to more accurately emulate this in the future?
   ret |= (1 << 27);

  ret |= CalcFIFOReadyBit() << 28;		// FIFO has room bit? (kinda).

  //
  //
  ret |= TexPageX >> 6;
  ret |= TexPageY >> 4;
  ret |= abr << 5;
  ret |= TexMode << 7;

  ret |= dtd << 9;
  ret |= dfe << 10;

  if(MaskSetOR)
   ret |= 1 << 11;

  if(MaskEvalAND)
   ret |= 1 << 12;
 }
 else		// "Data"
  ret = ReadData();

 if(DMAControl & 2)
 {
  //PSX_WARNING("[GPU READ WHEN (DMACONTROL&2)] 0x%08x - ret=0x%08x, scanline=%d", A, ret, scanline);
 }

 return(ret >> ((A & 3) * 8));
}

/*
static INLINE uint32_t ShiftHelper(uint32_t val, int shamt, uint32_t mask)
{
 if(shamt < 0)
  return((val >> (-shamt)) & mask);
 else
  return((val << shamt) & mask);
}
*/
INLINE void PS_GPU::ReorderRGB_Var(uint32_t out_Rshift, uint32_t out_Gshift, uint32_t out_Bshift, bool bpp24, const uint16_t *src, uint32_t *dest, const int32 dx_start, const int32 dx_end, int32 fb_x)
{
   if(bpp24)	// 24bpp
   {
      for(int32 x = dx_start; x < dx_end; x++)
      {
         uint32_t srcpix;

         srcpix = src[(fb_x >> 1) + 0] | (src[((fb_x >> 1) + 1) & 0x7FF] << 16);
         srcpix >>= (fb_x & 1) * 8;

         dest[x] = (((srcpix >> 0) << RED_SHIFT) & (0xFF << RED_SHIFT)) | (((srcpix >> 8) << GREEN_SHIFT) & (0xFF << GREEN_SHIFT)) |
            (((srcpix >> 16) << BLUE_SHIFT) & (0xFF << BLUE_SHIFT));

         fb_x = (fb_x + 3) & 0x7FF;
      }
   }				// 15bpp
   else
   {
      for(int32 x = dx_start; x < dx_end; x++)
      {
         uint32_t srcpix = src[fb_x >> 1];
         dest[x] = MAKECOLOR((((srcpix >> 0) & 0x1F) << 3), (((srcpix >> 5) & 0x1F) << 3), (((srcpix >> 10) & 0x1F) << 3), 0);

         fb_x = (fb_x + 2) & 0x7FF;
      }
   }

}

pscpu_timestamp_t PS_GPU::Update(const pscpu_timestamp_t sys_timestamp)
{
   static const uint32_t DotClockRatios[5] = { 10, 8, 5, 4, 7 };
   const uint32_t dmc = (DisplayMode & 0x40) ? 4 : (DisplayMode & 0x3);
   const uint32_t dmw = 2800 / DotClockRatios[dmc];	// Must be <= 768

   int32 sys_clocks = sys_timestamp - lastts;
   int32 gpu_clocks;

   //printf("GPUISH: %d\n", sys_timestamp - lastts);

   if(!sys_clocks)
      goto TheEnd;

   DrawTimeAvail += sys_clocks << 1;

   if(DrawTimeAvail > 256)
      DrawTimeAvail = 256;

   ProcessFIFO();

   //puts("GPU Update Start");

   GPUClockCounter += (uint64)sys_clocks * GPUClockRatio;

   gpu_clocks = GPUClockCounter >> 16;
   GPUClockCounter -= gpu_clocks << 16;

   while(gpu_clocks > 0)
   {
      int32 chunk_clocks = gpu_clocks;
      int32 dot_clocks;

      if(chunk_clocks > LineClockCounter)
      {
         //printf("Chunk: %u, LCC: %u\n", chunk_clocks, LineClockCounter);
         chunk_clocks = LineClockCounter;
      }

      gpu_clocks -= chunk_clocks;
      LineClockCounter -= chunk_clocks;

      DotClockCounter += chunk_clocks;
      dot_clocks = DotClockCounter / DotClockRatios[DisplayMode & 0x3];
      DotClockCounter -= dot_clocks * DotClockRatios[DisplayMode & 0x3];

      TIMER_AddDotClocks(dot_clocks);


      if(!LineClockCounter)
      {
         PSX_SetEventNT(PSX_EVENT_TIMER, TIMER_Update(sys_timestamp));  // We could just call this at the top of GPU_Update(), but do it here for slightly less CPU usage(presumably).

         LinePhase = (LinePhase + 1) & 1;

         if(LinePhase)
         {
            TIMER_SetHRetrace(true);
            LineClockCounter = 200;
            TIMER_ClockHRetrace();
         }
         else
         {
            const unsigned int FirstVisibleLine = LineVisFirst + (HardwarePALType ? 20 : 16);
            const unsigned int VisibleLineCount = LineVisLast + 1 - LineVisFirst; //HardwarePALType ? 288 : 240;

            TIMER_SetHRetrace(false);

            if(DisplayMode & 0x08)
               LineClockCounter = 3405 - 200;
            else
               LineClockCounter = 3412 + PhaseChange - 200;

            scanline = (scanline + 1) % LinesPerField;
            PhaseChange = !PhaseChange;

#ifdef WANT_DEBUGGER
            DBG_GPUScanlineHook(scanline);
#endif

            //
            //
            //
            if(scanline == (HardwarePALType ? 308 : 256))	// Will need to be redone if we ever allow for visible vertical overscan with NTSC.
            {
               if(sl_zero_reached)
               {
                  //printf("Req Exit(visible fallthrough case): %u\n", scanline);
                  PSX_RequestMLExit();
               }
            }

            if(scanline == (LinesPerField - 1))
            {
               if(sl_zero_reached)
               {
                  //printf("Req Exit(final fallthrough case): %u\n", scanline);
                  PSX_RequestMLExit();
               }

               if(DisplayMode & 0x20)
                  field = !field;
               else
                  field = 0;
            }

            if(scanline == 0)
            {
               assert(sl_zero_reached == false);
               sl_zero_reached = true;

               if(DisplayMode & 0x08)	// PAL
                  LinesPerField = 314;
               else			// NTSC
                  LinesPerField = 263;

               if(DisplayMode & 0x20)
                  LinesPerField -= field;
               else
                  field = 0;	// May not be the correct place for this?

               if(espec)
               {
                  if((bool)(DisplayMode & 0x08) != HardwarePALType)
                  {
                     DisplayRect->x = 0;
                     DisplayRect->y = 0;
                     DisplayRect->w = 384;
                     DisplayRect->h = VisibleLineCount;

                     for(int32 y = 0; y < DisplayRect->h; y++)
                     {
                        uint32_t *dest = surface->pixels + y * surface->pitch32;

                        LineWidths[y] = 384;

                        memset(dest, 0, 384 * sizeof(int32));
                     }
                     char buffer[256];

                     //snprintf(buffer, sizeof(buffer), _("VIDEO STANDARD MISMATCH"));
                     //DrawTextTrans(surface->pixels + ((DisplayRect->h / 2) - (13 / 2)) * surface->pitch32, surface->pitch32 << 2, DisplayRect->w, (UTF8*)buffer,
                     //MAKECOLOR(0x00, 0xFF, 0x00), true, MDFN_FONT_6x13_12x13, 0);
                  }
                  else
                  {
                     espec->InterlaceOn = (bool)(DisplayMode & 0x20);
                     espec->InterlaceField = (bool)(DisplayMode & 0x20) && field;

                     DisplayRect->x = 0;
                     DisplayRect->y = 0;
                     DisplayRect->w = 0;
                     DisplayRect->h = VisibleLineCount << (bool)(DisplayMode & 0x20);

                     // Clear ~0 state.
                     LineWidths[0] = 0;

                     for(int i = 0; i < (DisplayRect->y + DisplayRect->h); i++)
                     {
                        surface->pixels[i * surface->pitch32 + 0] =
                           surface->pixels[i * surface->pitch32 + 1] = 0;
                        LineWidths[i] = 2;
                     }
                  }
               }
            }

            //
            // Don't mess with the order of evaluation of these scanline == VertXXX && (InVblankwhatever) if statements and the following IRQ/timer vblank stuff
            // unless you know what you're doing!!! (IE you've run further tests to refine the behavior)
            //
            if(scanline == VertEnd && !InVBlank)
            {
               if(sl_zero_reached)
               {
                  // Gameplay in Descent(NTSC) has vblank at scanline 236
                  if(scanline >= (FirstVisibleLine + VisibleLineCount) || (scanline >= (HardwarePALType ? 260 : 232)))
                  {
                     //printf("Req Exit(vblank case): %u\n", scanline);
                     PSX_RequestMLExit();
                  }
                  else
                  {
                     //printf("VBlank too early, chickening out early exit!\n");
                  }
               }

               //printf("VBLANK: %u\n", scanline);
               InVBlank = true;

               DisplayFB_CurYOffset = 0;

               if((DisplayMode & 0x24) == 0x24)
                  field_ram_readout = !field;
               else
                  field_ram_readout = 0;
            }

            if(scanline == VertStart && InVBlank)
            {
               InVBlank = false;

               // Note to self: X-Men Mutant Academy relies on this being set on the proper scanline in 480i mode(otherwise it locks up on startup).
               //if(HeightMode)
               // DisplayFB_CurYOffset = field;
            }

            IRQ_Assert(IRQ_VBLANK, InVBlank);
            TIMER_SetVBlank(InVBlank);
            //
            //
            //

            // Needs to occur even in vblank.
            // Not particularly confident about the timing of this in regards to vblank and the upper bit(ODE) of the GPU status port, though the test that
            // showed an oddity was pathological in that VertEnd < VertStart in it.
            if((DisplayMode & 0x24) == 0x24)
               DisplayFB_CurLineYReadout = (DisplayFB_YStart + (DisplayFB_CurYOffset << 1) + (InVBlank ? 0 : field_ram_readout)) & 0x1FF;
            else
               DisplayFB_CurLineYReadout = (DisplayFB_YStart + DisplayFB_CurYOffset) & 0x1FF;

            unsigned dmw_width = 0;
            unsigned pix_clock_offset = 0;
            unsigned pix_clock = 0;
            unsigned pix_clock_div = 0;
            uint32_t *dest = NULL;
            if((bool)(DisplayMode & 0x08) == HardwarePALType && scanline >= FirstVisibleLine && scanline < (FirstVisibleLine + VisibleLineCount))
            {
               int32 dest_line;
               int32 fb_x = DisplayFB_XStart * 2;
               int32 dx_start = HorizStart, dx_end = HorizEnd;

               dest_line = ((scanline - FirstVisibleLine) << espec->InterlaceOn) + espec->InterlaceField;
               dest = surface->pixels + dest_line * surface->pitch32;

               if(dx_end < dx_start)
                  dx_end = dx_start;

               dx_start = dx_start / DotClockRatios[dmc];
               dx_end = dx_end / DotClockRatios[dmc];

               dx_start -= 488 / DotClockRatios[dmc];
               dx_end -= 488 / DotClockRatios[dmc];

               if(dx_start < 0)
               {
                  fb_x -= dx_start * ((DisplayMode & 0x10) ? 3 : 2);
                  fb_x &= 0x7FF; //0x3FF;
                  dx_start = 0;
               }

               if((uint32)dx_end > dmw)
                  dx_end = dmw;

               if(InVBlank || DisplayOff)
                  dx_start = dx_end = 0;

               // TODO, but there are problems with this, as not all blitter busy cycles(crudely abstracted with DrawTimeAvail) are GPU RAM access cycles.
               // Also, it shouldn't be here per-se, since this code won't be all if we're frameskipping or there's a video standard mismatch
               //DrawTimeAvail -= (dx_end - dx_start) + ((DisplayMode & 0x10) ? ((dx_end - dx_start + 1) >> 1) : 0);

               LineWidths[dest_line] = dmw;

               {
                  uint32_t x;
                  const uint16_t *src = GPURAM[DisplayFB_CurLineYReadout];

                  memset(dest, 0, dx_start * sizeof(int32));

                  //printf("%d %d %d - %d %d\n", scanline, dx_start, dx_end, HorizStart, HorizEnd);
                  ReorderRGB_Var(RED_SHIFT, GREEN_SHIFT, BLUE_SHIFT, DisplayMode & 0x10, src, dest, dx_start, dx_end, fb_x);

                  for(x = dx_end; x < dmw; x++)
                     dest[x] = 0;
               }

               //if(scanline == 64)
               // printf("%u\n", sys_timestamp - ((uint64)gpu_clocks * 65536) / GPUClockRatio);

               dmw_width = dmw;
               pix_clock_offset = (488 - 146) / DotClockRatios[dmc];
               pix_clock = (HardwarePALType ? 53203425 : 53693182) / DotClockRatios[dmc];
               pix_clock_div = DotClockRatios[dmc];
            }
            PSX_GPULineHook(sys_timestamp, sys_timestamp - ((uint64)gpu_clocks * 65536) / GPUClockRatio, scanline == 0, dest, &surface->format, dmw_width, pix_clock_offset, pix_clock, pix_clock_div);

            if(!InVBlank)
            {
               DisplayFB_CurYOffset = (DisplayFB_CurYOffset + 1) & 0x1FF;
            }
         }
      }
   }

   //puts("GPU Update End");

TheEnd:
   lastts = sys_timestamp;

   {
      int32 next_dt = LineClockCounter;

      next_dt = (((int64)next_dt << 16) - GPUClockCounter + GPUClockRatio - 1) / GPUClockRatio;

      next_dt = std::max<int32>(1, next_dt);
      next_dt = std::min<int32>(128, next_dt);

      //printf("%d\n", next_dt);

      return(sys_timestamp + next_dt);
   }
}

void PS_GPU::StartFrame(EmulateSpecStruct *espec_arg)
{
   sl_zero_reached = false;

   espec = espec_arg;

   surface = espec->surface;
   DisplayRect = &espec->DisplayRect;
   LineWidths = espec->LineWidths;
}

int PS_GPU::StateAction(StateMem *sm, int load, int data_only)
{
   SFORMAT StateRegs[] =
   {
      SFARRAY16(&GPURAM[0][0], sizeof(GPURAM) / sizeof(GPURAM[0][0])),

      SFVAR(DMAControl),

      SFVAR(ClipX0),
      SFVAR(ClipY0),
      SFVAR(ClipX1),
      SFVAR(ClipY1),

      SFVAR(OffsX),
      SFVAR(OffsY),

      SFVAR(dtd),
      SFVAR(dfe),

      SFVAR(MaskSetOR),
      SFVAR(MaskEvalAND),

      SFVAR(tww),
      SFVAR(twh),
      SFVAR(twx),
      SFVAR(twy),

      SFVAR(TexPageX),
      SFVAR(TexPageY),

      SFVAR(SpriteFlip),

      SFVAR(abr),
      SFVAR(TexMode),

      SFARRAY32(&BlitterFIFO.data[0], BlitterFIFO.size),
      SFVAR(BlitterFIFO.read_pos),
      SFVAR(BlitterFIFO.write_pos),
      SFVAR(BlitterFIFO.in_count),

      SFVAR(DataReadBuffer),

      SFVAR(IRQPending),

      SFVAR(InCmd),
      SFVAR(InCmd_CC),

#define TVHELPER(n)	SFVAR(n.x), SFVAR(n.y), SFVAR(n.u), SFVAR(n.v), SFVAR(n.r), SFVAR(n.g), SFVAR(n.b)
      TVHELPER(InQuad_F3Vertices[0]),
      TVHELPER(InQuad_F3Vertices[1]),
      TVHELPER(InQuad_F3Vertices[2]),
#undef TVHELPER
      SFVAR(InQuad_clut),

      SFVAR(InPLine_PrevPoint.x),
      SFVAR(InPLine_PrevPoint.y),
      SFVAR(InPLine_PrevPoint.r),
      SFVAR(InPLine_PrevPoint.g),
      SFVAR(InPLine_PrevPoint.b),

      SFVAR(FBRW_X),
      SFVAR(FBRW_Y),
      SFVAR(FBRW_W),
      SFVAR(FBRW_H),
      SFVAR(FBRW_CurY),
      SFVAR(FBRW_CurX),

      SFVAR(DisplayMode),
      SFVAR(DisplayOff),
      SFVAR(DisplayFB_XStart),
      SFVAR(DisplayFB_YStart),

      SFVAR(HorizStart),
      SFVAR(HorizEnd),

      SFVAR(VertStart),
      SFVAR(VertEnd),

      SFVAR(DisplayFB_CurYOffset),
      SFVAR(DisplayFB_CurLineYReadout),

      SFVAR(InVBlank),

      SFVAR(LinesPerField),
      SFVAR(scanline),
      SFVAR(field),
      SFVAR(field_ram_readout),
      SFVAR(PhaseChange),

      SFVAR(DotClockCounter),

      SFVAR(GPUClockCounter),
      SFVAR(LineClockCounter),
      SFVAR(LinePhase),

      SFVAR(DrawTimeAvail),

      SFEND
   };
   int ret = MDFNSS_StateAction(sm, load, StateRegs, "GPU");

   if(load)
   {
      RecalcTexWindowLUT();
      BlitterFIFO.SaveStatePostLoad();

      HorizStart &= 0xFFF;
      HorizEnd &= 0xFFF;

      IRQ_Assert(IRQ_GPU, IRQPending);
   }

   return(ret);
}

}

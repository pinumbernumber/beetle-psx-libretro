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
PS_GPU::PS_GPU(bool pal_clock_and_tv, int sls, int sle)
{
   SimpleFIFO_New(BlitterFIFO, SimpleFIFOU32, uint32, 0x20);
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
   SimpleFIFO_Free(BlitterFIFO);
}

void PS_GPU::FillVideoParams(MDFNGI* gi)
{
   gi->lcm_width = 2800;
   gi->lcm_height = (LineVisLast + 1 - LineVisFirst) * 2;
   gi->nominal_height = LineVisLast + 1 - LineVisFirst;

   if(HardwarePALType)
   {

      gi->nominal_width = 377;	// Dunno. :(

      gi->fb_width = 768;
      gi->fb_height = 576;

      gi->fps = 836203078; // 49.842

      gi->VideoSystem = VIDSYS_PAL;
   }
   else
   {
      gi->nominal_width = 320;

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

   SimpleFIFO_Flush(BlitterFIFO);
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

   SimpleFIFO_Flush(BlitterFIFO);

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

#define ModTexel(texel, r, g, b, dither_x, dither_y) (((texel) & 0x8000) | (DitherLUT[(dither_y)][(dither_x)][((((texel) & 0x1F)   * (r)) >> (5  - 1))] << 0)   | (DitherLUT[(dither_y)][(dither_x)][((((texel) & 0x3E0)  * (g)) >> (10 - 1))] << 5) | (DitherLUT[(dither_y)][(dither_x)][((((texel) & 0x7C00) * (b)) >> (15 - 1))] << 10))

template<uint32_t TexMode_TA>
INLINE uint16_t PS_GPU::GetTexel(const uint32_t clut_offset, int32 u_arg, int32 v_arg)
{
   uint32_t u = TexWindowXLUT[u_arg];
   uint32_t v = TexWindowYLUT[v_arg];
   uint32_t fbtex_x = TexPageX + (u >> (2 - TexMode_TA));
   uint32_t fbtex_y = TexPageY + v;
   uint16_t fbw = GPURAM[fbtex_y][fbtex_x & 1023];

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

#define COORD_FBS 12
#define COORD_MF_INT(n) ((n) << COORD_FBS)

/*
 Store and do most math with interpolant coordinates and deltas as unsigned to avoid violating strict overflow(due to biasing),
 but when actually grabbing the coordinates, treat them as signed(with signed right shift) so we can do saturation properly.
*/
#define COORD_GET_INT(n) ((n) >> COORD_FBS)

struct i_group
{
 uint32 u, v;
 uint32 r, g, b;
 uint32 dummy0[3];
};

struct i_deltas
{
 uint32 du_dx, dv_dx;
 uint32 dr_dx, dg_dx, db_dx;
 uint32 dummy0[3];

 uint32 du_dy, dv_dy;
 uint32 dr_dy, dg_dy, db_dy;
 uint32 dummy1[3];
};

#define MakePolyXFP(x) (((int64)(x) << 32) + ((1LL << 32) - (1 << 11)))

static INLINE int64 MakePolyXFPStep(int32 dx, int32 dy)
{
   int64 dx_ex = (int64)dx << 32;

   if(dx_ex < 0)
      dx_ex -= dy - 1;

   if(dx_ex > 0)
      dx_ex += dy - 1;

   return (dx_ex / dy);
}

#define GetPolyXFP_Int(xfp) ((xfp) >> 32)

#define CALCIS(x,y) (((B->x - A->x) * (C->y - B->y)) - ((C->x - B->x) * (B->y - A->y)))

static INLINE bool CalcIDeltas(i_deltas *idl, const tri_vertex *A, const tri_vertex *B, const tri_vertex *C)
{
 // 32 is const SA
 int64 num = ((int64)COORD_MF_INT(1)) << 32;
 int64 denom = CALCIS(x, y);
 int64 one_div = num / denom;

 if(!denom)
  return(false);

 idl->dr_dx = ((one_div * CALCIS(r, y)) + 0x00000000) >> 32;
 idl->dr_dy = ((one_div * CALCIS(x, r)) + 0x00000000) >> 32;

 idl->dg_dx = ((one_div * CALCIS(g, y)) + 0x00000000) >> 32;
 idl->dg_dy = ((one_div * CALCIS(x, g)) + 0x00000000) >> 32;

 idl->db_dx = ((one_div * CALCIS(b, y)) + 0x00000000) >> 32;
 idl->db_dy = ((one_div * CALCIS(x, b)) + 0x00000000) >> 32;

 idl->du_dx = ((one_div * CALCIS(u, y)) + 0x00000000) >> 32;
 idl->du_dy = ((one_div * CALCIS(x, u)) + 0x00000000) >> 32;

 idl->dv_dx = ((one_div * CALCIS(v, y)) + 0x00000000) >> 32;
 idl->dv_dy = ((one_div * CALCIS(x, v)) + 0x00000000) >> 32;

 // idl->du_dx = ((int64)CALCIS(u, y) << COORD_FBS) / denom;
 // idl->du_dy = ((int64)CALCIS(x, u) << COORD_FBS) / denom;

 // idl->dv_dx = ((int64)CALCIS(v, y) << COORD_FBS) / denom;
 // idl->dv_dy = ((int64)CALCIS(x, v) << COORD_FBS) / denom;

 //printf("Denom=%lld - CIS_UY=%d, CIS_XU=%d, CIS_VY=%d, CIS_XV=%d\n", denom, CALCIS(u, y), CALCIS(x, u), CALCIS(v, y), CALCIS(x, v));
 //printf("  du_dx=0x%08x, du_dy=0x%08x --- dv_dx=0x%08x, dv_dy=0x%08x\n", idl->du_dx, idl->du_dy, idl->dv_dx, idl->dv_dy);

 return(true);
}
#undef CALCIS

template<bool goraud, bool textured>
static INLINE void AddIDeltas_DX(i_group &ig, const i_deltas &idl, uint32 count = 1)
{
 if(textured)
 {
  ig.u += idl.du_dx * count;
  ig.v += idl.dv_dx * count;
 }

 if(goraud)
 {
  ig.r += idl.dr_dx * count;
  ig.g += idl.dg_dx * count;
  ig.b += idl.db_dx * count;
 }
}

template<bool goraud, bool textured>
static INLINE void AddIDeltas_DY(i_group &ig, const i_deltas &idl, uint32 count = 1)
{
 if(textured)
 {
  ig.u += idl.du_dy * count;
  ig.v += idl.dv_dy * count;
 }

 if(goraud)
 {
  ig.r += idl.dr_dy * count;
  ig.g += idl.dg_dy * count;
  ig.b += idl.db_dy * count;
 }
}

template<bool goraud, bool textured, int BlendMode, bool TexMult, uint32 TexMode_TA, bool MaskEval_TA>
INLINE void PS_GPU::DrawSpan(int y, uint32 clut_offset, const int32 x_start, const int32 x_bound, i_group ig, const i_deltas &idl)
{
  int32 xs = x_start, xb = x_bound;

  if(LineSkipTest(this, y))
   return;

  if(xs < xb)	// (xs != xb)
  {
   if(xs < ClipX0)
    xs = ClipX0;

   if(xb > (ClipX1 + 1))
    xb = ClipX1 + 1;

   if(xs < xb)
   {
    DrawTimeAvail -= (xb - xs);

    if(goraud || textured)
    {
     DrawTimeAvail -= (xb - xs);
    }
    else if((BlendMode >= 0) || MaskEval_TA)
    {
     DrawTimeAvail -= (((xb + 1) & ~1) - (xs & ~1)) >> 1;
    }
   }

   if(textured)
   {
    ig.u += (xs * idl.du_dx) + (y * idl.du_dy);
    ig.v += (xs * idl.dv_dx) + (y * idl.dv_dy);
   }

   if(goraud)
   {
    ig.r += (xs * idl.dr_dx) + (y * idl.dr_dy);
    ig.g += (xs * idl.dg_dx) + (y * idl.dg_dy);
    ig.b += (xs * idl.db_dx) + (y * idl.db_dy);
   }

   for(int32 x = xs; MDFN_LIKELY(x < xb); x++)
   {
    uint32 r, g, b;

    if(goraud)
    {
     r = RGB8SAT[COORD_GET_INT(ig.r)];
     g = RGB8SAT[COORD_GET_INT(ig.g)];
     b = RGB8SAT[COORD_GET_INT(ig.b)];
    }
    else
    {
     r = COORD_GET_INT(ig.r);
     g = COORD_GET_INT(ig.g);
     b = COORD_GET_INT(ig.b);
    }

    if(textured)
    {
     uint16 fbw = GetTexel<TexMode_TA>(clut_offset, COORD_GET_INT(ig.u), COORD_GET_INT(ig.v));

     if(fbw)
     {
      if(TexMult)
        fbw = ModTexel(fbw, r, g, b, (dtd) ? (x & 3) : 3, (dtd) ? (y & 3) : 2);
      PlotPixel<BlendMode, MaskEval_TA, true>(x, y, fbw);
     }
    }
    else
    {
     uint16 pix = 0x8000;

     if(goraud && dtd)
     {
      pix |= DitherLUT[y & 3][x & 3][r] << 0;
      pix |= DitherLUT[y & 3][x & 3][g] << 5;
      pix |= DitherLUT[y & 3][x & 3][b] << 10;
     }
     else
     {
      pix |= (r >> 3) << 0;
      pix |= (g >> 3) << 5;
      pix |= (b >> 3) << 10;
     }
    
     PlotPixel<BlendMode, MaskEval_TA, false>(x, y, pix);
    }

    AddIDeltas_DX<goraud, textured>(ig, idl);
    //AddStep<goraud, textured>(perp_coord, perp_step);
   }
  }
}

template<bool goraud, bool textured, int BlendMode, bool TexMult, uint32 TexMode_TA, bool MaskEval_TA>
void PS_GPU::DrawTriangle(tri_vertex *vertices, uint32 clut)
{
 i_deltas idl;

 //
 // Sort vertices by y.
 //
 if(vertices[2].y < vertices[1].y)
 {
  tri_vertex tmp = vertices[1];
  vertices[1] = vertices[2];
  vertices[2] = tmp;
 }

 if(vertices[1].y < vertices[0].y)
 {
  tri_vertex tmp = vertices[0];
  vertices[0] = vertices[1];
  vertices[1] = tmp;
 }

 if(vertices[2].y < vertices[1].y)
 {
  tri_vertex tmp = vertices[1];
  vertices[1] = vertices[2];
  vertices[2] = tmp;
 }

 if(vertices[0].y == vertices[2].y)
  return;

 if((vertices[2].y - vertices[0].y) >= 512)
 {
  //PSX_WARNING("[GPU] Triangle height too large: %d", (vertices[2].y - vertices[0].y));
  return;
 }

 if(abs(vertices[2].x - vertices[0].x) >= 1024 ||
    abs(vertices[2].x - vertices[1].x) >= 1024 ||
    abs(vertices[1].x - vertices[0].x) >= 1024)
 {
  //PSX_WARNING("[GPU] Triangle width too large: %d %d %d", abs(vertices[2].x - vertices[0].x), abs(vertices[2].x - vertices[1].x), abs(vertices[1].x - vertices[0].x));
  return;
 }

 if(!CalcIDeltas(&idl, &vertices[0], &vertices[1], &vertices[2]))
  return;

 // [0] should be top vertex, [2] should be bottom vertex, [1] should be off to the side vertex.
 //
 //
 int32 y_start = vertices[0].y;
 int32 y_middle = vertices[1].y;
 int32 y_bound = vertices[2].y;

 int64 base_coord;
 int64 base_step;

 int64 bound_coord_ul;
 int64 bound_coord_us;

 int64 bound_coord_ll;
 int64 bound_coord_ls;

 bool right_facing;
 //bool bottom_up;
 i_group ig;

 //
 // Find vertex with lowest X coordinate, and use as the base for calculating interpolants from.
 //
 {
  unsigned iggvi = 0;

  //
  // <=, not <
  //
  if(vertices[1].x <= vertices[iggvi].x)
   iggvi = 1;

  if(vertices[2].x <= vertices[iggvi].x)
   iggvi = 2;

  ig.u = COORD_MF_INT(vertices[iggvi].u) + (1 << (COORD_FBS - 1));
  ig.v = COORD_MF_INT(vertices[iggvi].v) + (1 << (COORD_FBS - 1));
  ig.r = COORD_MF_INT(vertices[iggvi].r);
  ig.g = COORD_MF_INT(vertices[iggvi].g);
  ig.b = COORD_MF_INT(vertices[iggvi].b);

  AddIDeltas_DX<goraud, textured>(ig, idl, -vertices[iggvi].x);
  AddIDeltas_DY<goraud, textured>(ig, idl, -vertices[iggvi].y);
 }

 base_coord = MakePolyXFP(vertices[0].x);
 base_step = MakePolyXFPStep((vertices[2].x - vertices[0].x), (vertices[2].y - vertices[0].y));

 bound_coord_ul = MakePolyXFP(vertices[0].x);
 bound_coord_ll = MakePolyXFP(vertices[1].x);

 //
 //
 //


 if(vertices[1].y == vertices[0].y)
 {
  bound_coord_us = 0;
  right_facing = (bool)(vertices[1].x > vertices[0].x);
 }
 else
 {
  bound_coord_us = MakePolyXFPStep((vertices[1].x - vertices[0].x), (vertices[1].y - vertices[0].y));
  right_facing = (bool)(bound_coord_us > base_step);
 }

 if(vertices[2].y == vertices[1].y)
  bound_coord_ls = 0;
 else
  bound_coord_ls = MakePolyXFPStep((vertices[2].x - vertices[1].x), (vertices[2].y - vertices[1].y));

 if(y_start < ClipY0)
 {
  int32 count = ClipY0 - y_start;

  y_start = ClipY0;
  base_coord += base_step * count;
  bound_coord_ul += bound_coord_us * count;

  if(y_middle < ClipY0)
  {
   int32 count_ls = ClipY0 - y_middle;

   y_middle = ClipY0;
   bound_coord_ll += bound_coord_ls * count_ls;
  }
 }

 if(y_bound > (ClipY1 + 1))
 {
  y_bound = ClipY1 + 1;

  if(y_middle > y_bound)
   y_middle = y_bound;
 }

 if(right_facing)
 {
  for(int32 y = y_start; y < y_middle; y++)
  {
   DrawSpan<goraud, textured, BlendMode, TexMult, TexMode_TA, MaskEval_TA>(y, clut, GetPolyXFP_Int(base_coord), GetPolyXFP_Int(bound_coord_ul), ig, idl);
   base_coord += base_step;
   bound_coord_ul += bound_coord_us;
  }

  for(int32 y = y_middle; y < y_bound; y++)
  {
   DrawSpan<goraud, textured, BlendMode, TexMult, TexMode_TA, MaskEval_TA>(y, clut, GetPolyXFP_Int(base_coord), GetPolyXFP_Int(bound_coord_ll), ig, idl);
   base_coord += base_step;
   bound_coord_ll += bound_coord_ls;
  }
 }
 else
 {
  for(int32 y = y_start; y < y_middle; y++)
  {
   DrawSpan<goraud, textured, BlendMode, TexMult, TexMode_TA, MaskEval_TA>(y, clut, GetPolyXFP_Int(bound_coord_ul), GetPolyXFP_Int(base_coord), ig, idl);
   base_coord += base_step;
   bound_coord_ul += bound_coord_us;
  }

  for(int32 y = y_middle; y < y_bound; y++)
  {
   DrawSpan<goraud, textured, BlendMode, TexMult, TexMode_TA, MaskEval_TA>(y, clut, GetPolyXFP_Int(bound_coord_ll), GetPolyXFP_Int(base_coord), ig, idl);
   base_coord += base_step;
   bound_coord_ll += bound_coord_ls;
  }
 }

#if 0
 printf("[GPU] Vertices: %d:%d(r=%d, g=%d, b=%d) -> %d:%d(r=%d, g=%d, b=%d) -> %d:%d(r=%d, g=%d, b=%d)\n\n\n", vertices[0].x, vertices[0].y,
	vertices[0].r, vertices[0].g, vertices[0].b,
	vertices[1].x, vertices[1].y,
	vertices[1].r, vertices[1].g, vertices[1].b,
	vertices[2].x, vertices[2].y,
	vertices[2].r, vertices[2].g, vertices[2].b);
#endif
}

template<int numvertices, bool goraud, bool textured, int BlendMode, bool TexMult, uint32 TexMode_TA, bool MaskEval_TA>
INLINE void PS_GPU::Command_DrawPolygon(const uint32 *cb)
{
 const unsigned cb0 = cb[0];
 tri_vertex vertices[3];
 uint32 clut = 0;
 unsigned sv = 0;
 //uint32 tpage = 0;

 // Base timing is approximate, and could be improved.
 if(numvertices == 4 && InCmd == INCMD_QUAD)
  DrawTimeAvail -= (28 + 18);
 else
  DrawTimeAvail -= (64 + 18);

 if(goraud && textured)
  DrawTimeAvail -= 150 * 3;
 else if(goraud)
  DrawTimeAvail -= 96 * 3;
 else if(textured)
  DrawTimeAvail -= 60 * 3;

 if(numvertices == 4)
 {
  if(InCmd == INCMD_QUAD)
  {
   memcpy(&vertices[0], &InQuad_F3Vertices[1], 2 * sizeof(tri_vertex));
   clut = InQuad_clut;
   sv = 2;
  }
 }
 //else
 // memset(vertices, 0, sizeof(vertices));

 for(unsigned v = sv; v < 3; v++)
 {
  if(v == 0 || goraud)
  {
   uint32 raw_color = (*cb & 0xFFFFFF);

   vertices[v].r = raw_color & 0xFF;
   vertices[v].g = (raw_color >> 8) & 0xFF;
   vertices[v].b = (raw_color >> 16) & 0xFF;

   cb++;
  }
  else
  {
   vertices[v].r = vertices[0].r;
   vertices[v].g = vertices[0].g;
   vertices[v].b = vertices[0].b;
  }

  vertices[v].x = sign_x_to_s32(11, ((int16)(*cb & 0xFFFF))) + OffsX;
  vertices[v].y = sign_x_to_s32(11, ((int16)(*cb >> 16))) + OffsY;
  cb++;

  if(textured)
  {
   vertices[v].u = (*cb & 0xFF);
   vertices[v].v = (*cb >> 8) & 0xFF;

   if(v == 0)
   {
    clut = ((*cb >> 16) & 0xFFFF) << 4;
   }

   cb++;
  }
 }

 if(numvertices == 4)
 {
  if(InCmd == INCMD_QUAD)
  {
   InCmd = INCMD_NONE;
  }
  else
  {
   InCmd = INCMD_QUAD;
   InCmd_CC = cb0 >> 24;
   memcpy(&InQuad_F3Vertices[0], &vertices[0], sizeof(tri_vertex) * 3);
   InQuad_clut = clut;
  }
 }

 DrawTriangle<goraud, textured, BlendMode, TexMult, TexMode_TA, MaskEval_TA>(vertices, clut);
}

#undef COORD_FBS
#undef COORD_MF_INT

template<bool textured, int BlendMode, bool TexMult, uint32 TexMode_TA, bool MaskEval_TA, bool FlipX, bool FlipY>
void PS_GPU::DrawSprite(int32 x_arg, int32 y_arg, int32 w, int32 h, uint8 u_arg, uint8 v_arg, uint32 color, uint32 clut_offset)
{
 const int32 r = color & 0xFF;
 const int32 g = (color >> 8) & 0xFF;
 const int32 b = (color >> 16) & 0xFF;
 const uint16 fill_color = 0x8000 | ((r >> 3) << 0) | ((g >> 3) << 5) | ((b >> 3) << 10);

 int32 x_start, x_bound;
 int32 y_start, y_bound;
 uint8 u, v;
 int v_inc = 1, u_inc = 1;

 //printf("[GPU] Sprite: x=%d, y=%d, w=%d, h=%d\n", x_arg, y_arg, w, h);

 x_start = x_arg;
 x_bound = x_arg + w;

 y_start = y_arg;
 y_bound = y_arg + h;

 if(textured)
 {
  u = u_arg;
  v = v_arg;

  //if(FlipX || FlipY || (u & 1) || (v & 1) || ((TexMode_TA == 0) && ((u & 3) || (v & 3))))
  // fprintf(stderr, "Flippy: %d %d 0x%02x 0x%02x\n", FlipX, FlipY, u, v);

  if(FlipX)
  {
   u_inc = -1;
   u |= 1;
  }
  // FIXME: Something weird happens when lower bit of u is set and we're not doing horizontal flip, but I'm not sure what it is exactly(needs testing)
  // It may only happen to the first pixel, so look for that case too during testing.
  //else
  // u = (u + 1) & ~1;

  if(FlipY)
  {
   v_inc = -1;
  }
 }

 if(x_start < ClipX0)
 {
  if(textured)
   u += (ClipX0 - x_start) * u_inc;

  x_start = ClipX0;
 }

 if(y_start < ClipY0)
 {
  if(textured)
   v += (ClipY0 - y_start) * v_inc;

  y_start = ClipY0;
 }

 if(x_bound > (ClipX1 + 1))
  x_bound = ClipX1 + 1;

 if(y_bound > (ClipY1 + 1))
  y_bound = ClipY1 + 1;

 if(y_bound > y_start && x_bound > x_start)
 {
  //
  // Note(TODO): From tests on a PS1, even a 0-width sprite takes up time to "draw" proportional to its height.
  //
  int32 suck_time = (x_bound - x_start) * (y_bound - y_start);

  // Disabled until we can get it to take into account texture windowing, which can cause large sprites to be drawn entirely from cache(and not suffer from a texturing
  // penalty); and disabled until we find a game that needs more accurate sprite draw timing. :b
#if 0
  if(textured)
  {
   // Empirically-observed approximations of time(66MHz cycles) taken to draw large textured sprites in the various texture depths, when the texture data and CLUT data
   // was zero(non-zero takes longer to draw, TODO test that more):
   //  4bpp - area * 2
   //  8bpp - area * 3
   //  15/16bpp - area * 5
   // (other factors come into more importance for smaller sprites)
   static const int cw[4] = { 64, 32, 32, 32 };
   static const int ch[4] = { 64, 64, 32, 32 };
   static const int mm[4] = { 2 - 1, 3 - 1, 5 - 1, 5 - 1 };

   // Parts of the first few(up to texture cache height) horizontal lines can be in cache, so assume they are.
   suck_time += mm[TexMode_TA] * std::max<int>(0, (x_bound - x_start) - cw[TexMode_TA]) * std::min<int>(ch[TexMode_TA], y_bound - y_start);

   // The rest of the horizontal lines should not possibly have parts in the cache now.
   suck_time += mm[TexMode_TA] * (x_bound - x_start) * std::max<int>(0, (y_bound - y_start) - ch[TexMode_TA]);
  }
  else
#endif

  if((BlendMode >= 0) || MaskEval_TA)
  {
   suck_time += ((((x_bound + 1) & ~1) - (x_start & ~1)) * (y_bound - y_start)) >> 1;
  }

  DrawTimeAvail -= suck_time;
 }


 //HeightMode && !dfe && ((y & 1) == ((DisplayFB_YStart + !field_atvs) & 1)) && !DisplayOff
 //printf("%d:%d, %d, %d ---- heightmode=%d displayfb_ystart=%d field_atvs=%d displayoff=%d\n", w, h, scanline, dfe, HeightMode, DisplayFB_YStart, field_atvs, DisplayOff);

 for(int32 y = y_start; MDFN_LIKELY(y < y_bound); y++)
 {
  uint8 u_r;

  if(textured)
   u_r = u;

  if(!LineSkipTest(this, y))
  {
   for(int32 x = x_start; MDFN_LIKELY(x < x_bound); x++)
   {
    if(textured)
    {
     uint16 fbw = GetTexel<TexMode_TA>(clut_offset, u_r, v);

     if(fbw)
     {
      if(TexMult)
      {
       fbw = ModTexel(fbw, r, g, b, 3, 2);
      }
      PlotPixel<BlendMode, MaskEval_TA, true>(x, y, fbw);
     }
    }
    else
     PlotPixel<BlendMode, MaskEval_TA, false>(x, y, fill_color);

    if(textured)
     u_r += u_inc;
   }
  }
  if(textured)
   v += v_inc;
 }
}

template<uint8 raw_size, bool textured, int BlendMode, bool TexMult, uint32 TexMode_TA, bool MaskEval_TA>
INLINE void PS_GPU::Command_DrawSprite(const uint32 *cb)
{
 int32 x, y;
 int32 w, h;
 uint8 u = 0, v = 0;
 uint32 color = 0;
 uint32 clut = 0;

 DrawTimeAvail -= 16;	// FIXME, correct time.

 color = *cb & 0x00FFFFFF;
 cb++;

 x = sign_x_to_s32(11, (*cb & 0xFFFF));
 y = sign_x_to_s32(11, (*cb >> 16));
 cb++;

 if(textured)
 {
  u = *cb & 0xFF;
  v = (*cb >> 8) & 0xFF;
  clut = ((*cb >> 16) & 0xFFFF) << 4;
  cb++;
 }

 switch(raw_size)
 {
  default:
  case 0:
	w = (*cb & 0x3FF);
	h = (*cb >> 16) & 0x1FF;
	cb++;
	break;

  case 1:
	w = 1;
	h = 1;
	break;

  case 2:
	w = 8;
	h = 8;
	break;

  case 3:
	w = 16;
	h = 16;
	break;
 }

 //printf("SPRITE: %d %d %d -- %d %d\n", raw_size, x, y, w, h);

 x = sign_x_to_s32(11, x + OffsX);
 y = sign_x_to_s32(11, y + OffsY);

 switch(SpriteFlip & 0x3000)
 {
  case 0x0000:
	if(!TexMult || color == 0x808080)
  	 DrawSprite<textured, BlendMode, false, TexMode_TA, MaskEval_TA, false, false>(x, y, w, h, u, v, color, clut);
	else
	 DrawSprite<textured, BlendMode, true, TexMode_TA, MaskEval_TA, false, false>(x, y, w, h, u, v, color, clut);
	break;

  case 0x1000:
	if(!TexMult || color == 0x808080)
  	 DrawSprite<textured, BlendMode, false, TexMode_TA, MaskEval_TA, true, false>(x, y, w, h, u, v, color, clut);
	else
	 DrawSprite<textured, BlendMode, true, TexMode_TA, MaskEval_TA, true, false>(x, y, w, h, u, v, color, clut);
	break;

  case 0x2000:
	if(!TexMult || color == 0x808080)
  	 DrawSprite<textured, BlendMode, false, TexMode_TA, MaskEval_TA, false, true>(x, y, w, h, u, v, color, clut);
	else
	 DrawSprite<textured, BlendMode, true, TexMode_TA, MaskEval_TA, false, true>(x, y, w, h, u, v, color, clut);
	break;

  case 0x3000:
	if(!TexMult || color == 0x808080)
  	 DrawSprite<textured, BlendMode, false, TexMode_TA, MaskEval_TA, true, true>(x, y, w, h, u, v, color, clut);
	else
	 DrawSprite<textured, BlendMode, true, TexMode_TA, MaskEval_TA, true, true>(x, y, w, h, u, v, color, clut);
	break;
 }
}

struct line_fxp_coord
{
 int64 x, y;
 int32 r, g, b;
};

struct line_fxp_step
{
 int64 dx_dk, dy_dk;
 int32 dr_dk, dg_dk, db_dk;
};

enum { Line_XY_FractBits = 32 };
enum { Line_RGB_FractBits = 12 };

template<bool goraud>
static INLINE void LinePointToFXPCoord(const line_point &point, const line_fxp_step &step, line_fxp_coord &coord)
{
 coord.x = ((int64)point.x << Line_XY_FractBits) | (1LL << (Line_XY_FractBits - 1));
 coord.y = ((int64)point.y << Line_XY_FractBits) | (1LL << (Line_XY_FractBits - 1));

 coord.x -= 1024;

 if(step.dy_dk < 0)
  coord.y -= 1024;

 if(goraud)
 {
  coord.r = (point.r << Line_RGB_FractBits) | (1 << (Line_RGB_FractBits - 1));
  coord.g = (point.g << Line_RGB_FractBits) | (1 << (Line_RGB_FractBits - 1));
  coord.b = (point.b << Line_RGB_FractBits) | (1 << (Line_RGB_FractBits - 1));
 }
}

template<typename T, unsigned bits>
static INLINE T LineDivide(T delta, int32 dk)
{
 delta <<= bits;

 if(delta < 0)
  delta -= dk - 1;
 if(delta > 0)
  delta += dk - 1;

 return(delta / dk);
}

template<bool goraud>
static INLINE void LinePointsToFXPStep(const line_point &point0, const line_point &point1, const int32 dk, line_fxp_step &step)
{
 if(!dk)
 {
  step.dx_dk = 0;
  step.dy_dk = 0;

  if(goraud)
  {
   step.dr_dk = 0;
   step.dg_dk = 0;
   step.db_dk = 0;
  }
  return;
 }

 step.dx_dk = LineDivide<int64, Line_XY_FractBits>(point1.x - point0.x, dk);
 step.dy_dk = LineDivide<int64, Line_XY_FractBits>(point1.y - point0.y, dk);

 if(goraud)
 {
  step.dr_dk = ((point1.r - point0.r) << Line_RGB_FractBits) / dk;
  step.dg_dk = ((point1.g - point0.g) << Line_RGB_FractBits) / dk;
  step.db_dk = ((point1.b - point0.b) << Line_RGB_FractBits) / dk;
 }
}

template<bool goraud>
static INLINE void AddLineStep(line_fxp_coord &point, const line_fxp_step &step, int32 count = 1)
{
 point.x += step.dx_dk * count;
 point.y += step.dy_dk * count;
 
 if(goraud)
 {
  point.r += step.dr_dk * count;
  point.g += step.dg_dk * count;
  point.b += step.db_dk * count;
 }
}

template<bool goraud, int BlendMode, bool MaskEval_TA>
void PS_GPU::DrawLine(line_point *points)
{
 int32 i_dx;
 int32 i_dy;
 int32 k;
 line_fxp_coord cur_point;
 line_fxp_step step;

 i_dx = abs(points[1].x - points[0].x);
 i_dy = abs(points[1].y - points[0].y);
 k = (i_dx > i_dy) ? i_dx : i_dy;

 if(i_dx >= 1024)
 {
  PSX_DBG(PSX_DBG_WARNING, "[GPU] Line too long: i_dx=%d\n", i_dx);
  return;
 }

 if(i_dy >= 512)
 {
  PSX_DBG(PSX_DBG_WARNING, "[GPU] Line too long: i_dy=%d\n", i_dy);
  return;
 }

 // May not be correct(do tests for the case of k == i_dy on real thing.
 if(points[0].x > points[1].x)
 {
  line_point tmp = points[1];

  points[1] = points[0];
  points[0] = tmp;  
 }

 DrawTimeAvail -= k * ((BlendMode >= 0) ? 2 : 1);

 //
 //
 //

 LinePointsToFXPStep<goraud>(points[0], points[1], k, step);
 LinePointToFXPCoord<goraud>(points[0], step, cur_point);
 
 //
 //
 //
 for(int32 i = 0; i <= k; i++)	// <= is not a typo.
 {
  // Sign extension is not necessary here for x and y, due to the maximum values that ClipX1 and ClipY1 can contain.
  const int32 x = (cur_point.x >> Line_XY_FractBits) & 2047;
  const int32 y = (cur_point.y >> Line_XY_FractBits) & 2047;
  uint16 pix = 0x8000;

  if(!LineSkipTest(this, y))
  {
   uint8 r, g, b;

   if(goraud)
   {
    r = cur_point.r >> Line_RGB_FractBits;
    g = cur_point.g >> Line_RGB_FractBits;
    b = cur_point.b >> Line_RGB_FractBits;
   }
   else
   {
    r = points[0].r;
    g = points[0].g;
    b = points[0].b;
   }

   if(goraud && dtd)
   {
    pix |= DitherLUT[y & 3][x & 3][r] << 0;
    pix |= DitherLUT[y & 3][x & 3][g] << 5;
    pix |= DitherLUT[y & 3][x & 3][b] << 10;
   }
   else
   {
    pix |= (r >> 3) << 0;
    pix |= (g >> 3) << 5;
    pix |= (b >> 3) << 10;
   }

   // FIXME: There has to be a faster way than checking for being inside the drawing area for each pixel.
   if(x >= ClipX0 && x <= ClipX1 && y >= ClipY0 && y <= ClipY1)
    PlotPixel<BlendMode, MaskEval_TA, false>(x, y, pix);
  }

  AddLineStep<goraud>(cur_point, step);
 }
}

template<bool polyline, bool goraud, int BlendMode, bool MaskEval_TA>
INLINE void PS_GPU::Command_DrawLine(const uint32 *cb)
{
 const uint8 cc = cb[0] >> 24; // For pline handling later.
 line_point points[2];

 DrawTimeAvail -= 16;	// FIXME, correct time.

 if(polyline && InCmd == INCMD_PLINE)
 {
  //printf("PLINE N\n");
  points[0] = InPLine_PrevPoint;
 }
 else
 {
  points[0].r = (*cb >> 0) & 0xFF;
  points[0].g = (*cb >> 8) & 0xFF;
  points[0].b = (*cb >> 16) & 0xFF;
  cb++;

  points[0].x = sign_x_to_s32(11, ((*cb >> 0) & 0xFFFF)) + OffsX;
  points[0].y = sign_x_to_s32(11, ((*cb >> 16) & 0xFFFF)) + OffsY;
  cb++;
 }

 if(goraud)
 {
  points[1].r = (*cb >> 0) & 0xFF;
  points[1].g = (*cb >> 8) & 0xFF;
  points[1].b = (*cb >> 16) & 0xFF;
  cb++;
 }
 else
 {
  points[1].r = points[0].r;
  points[1].g = points[0].g;
  points[1].b = points[0].b;
 }

 points[1].x = sign_x_to_s32(11, ((*cb >> 0) & 0xFFFF)) + OffsX;
 points[1].y = sign_x_to_s32(11, ((*cb >> 16) & 0xFFFF)) + OffsY;
 cb++;

 if(polyline)
 {
  InPLine_PrevPoint = points[1];

  if(InCmd != INCMD_PLINE)
  {
   InCmd = INCMD_PLINE;
   InCmd_CC = cc;
  }
 }

 DrawLine<goraud, BlendMode, MaskEval_TA>(points);
}


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
   int32_t x, y;
   int32_t r = cb[0] & 0xFF;
   int32_t g = (cb[0] >> 8) & 0xFF;
   int32_t b = (cb[0] >> 16) & 0xFF;
   const uint16_t fill_value = ((r >> 3) << 0) | ((g >> 3) << 5) | ((b >> 3) << 10);

   int32_t destX = (cb[1] >>  0) & 0x3F0;
   int32_t destY = (cb[1] >> 16) & 0x3FF;

   int32_t width =  (((cb[2] >> 0) & 0x3FF) + 0xF) & ~0xF;
   int32_t height = (cb[2] >> 16) & 0x1FF;

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

#define POLY_HELPER_SUB(bm, cv, tm, mam)	\
   G_Command_DrawPolygon<3 + ((cv & 0x8) >> 3), ((cv & 0x10) >> 4), ((cv & 0x4) >> 2), ((cv & 0x2) >> 1) ? bm : -1, ((cv & 1) ^ 1) & ((cv & 0x4) >> 2), tm, mam >

#define POLY_HELPER_FG(bm, cv)						\
   {								\
      POLY_HELPER_SUB(bm, cv, ((cv & 0x4) ? 0 : 0), 0),	\
      POLY_HELPER_SUB(bm, cv, ((cv & 0x4) ? 1 : 0), 0),	\
      POLY_HELPER_SUB(bm, cv, ((cv & 0x4) ? 2 : 0), 0),	\
      POLY_HELPER_SUB(bm, cv, ((cv & 0x4) ? 2 : 0), 0),	\
      POLY_HELPER_SUB(bm, cv, ((cv & 0x4) ? 0 : 0), 1),	\
      POLY_HELPER_SUB(bm, cv, ((cv & 0x4) ? 1 : 0), 1),	\
      POLY_HELPER_SUB(bm, cv, ((cv & 0x4) ? 2 : 0), 1),	\
      POLY_HELPER_SUB(bm, cv, ((cv & 0x4) ? 2 : 0), 1),	\
   }

#define POLY_HELPER(cv)														\
   { 															\
      { POLY_HELPER_FG(0, cv), POLY_HELPER_FG(1, cv), POLY_HELPER_FG(2, cv), POLY_HELPER_FG(3, cv) },			\
      1 + (3 /*+ ((cv & 0x8) >> 3)*/) * ( 1 + ((cv & 0x4) >> 2) + ((cv & 0x10) >> 4) ) - ((cv & 0x10) >> 4),			\
      1,															\
      false															\
   }

   //
   //

#define SPR_HELPER_SUB(bm, cv, tm, mam) G_Command_DrawSprite<(cv >> 3) & 0x3,	((cv & 0x4) >> 2), ((cv & 0x2) >> 1) ? bm : -1, ((cv & 1) ^ 1) & ((cv & 0x4) >> 2), tm, mam>

#define SPR_HELPER_FG(bm, cv)						\
   {								\
      SPR_HELPER_SUB(bm, cv, ((cv & 0x4) ? 0 : 0), 0),	\
      SPR_HELPER_SUB(bm, cv, ((cv & 0x4) ? 1 : 0), 0),	\
      SPR_HELPER_SUB(bm, cv, ((cv & 0x4) ? 2 : 0), 0),	\
      SPR_HELPER_SUB(bm, cv, ((cv & 0x4) ? 2 : 0), 0),	\
      SPR_HELPER_SUB(bm, cv, ((cv & 0x4) ? 0 : 0), 1),	\
      SPR_HELPER_SUB(bm, cv, ((cv & 0x4) ? 1 : 0), 1),	\
      SPR_HELPER_SUB(bm, cv, ((cv & 0x4) ? 2 : 0), 1),	\
      SPR_HELPER_SUB(bm, cv, ((cv & 0x4) ? 2 : 0), 1),	\
   }


#define SPR_HELPER(cv)												\
   {													\
      { SPR_HELPER_FG(0, cv), SPR_HELPER_FG(1, cv), SPR_HELPER_FG(2, cv), SPR_HELPER_FG(3, cv) },		\
      2 + ((cv & 0x4) >> 2) + ((cv & 0x18) ? 0 : 1),								\
      2 | ((cv & 0x4) >> 2) | ((cv & 0x18) ? 0 : 1),		/* |, not +, for this */			\
      false													\
   }

   //
   //

#define LINE_HELPER_SUB(bm, cv, mam) G_Command_DrawLine<((cv & 0x08) >> 3), ((cv & 0x10) >> 4), ((cv & 0x2) >> 1) ? bm : -1, mam>

#define LINE_HELPER_FG(bm, cv)											\
   {													\
      LINE_HELPER_SUB(bm, cv, 0),									\
      LINE_HELPER_SUB(bm, cv, 0),									\
      LINE_HELPER_SUB(bm, cv, 0),									\
      LINE_HELPER_SUB(bm, cv, 0),									\
      LINE_HELPER_SUB(bm, cv, 1),									\
      LINE_HELPER_SUB(bm, cv, 1),									\
      LINE_HELPER_SUB(bm, cv, 1),									\
      LINE_HELPER_SUB(bm, cv, 1)									\
   }

#define LINE_HELPER(cv)												\
   { 													\
      { LINE_HELPER_FG(0, cv), LINE_HELPER_FG(1, cv), LINE_HELPER_FG(2, cv), LINE_HELPER_FG(3, cv) },	\
      3 + ((cv & 0x10) >> 4),										\
      1,													\
      false													\
   }

   //
   //


#define OTHER_HELPER_FG(bm, arg_ptr) { arg_ptr, arg_ptr, arg_ptr, arg_ptr, arg_ptr, arg_ptr, arg_ptr, arg_ptr }
#define OTHER_HELPER(arg_cs, arg_fbcs, arg_ss, arg_ptr) { { OTHER_HELPER_FG(0, arg_ptr), OTHER_HELPER_FG(1, arg_ptr), OTHER_HELPER_FG(2, arg_ptr), OTHER_HELPER_FG(3, arg_ptr) }, arg_cs, arg_fbcs, arg_ss }
#define OTHER_HELPER_X2(arg_cs, arg_fbcs, arg_ss, arg_ptr)	OTHER_HELPER(arg_cs, arg_fbcs, arg_ss, arg_ptr), OTHER_HELPER(arg_cs, arg_fbcs, arg_ss, arg_ptr)
#define OTHER_HELPER_X4(arg_cs, arg_fbcs, arg_ss, arg_ptr)	OTHER_HELPER_X2(arg_cs, arg_fbcs, arg_ss, arg_ptr), OTHER_HELPER_X2(arg_cs, arg_fbcs, arg_ss, arg_ptr)
#define OTHER_HELPER_X8(arg_cs, arg_fbcs, arg_ss, arg_ptr)	OTHER_HELPER_X4(arg_cs, arg_fbcs, arg_ss, arg_ptr), OTHER_HELPER_X4(arg_cs, arg_fbcs, arg_ss, arg_ptr)
#define OTHER_HELPER_X16(arg_cs, arg_fbcs, arg_ss, arg_ptr)	OTHER_HELPER_X8(arg_cs, arg_fbcs, arg_ss, arg_ptr), OTHER_HELPER_X8(arg_cs, arg_fbcs, arg_ss, arg_ptr)
#define OTHER_HELPER_X32(arg_cs, arg_fbcs, arg_ss, arg_ptr)	OTHER_HELPER_X16(arg_cs, arg_fbcs, arg_ss, arg_ptr), OTHER_HELPER_X16(arg_cs, arg_fbcs, arg_ss, arg_ptr)

#define NULLCMD_FG(bm) { NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL } 
#define NULLCMD() { { NULLCMD_FG(0), NULLCMD_FG(1), NULLCMD_FG(2), NULLCMD_FG(3) }, 1, 1, true }

//#define BM_HELPER(fg) { fg(0), fg(1), fg(2), fg(3) }

CTEntry PS_GPU::Commands[256] =
{
   /* 0x00 */
   NULLCMD(),
   OTHER_HELPER(1, 2, false, G_Command_ClearCache),
   OTHER_HELPER(3, 3, false, G_Command_FBFill),

   NULLCMD(), NULLCMD(), NULLCMD(), NULLCMD(), NULLCMD(),
   NULLCMD(), NULLCMD(), NULLCMD(), NULLCMD(), NULLCMD(), NULLCMD(), NULLCMD(), NULLCMD(),

   /* 0x10 */
   NULLCMD(), NULLCMD(), NULLCMD(), NULLCMD(), NULLCMD(), NULLCMD(), NULLCMD(), NULLCMD(),
   NULLCMD(), NULLCMD(), NULLCMD(), NULLCMD(), NULLCMD(), NULLCMD(), NULLCMD(),

   OTHER_HELPER(1, 1, false,  G_Command_IRQ),

   /* 0x20 */
   POLY_HELPER(0x20),
   POLY_HELPER(0x21),
   POLY_HELPER(0x22),
   POLY_HELPER(0x23),
   POLY_HELPER(0x24),
   POLY_HELPER(0x25),
   POLY_HELPER(0x26),
   POLY_HELPER(0x27),
   POLY_HELPER(0x28),
   POLY_HELPER(0x29),
   POLY_HELPER(0x2a),
   POLY_HELPER(0x2b),
   POLY_HELPER(0x2c),
   POLY_HELPER(0x2d),
   POLY_HELPER(0x2e),
   POLY_HELPER(0x2f),
   POLY_HELPER(0x30),
   POLY_HELPER(0x31),
   POLY_HELPER(0x32),
   POLY_HELPER(0x33),
   POLY_HELPER(0x34),
   POLY_HELPER(0x35),
   POLY_HELPER(0x36),
   POLY_HELPER(0x37),
   POLY_HELPER(0x38),
   POLY_HELPER(0x39),
   POLY_HELPER(0x3a),
   POLY_HELPER(0x3b),
   POLY_HELPER(0x3c),
   POLY_HELPER(0x3d),
   POLY_HELPER(0x3e),
   POLY_HELPER(0x3f),

   LINE_HELPER(0x40),
   LINE_HELPER(0x41),
   LINE_HELPER(0x42),
   LINE_HELPER(0x43),
   LINE_HELPER(0x44),
   LINE_HELPER(0x45),
   LINE_HELPER(0x46),
   LINE_HELPER(0x47),
   LINE_HELPER(0x48),
   LINE_HELPER(0x49),
   LINE_HELPER(0x4a),
   LINE_HELPER(0x4b),
   LINE_HELPER(0x4c),
   LINE_HELPER(0x4d),
   LINE_HELPER(0x4e),
   LINE_HELPER(0x4f),
   LINE_HELPER(0x50),
   LINE_HELPER(0x51),
   LINE_HELPER(0x52),
   LINE_HELPER(0x53),
   LINE_HELPER(0x54),
   LINE_HELPER(0x55),
   LINE_HELPER(0x56),
   LINE_HELPER(0x57),
   LINE_HELPER(0x58),
   LINE_HELPER(0x59),
   LINE_HELPER(0x5a),
   LINE_HELPER(0x5b),
   LINE_HELPER(0x5c),
   LINE_HELPER(0x5d),
   LINE_HELPER(0x5e),
   LINE_HELPER(0x5f),

   SPR_HELPER(0x60),
   SPR_HELPER(0x61),
   SPR_HELPER(0x62),
   SPR_HELPER(0x63),
   SPR_HELPER(0x64),
   SPR_HELPER(0x65),
   SPR_HELPER(0x66),
   SPR_HELPER(0x67),
   SPR_HELPER(0x68),
   SPR_HELPER(0x69),
   SPR_HELPER(0x6a),
   SPR_HELPER(0x6b),
   SPR_HELPER(0x6c),
   SPR_HELPER(0x6d),
   SPR_HELPER(0x6e),
   SPR_HELPER(0x6f),
   SPR_HELPER(0x70),
   SPR_HELPER(0x71),
   SPR_HELPER(0x72),
   SPR_HELPER(0x73),
   SPR_HELPER(0x74),
   SPR_HELPER(0x75),
   SPR_HELPER(0x76),
   SPR_HELPER(0x77),
   SPR_HELPER(0x78),
   SPR_HELPER(0x79),
   SPR_HELPER(0x7a),
   SPR_HELPER(0x7b),
   SPR_HELPER(0x7c),
   SPR_HELPER(0x7d),
   SPR_HELPER(0x7e),
   SPR_HELPER(0x7f),

   /* 0x80 ... 0x9F */
   OTHER_HELPER_X32(4, 2, false, G_Command_FBCopy),

   /* 0xA0 ... 0xBF */
   OTHER_HELPER_X32(3, 2, false, G_Command_FBWrite),

   /* 0xC0 ... 0xDF */
   OTHER_HELPER_X32(3, 2, false, G_Command_FBRead),

   /* 0xE0 */

   NULLCMD(),
   OTHER_HELPER(1, 2, false, G_Command_DrawMode),
   OTHER_HELPER(1, 2, false, G_Command_TexWindow),
   OTHER_HELPER(1, 1, true,  G_Command_Clip0),
   OTHER_HELPER(1, 1, true,  G_Command_Clip1),
   OTHER_HELPER(1, 1, true,  G_Command_DrawingOffset),
   OTHER_HELPER(1, 2, false, G_Command_MaskSetting),

   NULLCMD(),
   NULLCMD(), NULLCMD(), NULLCMD(), NULLCMD(), NULLCMD(), NULLCMD(), NULLCMD(), NULLCMD(),

   /* 0xF0 */
   NULLCMD(), NULLCMD(), NULLCMD(), NULLCMD(), NULLCMD(), NULLCMD(), NULLCMD(), NULLCMD(),
   NULLCMD(), NULLCMD(), NULLCMD(), NULLCMD(), NULLCMD(), NULLCMD(), NULLCMD(), NULLCMD(),

};


void PS_GPU::ProcessFIFO(void)
{
   if(!BlitterFIFO->in_count)
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
            uint32_t InData = SimpleFIFO_ReadUnit(BlitterFIFO);
            SimpleFIFO_ReadUnitIncrement(BlitterFIFO);

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

            if(BlitterFIFO->in_count >= vl)
            {
               for(unsigned i = 0; i < vl; i++)
               {
                  CB[i] = SimpleFIFO_ReadUnit(BlitterFIFO);
                  SimpleFIFO_ReadUnitIncrement(BlitterFIFO);
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

            if((SimpleFIFO_ReadUnit(BlitterFIFO) & 0xF000F000) == 0x50005000)
            {
               SimpleFIFO_ReadUnitIncrement(BlitterFIFO);
               InCmd = INCMD_NONE;
               return;
            }

            if(BlitterFIFO->in_count >= vl)
            {
               for(unsigned i = 0; i < vl; i++)
               {
                  CB[i] = SimpleFIFO_ReadUnit(BlitterFIFO);
                  SimpleFIFO_ReadUnitIncrement(BlitterFIFO);
               }

               command->func[abr][TexMode | (MaskEvalAND ? 0x4 : 0x0)](this, CB);
            }
            return;
         }
         break;
   }

   const uint32_t cc = SimpleFIFO_ReadUnit(BlitterFIFO) >> 24;
   const CTEntry *command = &Commands[cc];

   if(DrawTimeAvail < 0 && !command->ss_cmd)
      return;

   if(BlitterFIFO->in_count >= command->len)
   {
      uint32_t CB[0x10];

      for(unsigned i = 0; i < command->len; i++)
      {
         CB[i] = SimpleFIFO_ReadUnit(BlitterFIFO);
         SimpleFIFO_ReadUnitIncrement(BlitterFIFO);
      }

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
   if(BlitterFIFO->in_count >= 0x10 && (InCmd != INCMD_NONE || (BlitterFIFO->in_count - 0x10) >= Commands[SimpleFIFO_ReadUnit(BlitterFIFO) >> 24].fifo_fb_len))
   {
      PSX_DBG(PSX_DBG_WARNING, "GPU FIFO overflow!!!\n");
      return;
   }

   SimpleFIFO_WriteUnit(BlitterFIFO, InData);
   ProcessFIFO();
}

void PS_GPU::Write(const int32_t timestamp, uint32_t A, uint32_t V)
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
            SimpleFIFO_Flush(BlitterFIFO);
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

uint32_t PS_GPU::ReadDMA(void)
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

uint32_t PS_GPU::Read(const int32_t timestamp, uint32_t A)
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

  if(InCmd == INCMD_NONE && DrawTimeAvail >= 0 && BlitterFIFO->in_count == 0x00)	// GPU idle bit.
   ret |= 1 << 26;

  if(InCmd == INCMD_FBREAD)	// Might want to more accurately emulate this in the future?
   ret |= (1 << 27);

  ret |= DMACanWrite() << 28;		// FIFO has room bit? (kinda).

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
  ret = ReadDMA();

 if(DMAControl & 2)
 {
  //PSX_WARNING("[GPU READ WHEN (DMACONTROL&2)] 0x%08x - ret=0x%08x, scanline=%d", A, ret, scanline);
 }

 return(ret >> ((A & 3) * 8));
}

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

int32_t PS_GPU::Update(const int32_t sys_timestamp)
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
      { ((&GPURAM[0][0])), (uint32)(((sizeof(GPURAM) / sizeof(GPURAM[0][0]))) * sizeof(uint16)), 0x20000000 | SF_FORCE_A16((&GPURAM[0][0])), "&GPURAM[0][0]" },

      { &((DMAControl)), sizeof((DMAControl)), 0x80000000 | 0, "DMAControl" },

      { &((ClipX0)), sizeof((ClipX0)), 0x80000000 | 0, "ClipX0" },
      { &((ClipY0)), sizeof((ClipY0)), 0x80000000 | 0, "ClipY0" },
      { &((ClipX1)), sizeof((ClipX1)), 0x80000000 | 0, "ClipX1" },
      { &((ClipY1)), sizeof((ClipY1)), 0x80000000 | 0, "ClipY1" },

      { &((OffsX)), sizeof((OffsX)), 0x80000000 | 0, "OffsX" },
      { &((OffsY)), sizeof((OffsY)), 0x80000000 | 0, "OffsY" },

      { &((dtd)), sizeof((dtd)), 0x80000000 | 0, "dtd" },
      { &((dfe)), sizeof((dfe)), 0x80000000 | 0, "dfe" },

      { &((MaskSetOR)), sizeof((MaskSetOR)), 0x80000000 | 0, "MaskSetOR" },
      { &((MaskEvalAND)), sizeof((MaskEvalAND)), 0x80000000 | 0, "MaskEvalAND" },

      { &((tww)), sizeof((tww)), 0x80000000 | 0, "tww" },
      { &((twh)), sizeof((twh)), 0x80000000 | 0, "twh" },
      { &((twx)), sizeof((twx)), 0x80000000 | 0, "twx" },
      { &((twy)), sizeof((twy)), 0x80000000 | 0, "twy" },

      { &((TexPageX)), sizeof((TexPageX)), 0x80000000 | 0, "TexPageX" },
      { &((TexPageY)), sizeof((TexPageY)), 0x80000000 | 0, "TexPageY" },

      { &((SpriteFlip)), sizeof((SpriteFlip)), 0x80000000 | 0, "SpriteFlip" },

      { &((abr)), SF_IS_BOOL(&((abr))) ? 1 : sizeof((abr)), 0x80000000 | (SF_IS_BOOL(&((abr))) ? 0x08000000 : 0), "abr" },
      { &((TexMode)), sizeof((TexMode)), 0x80000000 | 0, "TexMode" },

      { ((BlitterFIFO->data)), (uint32)(((BlitterFIFO->size)) * sizeof(uint32)), 0x40000000 | SF_FORCE_A32((BlitterFIFO->data)), "&BlitterFIFO.data[0]" },
      { &((BlitterFIFO->read_pos)), sizeof((BlitterFIFO->read_pos)), 0x80000000 | 0, "BlitterFIFO.read_pos" },
      { &((BlitterFIFO->write_pos)), sizeof((BlitterFIFO->write_pos)), 0x80000000 | 0, "BlitterFIFO.write_pos" },
      { &((BlitterFIFO->in_count)), sizeof((BlitterFIFO->in_count)), 0x80000000 | 0, "BlitterFIFO.in_count" },

      { &((DataReadBuffer)), sizeof((DataReadBuffer)), 0x80000000 | 0, "DataReadBuffer" },

      { &((IRQPending)), 1, 0x80000000 | 0x08000000, "IRQPending" },

      { &((InCmd)), sizeof((InCmd)), 0x80000000 | 0, "InCmd" },
      { &((InCmd_CC)), sizeof((InCmd_CC)), 0x80000000 | 0, "InCmd_CC" },


      { &((InQuad_F3Vertices[0].x)), SF_IS_BOOL(&((InQuad_F3Vertices[0].x))) ? 1 : sizeof((InQuad_F3Vertices[0].x)), 0x80000000 | (SF_IS_BOOL(&((InQuad_F3Vertices[0].x))) ? 0x08000000 : 0), "InQuad_F3Vertices[0].x" },
      { &((InQuad_F3Vertices[0].y)), SF_IS_BOOL(&((InQuad_F3Vertices[0].y))) ? 1 : sizeof((InQuad_F3Vertices[0].y)), 0x80000000 | (SF_IS_BOOL(&((InQuad_F3Vertices[0].y))) ? 0x08000000 : 0), "InQuad_F3Vertices[0].y" },
      { &((InQuad_F3Vertices[0].u)), SF_IS_BOOL(&((InQuad_F3Vertices[0].u))) ? 1 : sizeof((InQuad_F3Vertices[0].u)), 0x80000000 | (SF_IS_BOOL(&((InQuad_F3Vertices[0].u))) ? 0x08000000 : 0), "InQuad_F3Vertices[0].u" },
      { &((InQuad_F3Vertices[0].v)), SF_IS_BOOL(&((InQuad_F3Vertices[0].v))) ? 1 : sizeof((InQuad_F3Vertices[0].v)), 0x80000000 | (SF_IS_BOOL(&((InQuad_F3Vertices[0].v))) ? 0x08000000 : 0), "InQuad_F3Vertices[0].v" },
      { &((InQuad_F3Vertices[0].r)), SF_IS_BOOL(&((InQuad_F3Vertices[0].r))) ? 1 : sizeof((InQuad_F3Vertices[0].r)), 0x80000000 | (SF_IS_BOOL(&((InQuad_F3Vertices[0].r))) ? 0x08000000 : 0), "InQuad_F3Vertices[0].r" },
      { &((InQuad_F3Vertices[0].g)), SF_IS_BOOL(&((InQuad_F3Vertices[0].g))) ? 1 : sizeof((InQuad_F3Vertices[0].g)), 0x80000000 | (SF_IS_BOOL(&((InQuad_F3Vertices[0].g))) ? 0x08000000 : 0), "InQuad_F3Vertices[0].g" },
      { &((InQuad_F3Vertices[0].b)), SF_IS_BOOL(&((InQuad_F3Vertices[0].b))) ? 1 : sizeof((InQuad_F3Vertices[0].b)), 0x80000000 | (SF_IS_BOOL(&((InQuad_F3Vertices[0].b))) ? 0x08000000 : 0), "InQuad_F3Vertices[0].b" },
      { &((InQuad_F3Vertices[1].x)), SF_IS_BOOL(&((InQuad_F3Vertices[1].x))) ? 1 : sizeof((InQuad_F3Vertices[1].x)), 0x80000000 | (SF_IS_BOOL(&((InQuad_F3Vertices[1].x))) ? 0x08000000 : 0), "InQuad_F3Vertices[1].x" },
      { &((InQuad_F3Vertices[1].y)), SF_IS_BOOL(&((InQuad_F3Vertices[1].y))) ? 1 : sizeof((InQuad_F3Vertices[1].y)), 0x80000000 | (SF_IS_BOOL(&((InQuad_F3Vertices[1].y))) ? 0x08000000 : 0), "InQuad_F3Vertices[1].y" },
      { &((InQuad_F3Vertices[1].u)), SF_IS_BOOL(&((InQuad_F3Vertices[1].u))) ? 1 : sizeof((InQuad_F3Vertices[1].u)), 0x80000000 | (SF_IS_BOOL(&((InQuad_F3Vertices[1].u))) ? 0x08000000 : 0), "InQuad_F3Vertices[1].u" },
      { &((InQuad_F3Vertices[1].v)), SF_IS_BOOL(&((InQuad_F3Vertices[1].v))) ? 1 : sizeof((InQuad_F3Vertices[1].v)), 0x80000000 | (SF_IS_BOOL(&((InQuad_F3Vertices[1].v))) ? 0x08000000 : 0), "InQuad_F3Vertices[1].v" },
      { &((InQuad_F3Vertices[1].r)), SF_IS_BOOL(&((InQuad_F3Vertices[1].r))) ? 1 : sizeof((InQuad_F3Vertices[1].r)), 0x80000000 | (SF_IS_BOOL(&((InQuad_F3Vertices[1].r))) ? 0x08000000 : 0), "InQuad_F3Vertices[1].r" },
      { &((InQuad_F3Vertices[1].g)), SF_IS_BOOL(&((InQuad_F3Vertices[1].g))) ? 1 : sizeof((InQuad_F3Vertices[1].g)), 0x80000000 | (SF_IS_BOOL(&((InQuad_F3Vertices[1].g))) ? 0x08000000 : 0), "InQuad_F3Vertices[1].g" },
      { &((InQuad_F3Vertices[1].b)), SF_IS_BOOL(&((InQuad_F3Vertices[1].b))) ? 1 : sizeof((InQuad_F3Vertices[1].b)), 0x80000000 | (SF_IS_BOOL(&((InQuad_F3Vertices[1].b))) ? 0x08000000 : 0), "InQuad_F3Vertices[1].b" },
      { &((InQuad_F3Vertices[2].x)), SF_IS_BOOL(&((InQuad_F3Vertices[2].x))) ? 1 : sizeof((InQuad_F3Vertices[2].x)), 0x80000000 | (SF_IS_BOOL(&((InQuad_F3Vertices[2].x))) ? 0x08000000 : 0), "InQuad_F3Vertices[2].x" },
      { &((InQuad_F3Vertices[2].y)), SF_IS_BOOL(&((InQuad_F3Vertices[2].y))) ? 1 : sizeof((InQuad_F3Vertices[2].y)), 0x80000000 | (SF_IS_BOOL(&((InQuad_F3Vertices[2].y))) ? 0x08000000 : 0), "InQuad_F3Vertices[2].y" },
      { &((InQuad_F3Vertices[2].u)), SF_IS_BOOL(&((InQuad_F3Vertices[2].u))) ? 1 : sizeof((InQuad_F3Vertices[2].u)), 0x80000000 | (SF_IS_BOOL(&((InQuad_F3Vertices[2].u))) ? 0x08000000 : 0), "InQuad_F3Vertices[2].u" },
      { &((InQuad_F3Vertices[2].v)), SF_IS_BOOL(&((InQuad_F3Vertices[2].v))) ? 1 : sizeof((InQuad_F3Vertices[2].v)), 0x80000000 | (SF_IS_BOOL(&((InQuad_F3Vertices[2].v))) ? 0x08000000 : 0), "InQuad_F3Vertices[2].v" },
      { &((InQuad_F3Vertices[2].r)), SF_IS_BOOL(&((InQuad_F3Vertices[2].r))) ? 1 : sizeof((InQuad_F3Vertices[2].r)), 0x80000000 | (SF_IS_BOOL(&((InQuad_F3Vertices[2].r))) ? 0x08000000 : 0), "InQuad_F3Vertices[2].r" },
      { &((InQuad_F3Vertices[2].g)), SF_IS_BOOL(&((InQuad_F3Vertices[2].g))) ? 1 : sizeof((InQuad_F3Vertices[2].g)), 0x80000000 | (SF_IS_BOOL(&((InQuad_F3Vertices[2].g))) ? 0x08000000 : 0), "InQuad_F3Vertices[2].g" },
      { &((InQuad_F3Vertices[2].b)), SF_IS_BOOL(&((InQuad_F3Vertices[2].b))) ? 1 : sizeof((InQuad_F3Vertices[2].b)), 0x80000000 | (SF_IS_BOOL(&((InQuad_F3Vertices[2].b))) ? 0x08000000 : 0), "InQuad_F3Vertices[2].b" },

      { &((InQuad_clut)), SF_IS_BOOL(&((InQuad_clut))) ? 1 : sizeof((InQuad_clut)), 0x80000000 | (SF_IS_BOOL(&((InQuad_clut))) ? 0x08000000 : 0), "InQuad_clut" },

      { &((InPLine_PrevPoint.x)), SF_IS_BOOL(&((InPLine_PrevPoint.x))) ? 1 : sizeof((InPLine_PrevPoint.x)), 0x80000000 | (SF_IS_BOOL(&((InPLine_PrevPoint.x))) ? 0x08000000 : 0), "InPLine_PrevPoint.x" },
      { &((InPLine_PrevPoint.y)), SF_IS_BOOL(&((InPLine_PrevPoint.y))) ? 1 : sizeof((InPLine_PrevPoint.y)), 0x80000000 | (SF_IS_BOOL(&((InPLine_PrevPoint.y))) ? 0x08000000 : 0), "InPLine_PrevPoint.y" },
      { &((InPLine_PrevPoint.r)), SF_IS_BOOL(&((InPLine_PrevPoint.r))) ? 1 : sizeof((InPLine_PrevPoint.r)), 0x80000000 | (SF_IS_BOOL(&((InPLine_PrevPoint.r))) ? 0x08000000 : 0), "InPLine_PrevPoint.r" },
      { &((InPLine_PrevPoint.g)), SF_IS_BOOL(&((InPLine_PrevPoint.g))) ? 1 : sizeof((InPLine_PrevPoint.g)), 0x80000000 | (SF_IS_BOOL(&((InPLine_PrevPoint.g))) ? 0x08000000 : 0), "InPLine_PrevPoint.g" },
      { &((InPLine_PrevPoint.b)), SF_IS_BOOL(&((InPLine_PrevPoint.b))) ? 1 : sizeof((InPLine_PrevPoint.b)), 0x80000000 | (SF_IS_BOOL(&((InPLine_PrevPoint.b))) ? 0x08000000 : 0), "InPLine_PrevPoint.b" },

      { &((FBRW_X)), sizeof((FBRW_X)), 0x80000000 | 0, "FBRW_X" },
      { &((FBRW_Y)), sizeof((FBRW_Y)), 0x80000000 | 0, "FBRW_Y" },
      { &((FBRW_W)), sizeof((FBRW_W)), 0x80000000 | 0, "FBRW_W" },
      { &((FBRW_H)), sizeof((FBRW_H)), 0x80000000 | 0, "FBRW_H" },
      { &((FBRW_CurY)), sizeof((FBRW_CurY)), 0x80000000 | 0, "FBRW_CurY" },
      { &((FBRW_CurX)), sizeof((FBRW_CurX)), 0x80000000 | 0, "FBRW_CurX" },

      { &((DisplayMode)), SF_IS_BOOL(&((DisplayMode))) ? 1 : sizeof((DisplayMode)), 0x80000000 | (SF_IS_BOOL(&((DisplayMode))) ? 0x08000000 : 0), "DisplayMode" },
      { &((DisplayOff)), SF_IS_BOOL(&((DisplayOff))) ? 1 : sizeof((DisplayOff)), 0x80000000 | (SF_IS_BOOL(&((DisplayOff))) ? 0x08000000 : 0), "DisplayOff" },
      { &((DisplayFB_XStart)), SF_IS_BOOL(&((DisplayFB_XStart))) ? 1 : sizeof((DisplayFB_XStart)), 0x80000000 | (SF_IS_BOOL(&((DisplayFB_XStart))) ? 0x08000000 : 0), "DisplayFB_XStart" },
      { &((DisplayFB_YStart)), SF_IS_BOOL(&((DisplayFB_YStart))) ? 1 : sizeof((DisplayFB_YStart)), 0x80000000 | (SF_IS_BOOL(&((DisplayFB_YStart))) ? 0x08000000 : 0), "DisplayFB_YStart" },

      { &((HorizStart)), SF_IS_BOOL(&((HorizStart))) ? 1 : sizeof((HorizStart)), 0x80000000 | (SF_IS_BOOL(&((HorizStart))) ? 0x08000000 : 0), "HorizStart" },
      { &((HorizEnd)), SF_IS_BOOL(&((HorizEnd))) ? 1 : sizeof((HorizEnd)), 0x80000000 | (SF_IS_BOOL(&((HorizEnd))) ? 0x08000000 : 0), "HorizEnd" },

      { &((VertStart)), sizeof((VertStart)), 0x80000000 | 0, "VertStart" },
      { &((VertEnd)), sizeof((VertEnd)), 0x80000000 | 0, "VertEnd" },

      { &((DisplayFB_CurYOffset)), SF_IS_BOOL(&((DisplayFB_CurYOffset))) ? 1 : sizeof((DisplayFB_CurYOffset)), 0x80000000 | (SF_IS_BOOL(&((DisplayFB_CurYOffset))) ? 0x08000000 : 0), "DisplayFB_CurYOffset" },
      { &((DisplayFB_CurLineYReadout)), sizeof((DisplayFB_CurLineYReadout)), 0x80000000 | 0, "DisplayFB_CurLineYReadout" },

      { &((InVBlank)), 1, 0x80000000 | 0x08000000, "InVBlank" },

      { &((LinesPerField)), sizeof((LinesPerField)), 0x80000000 | 0, "LinesPerField" },
      { &((scanline)), SF_IS_BOOL(&((scanline))) ? 1 : sizeof((scanline)), 0x80000000 | (SF_IS_BOOL(&((scanline))) ? 0x08000000 : 0), "scanline" },
      { &((field)), SF_IS_BOOL(&((field))) ? 1 : sizeof((field)), 0x80000000 | (SF_IS_BOOL(&((field))) ? 0x08000000 : 0), "field" },
      { &((field_ram_readout)), SF_IS_BOOL(&((field_ram_readout))) ? 1 : sizeof((field_ram_readout)), 0x80000000 | (SF_IS_BOOL(&((field_ram_readout))) ? 0x08000000 : 0), "field_ram_readout" },
      { &((PhaseChange)), SF_IS_BOOL(&((PhaseChange))) ? 1 : sizeof((PhaseChange)), 0x80000000 | (SF_IS_BOOL(&((PhaseChange))) ? 0x08000000 : 0), "PhaseChange" },

      { &((DotClockCounter)), SF_IS_BOOL(&((DotClockCounter))) ? 1 : sizeof((DotClockCounter)), 0x80000000 | (SF_IS_BOOL(&((DotClockCounter))) ? 0x08000000 : 0), "DotClockCounter" },

      { &((GPUClockCounter)), SF_IS_BOOL(&((GPUClockCounter))) ? 1 : sizeof((GPUClockCounter)), 0x80000000 | (SF_IS_BOOL(&((GPUClockCounter))) ? 0x08000000 : 0), "GPUClockCounter" },
      { &((LineClockCounter)), SF_IS_BOOL(&((LineClockCounter))) ? 1 : sizeof((LineClockCounter)), 0x80000000 | (SF_IS_BOOL(&((LineClockCounter))) ? 0x08000000 : 0), "LineClockCounter" },
      { &((LinePhase)), SF_IS_BOOL(&((LinePhase))) ? 1 : sizeof((LinePhase)), 0x80000000 | (SF_IS_BOOL(&((LinePhase))) ? 0x08000000 : 0), "LinePhase" },

      { &((DrawTimeAvail)), SF_IS_BOOL(&((DrawTimeAvail))) ? 1 : sizeof((DrawTimeAvail)), 0x80000000 | (SF_IS_BOOL(&((DrawTimeAvail))) ? 0x08000000 : 0), "DrawTimeAvail" },

      { 0, 0, 0, 0 }
   };
   int ret = MDFNSS_StateAction(sm, load, StateRegs, "GPU");

   if(load)
   {
      RecalcTexWindowLUT();
      SimpleFIFO_SaveStatePostLoad(BlitterFIFO);

      HorizStart &= 0xFFF;
      HorizEnd &= 0xFFF;

      IRQ_Assert(IRQ_GPU, IRQPending);
   }

   return(ret);
}

}

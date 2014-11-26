#ifndef __MDFN_SURFACE_H
#define __MDFN_SURFACE_H

#define RED_SHIFT 16
#define GREEN_SHIFT 8
#define BLUE_SHIFT 0
#define ALPHA_SHIFT 24
#define MAKECOLOR(r, g, b, a) ((r << RED_SHIFT) | (g << GREEN_SHIFT) | (b << BLUE_SHIFT) | (a << ALPHA_SHIFT))

typedef struct
{
 int32 x, y, w, h;
} MDFN_Rect;


class MDFN_PixelFormat
{
 public:

 MDFN_PixelFormat();
 MDFN_PixelFormat(const uint8 p_rs, const uint8 p_gs, const uint8 p_bs, const uint8 p_as);

 uint8 Rshift;  // Bit position of the lowest bit of the red component
 uint8 Gshift;  // [...] green component
 uint8 Bshift;  // [...] blue component
 uint8 Ashift;  // [...] alpha component.

 // Gets the R/G/B/A values for the passed 32-bit surface pixel value
 INLINE void DecodeColor(uint32 value, int &r, int &g, int &b, int &a) const
 {
    r = (value >> RED_SHIFT) & 0xFF;
    g = (value >> GREEN_SHIFT) & 0xFF;
    b = (value >> BLUE_SHIFT) & 0xFF;
    a = (value >> ALPHA_SHIFT) & 0xFF;
 }

}; // MDFN_PixelFormat;

// Supports 32-bit RGBA
//  16-bit is WIP
class MDFN_Surface //typedef struct
{
 public:

 MDFN_Surface();
 MDFN_Surface(void *const p_pixels, const uint32 p_width, const uint32 p_height, const uint32 p_pitchinpix, const MDFN_PixelFormat &nf);

 ~MDFN_Surface();

 uint32 *pixels;

 // w, h, and pitch32 should always be > 0
 int32 w;
 int32 h;

 union
 {
  int32 pitch32; // In pixels, not in bytes.
  int32 pitchinpix;	// New name, new code should use this.
 };

 MDFN_PixelFormat format;
};

#endif

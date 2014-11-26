#ifndef _GIT_H
#define _GIT_H

#include <string>
#include <vector>

#include "video.h"

typedef struct
{
 const char *extension; // Example ".nes"
 const char *description; // Example "iNES Format ROM Image"
} FileExtensionSpecStruct;

#include "file.h"

typedef enum
{
 VIDSYS_NONE, // Can be used internally in system emulation code, but it is an error condition to let it continue to be
	      // after the Load() or LoadCD() function returns!
 VIDSYS_PAL,
 VIDSYS_PAL_M, // Same timing as NTSC, but uses PAL-style colour encoding
 VIDSYS_NTSC,
 VIDSYS_SECAM
} VideoSystems;

#include "state.h"
#include "settings-common.h"

typedef enum
{
 IDIT_BUTTON,		// 1-bit
 IDIT_BUTTON_CAN_RAPID, // 1-bit

 IDIT_X_AXIS,		// (mouse) 16-bits, signed - in-screen/window range: [0.0, nominal_width)
 IDIT_Y_AXIS,		// (mouse) 16-bits, signed - in-screen/window range: [0.0, nominal_height)

 IDIT_X_AXIS_REL,  // (mouse) 32-bits, signed
 IDIT_Y_AXIS_REL,  // (mouse) 32-bits, signed

 IDIT_BYTE_SPECIAL,

 IDIT_BUTTON_ANALOG, // 32-bits, 0 - 32767
 IDIT_RUMBLE,	// 32-bits, lower 8 bits are weak rumble(0-255), next 8 bits are strong rumble(0-255), 0=no rumble, 255=max rumble.  Somewhat subjective, too...
		// May extend to 16-bit each in the future.
		// It's also rather a special case of game module->driver code communication.
} InputDeviceInputType;

#define IDIT_BUTTON_ANALOG_FLAG_SQLR	0x00000001	// Denotes analog data that may need to be scaled to ensure a more squareish logical range(for emulated
							// analog sticks).

typedef struct
{
	const char *SettingName;	// No spaces, shouldbe all a-z0-9 and _. Definitely no ~!
	const char *Name;
	/*const InputDeviceInputVB VirtButton;*/
        const int ConfigOrder;          // Configuration order during in-game config process, -1 for no config.
	const InputDeviceInputType Type;
	const char *ExcludeName;	// SettingName of a button that can't be pressed at the same time as this button
					// due to physical limitations.

	const char *RotateName[3];	// 90, 180, 270
	unsigned Flags;
} InputDeviceInputInfoStruct;

typedef struct
{
 const char *ShortName;
 const char *FullName;
 const char *Description;

 //struct InputPortInfoStruct *PortExpanderDeviceInfo;
 const void *PortExpanderDeviceInfo;	// DON'T USE, IT'S NOT IMPLEMENTED PROPERLY CURRENTLY.
 int NumInputs; // Usually just the number of buttons....OR if PortExpanderDeviceInfo is non-NULL, it's the number of input
		// ports this port expander device provides.
 const InputDeviceInputInfoStruct *IDII;
} InputDeviceInfoStruct;

typedef struct
{
 const char *ShortName;
 const char *FullName;
 int NumTypes; // Number of unique input devices available for this input port
 InputDeviceInfoStruct *DeviceInfo;
 const char *DefaultDevice;	// Default device for this port.
} InputPortInfoStruct;

typedef struct
{
 int InputPorts;
 const InputPortInfoStruct *Types;
} InputInfoStruct;

enum
{
   MDFN_MSC_RESET = 0x01,
   MDFN_MSC_POWER = 0x02,
   MDFN_MSC_INSERT_COIN = 0x07,
   // If we ever support arcade systems, we'll abstract DIP switches differently...maybe.
   MDFN_MSC_TOGGLE_DIP0 = 0x10,
   MDFN_MSC_TOGGLE_DIP1,
   MDFN_MSC_TOGGLE_DIP2,
   MDFN_MSC_TOGGLE_DIP3,
   MDFN_MSC_TOGGLE_DIP4,
   MDFN_MSC_TOGGLE_DIP5,
   MDFN_MSC_TOGGLE_DIP6,
   MDFN_MSC_TOGGLE_DIP7,
   MDFN_MSC_TOGGLE_DIP8,
   MDFN_MSC_TOGGLE_DIP9,
   MDFN_MSC_TOGGLE_DIP10,
   MDFN_MSC_TOGGLE_DIP11,
   MDFN_MSC_TOGGLE_DIP12,
   MDFN_MSC_TOGGLE_DIP13,
   MDFN_MSC_TOGGLE_DIP14,
   MDFN_MSC_TOGGLE_DIP15,
   // n of DISKn translates to is emulation module specific.
   MDFN_MSC_INSERT_DISK0 = 0x20,
   MDFN_MSC_INSERT_DISK1,
   MDFN_MSC_INSERT_DISK2,
   MDFN_MSC_INSERT_DISK3,
   MDFN_MSC_INSERT_DISK4,
   MDFN_MSC_INSERT_DISK5,
   MDFN_MSC_INSERT_DISK6,
   MDFN_MSC_INSERT_DISK7,
   MDFN_MSC_INSERT_DISK8,
   MDFN_MSC_INSERT_DISK9,
   MDFN_MSC_INSERT_DISK10,
   MDFN_MSC_INSERT_DISK11,
   MDFN_MSC_INSERT_DISK12,
   MDFN_MSC_INSERT_DISK13,
   MDFN_MSC_INSERT_DISK14,
   MDFN_MSC_INSERT_DISK15,
   MDFN_MSC_INSERT_DISK = 0x30,
   MDFN_MSC_EJECT_DISK = 0x31,
   // This command should select the next disk or disk side in the set and use MDFN_DispMessage() to show which disk is selected.
   // (If it's only allowed while a disk is ejected, or not, is emulation module specific.
   MDFN_MSC_SELECT_DISK = 0x32,
   MDFN_MSC__LAST = 0x3F // WARNING: Increasing(or having the enum'd value of a command greater than this :b) this will necessitate a change to the netplay protocol.
};

struct MemoryPatch;

typedef struct
{
	// Pitch(32-bit) must be equal to width and >= the "fb_width" specified in the MDFNGI struct for the emulated system.
	// Height must be >= to the "fb_height" specified in the MDFNGI struct for the emulated system.
	// The framebuffer pointed to by surface->pixels is written to by the system emulation code.
	MDFN_Surface *surface;

	// Will be set to TRUE if the video pixel format has changed since the last call to Emulate(), FALSE otherwise.
	// Will be set to TRUE on the first call to the Emulate() function/method
	bool VideoFormatChanged;

	// Set by the system emulation code every frame, to denote the horizontal and vertical offsets of the image, and the size
	// of the image.  If the emulated system sets the elements of LineWidths, then the horizontal offset(x) and width(w) of this structure
	// are ignored while drawing the image.
	MDFN_Rect DisplayRect;

	// Pointer to an array of MDFN_Rect, number of elements = fb_height, set by the driver code.  Individual MDFN_Rect structs written
	// to by system emulation code.  If the emulated system doesn't support multiple screen widths per frame, or if you handle
	// such a situation by outputting at a constant width-per-frame that is the least-common-multiple of the screen widths, then
	// you can ignore this.  If you do wish to use this, you must set all elements every frame.
	int32 *LineWidths;

	// Set(optionally) by emulation code.  If InterlaceOn is true, then assume field height is 1/2 DisplayRect.h, and
	// only every other line in surface (with the start line defined by InterlacedField) has valid data
	// (it's up to internal Mednafen code to deinterlace it).
	bool InterlaceOn;
	bool InterlaceField;

	// Skip rendering this frame if true.  Set by the driver code.
	int skip;

	// Sound rate.  Set by driver side.
	double SoundRate;

	// Pointer to sound buffer, set by the driver code, that the emulation code should render sound to.
	// Guaranteed to be at least 500ms in length, but emulation code really shouldn't exceed 40ms or so.  Additionally, if emulation code
	// generates >= 100ms, 
	// DEPRECATED: Emulation code may set this pointer to a sound buffer internal to the emulation module.
	int16 *SoundBuf;

	// Number of frames currently in internal sound buffer.  Set by the system emulation code, to be read by the driver code.
	int32 SoundBufSize;
	int32 SoundBufSizeALMS;	// SoundBufSize value at last MidSync(), 0
				// if mid sync isn't implemented for the emulation module in use.

	// Number of cycles that this frame consumed, using MDFNGI::MasterClock as a time base.
	// Set by emulation code.
	int64 MasterCycles;
	int64 MasterCyclesALMS;	// MasterCycles value at last MidSync(), 0
				// if mid sync isn't implemented for the emulation module in use.
} EmulateSpecStruct;

#include "cdrom/cdromif.h"

typedef struct
{
 /* Private functions to Mednafen.  Do not call directly
    from the driver code, or else bad things shall happen.  Maybe.  Probably not, but don't
    do it(yet)!
 */
 // Short system name, lowercase a-z, 0-9, and _ are the only allowable characters!
 const char *shortname;

 // Full system name.  Preferably English letters, but can be UTF8
 const char *fullname;

 // Pointer to an array of FileExtensionSpecStruct, with the last entry being { NULL, NULL } to terminate the list.
 // This list is used to make best-guess choices, when calling the TestMagic*() functions would be unreasonable, such
 // as when scanning a ZIP archive for a file to load.  The list may also be used in the future for GUI file open windows.
 const FileExtensionSpecStruct *FileExtensions;

 InputInfoStruct *InputInfo;

 // Returns 1 on successful load, 0 on fatal error(deprecated: -1 on unrecognized format)
 int (*Load)(const char *name, MDFNFILE *fp);

 // Return TRUE if the file is a recognized type, FALSE if not.
 bool (*TestMagic)(const char *name, MDFNFILE *fp);

 //
 // (*CDInterfaces).size() is guaranteed to be >= 1.
 int (*LoadCD)(std::vector<CDIF *> *CDInterfaces);
 bool (*TestMagicCD)(std::vector<CDIF *> *CDInterfaces);

 void (*CloseGame)(void);

 void (*SetLayerEnableMask)(uint64 mask);	// Video
 const char *LayerNames;

 void (*SetChanEnableMask)(uint64 mask);	// Audio(TODO, placeholder)
 const char *ChanNames;

 void (*InstallReadPatch)(uint32 address);
 void (*RemoveReadPatches)(void);
 uint8 (*MemRead)(uint32 addr);

 int (*StateAction)(StateMem *sm, int load, int data_only);

 void (*Emulate)(EmulateSpecStruct *espec);
 void (*SetInput)(int port, const char *type, void *ptr);

 void (*DoSimpleCommand)(int cmd);

 const MDFNSetting *Settings;

 // Time base for EmulateSpecStruct::MasterCycles
 #define MDFN_MASTERCLOCK_FIXED(n)	((int64)((double)(n) * (1LL << 32)))
 int64 MasterClock;

 uint32 fps; // frames per second * 65536 * 256, truncated

 // multires is a hint that, if set, indicates that the system has fairly programmable video modes(particularly, the ability
 // to display multiple horizontal resolutions, such as the PCE, PC-FX, or Genesis).  In practice, it will cause the driver
 // code to set the linear interpolation on by default.
 //
 // lcm_width and lcm_height are the least common multiples of all possible
 // resolutions in the frame buffer as specified by DisplayRect/LineWidths(Ex for PCE: widths of 256, 341.333333, 512,
 // lcm = 1024)
 //
 // nominal_width and nominal_height specify the resolution that Mednafen should display
 // the framebuffer image in at 1x scaling, scaled from the dimensions of DisplayRect, and optionally the LineWidths array
 // passed through espec to the Emulate() function.
 //
 bool multires;

 int lcm_width;
 int lcm_height;

 void *dummy_separator;	//

 int nominal_width;
 int nominal_height;

 int fb_width;		// Width of the framebuffer(not necessarily width of the image).  MDFN_Surface width should be >= this.
 int fb_height;		// Height of the framebuffer passed to the Emulate() function(not necessarily height of the image)

 int soundchan; 	// Number of output sound channels.


 int rotated;

 uint8 *name;    /* Game name, UTF8 encoding */
 uint8 MD5[16];
 uint8 GameSetMD5[16];	/* A unique ID for the game set this CD belongs to, only used in PC-FX emulation. */
 bool GameSetMD5Valid; /* True if GameSetMD5 is valid. */

 uint8 StateMD5[16];	// ID to use in save state naming and netplay session IDs, if
 bool StateMD5Valid;	// StateMD5Valid is true(useful for systems with multiple BIOS revisions, e.g. PS1).

 int soundrate;  /* For Ogg Vorbis expansion sound wacky support.  0 for default. */

 VideoSystems VideoSystem;

 //int DiskLogicalCount;	// A single double-sided disk would be 2 here.
 //const char *DiskNames;	// Null-terminated.

 const char *cspecial;  /* Special cart expansion: DIP switches, barcode reader, etc. */

 // For mouse relative motion.
 double mouse_sensitivity;
 

 //
 // For absolute coordinates(IDIT_X_AXIS and IDIT_Y_AXIS), usually mapped to a mouse(hence the naming).
 //
 float mouse_scale_x, mouse_scale_y;
 float mouse_offs_x, mouse_offs_y;
} MDFNGI;

#endif

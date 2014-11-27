#ifndef __MDFN_PSX_FRONTIO_H
#define __MDFN_PSX_FRONTIO_H

class InputDevice_Multitap;

class InputDevice
{
 public:

 InputDevice();
 virtual ~InputDevice();

 virtual void Power(void);
 virtual void UpdateInput(const void *data);
 virtual int StateAction(StateMem* sm, int load, int data_only, const char* section_name);

 virtual bool RequireNoFrameskip(void);
 // Divide mouse X coordinate by pix_clock_divider in the lightgun code to get the coordinate in pixel(clocks).
 virtual int32_t GPULineHook(const int32_t line_timestamp, bool vsync, uint32 *pixels, const MDFN_PixelFormat* const format, const unsigned width, const unsigned pix_clock_offset, const unsigned pix_clock, const unsigned pix_clock_divider);

 virtual void Update(const int32_t timestamp);	// Partially-implemented, don't rely on for timing any more fine-grained than a video frame for now.
 virtual void ResetTS(void);

 void DrawCrosshairs(uint32 *pixels, const MDFN_PixelFormat* const format, const unsigned width, const unsigned pix_clock);

 //
 //
 //
 virtual void SetAMCT(bool enabled);
 virtual void SetCrosshairsColor(uint32_t color);

 //
 //
 //
 virtual void SetDTR(bool new_dtr);
 virtual bool GetDSR(void);	// Currently unused.

 virtual bool Clock(bool TxD, int32_t &dsr_pulse_delay);

 //
 //
 virtual uint8 *GetNVData() { return NULL; }
 virtual uint32_t GetNVSize(void);
 virtual void ReadNV(uint8_t *buffer, uint32_t offset, uint32_t count);
 virtual void WriteNV(const uint8_t *buffer, uint32_t offset, uint32_t count);

 //
 // Dirty count should be incremented on each call to a method this class that causes at least 1 write to occur to the
 // nonvolatile memory(IE Clock() in the correct command phase, and WriteNV()).
 //
 virtual uint64_t GetNVDirtyCount(void);
 virtual void ResetNVDirtyCount(void);

 private:
 unsigned chair_r, chair_g, chair_b;
 bool draw_chair;
 protected:
 int32 chair_x, chair_y;
};

void FrontIO_New(bool emulate_memcards_[8], bool emulate_multitap_[2]);
void FrontIO_Free();

void FrontIO_Power(void);
void FrontIO_Write(int32_t timestamp, uint32_t A, uint32_t V);
uint32_t FrontIO_Read(int32_t timestamp, uint32_t A);
int32_t FrontIO_CalcNextEventTS(int32_t timestamp, int32_t next_event);
int32_t FrontIO_Update(int32_t timestamp);
void FrontIO_ResetTS(void);

bool FrontIO_RequireNoFrameskip(void);

void FrontIO_GPULineHook(const int32_t timestamp,
      const int32_t line_timestamp, bool vsync,
      uint32 *pixels, const MDFN_PixelFormat* const format,
      const unsigned width, const unsigned pix_clock_offset,
      const unsigned pix_clock, const unsigned pix_clock_divider);

void FrontIO_UpdateInput(void);
void FrontIO_SetInput(unsigned int port, const char *type, void *ptr);
void FrontIO_SetAMCT(bool enabled);
void FrontIO_SetCrosshairsColor(unsigned port, uint32_t color);

InputDevice *FrontIO_GetMemcardDevice(unsigned int which);
uint64_t FrontIO_GetMemcardDirtyCount(unsigned int which);
void FrontIO_LoadMemcard(unsigned int which, const char *path);
void FrontIO_LoadMemcard(unsigned int which);
void FrontIO_SaveMemcard(unsigned int which, const char *path); //, bool force_save = false);
void FrontIO_SaveMemcard(unsigned int which);

int FrontIO_StateAction(StateMem* sm, int load, int data_only);

extern InputInfoStruct FIO_InputInfo;

#endif

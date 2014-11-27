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
#include "frontio.h"
#include "../msvc_compat.h"

#include "../video/surface.h"

#include "../error.h"

#define INPUTDEVICE_NONE            0
#define INPUTDEVICE_GAMEPAD         1
#define INPUTDEVICE_DUALSHOCK       2
#define INPUTDEVICE_DUALANALOG      3
#define INPUTDEVICE_ANALOG_JOYSTICK 4
#define INPUTDEVICE_MOUSE           5
#define INPUTDEVICE_NEGCON          6
#define INPUTDEVICE_GUNCON          7
#define INPUTDEVICE_JUSTIFIER       8
#define INPUTDEVICE_DANCEPAD        9

#define INPUTDEVICE_MULTITAP        10
#define INPUTDEVICE_MEMCARD         11

//#define PSX_FIODBGINFO(format, ...) { /* printf(format " -- timestamp=%d -- PAD temp\n", ## __VA_ARGS__, timestamp); */  }
static void PSX_FIODBGINFO(const char *format, ...)
{
}


static bool emulate_memcards[8];
static bool emulate_multitap[2];

static InputDevice *Ports[2];
static unsigned     PortsType[2];
static InputDevice *MCPorts[2];
static unsigned     MCPortsType[2];

static InputDevice *DummyDevice;
static InputDevice_Multitap *DevicesTap[2];

static InputDevice *DevicesMC[8];
static InputDevice *Devices[8];
static unsigned DevicesType[8];

static void *DeviceData[8];

static int32_t ClockDivider;

static bool ReceivePending;
static bool TransmitPending;

static bool ReceiveInProgress;
static bool TransmitInProgress;

static bool ReceiveBufferAvail;

static uint8_t ReceiveBuffer;
static uint8_t TransmitBuffer;

static int32_t ReceiveBitCounter;
static int32_t TransmitBitCounter;

static uint16_t Mode;
static uint16_t Control;
static uint16_t Baudrate;

static bool istatus;
 //
 //
static int32_t irq10_pulse_ts[2];

static int32_t dsr_pulse_delay[4];
static int32_t dsr_active_until_ts[4];
static int32_t lastts;

static bool amct_enabled;
static uint32_t chair_colors[8];

class InputDevice_Gamepad : public InputDevice
{
 public:

 InputDevice_Gamepad();
 virtual ~InputDevice_Gamepad();

 virtual void Power(void);
 virtual int StateAction(StateMem* sm, int load, int data_only, const char* section_name);

 //
 //
 //
 virtual void SetDTR(bool new_dtr);
 virtual bool Clock(bool TxD, int32 &dsr_pulse_delay);

 bool dtr;

 uint8 buttons[2];

 int32 command_phase;
 uint32 bitpos;
 uint8 receive_buffer;

 uint8 command;

 uint8 transmit_buffer[3];
 uint32 transmit_pos;
 uint32 transmit_count;
};

class InputDevice_DualAnalog : public InputDevice
{
 public:

 InputDevice_DualAnalog(bool joystick_mode_);
 virtual ~InputDevice_DualAnalog();

 virtual void Power(void);
 virtual int StateAction(StateMem* sm, int load, int data_only, const char* section_name);

 //
 //
 //
 virtual void SetDTR(bool new_dtr);
 virtual bool Clock(bool TxD, int32 &dsr_pulse_delay);

 bool joystick_mode;
 bool dtr;

 uint8 buttons[2];
 uint8 axes[2][2];

 int32 command_phase;
 uint32 bitpos;
 uint8 receive_buffer;

 uint8 command;

 uint8 transmit_buffer[8];
 uint32 transmit_pos;
 uint32 transmit_count;
};

class InputDevice_DualShock : public InputDevice
{
 public:

 InputDevice_DualShock(const std::string &arg_name);
 virtual ~InputDevice_DualShock();

 virtual void Power(void);
 virtual int StateAction(StateMem* sm, int load, int data_only, const char* section_name);
 virtual void Update(const int32_t timestamp);
 virtual void ResetTS(void);

 virtual void SetAMCT(bool enabled);
 //
 //
 //
 virtual void SetDTR(bool new_dtr);
 virtual bool Clock(bool TxD, int32 &dsr_pulse_delay);

 void CheckManualAnaModeChange(void);

 //
 //
 bool cur_ana_button_state;
 bool prev_ana_button_state;
 int64 combo_anatoggle_counter;
 //

 bool da_rumble_compat;

 bool analog_mode;
 bool analog_mode_locked;

 bool mad_munchkins;
 uint8 rumble_magic[6];

 uint8 rumble_param[2];

 bool dtr;

 uint8 buttons[2];
 uint8 axes[2][2];

 int32 command_phase;
 uint32 bitpos;
 uint8 receive_buffer;

 uint8 command;

 uint8 transmit_buffer[8];
 uint32 transmit_pos;
 uint32 transmit_count;

 //
 //
 //
 bool am_prev_info;
 bool aml_prev_info;
 std::string gp_name;
 int32_t lastts;

 //
 //
 bool amct_enabled;

};

class InputDevice_Multitap : public InputDevice
{
 public:

 InputDevice_Multitap();
 virtual ~InputDevice_Multitap();
 virtual void Power(void);
 virtual int StateAction(StateMem* sm, int load, int data_only, const char* section_name);

 void SetSubDevice(unsigned int sub_index, InputDevice *device, InputDevice *mc_device);

 //
 //
 //
 virtual void SetDTR(bool new_dtr);
 virtual bool Clock(bool TxD, int32 &dsr_pulse_delay);

 InputDevice *pad_devices[4];
 InputDevice *mc_devices[4];

 bool dtr;

 int selected_device;
 bool full_mode_setting;

 bool full_mode;
 bool mc_mode;

 uint8 fm_dp;	// Device-present.
 uint8 fm_buffer[4][8];

 bool fm_deferred_error_temp;
 bool fm_deferred_error;
 bool fm_command_error;

 uint8 command;
 uint8 receive_buffer;
 uint8 bit_counter;
 uint8 byte_counter;
};

class InputDevice_neGcon : public InputDevice
{
 public:

 InputDevice_neGcon(void);
 virtual ~InputDevice_neGcon();

 virtual void Power(void);

 //
 //
 //
 virtual void SetDTR(bool new_dtr);
 virtual bool Clock(bool TxD, int32 &dsr_pulse_delay);

 bool dtr;

 uint8 buttons[2];
 uint8 twist;
 uint8 anabuttons[3];

 int32 command_phase;
 uint32 bitpos;
 uint8 receive_buffer;

 uint8 command;

 uint8 transmit_buffer[8];
 uint32 transmit_pos;
 uint32 transmit_count;
};

class InputDevice_GunCon : public InputDevice
{
 public:

 InputDevice_GunCon(void);
 virtual ~InputDevice_GunCon();

 virtual void Power(void);
 virtual int StateAction(StateMem* sm, int load, int data_only, const char* section_name);
 virtual bool RequireNoFrameskip(void);
 virtual int32_t GPULineHook(const int32_t line_timestamp, bool vsync, uint32 *pixels, const MDFN_PixelFormat* const format, const unsigned width, const unsigned pix_clock_offset, const unsigned pix_clock, const unsigned pix_clock_divider);

 //
 //
 //
 virtual void SetDTR(bool new_dtr);
 virtual bool Clock(bool TxD, int32 &dsr_pulse_delay);

 bool dtr;

 uint8 buttons;
 bool trigger_eff;
 bool trigger_noclear;
 uint16 hit_x, hit_y;

 int16 nom_x, nom_y;
 int32 os_shot_counter;
 bool prev_oss;

 int32 command_phase;
 uint32 bitpos;
 uint8 receive_buffer;

 uint8 command;

 uint8 transmit_buffer[16];
 uint32 transmit_pos;
 uint32 transmit_count;

 //
 // Video timing stuff
 bool prev_vsync;
 int line_counter;
};


InputDevice_DualAnalog::InputDevice_DualAnalog(bool joystick_mode_) : joystick_mode(joystick_mode_)
{
 Power();
}

InputDevice_DualAnalog::~InputDevice_DualAnalog()
{

}

void InputDevice_DualAnalog::Power(void)
{
 dtr = 0;

 buttons[0] = buttons[1] = 0;

 command_phase = 0;

 bitpos = 0;

 receive_buffer = 0;

 command = 0;

 memset(transmit_buffer, 0, sizeof(transmit_buffer));

 transmit_pos = 0;
 transmit_count = 0;
}

int InputDevice_DualAnalog::StateAction(StateMem* sm, int load, int data_only, const char* section_name)
{
 SFORMAT StateRegs[] =
 {
  { &((dtr)), 1, 0x80000000 | 0x08000000, "dtr" },

  { ((buttons)), (uint32)((sizeof(buttons))), 0 | 0, "buttons" },
  { ((&axes[0][0])), (uint32)((sizeof(axes))), 0 | 0, "&axes[0][0]" },

  { &((command_phase)), sizeof((command_phase)), 0x80000000 | 0, "command_phase" },
  { &((bitpos)), sizeof((bitpos)), 0x80000000 | 0, "bitpos" },
  { &((receive_buffer)), sizeof((receive_buffer)), 0x80000000 | 0, "receive_buffer" },

  { &((command)), sizeof((command)), 0x80000000 | 0, "command" },

  { ((transmit_buffer)), (uint32)((sizeof(transmit_buffer))), 0 | 0, "transmit_buffer" },
  { &((transmit_pos)), sizeof((transmit_pos)), 0x80000000 | 0, "transmit_pos" },
  { &((transmit_count)), sizeof((transmit_count)), 0x80000000 | 0, "transmit_count" },

  { 0, 0, 0, 0 }
 };
 int ret = MDFNSS_StateAction(sm, load, StateRegs, section_name);

 if(load)
 {
  if((transmit_pos + transmit_count) > sizeof(transmit_buffer))
  {
   transmit_pos = 0;
   transmit_count = 0;
  }
 }

 return(ret);
}

void InputDevice_DualAnalog::SetDTR(bool new_dtr)
{
 if(!dtr && new_dtr)
 {
  command_phase = 0;
  bitpos = 0;
  transmit_pos = 0;
  transmit_count = 0;
 }
#if 0
 else if(dtr && !new_dtr)
 {
  //if(bitpos || transmit_count)
  // printf("[PAD] Abort communication!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
 }
#endif

 dtr = new_dtr;
}

bool InputDevice_DualAnalog::Clock(bool TxD, int32 &dsr_pulse_delay)
{
 bool ret = 1;

 dsr_pulse_delay = 0;

 if(!dtr)
  return(1);

 if(transmit_count)
  ret = (transmit_buffer[transmit_pos] >> bitpos) & 1;

 receive_buffer &= ~(1 << bitpos);
 receive_buffer |= TxD << bitpos;
 bitpos = (bitpos + 1) & 0x7;

 if(!bitpos)
 {
  //printf("[PAD] Receive: %02x -- command_phase=%d\n", receive_buffer, command_phase);

  if(transmit_count)
  {
   transmit_pos++;
   transmit_count--;
  }


  switch(command_phase)
  {
   case 0:
 	  if(receive_buffer != 0x01)
	    command_phase = -1;
	  else
	  {
	   transmit_buffer[0] = joystick_mode ? 0x53 : 0x73;
	   transmit_pos = 0;
	   transmit_count = 1;
	   command_phase++;
	  }
	  break;

   case 1:
	command = receive_buffer;
	command_phase++;

	transmit_buffer[0] = 0x5A;

	//if(command != 0x42)
	// fprintf(stderr, "Gamepad unhandled command: 0x%02x\n", command);

	if(command == 0x42)
	{
	 transmit_buffer[1] = 0xFF ^ buttons[0];
	 transmit_buffer[2] = 0xFF ^ buttons[1];
	 transmit_buffer[3] = axes[0][0];
	 transmit_buffer[4] = axes[0][1];
	 transmit_buffer[5] = axes[1][0];
	 transmit_buffer[6] = axes[1][1];
         transmit_pos = 0;
         transmit_count = 7;
	}
	else
	{
	 command_phase = -1;
	 transmit_buffer[1] = 0;
	 transmit_buffer[2] = 0;
         transmit_pos = 0;
         transmit_count = 0;
	}
	break;
   case 2:
	//if(receive_buffer)
	// printf("%d: %02x\n", 7 - transmit_count, receive_buffer);
	break;
  }
 }

 if(!bitpos && transmit_count)
  dsr_pulse_delay = 0x40; //0x100;

 return(ret);
}

InputDeviceInputInfoStruct Device_DualAnalog_IDII[24] =
{
 { "select", "SELECT", 4, IDIT_BUTTON, NULL },
 { "l3", "Left Stick, Button(L3)", 18, IDIT_BUTTON, NULL },
 { "r3", "Right stick, Button(R3)", 23, IDIT_BUTTON, NULL },
 { "start", "START", 5, IDIT_BUTTON, NULL },
 { "up", "D-Pad UP ↑", 0, IDIT_BUTTON, "down" },
 { "right", "D-Pad RIGHT →", 3, IDIT_BUTTON, "left" },
 { "down", "D-Pad DOWN ↓", 1, IDIT_BUTTON, "up" },
 { "left", "D-Pad LEFT ←", 2, IDIT_BUTTON, "right" },

 { "l2", "L2 (rear left shoulder)", 11, IDIT_BUTTON, NULL },
 { "r2", "R2 (rear right shoulder)", 13, IDIT_BUTTON, NULL },
 { "l1", "L1 (front left shoulder)", 10, IDIT_BUTTON, NULL },
 { "r1", "R1 (front right shoulder)", 12, IDIT_BUTTON, NULL },

 { "triangle", "△ (upper)", 6, IDIT_BUTTON_CAN_RAPID, NULL },
 { "circle", "○ (right)", 9, IDIT_BUTTON_CAN_RAPID, NULL },
 { "cross", "x (lower)", 7, IDIT_BUTTON_CAN_RAPID, NULL },
 { "square", "□ (left)", 8, IDIT_BUTTON_CAN_RAPID, NULL },

 { "rstick_right", "Right Stick RIGHT →", 22, IDIT_BUTTON_ANALOG, NULL, { NULL, NULL, NULL }, IDIT_BUTTON_ANALOG_FLAG_SQLR },
 { "rstick_left", "Right Stick LEFT ←", 21, IDIT_BUTTON_ANALOG, NULL, { NULL, NULL, NULL }, IDIT_BUTTON_ANALOG_FLAG_SQLR },
 { "rstick_down", "Right Stick DOWN ↓", 20, IDIT_BUTTON_ANALOG, NULL, { NULL, NULL, NULL }, IDIT_BUTTON_ANALOG_FLAG_SQLR },
 { "rstick_up", "Right Stick UP ↑", 19, IDIT_BUTTON_ANALOG, NULL, { NULL, NULL, NULL }, IDIT_BUTTON_ANALOG_FLAG_SQLR },

 { "lstick_right", "Left Stick RIGHT →", 17, IDIT_BUTTON_ANALOG, NULL, { NULL, NULL, NULL }, IDIT_BUTTON_ANALOG_FLAG_SQLR },
 { "lstick_left", "Left Stick LEFT ←", 16, IDIT_BUTTON_ANALOG, NULL, { NULL, NULL, NULL }, IDIT_BUTTON_ANALOG_FLAG_SQLR },
 { "lstick_down", "Left Stick DOWN ↓", 15, IDIT_BUTTON_ANALOG, NULL, { NULL, NULL, NULL }, IDIT_BUTTON_ANALOG_FLAG_SQLR },
 { "lstick_up", "Left Stick UP ↑", 14, IDIT_BUTTON_ANALOG, NULL, { NULL, NULL, NULL }, IDIT_BUTTON_ANALOG_FLAG_SQLR },

};

// Not sure if all these buttons are named correctly!
InputDeviceInputInfoStruct Device_AnalogJoy_IDII[24] =
{
 { "select", "SELECT", 8, IDIT_BUTTON, NULL },
 { NULL, "empty", 0, IDIT_BUTTON },
 { NULL, "empty", 0, IDIT_BUTTON },
 { "start", "START", 9, IDIT_BUTTON, NULL },
 { "up", "Thumbstick UP ↑", 14, IDIT_BUTTON, "down" },
 { "right", "Thumbstick RIGHT →", 17, IDIT_BUTTON, "left" },
 { "down", "Thumbstick DOWN ↓", 15, IDIT_BUTTON, "up" },
 { "left", "Thumbstick LEFT ←", 16, IDIT_BUTTON, "right" },

 { "l2", "Left stick, Trigger", 2, IDIT_BUTTON, NULL },
 { "r2", "Left stick, Pinky", 3, IDIT_BUTTON, NULL },
 { "l1", "Left stick, L-thumb", 0, IDIT_BUTTON, NULL },
 { "r1", "Left stick, R-thumb", 1, IDIT_BUTTON, NULL },

 { "triangle", "Right stick, Pinky", 13, IDIT_BUTTON, NULL },
 { "circle", "Right stick, R-thumb", 11, IDIT_BUTTON, NULL },
 { "cross",  "Right stick, L-thumb", 10, IDIT_BUTTON, NULL },
 { "square", "Right stick, Trigger", 12, IDIT_BUTTON, NULL },

 { "rstick_right", "Right Stick, RIGHT →", 21, IDIT_BUTTON_ANALOG, NULL, { NULL, NULL, NULL }, IDIT_BUTTON_ANALOG_FLAG_SQLR },
 { "rstick_left", "Right Stick, LEFT ←", 20, IDIT_BUTTON_ANALOG, NULL, { NULL, NULL, NULL }, IDIT_BUTTON_ANALOG_FLAG_SQLR },
 { "rstick_down", "Right Stick, BACK ↓", 19, IDIT_BUTTON_ANALOG, NULL, { NULL, NULL, NULL }, IDIT_BUTTON_ANALOG_FLAG_SQLR },
 { "rstick_up", "Right Stick, FORE ↑", 18, IDIT_BUTTON_ANALOG, NULL, { NULL, NULL, NULL }, IDIT_BUTTON_ANALOG_FLAG_SQLR },

 { "lstick_right", "Left Stick, RIGHT →", 7, IDIT_BUTTON_ANALOG, NULL, { NULL, NULL, NULL }, IDIT_BUTTON_ANALOG_FLAG_SQLR },
 { "lstick_left", "Left Stick, LEFT ←", 6, IDIT_BUTTON_ANALOG, NULL, { NULL, NULL, NULL }, IDIT_BUTTON_ANALOG_FLAG_SQLR },
 { "lstick_down", "Left Stick, BACK ↓", 5, IDIT_BUTTON_ANALOG, NULL, { NULL, NULL, NULL }, IDIT_BUTTON_ANALOG_FLAG_SQLR },
 { "lstick_up", "Left Stick, FORE ↑", 4, IDIT_BUTTON_ANALOG, NULL, { NULL, NULL, NULL }, IDIT_BUTTON_ANALOG_FLAG_SQLR },

};

/*
   TODO:
	If we ever call Update() more than once per video frame(IE 50/60Hz), we'll need to add debounce logic to the analog mode button evaluation code.
*/

/* Notes:

     Both DA and DS style rumblings work in both analog and digital modes.

     Regarding getting Dual Shock style rumble working, Sony is evil and/or mean.  The owl tells me to burn Sony with boiling oil.

     To enable Dual Shock-style rumble, the game has to at least enter MAD MUNCHKINS MODE with command 0x43, and send the appropriate data(not the actual rumble type data per-se)
     with command 0x4D.

     DualAnalog-style rumble support seems to be borked until power loss if MAD MUNCHKINS MODE is even entered once...investigate further.

     Command 0x44 in MAD MUNCHKINS MODE can turn on/off analog mode(and the light with it).

     Command 0x42 in MAD MUNCHKINS MODE will return the analog mode style gamepad data, even when analog mode is off.  In combination with command 0x44, this could hypothetically
     be used for using the light in the gamepad as some kind of game mechanic).

     Dual Analog-style rumble notes(some of which may apply to DS too):
	Rumble appears to stop if you hold DTR active(TODO: for how long? instant?). (TODO: investigate if it's still stopped even if a memory card device number is sent.  It may be, since rumble may
							      cause excessive current draw in combination with memory card access)

	Rumble will work even if you interrupt the communication process after you've sent the rumble data(via command 0x42).
		Though if you interrupt it when you've only sent partial rumble data, dragons will eat you and I don't know(seems to have timing-dependent or random effects or something;
	        based on VERY ROUGH testing).
*/


InputDevice_DualShock::InputDevice_DualShock(const std::string &name)
{
 gp_name = name;
 Power();
 am_prev_info = analog_mode;
 aml_prev_info = analog_mode_locked;
 amct_enabled = false;
}

InputDevice_DualShock::~InputDevice_DualShock()
{

}

void InputDevice_DualShock::Update(const int32_t timestamp)
{
 lastts = timestamp;
}

void InputDevice_DualShock::ResetTS(void)
{
 //printf("%lld\n", combo_anatoggle_counter);
 if(combo_anatoggle_counter >= 0)
  combo_anatoggle_counter += lastts;
 lastts = 0;
}

void InputDevice_DualShock::SetAMCT(bool enabled)
{
 amct_enabled = enabled;
 if(amct_enabled)
     analog_mode = false;
 else
     analog_mode = true;

  MDFN_DispMessage(_("%s: Analog toggle is %s, sticks are %s"), gp_name.c_str(), amct_enabled ? _("ENABLED") : _("DISABLED"), analog_mode ? _("ON") : _("OFF"));  
}

//
// This simulates the behavior of the actual DualShock(analog toggle button evaluation is suspended while DTR is active).
// Call in Update(), and whenever dtr goes inactive in the port access code.
void InputDevice_DualShock::CheckManualAnaModeChange(void)
{
 if(!dtr)
 {
  bool need_mode_toggle = false;

  if(amct_enabled)
  {
   if(buttons[0] == 0x09 && buttons[1] == 0x0f)
   {
    if(combo_anatoggle_counter == -1)
     combo_anatoggle_counter = 0;
    else if(combo_anatoggle_counter >= (44100 * 768))
    {
     need_mode_toggle = true;
     combo_anatoggle_counter = -2;
    }
   }
   else
    combo_anatoggle_counter = -1;
  }  
  else
  {
   combo_anatoggle_counter = -1;
   if(cur_ana_button_state && (cur_ana_button_state != prev_ana_button_state))
   {
    need_mode_toggle = true;
   }
  }

  if(need_mode_toggle)
  {
   if(analog_mode_locked)
   {
     //MDFN_DispMessage(_("%s: Analog mode is  %s."), gp_name.c_str(), analog_mode ? _("on") : _("off"));
     MDFN_DispMessage(_("%s: 2 Analog toggle is DISABLED, sticks are %s"), gp_name.c_str(), analog_mode ? _("ON") : _("OFF"));         
   }
   else
    analog_mode = !analog_mode;
  }

  prev_ana_button_state = cur_ana_button_state; 	// Don't move this outside of the if(!dtr) block!
 }
}

void InputDevice_DualShock::Power(void)
{
 combo_anatoggle_counter = -2;
 lastts = 0;
 //
 //

 dtr = 0;

 buttons[0] = buttons[1] = 0;

 command_phase = 0;

 bitpos = 0;

 receive_buffer = 0;

 command = 0;

 memset(transmit_buffer, 0, sizeof(transmit_buffer));

 transmit_pos = 0;
 transmit_count = 0;

 analog_mode_locked = false;

 mad_munchkins = false;
 memset(rumble_magic, 0xFF, sizeof(rumble_magic));
 memset(rumble_param, 0, sizeof(rumble_param));

 da_rumble_compat = true;

 prev_ana_button_state = false;
}

int InputDevice_DualShock::StateAction(StateMem* sm, int load, int data_only, const char* section_name)
{
 SFORMAT StateRegs[] =
 {
  { &((cur_ana_button_state)), 1, 0x80000000 | 0x08000000, "cur_ana_button_state" },
  { &((prev_ana_button_state)), 1, 0x80000000 | 0x08000000, "prev_ana_button_state" },
  { &((combo_anatoggle_counter)), sizeof((combo_anatoggle_counter)), 0x80000000 | 0, "combo_anatoggle_counter" },

  { &((da_rumble_compat)), 1, 0x80000000 | 0x08000000, "da_rumble_compat" },

  { &((analog_mode)), 1, 0x80000000 | 0x08000000, "analog_mode" },
  { &((analog_mode_locked)), 1, 0x80000000 | 0x08000000, "analog_mode_locked" },

  { &((mad_munchkins)), 1, 0x80000000 | 0x08000000, "mad_munchkins" },
  { ((rumble_magic)), (uint32)((sizeof(rumble_magic))), 0 | 0, "rumble_magic" },

  { ((rumble_param)), (uint32)((sizeof(rumble_param))), 0 | 0, "rumble_param" },

  { &((dtr)), 1, 0x80000000 | 0x08000000, "dtr" },

  { ((buttons)), (uint32)((sizeof(buttons))), 0 | 0, "buttons" },
  { ((&axes[0][0])), (uint32)((sizeof(axes))), 0 | 0, "&axes[0][0]" },

  { &((command_phase)), sizeof((command_phase)), 0x80000000 | 0, "command_phase" },
  { &((bitpos)), sizeof((bitpos)), 0x80000000 | 0, "bitpos" },
  { &((receive_buffer)), sizeof((receive_buffer)), 0x80000000 | 0, "receive_buffer" },

  { &((command)), sizeof((command)), 0x80000000 | 0, "command" },

  { ((transmit_buffer)), (uint32)((sizeof(transmit_buffer))), 0 | 0, "transmit_buffer" },
  { &((transmit_pos)), sizeof((transmit_pos)), 0x80000000 | 0, "transmit_pos" },
  { &((transmit_count)), sizeof((transmit_count)), 0x80000000 | 0, "transmit_count" },

  { 0, 0, 0, 0 }
 };
 int ret = MDFNSS_StateAction(sm, load, StateRegs, section_name);

 if(load)
 {
  if((transmit_pos + transmit_count) > sizeof(transmit_buffer))
  {
   transmit_pos = 0;
   transmit_count = 0;
  }
 }

 return(ret);
}

void InputDevice_DualShock::SetDTR(bool new_dtr)
{
 const bool old_dtr = dtr;
 dtr = new_dtr;	// Set it to new state before we call CheckManualAnaModeChange().

 if(!old_dtr && dtr)
 {
  command_phase = 0;
  bitpos = 0;
  transmit_pos = 0;
  transmit_count = 0;
 }
 else if(old_dtr && !dtr)
 {
  CheckManualAnaModeChange();
  //if(bitpos || transmit_count)
  // printf("[PAD] Abort communication!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
 }
}

bool InputDevice_DualShock::Clock(bool TxD, int32 &dsr_pulse_delay)
{
 bool ret = 1;

 dsr_pulse_delay = 0;

 if(!dtr)
  return(1);

 if(transmit_count)
  ret = (transmit_buffer[transmit_pos] >> bitpos) & 1;

 receive_buffer &= ~(1 << bitpos);
 receive_buffer |= TxD << bitpos;
 bitpos = (bitpos + 1) & 0x7;

 if(!bitpos)
 {
  //if(command == 0x44)
  //if(command == 0x4D) //mad_munchkins) // || command == 0x43)
  // fprintf(stderr, "[PAD] Receive: %02x -- command=%02x, command_phase=%d, transmit_pos=%d\n", receive_buffer, command, command_phase, transmit_pos);

  if(transmit_count)
  {
   transmit_pos++;
   transmit_count--;
  }

  switch(command_phase)
  {
   case 0:
 	  if(receive_buffer != 0x01)
	    command_phase = -1;
	  else
	  {
	   if(mad_munchkins)
	   {
	    transmit_buffer[0] = 0xF3;
	    transmit_pos = 0;
	    transmit_count = 1;
	    command_phase = 101;
	   }
	   else
	   {
	    transmit_buffer[0] = analog_mode ? 0x73 : 0x41;
	    transmit_pos = 0;
	    transmit_count = 1;
	    command_phase++;
	   }
	  }
	  break;

   case 1:
	command = receive_buffer;
	command_phase++;

	transmit_buffer[0] = 0x5A;

	//fprintf(stderr, "Gamepad command: 0x%02x\n", command);
	//if(command != 0x42 && command != 0x43)
	// fprintf(stderr, "Gamepad unhandled command: 0x%02x\n", command);

	if(command == 0x42)
	{
	 transmit_buffer[0] = 0x5A;
	 transmit_pos = 0;
	 transmit_count = 1;
	 command_phase = (command << 8) | 0x00;
	}
	else if(command == 0x43)
	{
	 transmit_pos = 0;
	 if(analog_mode)
	 {
	  transmit_buffer[1] = 0xFF ^ buttons[0];
	  transmit_buffer[2] = 0xFF ^ buttons[1];
	  transmit_buffer[3] = axes[0][0];
	  transmit_buffer[4] = axes[0][1];
	  transmit_buffer[5] = axes[1][0];
	  transmit_buffer[6] = axes[1][1];
          transmit_count = 7;
	 }
	 else
	 {
	  transmit_buffer[1] = 0xFF ^ buttons[0];
 	  transmit_buffer[2] = 0xFF ^ buttons[1];
 	  transmit_count = 3;
	 }
	}
	else
	{
	 command_phase = -1;
	 transmit_buffer[1] = 0;
	 transmit_buffer[2] = 0;
         transmit_pos = 0;
         transmit_count = 0;
	}
	break;

   case 2:
	{
	 if(command == 0x43 && transmit_pos == 2 && (receive_buffer == 0x01))
	 {
	  //fprintf(stderr, "Mad Munchkins mode entered!\n");
	  mad_munchkins = true;

	  if(da_rumble_compat)
	  {
	   rumble_param[0] = 0;
	   rumble_param[1] = 0;
	   da_rumble_compat = false;
	  }
	  command_phase = -1;
	 }
	}
	break;

   case 101:
	command = receive_buffer;

	//fprintf(stderr, "Mad Munchkins DualShock command: 0x%02x\n", command);

	if(command >= 0x40 && command <= 0x4F)
	{
	 transmit_buffer[0] = 0x5A;
	 transmit_pos = 0;
	 transmit_count = 1;
	 command_phase = (command << 8) | 0x00;
	}
	else
	{
	 transmit_count = 0;
	 command_phase = -1;
	}
	break;

  /************************/
  /* MMMode 1, Command 0x40 */
  /************************/
  case 0x4000:
	if(receive_buffer == 0x00)
	{
	 transmit_buffer[0] = 0; /**/ transmit_pos = 0; transmit_count = 1; /**/
	 command_phase++;
	}
	else
	 command_phase = -1;
	break;

  case 0x4001:
	transmit_buffer[0] = 0x00;
	transmit_buffer[1] = 0x00;
	transmit_buffer[2] = 0x00;
	transmit_buffer[3] = 0x00;
	transmit_buffer[4] = 0x00;
	transmit_pos = 0;
	transmit_count = 5;
	command_phase = -1;
	break;


  /************************/
  /* MMMode 1, Command 0x41 */
  /************************/
  case 0x4100:
	if(receive_buffer == 0x00)
	{
	 transmit_buffer[0] = 0; /**/ transmit_pos = 0; transmit_count = 1; /**/
	 command_phase++;
	}
	else
	 command_phase = -1;
	break;

  case 0x4101:
	transmit_buffer[0] = 0x00;
	transmit_buffer[1] = 0x00;
	transmit_buffer[2] = 0x00;
	transmit_buffer[3] = 0x00;
	transmit_buffer[4] = 0x00;
	transmit_pos = 0;
	transmit_count = 5;
	command_phase = -1;
	break;

  /**************************/
  /* MMMode 0&1, Command 0x42 */
  /**************************/
  case 0x4200:
	transmit_pos = 0;
	if(analog_mode || mad_munchkins)
	{
	 transmit_buffer[0] = 0xFF ^ buttons[0];
	 transmit_buffer[1] = 0xFF ^ buttons[1];
	 transmit_buffer[2] = axes[0][0];
	 transmit_buffer[3] = axes[0][1];
	 transmit_buffer[4] = axes[1][0];
	 transmit_buffer[5] = axes[1][1];
 	 transmit_count = 6;
	}
	else
	{
	 transmit_buffer[0] = 0xFF ^ buttons[0];
	 transmit_buffer[1] = 0xFF ^ buttons[1];
	 transmit_count = 2;

	 if(!(rumble_magic[2] & 0xFE))
	 {
	  transmit_buffer[transmit_count++] = 0x00;
	  transmit_buffer[transmit_count++] = 0x00;
	 }
	}
	command_phase++;
	break;
 
  case 0x4201:			// Weak(in DS mode)
	if(da_rumble_compat)
	 rumble_param[0] = receive_buffer;
	// Dualshock weak
	else if(rumble_magic[0] == 0x00 && rumble_magic[2] != 0x00 && rumble_magic[3] != 0x00 && rumble_magic[4] != 0x00 && rumble_magic[5] != 0x00)
	 rumble_param[0] = receive_buffer;
	command_phase++;
	break;

  case 0x4202:
	if(da_rumble_compat)
	 rumble_param[1] = receive_buffer;
	else if(rumble_magic[1] == 0x01)	// DualShock strong
	 rumble_param[1] = receive_buffer;
	else if(rumble_magic[1] == 0x00 && rumble_magic[2] != 0x00 && rumble_magic[3] != 0x00 && rumble_magic[4] != 0x00 && rumble_magic[5] != 0x00)	// DualShock weak
	 rumble_param[0] = receive_buffer;

	command_phase++;
	break;

  case 0x4203:
	if(da_rumble_compat)
	{

	}
	else if(rumble_magic[1] == 0x00 && rumble_magic[2] == 0x01)
	 rumble_param[1] = receive_buffer;	// DualShock strong.
	command_phase++;	// Nowhere here we come!
	break;

  /************************/
  /* MMMode 1, Command 0x43 */
  /************************/
  case 0x4300:
	if(receive_buffer == 0x00)
	{
	 transmit_buffer[0] = 0; /**/ transmit_pos = 0; transmit_count = 1; /**/
	 command_phase++;
	}
	else
	 command_phase = -1;
	break;

  case 0x4301:
	if(receive_buffer == 0x00)
	{
	 //fprintf(stderr, "Mad Munchkins mode left!\n");
	 mad_munchkins = false;
	}
	transmit_buffer[0] = 0x00;
	transmit_buffer[1] = 0x00;
	transmit_buffer[2] = 0x00;
	transmit_buffer[3] = 0x00;
	transmit_buffer[4] = 0x00;
	transmit_pos = 0;
	transmit_count = 5;
	command_phase = -1;
	break;

  /************************/
  /* MMMode 1, Command 0x44 */
  /************************/
  case 0x4400:
	if(receive_buffer == 0x00)
	{
	 transmit_buffer[0] = 0; /**/ transmit_pos = 0; transmit_count = 1; /**/
	 command_phase++;
	}
	else
	 command_phase = -1;
	break;

  case 0x4401:
	transmit_buffer[0] = 0x00;
	transmit_buffer[1] = 0x00;
	transmit_buffer[2] = 0x00;
	transmit_buffer[3] = 0x00;
	transmit_buffer[4] = 0x00;
	transmit_pos = 0;
	transmit_count = 5;
	command_phase++;

	// Ignores locking state.
	switch(receive_buffer)
	{
	 case 0x00:
		analog_mode = false;
		//fprintf(stderr, "Analog mode disabled\n");
		break;

	 case 0x01:
		analog_mode = true;
		//fprintf(stderr, "Analog mode enabled\n");
		break;
	}
	break;

  case 0x4402:
	switch(receive_buffer)
	{
	 case 0x02:
		analog_mode_locked = false;
		break;

	 case 0x03:
		analog_mode_locked = true;
		break;
	}
	command_phase = -1;
	break;

  /************************/
  /* MMMode 1, Command 0x45 */
  /************************/
  case 0x4500:
	if(receive_buffer == 0x00)
	{
	 transmit_buffer[0] = 0x01; /**/ transmit_pos = 0; transmit_count = 1; /**/
	 command_phase++;
	}
	else
	 command_phase = -1;
	break;

  case 0x4501:
	transmit_buffer[0] = 0x02;
	transmit_buffer[1] = analog_mode ? 0x01 : 0x00;
	transmit_buffer[2] = 0x02;
	transmit_buffer[3] = 0x01;
	transmit_buffer[4] = 0x00;
	transmit_pos = 0;
	transmit_count = 5;
	command_phase = -1;
	break;


  /************************/
  /* MMMode 1, Command 0x46 */
  /************************/
  case 0x4600:
	if(receive_buffer == 0x00)
	{
	 transmit_buffer[0] = 0; /**/ transmit_pos = 0; transmit_count = 1; /**/
	 command_phase++;
	}
	else
	 command_phase = -1;
	break;

  case 0x4601:
	if(receive_buffer == 0x00)
	{
	 transmit_buffer[0] = 0x00;
	 transmit_buffer[1] = 0x01;
	 transmit_buffer[2] = 0x02;
	 transmit_buffer[3] = 0x00;
	 transmit_buffer[4] = 0x0A;
	}
	else if(receive_buffer == 0x01)
	{
	 transmit_buffer[0] = 0x00;
	 transmit_buffer[1] = 0x01;
	 transmit_buffer[2] = 0x01;
	 transmit_buffer[3] = 0x01;
	 transmit_buffer[4] = 0x14;
	}
	else
	{
	 transmit_buffer[0] = 0x00;
	 transmit_buffer[1] = 0x00;
	 transmit_buffer[2] = 0x00;
	 transmit_buffer[3] = 0x00;
	 transmit_buffer[4] = 0x00;
	}
	transmit_pos = 0;
	transmit_count = 5;
	command_phase = -1;
	break;

  /************************/
  /* MMMode 1, Command 0x47 */
  /************************/
  case 0x4700:
	if(receive_buffer == 0x00)
	{
	 transmit_buffer[0] = 0; /**/ transmit_pos = 0; transmit_count = 1; /**/
	 command_phase++;
	}
	else
	 command_phase = -1;
	break;

  case 0x4701:
	if(receive_buffer == 0x00)
	{
	 transmit_buffer[0] = 0x00;
	 transmit_buffer[1] = 0x02;
	 transmit_buffer[2] = 0x00;
	 transmit_buffer[3] = 0x01;
	 transmit_buffer[4] = 0x00;
	}
	else
	{
	 transmit_buffer[0] = 0x00;
	 transmit_buffer[1] = 0x00;
	 transmit_buffer[2] = 0x00;
	 transmit_buffer[3] = 0x00;
	 transmit_buffer[4] = 0x00;
	}
	transmit_pos = 0;
	transmit_count = 5;
	command_phase = -1;
	break;

  /************************/
  /* MMMode 1, Command 0x48 */
  /************************/
  case 0x4800:
	if(receive_buffer == 0x00)
	{
	 transmit_buffer[0] = 0; /**/ transmit_pos = 0; transmit_count = 1; /**/
	 command_phase++;
	}
	else
	 command_phase = -1;
	break;

  case 0x4801:
	if(receive_buffer == 0x00)
	{
	 transmit_buffer[0] = 0x00;
	 transmit_buffer[1] = 0x00;
	 transmit_buffer[2] = 0x00;
	 transmit_buffer[3] = 0x01;
	 transmit_buffer[4] = rumble_param[0];
	}
	else if(receive_buffer == 0x01)
	{
	 transmit_buffer[0] = 0x00;
	 transmit_buffer[1] = 0x00;
	 transmit_buffer[2] = 0x00;
	 transmit_buffer[3] = 0x01;
	 transmit_buffer[4] = rumble_param[1];
	}
	else
	{
	 transmit_buffer[0] = 0x00;
	 transmit_buffer[1] = 0x00;
	 transmit_buffer[2] = 0x00;
	 transmit_buffer[3] = 0x00;
	 transmit_buffer[4] = 0x00;
	}
	transmit_pos = 0;
	transmit_count = 5;
	command_phase = -1;
	break;

  /************************/
  /* MMMode 1, Command 0x49 */
  /************************/
  case 0x4900:
	if(receive_buffer == 0x00)
	{
	 transmit_buffer[0] = 0; /**/ transmit_pos = 0; transmit_count = 1; /**/
	 command_phase++;
	}
	else
	 command_phase = -1;
	break;

  case 0x4901:
	transmit_buffer[0] = 0x00;
	transmit_buffer[1] = 0x00;
	transmit_buffer[2] = 0x00;
	transmit_buffer[3] = 0x00;
	transmit_buffer[4] = 0x00;
	transmit_pos = 0;
	transmit_count = 5;
	command_phase = -1;
	break;

  /************************/
  /* MMMode 1, Command 0x4A */
  /************************/
  case 0x4A00:
	if(receive_buffer == 0x00)
	{
	 transmit_buffer[0] = 0; /**/ transmit_pos = 0; transmit_count = 1; /**/
	 command_phase++;
	}
	else
	 command_phase = -1;
	break;

  case 0x4A01:
	transmit_buffer[0] = 0x00;
	transmit_buffer[1] = 0x00;
	transmit_buffer[2] = 0x00;
	transmit_buffer[3] = 0x00;
	transmit_buffer[4] = 0x00;
	transmit_pos = 0;
	transmit_count = 5;
	command_phase = -1;
	break;

  /************************/
  /* MMMode 1, Command 0x4B */
  /************************/
  case 0x4B00:
	if(receive_buffer == 0x00)
	{
	 transmit_buffer[0] = 0; /**/ transmit_pos = 0; transmit_count = 1; /**/
	 command_phase++;
	}
	else
	 command_phase = -1;
	break;

  case 0x4B01:
	transmit_buffer[0] = 0x00;
	transmit_buffer[1] = 0x00;
	transmit_buffer[2] = 0x00;
	transmit_buffer[3] = 0x00;
	transmit_buffer[4] = 0x00;
	transmit_pos = 0;
	transmit_count = 5;
	command_phase = -1;
	break;

  /************************/
  /* MMMode 1, Command 0x4C */
  /************************/
  case 0x4C00:
	if(receive_buffer == 0x00)
	{
	 transmit_buffer[0] = 0; /**/ transmit_pos = 0; transmit_count = 1; /**/
	 command_phase++;
	}
	else
	 command_phase = -1;
	break;

  case 0x4C01:
	if(receive_buffer == 0x00)
	{
	 transmit_buffer[0] = 0x00;
	 transmit_buffer[1] = 0x00;
	 transmit_buffer[2] = 0x04;
	 transmit_buffer[3] = 0x00;
	 transmit_buffer[4] = 0x00;
	}
	else if(receive_buffer == 0x01)
	{
	 transmit_buffer[0] = 0x00;
	 transmit_buffer[1] = 0x00;
	 transmit_buffer[2] = 0x07;
	 transmit_buffer[3] = 0x00;
	 transmit_buffer[4] = 0x00;
	}
	else
	{
	 transmit_buffer[0] = 0x00;
	 transmit_buffer[1] = 0x00;
	 transmit_buffer[2] = 0x00;
	 transmit_buffer[3] = 0x00;
	 transmit_buffer[4] = 0x00;
	}

	transmit_pos = 0;
	transmit_count = 5;
	command_phase = -1;
	break;

  /************************/
  /* MMMode 1, Command 0x4D */
  /************************/
  case 0x4D00:
	if(receive_buffer == 0x00)
	{
	 transmit_buffer[0] = rumble_magic[0]; /**/ transmit_pos = 0; transmit_count = 1; /**/
	 command_phase++;
	}
	else
	 command_phase = -1;
	break;

  case 0x4D01:
  case 0x4D02:
  case 0x4D03:
  case 0x4D04:
  case 0x4D05:
  case 0x4D06:
	{
	 unsigned index = command_phase - 0x4D01;

	 if(index < 5)
	 {
 	  transmit_buffer[0] = rumble_magic[1 + index];
	  transmit_pos = 0;
	  transmit_count = 1;
	  command_phase++;
	 }
	 else
	  command_phase = -1;

 	 rumble_magic[index] = receive_buffer;	 
	}
	break;

  /************************/
  /* MMMode 1, Command 0x4E */
  /************************/
  case 0x4E00:
	if(receive_buffer == 0x00)
	{
	 transmit_buffer[0] = 0; /**/ transmit_pos = 0; transmit_count = 1; /**/
	 command_phase++;
	}
	else
	 command_phase = -1;
	break;

  case 0x4E01:
	transmit_buffer[0] = 0x00;
	transmit_buffer[1] = 0x00;
	transmit_buffer[2] = 0x00;
	transmit_buffer[3] = 0x00;
	transmit_buffer[4] = 0x00;
	transmit_pos = 0;
	transmit_count = 5;
	command_phase = -1;
	break;


  /************************/
  /* MMMode 1, Command 0x4F */
  /************************/
  case 0x4F00:
	if(receive_buffer == 0x00)
	{
	 transmit_buffer[0] = 0; /**/ transmit_pos = 0; transmit_count = 1; /**/
	 command_phase++;
	}
	else
	 command_phase = -1;
	break;

  case 0x4F01:
	transmit_buffer[0] = 0x00;
	transmit_buffer[1] = 0x00;
	transmit_buffer[2] = 0x00;
	transmit_buffer[3] = 0x00;
	transmit_buffer[4] = 0x00;
	transmit_pos = 0;
	transmit_count = 5;
	command_phase = -1;
	break;
  }
 }

 if(!bitpos && transmit_count)
  dsr_pulse_delay = 0x40; //0x100;

 return(ret);
}

InputDeviceInputInfoStruct Device_DualShock_IDII[26] =
{
 { "select", "SELECT", 4, IDIT_BUTTON, NULL },
 { "l3", "Left Stick, Button(L3)", 18, IDIT_BUTTON, NULL },
 { "r3", "Right stick, Button(R3)", 23, IDIT_BUTTON, NULL },
 { "start", "START", 5, IDIT_BUTTON, NULL },
 { "up", "D-Pad UP ↑", 0, IDIT_BUTTON, "down" },
 { "right", "D-Pad RIGHT →", 3, IDIT_BUTTON, "left" },
 { "down", "D-Pad DOWN ↓", 1, IDIT_BUTTON, "up" },
 { "left", "D-Pad LEFT ←", 2, IDIT_BUTTON, "right" },

 { "l2", "L2 (rear left shoulder)", 11, IDIT_BUTTON, NULL },
 { "r2", "R2 (rear right shoulder)", 13, IDIT_BUTTON, NULL },
 { "l1", "L1 (front left shoulder)", 10, IDIT_BUTTON, NULL },
 { "r1", "R1 (front right shoulder)", 12, IDIT_BUTTON, NULL },

 { "triangle", "△ (upper)", 6, IDIT_BUTTON_CAN_RAPID, NULL },
 { "circle", "○ (right)", 9, IDIT_BUTTON_CAN_RAPID, NULL },
 { "cross", "x (lower)", 7, IDIT_BUTTON_CAN_RAPID, NULL },
 { "square", "□ (left)", 8, IDIT_BUTTON_CAN_RAPID, NULL },

 { "analog", "Analog(mode toggle)", 24, IDIT_BUTTON, NULL },

 { "rstick_right", "Right Stick RIGHT →", 22, IDIT_BUTTON_ANALOG, NULL, { NULL, NULL, NULL }, IDIT_BUTTON_ANALOG_FLAG_SQLR },
 { "rstick_left", "Right Stick LEFT ←", 21, IDIT_BUTTON_ANALOG, NULL, { NULL, NULL, NULL }, IDIT_BUTTON_ANALOG_FLAG_SQLR },
 { "rstick_down", "Right Stick DOWN ↓", 20, IDIT_BUTTON_ANALOG, NULL, { NULL, NULL, NULL }, IDIT_BUTTON_ANALOG_FLAG_SQLR },
 { "rstick_up", "Right Stick UP ↑", 19, IDIT_BUTTON_ANALOG, NULL, { NULL, NULL, NULL }, IDIT_BUTTON_ANALOG_FLAG_SQLR },

 { "lstick_right", "Left Stick RIGHT →", 17, IDIT_BUTTON_ANALOG, NULL, { NULL, NULL, NULL }, IDIT_BUTTON_ANALOG_FLAG_SQLR },
 { "lstick_left", "Left Stick LEFT ←", 16, IDIT_BUTTON_ANALOG, NULL, { NULL, NULL, NULL }, IDIT_BUTTON_ANALOG_FLAG_SQLR },
 { "lstick_down", "Left Stick DOWN ↓", 15, IDIT_BUTTON_ANALOG, NULL, { NULL, NULL, NULL }, IDIT_BUTTON_ANALOG_FLAG_SQLR },
 { "lstick_up", "Left Stick UP ↑", 14, IDIT_BUTTON_ANALOG, NULL, { NULL, NULL, NULL }, IDIT_BUTTON_ANALOG_FLAG_SQLR },

 { "rumble", "RUMBLE MONSTER RUMBA", 100, IDIT_RUMBLE },
};


InputDevice_Gamepad::InputDevice_Gamepad()
{
 Power();
}

InputDevice_Gamepad::~InputDevice_Gamepad()
{

}

void InputDevice_Gamepad::Power(void)
{
 dtr = 0;

 buttons[0] = buttons[1] = 0;

 command_phase = 0;

 bitpos = 0;

 receive_buffer = 0;

 command = 0;

 memset(transmit_buffer, 0, sizeof(transmit_buffer));

 transmit_pos = 0;
 transmit_count = 0;
}

int InputDevice_Gamepad::StateAction(StateMem* sm, int load, int data_only, const char* section_name)
{
 SFORMAT StateRegs[] =
 {
  { &((dtr)), 1, 0x80000000 | 0x08000000, "dtr" },

  { ((buttons)), (uint32)((sizeof(buttons))), 0 | 0, "buttons" },

  { &((command_phase)), sizeof((command_phase)), 0x80000000 | 0, "command_phase" },
  { &((bitpos)), sizeof((bitpos)), 0x80000000 | 0, "bitpos" },
  { &((receive_buffer)), sizeof((receive_buffer)), 0x80000000 | 0, "receive_buffer" },

  { &((command)), sizeof((command)), 0x80000000 | 0, "command" },

  { ((transmit_buffer)), (uint32)((sizeof(transmit_buffer))), 0 | 0, "transmit_buffer" },
  { &((transmit_pos)), sizeof((transmit_pos)), 0x80000000 | 0, "transmit_pos" },
  { &((transmit_count)), sizeof((transmit_count)), 0x80000000 | 0, "transmit_count" },

  { 0, 0, 0, 0 }
 };
 int ret = MDFNSS_StateAction(sm, load, StateRegs, section_name);

 if(load)
 {
  if((transmit_pos + transmit_count) > sizeof(transmit_buffer))
  {
   transmit_pos = 0;
   transmit_count = 0;
  }
 }

 return(ret);
}

void InputDevice_Gamepad::SetDTR(bool new_dtr)
{
 if(!dtr && new_dtr)
 {
  command_phase = 0;
  bitpos = 0;
  transmit_pos = 0;
  transmit_count = 0;
 }
#if 0
 else if(dtr && !new_dtr)
 {
  //if(bitpos || transmit_count)
  // printf("[PAD] Abort communication!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
 }
#endif

 dtr = new_dtr;
}

bool InputDevice_Gamepad::Clock(bool TxD, int32 &dsr_pulse_delay)
{
 bool ret = 1;

 dsr_pulse_delay = 0;

 if(!dtr)
  return(1);

 if(transmit_count)
  ret = (transmit_buffer[transmit_pos] >> bitpos) & 1;

 receive_buffer &= ~(1 << bitpos);
 receive_buffer |= TxD << bitpos;
 bitpos = (bitpos + 1) & 0x7;

 if(!bitpos)
 {
  //printf("[PAD] Receive: %02x -- command_phase=%d\n", receive_buffer, command_phase);

  if(transmit_count)
  {
   transmit_pos++;
   transmit_count--;
  }


  switch(command_phase)
  {
   case 0:
 	  if(receive_buffer != 0x01)
	    command_phase = -1;
	  else
	  {
	   transmit_buffer[0] = 0x41;
	   transmit_pos = 0;
	   transmit_count = 1;
	   command_phase++;
	  }
	  break;

   case 1:
	command = receive_buffer;
	command_phase++;

	transmit_buffer[0] = 0x5A;

	//if(command != 0x42)
	// fprintf(stderr, "Gamepad unhandled command: 0x%02x\n", command);
	//assert(command == 0x42);
	if(command == 0x42)
	{
	 //printf("PAD COmmand 0x42, sl=%u\n", GPU->GetScanlineNum());

	 transmit_buffer[1] = 0xFF ^ buttons[0];
	 transmit_buffer[2] = 0xFF ^ buttons[1];
         transmit_pos = 0;
         transmit_count = 3;
	}
	else
	{
	 command_phase = -1;
	 transmit_buffer[1] = 0;
	 transmit_buffer[2] = 0;
         transmit_pos = 0;
         transmit_count = 0;
	}
	break;

  }
 }

 if(!bitpos && transmit_count)
  dsr_pulse_delay = 0x40; //0x100;

 return(ret);
}

InputDeviceInputInfoStruct Device_Gamepad_IDII[16] =
{
 { "select", "SELECT", 4, IDIT_BUTTON, NULL },
 { NULL, "empty", 0, IDIT_BUTTON },
 { NULL, "empty", 0, IDIT_BUTTON },
 { "start", "START", 5, IDIT_BUTTON, NULL },
 { "up", "UP ↑", 0, IDIT_BUTTON, "down" },
 { "right", "RIGHT →", 3, IDIT_BUTTON, "left" },
 { "down", "DOWN ↓", 1, IDIT_BUTTON, "up" },
 { "left", "LEFT ←", 2, IDIT_BUTTON, "right" },

 { "l2", "L2 (rear left shoulder)", 11, IDIT_BUTTON, NULL },
 { "r2", "R2 (rear right shoulder)", 13, IDIT_BUTTON, NULL },
 { "l1", "L1 (front left shoulder)", 10, IDIT_BUTTON, NULL },
 { "r1", "R1 (front right shoulder)", 12, IDIT_BUTTON, NULL },

 { "triangle", "△ (upper)", 6, IDIT_BUTTON_CAN_RAPID, NULL },
 { "circle", "○ (right)", 9, IDIT_BUTTON_CAN_RAPID, NULL },
 { "cross", "x (lower)", 7, IDIT_BUTTON_CAN_RAPID, NULL },
 { "square", "□ (left)", 8, IDIT_BUTTON_CAN_RAPID, NULL },
};

InputDeviceInputInfoStruct Device_Dancepad_IDII[16] =
{
 { "select", "SELECT", 0, IDIT_BUTTON, NULL },
 { NULL, "empty", 0, IDIT_BUTTON },
 { NULL, "empty", 0, IDIT_BUTTON },
 { "start", "START", 1, IDIT_BUTTON, NULL },

 { "up", "UP ↑", 3, IDIT_BUTTON, 	NULL },
 { "right", "RIGHT →", 6, IDIT_BUTTON, 	NULL },
 { "down", "DOWN ↓", 8, IDIT_BUTTON, 	NULL },
 { "left", "LEFT ←", 5, IDIT_BUTTON, 	NULL },

 { NULL, "empty", 0, IDIT_BUTTON, NULL },
 { NULL, "empty", 0, IDIT_BUTTON, NULL },
 { NULL, "empty", 0, IDIT_BUTTON, NULL },
 { NULL, "empty", 0, IDIT_BUTTON, NULL },

 { "triangle", "△ (lower left)", 7, IDIT_BUTTON, NULL },
 { "circle", "○ (upper right)", 4, IDIT_BUTTON, NULL },
 { "cross", "x (upper left)", 2, IDIT_BUTTON, NULL },
 { "square", "□ (lower right)", 9, IDIT_BUTTON, NULL },
};


InputDevice_GunCon::InputDevice_GunCon(void)
{
 Power();
}

InputDevice_GunCon::~InputDevice_GunCon()
{

}

void InputDevice_GunCon::Power(void)
{
 dtr = 0;

 buttons = 0;
 trigger_eff = 0;
 trigger_noclear = 0;
 hit_x = 0;
 hit_y = 0;

 nom_x = 0;
 nom_y = 0;

 os_shot_counter = 0;
 prev_oss = 0;

 command_phase = 0;

 bitpos = 0;

 receive_buffer = 0;

 command = 0;

 memset(transmit_buffer, 0, sizeof(transmit_buffer));

 transmit_pos = 0;
 transmit_count = 0;

 prev_vsync = 0;
 line_counter = 0;
}

int InputDevice_GunCon::StateAction(StateMem* sm, int load, int data_only, const char* section_name)
{
   SFORMAT StateRegs[] =
   {
      { &((dtr)), 1, 0x80000000 | 0x08000000, "dtr" },

      { &((buttons)), sizeof((buttons)), 0x80000000 | 0, "buttons" },
      { &((trigger_eff)), 1, 0x80000000 | 0x08000000, "trigger_eff" },
      { &((trigger_noclear)), 1, 0x80000000 | 0x08000000, "trigger_noclear" },
      { &((hit_x)), sizeof((hit_x)), 0x80000000 | 0, "hit_x" },
      { &((hit_y)), sizeof((hit_y)), 0x80000000 | 0, "hit_y" },

      { &((nom_x)), sizeof((nom_x)), 0x80000000 | 0, "nom_x" },
      { &((nom_y)), sizeof((nom_y)), 0x80000000 | 0, "nom_y" },
      { &((os_shot_counter)), sizeof((os_shot_counter)), 0x80000000 | 0, "os_shot_counter" },
      { &((prev_oss)), 1, 0x80000000 | 0x08000000, "prev_oss" },

      { &((command_phase)), sizeof((command_phase)), 0x80000000 | 0, "command_phase" },
      { &((bitpos)), sizeof((bitpos)), 0x80000000 | 0, "bitpos" },
      { &((receive_buffer)), sizeof((receive_buffer)), 0x80000000 | 0, "receive_buffer" },

      { &((command)), sizeof((command)), 0x80000000 | 0, "command" },

      { ((transmit_buffer)), (uint32)((sizeof(transmit_buffer))), 0 | 0, "transmit_buffer" },
      { &((transmit_pos)), sizeof((transmit_pos)), 0x80000000 | 0, "transmit_pos" },
      { &((transmit_count)), sizeof((transmit_count)), 0x80000000 | 0, "transmit_count" },

      { &((prev_vsync)), 1, 0x80000000 | 0x08000000, "prev_vsync" },
      { &((line_counter)), sizeof((line_counter)), 0x80000000 | 0, "line_counter" },

      { 0, 0, 0, 0 }
   };
   int ret = MDFNSS_StateAction(sm, load, StateRegs, section_name);

   if(load)
   {
      if((transmit_pos + transmit_count) > sizeof(transmit_buffer))
      {
         transmit_pos = 0;
         transmit_count = 0;
      }
   }

   return(ret);
}

bool InputDevice_GunCon::RequireNoFrameskip(void)
{
 return(true);
}

int32_t InputDevice_GunCon::GPULineHook(const int32_t line_timestamp, bool vsync, uint32 *pixels, const MDFN_PixelFormat* const format, const unsigned width,
      const unsigned pix_clock_offset, const unsigned pix_clock, const unsigned pix_clock_divider)
{
 if(vsync && !prev_vsync)
  line_counter = 0;

 if(pixels && pix_clock)
 {
  const int avs = 16; // Not 16 for PAL, fixme.
  int32 gx;
  int32 gy;

  gx = (nom_x * 2 + pix_clock_divider) / (pix_clock_divider * 2);
  gy = nom_y;

  for(int32 ix = gx; ix < (gx + (int32)(pix_clock / 762925)); ix++)
  {
     if(ix >= 0 && ix < (int)width && line_counter >= (avs + gy) && line_counter < (avs + gy + 8))
   {
    int r, g, b, a;

    DecodeColor(pixels[ix], r, g, b, a);

    if((r + g + b) >= 0x40)	// Wrong, but not COMPLETELY ABSOLUTELY wrong, at least. ;)
    {
     hit_x = (int64)(ix + pix_clock_offset) * 8000000 / pix_clock;	// GunCon has what appears to be an 8.00MHz ceramic resonator in it.
     hit_y = line_counter;
    }
   }
  }

  chair_x = gx;
  chair_y = (avs + gy) - line_counter;
 }

 line_counter++;

 return(PSX_EVENT_MAXTS);
}

void InputDevice_GunCon::SetDTR(bool new_dtr)
{
 if(!dtr && new_dtr)
 {
  command_phase = 0;
  bitpos = 0;
  transmit_pos = 0;
  transmit_count = 0;
 }
#if 0
 else if(dtr && !new_dtr)
 {
  //if(bitpos || transmit_count)
  // printf("[PAD] Abort communication!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
 }
#endif

 dtr = new_dtr;
}

bool InputDevice_GunCon::Clock(bool TxD, int32 &dsr_pulse_delay)
{
 bool ret = 1;

 dsr_pulse_delay = 0;

 if(!dtr)
  return(1);

 if(transmit_count)
  ret = (transmit_buffer[transmit_pos] >> bitpos) & 1;

 receive_buffer &= ~(1 << bitpos);
 receive_buffer |= TxD << bitpos;
 bitpos = (bitpos + 1) & 0x7;

 if(!bitpos)
 {
  //printf("[PAD] Receive: %02x -- command_phase=%d\n", receive_buffer, command_phase);

  if(transmit_count)
  {
   transmit_pos++;
   transmit_count--;
  }


  switch(command_phase)
  {
   case 0:
 	  if(receive_buffer != 0x01)
	    command_phase = -1;
	  else
	  {
	   transmit_buffer[0] = 0x63;
	   transmit_pos = 0;
	   transmit_count = 1;
	   command_phase++;
	  }
	  break;

   case 2:
	//if(receive_buffer)
	// printf("%02x\n", receive_buffer);
	break;

   case 1:
	command = receive_buffer;
	command_phase++;

	transmit_buffer[0] = 0x5A;

	//puts("MOO");
	//if(command != 0x42)
	// fprintf(stderr, "GunCon unhandled command: 0x%02x\n", command);
	//assert(command == 0x42);
	if(command == 0x42)
	{
	 transmit_buffer[1] = 0xFF ^ ((buttons & 0x01) << 3);
	 transmit_buffer[2] = 0xFF ^ (trigger_eff << 5) ^ ((buttons & 0x02) << 5);

	 if(os_shot_counter > 0)
	 {
	  hit_x = 0x01;
	  hit_y = 0x0A;
	  transmit_buffer[2] |= (1 << 5);
	  if(os_shot_counter == 2 || os_shot_counter == 3)
	  {
	   transmit_buffer[2] &= ~(1 << 5);
	  }
	 }

	 MDFN_en16lsb(&transmit_buffer[3], hit_x);
	 MDFN_en16lsb(&transmit_buffer[5], hit_y);

	 hit_x = 0x01;
	 hit_y = 0x0A;

         transmit_pos = 0;
         transmit_count = 7;

	 trigger_eff = trigger_noclear;
	}
	else
	{
	 command_phase = -1;
	 transmit_buffer[1] = 0;
	 transmit_buffer[2] = 0;
         transmit_pos = 0;
         transmit_count = 0;
	}
	break;

  }
 }

 if(!bitpos && transmit_count)
  dsr_pulse_delay = 100; //0x80; //0x40;

 return(ret);
}

InputDeviceInputInfoStruct Device_GunCon_IDII[6] =
{
 { "x_axis", "X Axis", -1, IDIT_X_AXIS },
 { "y_axis", "Y Axis", -1, IDIT_Y_AXIS },

 { "trigger", "Trigger", 0, IDIT_BUTTON, NULL  },

 { "a",	"A",		 1, IDIT_BUTTON,	NULL },
 { "b", "B",		 2, IDIT_BUTTON,	NULL },

 { "offscreen_shot", "Offscreen Shot(Simulated)", 3, IDIT_BUTTON, NULL },	// Useful for "Judge Dredd", and probably not much else.
};

class InputDevice_Justifier : public InputDevice
{
 public:

 InputDevice_Justifier(void);
 virtual ~InputDevice_Justifier();

 virtual void Power(void);
 virtual int StateAction(StateMem* sm, int load, int data_only, const char* section_name);
 virtual bool RequireNoFrameskip(void);
 virtual int32_t GPULineHook(const int32_t timestamp, bool vsync, uint32 *pixels, const MDFN_PixelFormat* const format, const unsigned width, const unsigned pix_clock_offset, const unsigned pix_clock, const unsigned pix_clock_divider);

 //
 //
 //
 virtual void SetDTR(bool new_dtr);
 virtual bool Clock(bool TxD, int32 &dsr_pulse_delay);

 bool dtr;

 uint8 buttons;
 bool trigger_eff;
 bool trigger_noclear;

 bool need_hit_detect;

 int16 nom_x, nom_y;
 int32 os_shot_counter;
 bool prev_oss;

 int32 command_phase;
 uint32 bitpos;
 uint8 receive_buffer;

 uint8 command;

 uint8 transmit_buffer[16];
 uint32 transmit_pos;
 uint32 transmit_count;

 //
 // Video timing stuff
 bool prev_vsync;
 int line_counter;

};

InputDevice_Justifier::InputDevice_Justifier(void)
{
 Power();
}

InputDevice_Justifier::~InputDevice_Justifier()
{

}

void InputDevice_Justifier::Power(void)
{
 dtr = 0;

 buttons = 0;
 trigger_eff = 0;
 trigger_noclear = 0;

 need_hit_detect = false;

 nom_x = 0;
 nom_y = 0;

 os_shot_counter = 0;
 prev_oss = 0;

 command_phase = 0;

 bitpos = 0;

 receive_buffer = 0;

 command = 0;

 memset(transmit_buffer, 0, sizeof(transmit_buffer));

 transmit_pos = 0;
 transmit_count = 0;

 prev_vsync = 0;
 line_counter = 0;
}

int InputDevice_Justifier::StateAction(StateMem* sm, int load, int data_only, const char* section_name)
{
   SFORMAT StateRegs[] =
   {
      { &((dtr)), 1, 0x80000000 | 0x08000000, "dtr" },

      { &((buttons)), sizeof((buttons)), 0x80000000 | 0, "buttons" },
      { &((trigger_eff)), 1, 0x80000000 | 0x08000000, "trigger_eff" },
      { &((trigger_noclear)), 1, 0x80000000 | 0x08000000, "trigger_noclear" },

      { &((need_hit_detect)), 1, 0x80000000 | 0x08000000, "need_hit_detect" },

      { &((nom_x)), sizeof((nom_x)), 0x80000000 | 0, "nom_x" },
      { &((nom_y)), sizeof((nom_y)), 0x80000000 | 0, "nom_y" },
      { &((os_shot_counter)), sizeof((os_shot_counter)), 0x80000000 | 0, "os_shot_counter" },
      { &((prev_oss)), 1, 0x80000000 | 0x08000000, "prev_oss" },

      { &((command_phase)), sizeof((command_phase)), 0x80000000 | 0, "command_phase" },
      { &((bitpos)), sizeof((bitpos)), 0x80000000 | 0, "bitpos" },
      { &((receive_buffer)), sizeof((receive_buffer)), 0x80000000 | 0, "receive_buffer" },

      { &((command)), sizeof((command)), 0x80000000 | 0, "command" },

      { ((transmit_buffer)), (uint32)((sizeof(transmit_buffer))), 0 | 0, "transmit_buffer" },
      { &((transmit_pos)), sizeof((transmit_pos)), 0x80000000 | 0, "transmit_pos" },
      { &((transmit_count)), sizeof((transmit_count)), 0x80000000 | 0, "transmit_count" },

      { &((prev_vsync)), 1, 0x80000000 | 0x08000000, "prev_vsync" },
      { &((line_counter)), 1, 0x80000000 | 0x08000000, "line_counter" },

      { 0, 0, 0, 0 }
   };
   int ret = MDFNSS_StateAction(sm, load, StateRegs, section_name);

   if(load)
   {
      if((transmit_pos + transmit_count) > sizeof(transmit_buffer))
      {
         transmit_pos = 0;
         transmit_count = 0;
      }
   }

   return(ret);
}

bool InputDevice_Justifier::RequireNoFrameskip(void)
{
 return(true);
}

int32_t InputDevice_Justifier::GPULineHook(const int32_t timestamp, bool vsync, uint32 *pixels, const MDFN_PixelFormat* const format, const unsigned width, const unsigned pix_clock_offset, const unsigned pix_clock, const unsigned pix_clock_divider)
{
 int32_t ret = PSX_EVENT_MAXTS;

 if(vsync && !prev_vsync)
  line_counter = 0;

 if(pixels && pix_clock)
 {
  const int avs = 16; // Not 16 for PAL, fixme.
  int32 gx;
  int32 gy;
  int32 gxa;

  gx = (nom_x * 2 + pix_clock_divider) / (pix_clock_divider * 2);
  gy = nom_y;
  gxa = gx; // - (pix_clock / 400000);
  //if(gxa < 0 && gx >= 0)
  // gxa = 0;

  if(!os_shot_counter && need_hit_detect && gxa >= 0 && gxa < (int)width && line_counter >= (avs + gy - 1) && line_counter <= (avs + gy + 1))
  {
   int r, g, b, a;

   DecodeColor(pixels[gxa], r, g, b, a);

   if((r + g + b) >= 0x40)	// Wrong, but not COMPLETELY ABSOLUTELY wrong, at least. ;)
   {
    ret = timestamp + (int64)(gxa + pix_clock_offset) * (44100 * 768) / pix_clock - 177;
   }
  }

  chair_x = gx;
  chair_y = (avs + gy) - line_counter;
 }

 line_counter++;

 return(ret);
}

void InputDevice_Justifier::SetDTR(bool new_dtr)
{
 if(!dtr && new_dtr)
 {
  command_phase = 0;
  bitpos = 0;
  transmit_pos = 0;
  transmit_count = 0;
 }
#if 0
 else if(dtr && !new_dtr)
 {
  //if(bitpos || transmit_count)
  // printf("[PAD] Abort communication!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
 }
#endif

 dtr = new_dtr;
}

bool InputDevice_Justifier::Clock(bool TxD, int32 &dsr_pulse_delay)
{
 bool ret = 1;

 dsr_pulse_delay = 0;

 if(!dtr)
  return(1);

 if(transmit_count)
  ret = (transmit_buffer[transmit_pos] >> bitpos) & 1;

 receive_buffer &= ~(1 << bitpos);
 receive_buffer |= TxD << bitpos;
 bitpos = (bitpos + 1) & 0x7;

 if(!bitpos)
 {
  //printf("[PAD] Receive: %02x -- command_phase=%d\n", receive_buffer, command_phase);

  if(transmit_count)
  {
   transmit_pos++;
   transmit_count--;
  }


  switch(command_phase)
  {
   case 0:
 	  if(receive_buffer != 0x01)
	    command_phase = -1;
	  else
	  {
	   transmit_buffer[0] = 0x31;
	   transmit_pos = 0;
	   transmit_count = 1;
	   command_phase++;
	  }
	  break;

   case 2:
	//if(receive_buffer)
	// printf("%02x\n", receive_buffer);
	command_phase++;
	break;

   case 3:
	need_hit_detect = receive_buffer & 0x10;	// TODO, see if it's (val&0x10) == 0x10, or some other mask value.
	command_phase++;
	break;

   case 1:
	command = receive_buffer;
	command_phase++;

	transmit_buffer[0] = 0x5A;

	//if(command != 0x42)
	// fprintf(stderr, "Justifier unhandled command: 0x%02x\n", command);
	//assert(command == 0x42);
	if(command == 0x42)
	{
	 transmit_buffer[1] = 0xFF ^ ((buttons & 2) << 2);
	 transmit_buffer[2] = 0xFF ^ (trigger_eff << 7) ^ ((buttons & 1) << 6);

	 if(os_shot_counter > 0)
	 {
	  transmit_buffer[2] |= (1 << 7);
	  if(os_shot_counter == 6 || os_shot_counter == 5)
	  {
	   transmit_buffer[2] &= ~(1 << 7);
	  }
	 }

         transmit_pos = 0;
         transmit_count = 3;

	 trigger_eff = trigger_noclear;
	}
	else
	{
	 command_phase = -1;
	 transmit_buffer[1] = 0;
	 transmit_buffer[2] = 0;
         transmit_pos = 0;
         transmit_count = 0;
	}
	break;

  }
 }

 if(!bitpos && transmit_count)
  dsr_pulse_delay = 200;

 return(ret);
}

InputDeviceInputInfoStruct Device_Justifier_IDII[6] =
{
 { "x_axis", "X Axis", -1, IDIT_X_AXIS },
 { "y_axis", "Y Axis", -1, IDIT_Y_AXIS },

 { "trigger", "Trigger", 0, IDIT_BUTTON, NULL  },

 { "o",	"O",		 1, IDIT_BUTTON,	NULL },
 { "start", "Start",	 2, IDIT_BUTTON,	NULL },

 { "offscreen_shot", "Offscreen Shot(Simulated)", 3, IDIT_BUTTON, NULL },
};

void InputDevice_Memcard::Format(void)
{
   memset(card_data, 0x00, sizeof(card_data));

   card_data[0x00] = 0x4D;
   card_data[0x01] = 0x43;
   card_data[0x7F] = 0x0E;

   for(unsigned int A = 0x80; A < 0x800; A += 0x80)
   {
      card_data[A + 0x00] = 0xA0;
      card_data[A + 0x08] = 0xFF;
      card_data[A + 0x09] = 0xFF;
      card_data[A + 0x7F] = 0xA0;
   }

   for(unsigned int A = 0x0800; A < 0x1200; A += 0x80)
   {
      card_data[A + 0x00] = 0xFF;
      card_data[A + 0x01] = 0xFF;
      card_data[A + 0x02] = 0xFF;
      card_data[A + 0x03] = 0xFF;
      card_data[A + 0x08] = 0xFF;
      card_data[A + 0x09] = 0xFF;
   }
}

InputDevice_Memcard::InputDevice_Memcard()
{
 Power();

 data_used = false;
 dirty_count = 0;

 // Init memcard as formatted.
 assert(sizeof(card_data) == (1 << 17));
 Format();
}

InputDevice_Memcard::~InputDevice_Memcard()
{

}

void InputDevice_Memcard::Power(void)
{
 dtr = 0;

 //buttons[0] = buttons[1] = 0;

 command_phase = 0;

 bitpos = 0;

 receive_buffer = 0;

 command = 0;

 transmit_buffer = 0;

 transmit_count = 0;

 addr = 0;

 presence_new = true;
}

int InputDevice_Memcard::StateAction(StateMem* sm, int load, int data_only, const char* section_name)
{
 // Don't save dirty_count.
 SFORMAT StateRegs[] =
 {
  { &((presence_new)), 1, 0x80000000 | 0x08000000, "presence_new" },

  { ((rw_buffer)), (uint32)((sizeof(rw_buffer))), 0 | 0, "rw_buffer" },
  { &((write_xor)), sizeof((write_xor)), 0x80000000 | 0, "write_xor" },

  { &((dtr)), 1, 0x80000000 | 0x08000000, "dtr" },
  { &((command_phase)), sizeof((command_phase)), 0x80000000 | 0, "command_phase" },
  { &((bitpos)), sizeof((bitpos)), 0x80000000 | 0, "bitpos" },
  { &((receive_buffer)), sizeof((receive_buffer)), 0x80000000 | 0, "receive_buffer" },

  { &((command)), sizeof((command)), 0x80000000 | 0, "command" },
  { &((addr)), sizeof((addr)), 0x80000000 | 0, "addr" },
  { &((calced_xor)), sizeof((calced_xor)), 0x80000000 | 0, "calced_xor" },

  { &((transmit_buffer)), sizeof((transmit_buffer)), 0x80000000 | 0, "transmit_buffer" },
  { &((transmit_count)), sizeof((transmit_count)), 0x80000000 | 0, "transmit_count" },

  { &((data_used)), 1, 0x80000000 | 0x08000000, "data_used" },

  { 0, 0, 0, 0 }
 };

 SFORMAT CD_StateRegs[] =
 {
    { ((card_data)), (uint32)((sizeof(card_data))), 0 | 0, "card_data" },
    { 0, 0, 0, 0 }
 };
 int ret = 1;
 
 if(MDFNSS_StateAction(sm, load, StateRegs, section_name) != 0)
 {
    //printf("%s data_used=%d\n", section_name, data_used);
    if(data_used)
    {
       std::string tmp_name = std::string(section_name) + "_DT";

       ret &= MDFNSS_StateAction(sm, load, CD_StateRegs, tmp_name.c_str());
    }

    if(load)
    {
       if(data_used)
          dirty_count++;
       else
       {
          //printf("Format: %s\n", section_name);
          Format();
       }
    }
 }
 else
    ret = 0;

 return(ret);
}

void InputDevice_Memcard::SetDTR(bool new_dtr)
{
 if(!dtr && new_dtr)
 {
  command_phase = 0;
  bitpos = 0;
  transmit_count = 0;
 }
#if 0
 else if(dtr && !new_dtr)
 {
  if(command_phase > 0)
   PSX_WARNING("[MCR] Communication aborted???");
 }
#endif
 dtr = new_dtr;
}

bool InputDevice_Memcard::Clock(bool TxD, int32 &dsr_pulse_delay)
{
 bool ret = 1;

 dsr_pulse_delay = 0;

 if(!dtr)
  return(1);

 if(transmit_count)
  ret = (transmit_buffer >> bitpos) & 1;

 receive_buffer &= ~(1 << bitpos);
 receive_buffer |= TxD << bitpos;
 bitpos = (bitpos + 1) & 0x7;

 if(!bitpos)
 {
  //if(command_phase > 0 || transmit_count)
  // printf("[MCRDATA] Received_data=0x%02x, Sent_data=0x%02x\n", receive_buffer, transmit_buffer);

  if(transmit_count)
  {
   transmit_count--;
  }

  if (command_phase >= 1024 && command_phase <= 1151)
  {
  // Transmit actual 128 bytes data
	transmit_buffer = card_data[(addr << 7) + (command_phase - 1024)];
	calced_xor ^= transmit_buffer;
	transmit_count = 1;
	command_phase++;
  }
  else if (command_phase >= 2048 && command_phase <= 2175)
  {
  	calced_xor ^= receive_buffer;
	rw_buffer[command_phase - 2048] = receive_buffer;

        transmit_buffer = receive_buffer;
        transmit_count = 1;
        command_phase++;
  }
  else
  switch(command_phase)
  {
   case 0:
          if(receive_buffer != 0x81)
            command_phase = -1;
          else
          {
	   //printf("[MCR] Device selected\n");
           transmit_buffer = presence_new ? 0x08 : 0x00;
           transmit_count = 1;
           command_phase++;
          }
          break;

   case 1:
        command = receive_buffer;
	//printf("[MCR] Command received: %c\n", command);
	if(command == 'R' || command == 'W')
	{
	 command_phase++;
         transmit_buffer = 0x5A;
         transmit_count = 1;
	}
	else
	{
#if 0
	 if(command == 'S')
	 {
	  PSX_WARNING("[MCR] Memcard S command unsupported.");
	 }
#endif

	 command_phase = -1;
	 transmit_buffer = 0;
	 transmit_count = 0;
	}
        break;

   case 2:
	transmit_buffer = 0x5D;
	transmit_count = 1;
	command_phase++;
	break;

   case 3:
	transmit_buffer = 0x00;
	transmit_count = 1;
	if(command == 'R')
	 command_phase = 1000;
	else if(command == 'W')
	 command_phase = 2000;
	break;

  //
  // Read
  //
  case 1000:
	addr = receive_buffer << 8;
	transmit_buffer = receive_buffer;
	transmit_count = 1;
	command_phase++;
	break;

  case 1001:
	addr |= receive_buffer & 0xFF;
	transmit_buffer = '\\';
	transmit_count = 1;
	command_phase++;
	break;

  case 1002:
	//printf("[MCR]   READ ADDR=0x%04x\n", addr);
	if(addr >= (sizeof(card_data) >> 7))
	 addr = 0xFFFF;

	calced_xor = 0;
	transmit_buffer = ']';
	transmit_count = 1;
	command_phase++;

	// TODO: enable this code(or something like it) when CPU instruction timing is a bit better.
	//
	//dsr_pulse_delay = 32000;
	//goto SkipDPD;
	//

	break;

  case 1003:
	transmit_buffer = addr >> 8;
	calced_xor ^= transmit_buffer;
	transmit_count = 1;
	command_phase++;
	break;

  case 1004:
	transmit_buffer = addr & 0xFF;
	calced_xor ^= transmit_buffer;

	if(addr == 0xFFFF)
	{
	 transmit_count = 1;
	 command_phase = -1;
	}
	else
	{
	 transmit_count = 1;
	 command_phase = 1024;
	}
	break;



  // XOR
  case (1024 + 128):
	transmit_buffer = calced_xor;
	transmit_count = 1;
	command_phase++;
	break;

  // End flag
  case (1024 + 129):
	transmit_buffer = 'G';
	transmit_count = 1;
	command_phase = -1;
	break;

  //
  // Write
  //
  case 2000:
	calced_xor = receive_buffer;
        addr = receive_buffer << 8;
        transmit_buffer = receive_buffer;
        transmit_count = 1;
        command_phase++;
	break;

  case 2001:
	calced_xor ^= receive_buffer;
        addr |= receive_buffer & 0xFF;
	//printf("[MCR]   WRITE ADDR=0x%04x\n", addr);
        transmit_buffer = receive_buffer;
        transmit_count = 1;
        command_phase = 2048;
        break;
  case (2048 + 128):	// XOR
	write_xor = receive_buffer;
	transmit_buffer = '\\';
	transmit_count = 1;
	command_phase++;
	break;

  case (2048 + 129):
	transmit_buffer = ']';
	transmit_count = 1;
	command_phase++;
	break;

  case (2048 + 130):	// End flag
	//MDFN_DispMessage("%02x %02x", calced_xor, write_xor);
	//printf("[MCR] Write End.  Actual_XOR=0x%02x, CW_XOR=0x%02x\n", calced_xor, write_xor);

	if(calced_xor != write_xor)
 	 transmit_buffer = 'N';
	else if(addr >= (sizeof(card_data) >> 7))
	 transmit_buffer = 0xFF;
	else
	{
	 transmit_buffer = 'G';
	 presence_new = false;

	 // If the current data is different from the data to be written, increment the dirty count.
	 // memcpy()'ing over to card_data is also conditionalized here for a slight optimization.
         if(memcmp(&card_data[addr << 7], rw_buffer, 128))
	 {
	  memcpy(&card_data[addr << 7], rw_buffer, 128);
	  dirty_count++;
     data_used = true;
	 }
	}

	transmit_count = 1;
	command_phase = -1;
	break;

  }

  //if(command_phase != -1 || transmit_count)
  // printf("[MCR] Receive: 0x%02x, Send: 0x%02x -- %d\n", receive_buffer, transmit_buffer, command_phase);
 }

 if(!bitpos && transmit_count)
  dsr_pulse_delay = 0x100;

 //SkipDPD: ;

 return(ret);
}

uint32 InputDevice_Memcard::GetNVSize(void)
{
 return(sizeof(card_data));
}

void InputDevice_Memcard::ReadNV(uint8 *buffer, uint32 offset, uint32 size)
{
 while(size--)
 {
  *buffer = card_data[offset & (sizeof(card_data) - 1)];
  buffer++;
  offset++;
 }
}

void InputDevice_Memcard::WriteNV(const uint8 *buffer, uint32 offset, uint32 size)
{
 if(size)
 {
  dirty_count++;
 }

 while(size--)
 {
    if(card_data[offset & (sizeof(card_data) - 1)] != *buffer)
       data_used = true;

    card_data[offset & (sizeof(card_data) - 1)] = *buffer;
    buffer++;
    offset++;
 }
}

uint64 InputDevice_Memcard::GetNVDirtyCount(void)
{
 return(dirty_count);
}

class InputDevice_Mouse : public InputDevice
{
 public:

 InputDevice_Mouse();
 virtual ~InputDevice_Mouse();

 virtual void Power(void);
 virtual int StateAction(StateMem* sm, int load, int data_only, const char* section_name);

 virtual void Update(const int32_t timestamp);
 virtual void ResetTS(void);

 //
 //
 //
 virtual void SetDTR(bool new_dtr);
 virtual bool Clock(bool TxD, int32 &dsr_pulse_delay);

 int32 lastts;
 int32 clear_timeout;

 bool dtr;

 uint8 button;
 uint8 button_post_mask;
 int32 accum_xdelta;
 int32 accum_ydelta;

 int32 command_phase;
 uint32 bitpos;
 uint8 receive_buffer;

 uint8 command;

 uint8 transmit_buffer[5];
 uint32 transmit_pos;
 uint32 transmit_count;
};

InputDevice_Mouse::InputDevice_Mouse()
{
 Power();
}

InputDevice_Mouse::~InputDevice_Mouse()
{
 
}

void InputDevice_Mouse::Update(const int32_t timestamp)
{
 int32 cycles = timestamp - lastts;

 clear_timeout += cycles;
 if(clear_timeout >= (33868800 / 4))
 {
  //puts("Mouse timeout\n");
  clear_timeout = 0;
  accum_xdelta = 0;
  accum_ydelta = 0;
  button &= button_post_mask;
 }

 lastts = timestamp;
}

void InputDevice_Mouse::ResetTS(void)
{
 lastts = 0;
}

void InputDevice_Mouse::Power(void)
{
 lastts = 0;
 clear_timeout = 0;

 dtr = 0;

 button = 0;
 button_post_mask = 0;
 accum_xdelta = 0;
 accum_ydelta = 0;

 command_phase = 0;

 bitpos = 0;

 receive_buffer = 0;

 command = 0;

 memset(transmit_buffer, 0, sizeof(transmit_buffer));

 transmit_pos = 0;
 transmit_count = 0;
}

int InputDevice_Mouse::StateAction(StateMem* sm, int load, int data_only, const char* section_name)
{
 SFORMAT StateRegs[] =
 {
  { &((clear_timeout)), sizeof((clear_timeout)), 0x80000000 | 0, "clear_timeout" },

  { &((dtr)), 1, 0x80000000 | 0x08000000, "dtr" },

  { &((button)), sizeof((button)), 0x80000000 | 0, "button" },
  { &((button_post_mask)), sizeof((button_post_mask)), 0x80000000 | 0, "button_post_mask" },
  { &((accum_xdelta)), sizeof((accum_xdelta)), 0x80000000 | 0, "accum_xdelta" },
  { &((accum_ydelta)), sizeof((accum_ydelta)), 0x80000000 | 0, "accum_ydelta" },

  { &((command_phase)), sizeof((command_phase)), 0x80000000 | 0, "command_phase" },
  { &((bitpos)), sizeof((bitpos)), 0x80000000 | 0, "bitpos" },
  { &((receive_buffer)), sizeof((receive_buffer)), 0x80000000 | 0, "receive_buffer" },

  { &((command)), sizeof((command)), 0x80000000 | 0, "command" },

  { ((transmit_buffer)), (uint32)((sizeof(transmit_buffer))), 0 | 0, "transmit_buffer" },
  { &((transmit_pos)), sizeof((transmit_pos)), 0x80000000 | 0, "transmit_pos" },
  { &((transmit_count)), sizeof((transmit_count)), 0x80000000 | 0, "transmit_count" },

  { 0, 0, 0, 0 }
 };
 int ret = MDFNSS_StateAction(sm, load, StateRegs, section_name);

 if(load)
 {
  if((transmit_pos + transmit_count) > sizeof(transmit_buffer))
  {
   transmit_pos = 0;
   transmit_count = 0;
  }
 }

 return(ret);
}

void InputDevice_Mouse::SetDTR(bool new_dtr)
{
 if(!dtr && new_dtr)
 {
  command_phase = 0;
  bitpos = 0;
  transmit_pos = 0;
  transmit_count = 0;
 }
#if 0
 else if(dtr && !new_dtr)
 {
  //if(bitpos || transmit_count)
  // printf("[PAD] Abort communication!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
 }
#endif

 dtr = new_dtr;
}

bool InputDevice_Mouse::Clock(bool TxD, int32 &dsr_pulse_delay)
{
 bool ret = 1;

 dsr_pulse_delay = 0;

 if(!dtr)
  return(1);

 if(transmit_count)
  ret = (transmit_buffer[transmit_pos] >> bitpos) & 1;

 receive_buffer &= ~(1 << bitpos);
 receive_buffer |= TxD << bitpos;
 bitpos = (bitpos + 1) & 0x7;

 if(!bitpos)
 {
  //printf("[PAD] Receive: %02x -- command_phase=%d\n", receive_buffer, command_phase);

  if(transmit_count)
  {
   transmit_pos++;
   transmit_count--;
  }


  switch(command_phase)
  {
   case 0:
 	  if(receive_buffer != 0x01)
	    command_phase = -1;
	  else
	  {
	   transmit_buffer[0] = 0x12;
	   transmit_pos = 0;
	   transmit_count = 1;
	   command_phase++;
	  }
	  break;

   case 1:
	command = receive_buffer;
	command_phase++;

	transmit_buffer[0] = 0x5A;

	if(command == 0x42)
	{
	 int32 xdelta = accum_xdelta;
	 int32 ydelta = accum_ydelta;

	 if(xdelta < -128) xdelta = -128;
	 if(xdelta > 127) xdelta = 127;

	 if(ydelta < -128) ydelta = -128;
	 if(ydelta > 127) ydelta = 127;

	 transmit_buffer[1] = 0xFF;
	 transmit_buffer[2] = 0xFC ^ (button << 2);
	 transmit_buffer[3] = xdelta;
         transmit_buffer[4] = ydelta;

	 accum_xdelta -= xdelta;
	 accum_ydelta -= ydelta;

	 button &= button_post_mask;

         transmit_pos = 0;
         transmit_count = 5;

	 clear_timeout = 0;
	}
	else
	{
	 command_phase = -1;
         transmit_pos = 0;
         transmit_count = 0;
	}
	break;

  }
 }

 if(!bitpos && transmit_count)
  dsr_pulse_delay = 0x40; //0x100;

 return(ret);
}

InputDeviceInputInfoStruct Device_Mouse_IDII[4] =
{
 { "x_axis", "X Axis", -1, IDIT_X_AXIS_REL },
 { "y_axis", "Y Axis", -1, IDIT_Y_AXIS_REL },
 { "right", "Right Button", 1, IDIT_BUTTON, NULL },
 { "left", "Left Button", 0, IDIT_BUTTON, NULL },
};

/*
 TODO: PS1 multitap appears to have some internal knowledge of controller IDs, so it won't get "stuck" waiting for data from a controller that'll never
       come.  We currently sort of "cheat" due to how the dsr_pulse_delay stuff works, but in the future we should try to emulate this multitap functionality.

       Also, full-mode read startup and subport controller ID read timing isn't quite right, so we should fix that too.
*/

/*
 Notes from tests on real thing(not necessarily emulated the same way here):

	Manual port selection read mode:
		Write 0x01-0x04 instead of 0x01 as first byte, selects port(1=A,2=B,3=C,4=D) to access.

		Ports that don't exist(0x00, 0x05-0xFF) or don't have a device plugged in will not respond(no DSR pulse).

	Full read mode:
		Bit0 of third byte(from-zero-index=0x02) should be set to 1 to enter full read mode, on subsequent reads.

		Appears to require a controller to be plugged into the port specified by the first byte as per manual port selection read mode,
		to write the byte necessary to enter full-read mode; but once the third byte with the bit set has been written, no controller in
		that port is required for doing full reads(and the manual port selection is ignored when doing a full read).

		However, if there are no controllers plugged in, the returned data will be short:
			% 0: 0xff
			% 1: 0x80
			% 2: 0x5a

		Example full-read bytestream(with controllers plugged into port A, port B, and port C, with port D empty):
			% 0: 0xff
			% 1: 0x80
			% 2: 0x5a

			% 3: 0x73	(Port A controller data start)
			% 4: 0x5a
			% 5: 0xff
			% 6: 0xff
			% 7: 0x80
			% 8: 0x8c
			% 9: 0x79
			% 10: 0x8f

			% 11: 0x53	(Port B controller data start)
			% 12: 0x5a
			% 13: 0xff
			% 14: 0xff
			% 15: 0x80
			% 16: 0x80
			% 17: 0x75
			% 18: 0x8e

			% 19: 0x41	(Port C controller data start)
			% 20: 0x5a
			% 21: 0xff
			% 22: 0xff
			% 23: 0xff
			% 24: 0xff
			% 25: 0xff
			% 26: 0xff

			% 27: 0xff	(Port D controller data start)
			% 28: 0xff
			% 29: 0xff
			% 30: 0xff
			% 31: 0xff
			% 32: 0xff
			% 33: 0xff
			% 34: 0xff

*/

InputDevice_Multitap::InputDevice_Multitap()
{
 for(int i = 0; i < 4; i++)
 {
  pad_devices[i] = NULL;
  mc_devices[i] = NULL;
 }
 Power();
}

InputDevice_Multitap::~InputDevice_Multitap()
{
}

void InputDevice_Multitap::SetSubDevice(unsigned int sub_index, InputDevice *device, InputDevice *mc_device)
{
 assert(sub_index < 4);

 //printf("%d\n", sub_index);

 pad_devices[sub_index] = device;
 mc_devices[sub_index] = mc_device;
}


void InputDevice_Multitap::Power(void)
{
 selected_device = -1;
 bit_counter = 0;
 receive_buffer = 0;
 byte_counter = 0;

 mc_mode = false;
 full_mode = false;
 full_mode_setting = false;

 fm_dp = 0;
 memset(fm_buffer, 0, sizeof(fm_buffer));
 fm_deferred_error_temp = false;
 fm_deferred_error = false;
 fm_command_error = false;

 for(int i = 0; i < 4; i++)
 {
  if(pad_devices[i])
   pad_devices[i]->Power();

  if(mc_devices[i])
   mc_devices[i]->Power();
 } 
}

int InputDevice_Multitap::StateAction(StateMem* sm, int load, int data_only, const char* section_name)
{
 SFORMAT StateRegs[] =
 {
  { &((dtr)), 1, 0x80000000 | 0x08000000, "dtr" },

  { &((selected_device)), sizeof((selected_device)), 0x80000000 | 0, "selected_device" },
  { &((full_mode_setting)), 1, 0x80000000 | 0x08000000, "full_mode_setting" },

  { &((full_mode)), 1, 0x80000000 | 0x08000000, "full_mode" },
  { &((mc_mode)), 1, 0x80000000 | 0x08000000, "mc_mode" },

  { &((fm_dp)), sizeof((fm_dp)), 0x80000000 | 0, "fm_dp" },
  { ((&fm_buffer[0][0])), (uint32)((sizeof(fm_buffer) / sizeof(fm_buffer[0][0]))), 0 | 0, "&fm_buffer[0][0]" },

  { &((fm_deferred_error_temp)), 1, 0x80000000 | 0x08000000, "fm_deferred_error_temp" },
  { &((fm_deferred_error)), 1, 0x80000000 | 0x08000000, "fm_deferred_error" },
  { &((fm_command_error)), 1, 0x80000000 | 0x08000000, "fm_command_error" },

  { &((command)), sizeof((command)), 0x80000000 | 0, "command" },
  { &((receive_buffer)), sizeof((receive_buffer)), 0x80000000 | 0, "receive_buffer" },
  { &((bit_counter)), sizeof((bit_counter)), 0x80000000 | 0, "bit_counter" },
  { &((byte_counter)), sizeof((byte_counter)), 0x80000000 | 0, "byte_counter" },

  { 0, 0, 0, 0 }
 };
 int ret = MDFNSS_StateAction(sm, load, StateRegs, section_name);

 if(load)
 {

 }

 return(ret);
}

void InputDevice_Multitap::SetDTR(bool new_dtr)
{
 bool old_dtr = dtr;
 dtr = new_dtr;

 if(!dtr)
 {
#if 0
  if(old_dtr)
  {
   //printf("Multitap stop.\n");
  }
#endif

  bit_counter = 0;
  receive_buffer = 0;
  selected_device = -1;
  mc_mode = false;
  full_mode = false;
 }

 if(!old_dtr && dtr)
 {
  full_mode = full_mode_setting;

  byte_counter = 0;

  //if(full_mode)
  // printf("Multitap start: %d\n", full_mode);
 }

 for(int i = 0; i < 4; i++)
 {
  pad_devices[i]->SetDTR(dtr);
  mc_devices[i]->SetDTR(dtr);
 }
}

bool InputDevice_Multitap::Clock(bool TxD, int32 &dsr_pulse_delay)
{
 if(!dtr)
  return(1);

 bool ret = 1;
 int32 tmp_pulse_delay[2][4] = { { 0, 0, 0, 0 }, { 0, 0, 0, 0 } };

 //printf("Receive bit: %d\n", TxD);
 //printf("TxD %d\n", TxD);

 receive_buffer &= ~ (1 << bit_counter);
 receive_buffer |= TxD << bit_counter;

 if(1)
 {
  if(byte_counter == 0)
  {
   bool mangled_txd = TxD;

   if(bit_counter < 4)
    mangled_txd = (0x01 >> bit_counter) & 1;

   for(unsigned i = 0; i < 4; i++)
   {
    pad_devices[i]->Clock(mangled_txd, tmp_pulse_delay[0][i]);
    mc_devices[i]->Clock(mangled_txd, tmp_pulse_delay[1][i]);
   }
  }
  else
  {
   if(full_mode)
   {
    if(byte_counter == 1)
    {
     ret = (0x80 >> bit_counter) & 1;

     for(unsigned i = 0; i < 4; i++)
     { 
      fm_buffer[i][0] &= (pad_devices[i]->Clock(TxD, tmp_pulse_delay[0][i]) << bit_counter) | (~(1U << bit_counter));
     }
    }
    else if(byte_counter == 2)
    {
     ret = (0x5A >> bit_counter) & 1;
    }
    // || byte_counter == (0x03 + 0x08 * 1) || byte_counter == (0x03 + 0x08 * 2) || byte_counter == (0x03 + 0x08 * 3))
    else if(byte_counter >= 0x03 && byte_counter < 0x03 + 0x08 * 4)
    {
     if(!fm_command_error && byte_counter >= (0x03 + 1) && byte_counter < (0x03 + 0x08))
     {
      for(unsigned i = 0; i < 4; i++)
      { 
       fm_buffer[i][byte_counter - 0x03] &= (pad_devices[i]->Clock(0, tmp_pulse_delay[0][i]) << bit_counter) | (~(1U << bit_counter));
      }
     }
     ret &= ((&fm_buffer[0][0])[byte_counter - 0x03] >> bit_counter) & 1;
    }
   }
   else // to if(full_mode)
   {
    if((unsigned)selected_device < 4)
    {
     ret &= pad_devices[selected_device]->Clock(TxD, tmp_pulse_delay[0][selected_device]);
     ret &= mc_devices[selected_device]->Clock(TxD, tmp_pulse_delay[1][selected_device]);
    }
   }
  } // end else to if(byte_counter == 0)
 }

 //
 //
 //

 bit_counter = (bit_counter + 1) & 0x7;
 if(bit_counter == 0)
 {
  //printf("Receive: 0x%02x\n", receive_buffer);
  if(byte_counter == 0)
  {
   mc_mode = (bool)(receive_buffer & 0xF0);
   if(mc_mode)
    full_mode = false;

   //printf("Zoomba: 0x%02x\n", receive_buffer);
   //printf("Full mode: %d %d %d\n", full_mode, bit_counter, byte_counter);

   if(full_mode)
   {
    memset(fm_buffer, 0xFF, sizeof(fm_buffer));
    selected_device = 0;
   }
   else
   {
    //printf("Device select: %02x\n", receive_buffer);
    fm_deferred_error = false;
    selected_device = ((receive_buffer & 0xF) - 1) & 0xFF;
   }
  }

  if(byte_counter == 1)
  {
   command = receive_buffer;

   //printf("Multitap sub-command: %02x\n", command);

   if(full_mode)
   {
    if(command != 0x42)
     fm_command_error = true;
    else
     fm_command_error = fm_deferred_error;
   }
   else
   {
    fm_command_error = false;
   }
   fm_deferred_error = false;
  }

  if((!mc_mode || full_mode) && byte_counter == 2)
  {
   //printf("Full mode setting: %02x\n", receive_buffer);
   full_mode_setting = receive_buffer & 0x01;
  }

  if(full_mode)
  {
   if(byte_counter == (3 + 8 * 0) || byte_counter == (3 + 8 * 1) || byte_counter == (3 + 8 * 2) || byte_counter == (3 + 8 * 3))
   {
    unsigned index = (byte_counter - 3) >> 3;
    assert(index < 4);

    if(index == 0)
     fm_deferred_error_temp = false;     

    if((fm_dp & (1U << index)) && receive_buffer != 0x42)
    {
     //printf("Multitap command check failed: %u, 0x%02x\n", byte_counter, receive_buffer);
     fm_deferred_error_temp = true;
    }
   }

   if(byte_counter == 33)
    fm_deferred_error = fm_deferred_error_temp;
  }

  // Handle DSR stuff
  if(full_mode)
  {
   if(byte_counter == 0)	// Next byte: 0x80
   {
    dsr_pulse_delay = 1000;

    fm_dp = 0;
    for(unsigned i = 0; i < 4; i++)
     fm_dp |= (((bool)(tmp_pulse_delay[0][i])) << i);
   }
   else if(byte_counter == 1)	// Next byte: 0x5A
    dsr_pulse_delay = 0x40;
   else if(byte_counter == 2)	// Next byte(typically, controller-dependent): 0x41
   {
    if(fm_dp)
     dsr_pulse_delay = 0x40;
    else
     dsr_pulse_delay = 0;
   }
   else if(byte_counter >= 3 && byte_counter < 34)	// Next byte when byte_counter==3 (typically, controller-dependent): 0x5A
   {
    if(byte_counter < 10)
    { 
     int d = 0x40;

     for(unsigned i = 0; i < 4; i++)
      if(tmp_pulse_delay[0][i] > d)
       d = tmp_pulse_delay[0][i];

     dsr_pulse_delay = d;
    }
    else
     dsr_pulse_delay = 0x20;

    if(byte_counter == 3 && fm_command_error)
     dsr_pulse_delay = 0;
   }
  } // end if(full_mode)
  else
  {
   if((unsigned)selected_device < 4)
   {
    dsr_pulse_delay = std::max<int32>(tmp_pulse_delay[0][selected_device], tmp_pulse_delay[1][selected_device]);
   }
  }


  //
  //
  //

  //printf("Byte Counter Increment\n");
  if(byte_counter < 255)
   byte_counter++;
 }



 return(ret);
}


InputDevice_neGcon::InputDevice_neGcon(void)
{
 Power();
}

InputDevice_neGcon::~InputDevice_neGcon()
{

}

void InputDevice_neGcon::Power(void)
{
 dtr = 0;

 buttons[0] = buttons[1] = 0;
 twist = 0;
 anabuttons[0] = 0;
 anabuttons[1] = 0;
 anabuttons[2] = 0;

 command_phase = 0;

 bitpos = 0;

 receive_buffer = 0;

 command = 0;

 memset(transmit_buffer, 0, sizeof(transmit_buffer));

 transmit_pos = 0;
 transmit_count = 0;
}

void InputDevice_neGcon::SetDTR(bool new_dtr)
{
 if(!dtr && new_dtr)
 {
  command_phase = 0;
  bitpos = 0;
  transmit_pos = 0;
  transmit_count = 0;
 }
#if 0
 else if(dtr && !new_dtr)
 {
  //if(bitpos || transmit_count)
  // printf("[PAD] Abort communication!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
 }
#endif

 dtr = new_dtr;
}

bool InputDevice_neGcon::Clock(bool TxD, int32 &dsr_pulse_delay)
{
 bool ret = 1;

 dsr_pulse_delay = 0;

 if(!dtr)
  return(1);

 if(transmit_count)
  ret = (transmit_buffer[transmit_pos] >> bitpos) & 1;

 receive_buffer &= ~(1 << bitpos);
 receive_buffer |= TxD << bitpos;
 bitpos = (bitpos + 1) & 0x7;

 if(!bitpos)
 {
  //printf("[PAD] Receive: %02x -- command_phase=%d\n", receive_buffer, command_phase);

  if(transmit_count)
  {
   transmit_pos++;
   transmit_count--;
  }


  switch(command_phase)
  {
   case 0:
 	  if(receive_buffer != 0x01)
	    command_phase = -1;
	  else
	  {
	   transmit_buffer[0] = 0x23;
	   transmit_pos = 0;
	   transmit_count = 1;
	   command_phase++;
	   dsr_pulse_delay = 256;
	  }
	  break;

   case 1:
	command = receive_buffer;
	command_phase++;

	transmit_buffer[0] = 0x5A;

	//if(command != 0x42)
	// fprintf(stderr, "Gamepad unhandled command: 0x%02x\n", command);

	if(command == 0x42)
	{
	 transmit_buffer[1] = 0xFF ^ buttons[0];
	 transmit_buffer[2] = 0xFF ^ buttons[1];
	 transmit_buffer[3] = twist;			// Twist, 0x00 through 0xFF, 0x80 center.
	 transmit_buffer[4] = anabuttons[0];		// Analog button I, 0x00 through 0xFF, 0x00 = no pressing, 0xFF = max.
	 transmit_buffer[5] = anabuttons[1];		// Analog button II, ""
	 transmit_buffer[6] = anabuttons[2];		// Left shoulder analog button, ""
         transmit_pos = 0;
         transmit_count = 7;
	 dsr_pulse_delay = 256;
	}
	else
	{
	 command_phase = -1;
	 transmit_buffer[1] = 0;
	 transmit_buffer[2] = 0;
         transmit_pos = 0;
         transmit_count = 0;
	}
	break;

   case 2:
	if(transmit_count > 0)
	 dsr_pulse_delay = 128;
	break;
  }
 }

 return(ret);
}

InputDeviceInputInfoStruct Device_neGcon_IDII[21] =
{
 { NULL, "empty", -1, IDIT_BUTTON, NULL },
 { NULL, "empty", -1, IDIT_BUTTON, NULL },
 { NULL, "empty", -1, IDIT_BUTTON, NULL },
 { "start", "START", 4, IDIT_BUTTON, NULL },
 { "up", "D-Pad UP ↑", 0, IDIT_BUTTON, "down" },
 { "right", "D-Pad RIGHT →", 3, IDIT_BUTTON, "left" },
 { "down", "D-Pad DOWN ↓", 1, IDIT_BUTTON, "up" },
 { "left", "D-Pad LEFT ←", 2, IDIT_BUTTON, "right" },

 { NULL, "empty", -1, IDIT_BUTTON, NULL },
 { NULL, "empty", -1, IDIT_BUTTON, NULL },
 { NULL, "empty", -1, IDIT_BUTTON, NULL },
 { "r", "Right Shoulder", 12, IDIT_BUTTON },

 { "b", "B", 9, IDIT_BUTTON, NULL },
 { "a", "A", 10, IDIT_BUTTON, NULL },
 { NULL, "empty", -1, IDIT_BUTTON, NULL },
 { NULL, "empty", -1, IDIT_BUTTON, NULL },

 { "twist_cwise",  "Twist ↓|↑ (Analog, Turn Right)", 6, IDIT_BUTTON_ANALOG },
 { "twist_ccwise", "Twist ↑|↓ (Analog, Turn Left)", 5, IDIT_BUTTON_ANALOG },
 { "i", "I (Analog)", 8, IDIT_BUTTON_ANALOG },
 { "ii", "II (Analog)", 7, IDIT_BUTTON_ANALOG },

 { "l", "Left Shoulder (Analog)", 11, IDIT_BUTTON_ANALOG },
};

InputDevice::InputDevice()
{
   chair_r    = 0;
   chair_g    = 0;
   chair_b    = 0;
   draw_chair = 0;
   chair_x    = -1000;
   chair_y    = -1000;
}

InputDevice::~InputDevice()
{
}

void InputDevice::Power(void)
{
}

int InputDevice::StateAction(StateMem* sm, int load, int data_only, const char* section_name)
{
 return(1);
}

void InputDevice::Update(const int32_t timestamp)
{

}

void InputDevice::ResetTS(void)
{

}

void InputDevice::SetAMCT(bool)
{

}

void InputDevice::SetCrosshairsColor(uint32_t color)
{
   chair_r = (color >> 16) & 0xFF;
   chair_g = (color >>  8) & 0xFF;
   chair_b = (color >>  0) & 0xFF;

   draw_chair = (color != (1 << 24));
}

INLINE void InputDevice::DrawCrosshairs(uint32 *pixels, const MDFN_PixelFormat* const format, const unsigned width, const unsigned pix_clock)
{
   if(draw_chair && chair_y >= -8 && chair_y <= 8)
   {
      int32 ic;
      int32 x_start, x_bound;

      if(chair_y == 0)
         ic = pix_clock / 762925;
      else
         ic = 0;

      x_start = std::max<int32>(0, chair_x - ic);
      x_bound = std::min<int32>(width, chair_x + ic + 1);

      for(int32 x = x_start; x < x_bound; x++)
      {
         int r, g, b, a;
         int nr, ng, nb;

         DecodeColor(pixels[x], r, g, b, a);

         nr = (r + chair_r * 3) >> 2;
         ng = (g + chair_g * 3) >> 2;
         nb = (b + chair_b * 3) >> 2;

         if((int)((abs(r - nr) - 0x40) & (abs(g - ng) - 0x40) & (abs(b - nb) - 0x40)) < 0)
         {
            if((nr | ng | nb) & 0x80)
            {
               nr >>= 1;
               ng >>= 1;
               nb >>= 1;
            }
            else
            {
               nr ^= 0x80;
               ng ^= 0x80;
               nb ^= 0x80;
            }
         }

         pixels[x] = MAKECOLOR(nr, ng, nb, a);
      }
   }
}

int FrontIO_StateAction(StateMem* sm, int load, int data_only)
{
   SFORMAT StateRegs[] =
   {
      { &((ClockDivider)), sizeof((ClockDivider)), 0x80000000 | 0, "ClockDivider" },

      { &((ReceivePending)), 1, 0x80000000 | 0x08000000, "ReceivePending" },
      { &((TransmitPending)), 1, 0x80000000 | 0x08000000, "TransmitPending" },

      { &((ReceiveInProgress)), 1, 0x80000000 | 0x08000000, "ReceiveInProgress" },
      { &((TransmitInProgress)), 1, 0x80000000 | 0x08000000, "TransmitInProgress" },

      { &((ReceiveBufferAvail)), 1, 0x80000000 | 0x08000000, "ReceiveBufferAvail" },

      { &((ReceiveBuffer)), sizeof((ReceiveBuffer)), 0x80000000 | 0, "ReceiveBuffer" },
      { &((TransmitBuffer)), sizeof((TransmitBuffer)), 0x80000000 | 0, "TransmitBuffer" },

      { &((ReceiveBitCounter)), sizeof((ReceiveBitCounter)), 0x80000000 | 0, "ReceiveBitCounter" },
      { &((TransmitBitCounter)), sizeof((TransmitBitCounter)), 0x80000000 | 0, "TransmitBitCounter" },

      { &((Mode)), sizeof((Mode)), 0x80000000 | 0, "Mode" },
      { &((Control)), sizeof((Control)), 0x80000000 | 0, "Control" },
      { &((Baudrate)), sizeof((Baudrate)), 0x80000000 | 0, "Baudrate" },

      { &((istatus)), 1, 0x80000000 | 0x08000000, "istatus" },


      { ((irq10_pulse_ts)), (uint32)(((sizeof(irq10_pulse_ts) / sizeof(irq10_pulse_ts[0]))) * sizeof(uint32)), 0x40000000 | 0, "irq10_pulse_ts" },
      { ((dsr_pulse_delay)), (uint32)(((sizeof(dsr_pulse_delay) / sizeof(dsr_pulse_delay[0]))) * sizeof(uint32)), 0x40000000 | 0, "dsr_pulse_delay" },
      { ((dsr_active_until_ts)), (uint32)(((sizeof(dsr_active_until_ts) / sizeof(dsr_active_until_ts[0]))) * sizeof(uint32)), 0x40000000 | 0, "dsr_active_until_ts" },

      { 0, 0, 0, 0 }
   };

   int ret = MDFNSS_StateAction(sm, load, StateRegs, "FIO");

   for(unsigned i = 0; i < 8; i++)
   {
      char tmpbuf[32];
      snprintf(tmpbuf, sizeof(tmpbuf), "FIODEV%u", i);

      ret &= Devices[i]->StateAction(sm, load, data_only, tmpbuf);
   }

   for(unsigned i = 0; i < 8; i++)
   {
      char tmpbuf[32];
      snprintf(tmpbuf, sizeof(tmpbuf), "FIOMC%u", i);

      ret &= DevicesMC[i]->StateAction(sm, load, data_only, tmpbuf);
   }

   for(unsigned i = 0; i < 2; i++)
   {
      char tmpbuf[32];
      snprintf(tmpbuf, sizeof(tmpbuf), "FIOTAP%u", i);

      ret &= DevicesTap[i]->StateAction(sm, load, data_only, tmpbuf);
   }

   if(load)
   {
      IRQ_Assert(IRQ_SIO, istatus);
   }

   return(ret);
}

bool InputDevice::RequireNoFrameskip(void)
{
 return false;
}

int32_t InputDevice::GPULineHook(const int32_t timestamp, bool vsync, uint32 *pixels, const MDFN_PixelFormat* const format, const unsigned width, const unsigned pix_clock_offset, const unsigned pix_clock, const unsigned pix_clock_divider)
{
 return(PSX_EVENT_MAXTS);
}


void InputDevice::SetDTR(bool new_dtr)
{

}

bool InputDevice::Clock(bool TxD, int32_t &dsr_pulse_delay)
{
   dsr_pulse_delay = 0;

   return 1;
}

uint32_t InputDevice::GetNVSize(void)
{
   return 0;
}

void InputDevice::ReadNV(uint8_t *buffer, uint32_t offset, uint32_t count)
{

}

void InputDevice::WriteNV(const uint8_t *buffer, uint32_t offset, uint32_t count)
{

}

uint64_t InputDevice::GetNVDirtyCount(void)
{
   return 0;
}

static unsigned EP_to_MP(bool emulate_multitap[2], unsigned ep)
{
   if(!emulate_multitap[0] && emulate_multitap[1])
   {
      if(ep == 0 || ep >= 5)
         return 0;
      return 1;
   }
   return(ep >= 4);
}

static INLINE unsigned EP_to_SP(bool emulate_multitap[2], unsigned ep)
{
   if(!emulate_multitap[0] && emulate_multitap[1])
   {
      if(ep == 0)
         return(0);
      if(ep < 5)
         return(ep - 1);
      return(ep - 4);
   }
   return(ep & 0x3);
}

void *FrontIO_GetMemcardDevice(unsigned int which)
{
   if (DevicesMC[which])
      return DevicesMC[which];
   return NULL;
}

static void FrontIO_MapDevicesToPorts(void)
{
   int i;

   if(emulate_multitap[0] && emulate_multitap[1])
   {
      for (i = 0; i < 2; i++)
      {
         Ports[i] = DevicesTap[i];
         MCPorts[i] = DummyDevice;

         PortsType[i]   = INPUTDEVICE_MULTITAP;
         MCPortsType[i] = INPUTDEVICE_NONE;
      }
   }
   else if(!emulate_multitap[0] && emulate_multitap[1])
   {
      Ports[0] = Devices[0];
      PortsType[0] = DevicesType[0];
      MCPorts[0] = emulate_memcards[0] ? DevicesMC[0] : DummyDevice;
      MCPortsType[0] = emulate_memcards[0] ? INPUTDEVICE_MEMCARD : INPUTDEVICE_NONE;

      Ports[1]       = DevicesTap[1];
      MCPorts[1]     = DummyDevice;
      PortsType[1]   = INPUTDEVICE_MULTITAP;
      MCPortsType[1] = INPUTDEVICE_NONE;
   }
   else if(emulate_multitap[0] && !emulate_multitap[1])
   {
      Ports[0] = DevicesTap[0];
      PortsType[0] = INPUTDEVICE_MULTITAP;
      MCPorts[0] = DummyDevice;
      MCPortsType[0] = INPUTDEVICE_NONE;

      Ports[1] = Devices[4];
      PortsType[1] = DevicesType[4];
      MCPorts[1] = emulate_memcards[4] ? DevicesMC[4] : DummyDevice;
      MCPortsType[1] = emulate_memcards[4] ? INPUTDEVICE_MEMCARD : INPUTDEVICE_NONE;
   }
   else
   {
      for(i = 0; i < 2; i++)
      {
         Ports[i] = Devices[i];
         PortsType[0] = DevicesType[i];
         MCPorts[i] = emulate_memcards[i] ? DevicesMC[i] : DummyDevice;
         MCPortsType[i] = emulate_memcards[i] ? INPUTDEVICE_MEMCARD : INPUTDEVICE_NONE;
      }
   }

   //printf("\n");
   for(i = 0; i < 8; i++)
   {
      unsigned mp = EP_to_MP(emulate_multitap, i);

      if(emulate_multitap[mp])
         DevicesTap[mp]->SetSubDevice(EP_to_SP(emulate_multitap, i), Devices[i], emulate_memcards[i] ? DevicesMC[i] : DummyDevice);
      else
         DevicesTap[mp]->SetSubDevice(EP_to_SP(emulate_multitap, i), DummyDevice, DummyDevice);

      //printf("%d-> multitap: %d, sub-port: %d\n", i, mp, EP_to_SP(emulate_multitap, i));
   }
}

void FrontIO_New(bool emulate_memcards_[8], bool emulate_multitap_[2])
{
   int i;
   memcpy(emulate_memcards, emulate_memcards_, sizeof(emulate_memcards));
   memcpy(emulate_multitap, emulate_multitap_, sizeof(emulate_multitap));

   DummyDevice = new InputDevice();

   for(i = 0; i < 8; i++)
   {
      DeviceData[i] = NULL;
      Devices[i] = new InputDevice();
      DevicesMC[i] = new InputDevice_Memcard();
      chair_colors[i] = 1 << 24;
      Devices[i]->SetCrosshairsColor(chair_colors[i]);
   }

   for(i = 0; i < 2; i++)
      DevicesTap[i] = new InputDevice_Multitap();

   FrontIO_MapDevicesToPorts();
}

void FrontIO_SetAMCT(bool enabled)
{
   int i;
   for(i = 0; i < 8; i++)
      Devices[i]->SetAMCT(enabled);
   amct_enabled = enabled;
}

void FrontIO_SetCrosshairsColor(unsigned port, uint32_t color)
{
   assert(port >= 0 && port < 8);

   chair_colors[port] = color;
   Devices[port]->SetCrosshairsColor(color);
}

void FrontIO_Free()
{
   int i;
   for(i = 0; i < 8; i++)
   {
      if(Devices[i])
         delete Devices[i];
      Devices[i] = NULL;
      if(DevicesMC[i])
         delete DevicesMC[i];
      DevicesMC[i] = NULL;
   }

   for(i = 0; i < 2; i++)
   {
      if(DevicesTap[i])
         delete DevicesTap[i];
      DevicesTap[i] = NULL;
   }

   if(DummyDevice)
      delete DummyDevice;
   DummyDevice = NULL;
}

static const uint8_t ScaleShift[4] = { 0, 0, 4, 6 };

int32_t FrontIO_CalcNextEventTS(int32_t timestamp, int32_t next_event)
{
   int32_t ret, i;

   if(ClockDivider > 0 && ClockDivider < next_event)
      next_event = ClockDivider;

   for(i = 0; i < 4; i++)
      if(dsr_pulse_delay[i] > 0 && next_event > dsr_pulse_delay[i])
         next_event = dsr_pulse_delay[i];

   ret = timestamp + next_event;

   if(irq10_pulse_ts[0] < ret)
      ret = irq10_pulse_ts[0];

   if(irq10_pulse_ts[1] < ret)
      ret = irq10_pulse_ts[1];

   return(ret);
}

static void FrontIO_CheckStartStopPending(int32_t timestamp, bool skip_event_set)
{
   //const bool prior_ReceiveInProgress = ReceiveInProgress;
   //const bool prior_TransmitInProgress = TransmitInProgress;
   bool trigger_condition = false;

   trigger_condition = (ReceivePending && (Control & 0x4)) || (TransmitPending && (Control & 0x1));

   if(trigger_condition)
   {
      if(ReceivePending)
      {
         ReceivePending = false;
         ReceiveInProgress = true;
         ReceiveBufferAvail = false;
         ReceiveBuffer = 0;
         ReceiveBitCounter = 0;
      }

      if(TransmitPending)
      {
         TransmitPending = false;
         TransmitInProgress = true;
         TransmitBitCounter = 0;
      }

      ClockDivider = std::max<uint32>(0x20, (Baudrate << ScaleShift[Mode & 0x3]) & ~1); // Minimum of 0x20 is an emulation sanity check to prevent severe performance degradation.
      //printf("CD: 0x%02x\n", ClockDivider);
   }

   if(!(Control & 0x5))
   {
      ReceiveInProgress = false;
      TransmitInProgress = false;
   }

   if(!ReceiveInProgress && !TransmitInProgress)
      ClockDivider = 0;

   if(!(skip_event_set))
      PSX_SetEventNT(MDFN_IEN_PSX::PSX_EVENT_FIO, FrontIO_CalcNextEventTS(timestamp, 0x10000000));
}

/* DSR IRQ bit setting appears(from indirect tests on real PS1) 
 * to be level-sensitive, not edge-sensitive.
 */
static INLINE void FrontIO_DoDSRIRQ(void)
{
   if(Control & 0x1000)
   {
      PSX_FIODBGINFO("[DSR] IRQ");
      istatus = true;
      IRQ_Assert(IRQ_SIO, true);
   }
}

void FrontIO_Write(int32_t timestamp, uint32_t A, uint32_t V)
{
   assert(!(A & 0x1));

   //PSX_FIODBGINFO("[FIO] Write: %08x %08x", A, V);

   FrontIO_Update(timestamp);

   switch(A & 0xF)
   {
      case 0x0:
         TransmitBuffer = V;
         TransmitPending = true;
         TransmitInProgress = false;
         break;

      case 0x8:
         Mode = V & 0x013F;
         break;

      case 0xa:
#if 0
         if(ClockDivider > 0 && ((V & 0x2000) != (Control & 0x2000)) && ((Control & 0x2) == (V & 0x2))  )
            PSX_DBG(PSX_DBG_WARNING, "FIO device selection changed during comm %04x->%04x\n", Control, V);
#endif

         //printf("Control: %d, %04x\n", timestamp, V);
         Control = V & 0x3F2F;

         if(V & 0x10)
         {
            istatus = false;
            IRQ_Assert(IRQ_SIO, false);
         }

         if(V & 0x40)	// Reset
         {
            istatus = false;
            IRQ_Assert(IRQ_SIO, false);

            ClockDivider = 0;
            ReceivePending = false;
            TransmitPending = false;

            ReceiveInProgress = false;
            TransmitInProgress = false;

            ReceiveBufferAvail = false;

            TransmitBuffer = 0;
            ReceiveBuffer = 0;

            ReceiveBitCounter = 0;
            TransmitBitCounter = 0;

            Mode = 0;
            Control = 0;
            Baudrate = 0;
         }

         Ports[0]->SetDTR((Control & 0x2) && !(Control & 0x2000));
         MCPorts[0]->SetDTR((Control & 0x2) && !(Control & 0x2000));
         Ports[1]->SetDTR((Control & 0x2) && (Control & 0x2000));
         MCPorts[1]->SetDTR((Control & 0x2) && (Control & 0x2000));

#if 1
         if(!((Control & 0x2) && !(Control & 0x2000)))
         {
            dsr_pulse_delay[0] = 0;
            dsr_pulse_delay[2] = 0;
            dsr_active_until_ts[0] = -1;
            dsr_active_until_ts[2] = -1;
         }

         if(!((Control & 0x2) && (Control & 0x2000)))
         {
            dsr_pulse_delay[1] = 0;
            dsr_pulse_delay[3] = 0;
            dsr_active_until_ts[1] = -1;
            dsr_active_until_ts[3] = -1;
         }

#endif
         // TODO: Uncomment out in the future once our CPU emulation is a bit more accurate with timing, to prevent causing problems with games
         // that may clear the IRQ in an unsafe pattern that only works because its execution was slow enough to allow DSR to go inactive.  (Whether or not
         // such games even exist though is unknown!)
         //if(timestamp < dsr_active_until_ts[0] || timestamp < dsr_active_until_ts[1] || timestamp < dsr_active_until_ts[2] || timestamp < dsr_active_until_ts[3])
         // DoDSRIRQ();

         break;

      case 0xe:
         Baudrate = V;
         //printf("%02x\n", V);
         //MDFN_DispMessage("%02x\n", V);
         break;
   }

   FrontIO_CheckStartStopPending(timestamp, false);
}

uint32_t FrontIO_Read(int32_t timestamp, uint32_t A)
{
   uint32_t ret = 0;

   assert(!(A & 0x1));

   FrontIO_Update(timestamp);

   switch(A & 0xF)
   {
      case 0x0:
         //printf("FIO Read: 0x%02x\n", ReceiveBuffer);
         ret = ReceiveBuffer | (ReceiveBuffer << 8) | (ReceiveBuffer << 16) | (ReceiveBuffer << 24);
         ReceiveBufferAvail = false;
         ReceivePending = true;
         ReceiveInProgress = false;
         FrontIO_CheckStartStopPending(timestamp, false);
         break;

      case 0x4:
         ret = 0;

         if(!TransmitPending && !TransmitInProgress)
            ret |= 0x1;

         if(ReceiveBufferAvail)
            ret |= 0x2;

         if(
               timestamp < dsr_active_until_ts[0] || 
               timestamp < dsr_active_until_ts[1] || 
               timestamp < dsr_active_until_ts[2] || 
               timestamp < dsr_active_until_ts[3])
            ret |= 0x80;

         if(istatus)
            ret |= 0x200;

         break;

      case 0x8:
         ret = Mode;
         break;

      case 0xa:
         ret = Control;
         break;

      case 0xe:
         ret = Baudrate;
         break;
   }

#if 0
   if((A & 0xF) != 0x4)
      PSX_FIODBGINFO("[FIO] Read: %08x %08x", A, ret);
#endif

   return(ret);
}

#if 0
bool FrontIO_GetDSR(void)
{
   //dualshock
   //gamepad
   //guncon
   //justifier
   //memcard
   ////negcon
   if(!dtr)
      return(0);

   if(!bitpos && transmit_count)
      return(1);

   //multitap
   return(0);
}
#endif

int32_t FrontIO_Update(int32_t timestamp)
{
   int32_t clocks, i;
   bool need_start_stop_check = false;

   clocks = timestamp - lastts;

   for(i = 0; i < 4; i++)
      if(dsr_pulse_delay[i] > 0)
      {
         dsr_pulse_delay[i] -= clocks;
         if(dsr_pulse_delay[i] <= 0)
         {
            dsr_active_until_ts[i] = timestamp + 32 + dsr_pulse_delay[i];
            FrontIO_DoDSRIRQ();
         }
      }

   for(i = 0; i < 2; i++)
   {
      if(timestamp >= irq10_pulse_ts[i])
      {
         //printf("Yay: %d %u\n", i, timestamp);
         irq10_pulse_ts[i] = PSX_EVENT_MAXTS;
         IRQ_Assert(IRQ_PIO, true);
         IRQ_Assert(IRQ_PIO, false);
      }
   }

   if(ClockDivider > 0)
   {
      ClockDivider -= clocks;

      while(ClockDivider <= 0)
      {
         if(ReceiveInProgress || TransmitInProgress)
         {
            bool rxd = 0, txd = 0;
            const uint32_t BCMask = 0x07;

            if(TransmitInProgress)
            {
               txd = (TransmitBuffer >> TransmitBitCounter) & 1;
               TransmitBitCounter = (TransmitBitCounter + 1) & BCMask;
               if(!TransmitBitCounter)
               {
                  need_start_stop_check = true;
                  PSX_FIODBGINFO("[FIO] Data transmitted: %08x", TransmitBuffer);
                  TransmitInProgress = false;

                  if(Control & 0x400)
                  {
                     istatus = true;
                     IRQ_Assert(IRQ_SIO, true);
                  }
               }
            }

            rxd = Ports[0]->Clock(txd, dsr_pulse_delay[0]) & Ports[1]->Clock(txd, dsr_pulse_delay[1]) &
               MCPorts[0]->Clock(txd, dsr_pulse_delay[2]) & MCPorts[1]->Clock(txd, dsr_pulse_delay[3]);

            if(ReceiveInProgress)
            {
               ReceiveBuffer &= ~(1 << ReceiveBitCounter);
               ReceiveBuffer |= rxd << ReceiveBitCounter;

               ReceiveBitCounter = (ReceiveBitCounter + 1) & BCMask;

               if(!ReceiveBitCounter)
               {
                  need_start_stop_check = true;
                  PSX_FIODBGINFO("[FIO] Data received: %08x", ReceiveBuffer);

                  ReceiveInProgress = false;
                  ReceiveBufferAvail = true;

                  if(Control & 0x800)
                  {
                     istatus = true;
                     IRQ_Assert(IRQ_SIO, true);
                  }
               }
            }
            ClockDivider += std::max<uint32>(0x20, (Baudrate << ScaleShift[Mode & 0x3]) & ~1); // Minimum of 0x20 is an emulation sanity check to prevent severe performance degradation.
         }
         else
            break;
      }
   }


   lastts = timestamp;


   if(need_start_stop_check)
      FrontIO_CheckStartStopPending(timestamp, true);

   return(FrontIO_CalcNextEventTS(timestamp, 0x10000000));
}

void FrontIO_ResetTS(void)
{
   int i;
   for(i = 0; i < 8; i++)
   {
      Devices[i]->Update(lastts);	// Maybe eventually call Update() from FrontIO_Update() and remove this(but would hurt speed)?
      Devices[i]->ResetTS();

      DevicesMC[i]->Update(lastts);	// Maybe eventually call Update() from FrontIO_Update() and remove this(but would hurt speed)?
      DevicesMC[i]->ResetTS();
   }

   for(i = 0; i < 2; i++)
   {
      DevicesTap[i]->Update(lastts);
      DevicesTap[i]->ResetTS();
   }

   for(i = 0; i < 2; i++)
   {
      if(irq10_pulse_ts[i] != PSX_EVENT_MAXTS)
         irq10_pulse_ts[i] -= lastts;
   }

   for(i = 0; i < 4; i++)
   {
      if(dsr_active_until_ts[i] >= 0)
      {
         dsr_active_until_ts[i] -= lastts;
         //printf("SPOOONY: %d %d\n", i, dsr_active_until_ts[i]);
      }
   }
   lastts = 0;
}


void FrontIO_Power(void)
{
   int i;
   for(i = 0; i < 4; i++)
   {
      dsr_pulse_delay[i] = 0;
      dsr_active_until_ts[i] = -1;
   }

   for(i = 0; i < 2; i++)
      irq10_pulse_ts[i] = PSX_EVENT_MAXTS;

   lastts = 0;

   //
   //

   ClockDivider = 0;

   ReceivePending = false;
   TransmitPending = false;

   ReceiveInProgress = false;
   TransmitInProgress = false;

   ReceiveBufferAvail = false;

   TransmitBuffer = 0;
   ReceiveBuffer = 0;

   ReceiveBitCounter = 0;
   TransmitBitCounter = 0;

   Mode = 0;
   Control = 0;
   Baudrate = 0;

   for(i = 0; i < 8; i++)
   {
      Devices[i]->Power();
      DevicesMC[i]->Power();
   }

   istatus = false;
}

void FrontIO_UpdateInput(void)
{
   int i;
   for(i = 0; i < 8; i++)
   {
      switch (DevicesType[i])
      {
         case INPUTDEVICE_GAMEPAD:
            {
               InputDevice_Gamepad *device = (InputDevice_Gamepad*)Devices[i];
               uint8 *d8 = (uint8 *)DeviceData[i];

               device->buttons[0] = d8[0];
               device->buttons[1] = d8[1];
            }
            break;
         case INPUTDEVICE_MOUSE:
            {
               InputDevice_Mouse *device = (InputDevice_Mouse*)Devices[i];
               device->accum_xdelta += (int32)MDFN_de32lsb((uint8*)DeviceData[i] + 0);
               device->accum_ydelta += (int32)MDFN_de32lsb((uint8*)DeviceData[i] + 4);

               if(device->accum_xdelta > 30 * 127)
                  device->accum_xdelta = 30 * 127;
               if(device->accum_xdelta < 30 * -128)
                  device->accum_xdelta = 30 * -128;

               if(device->accum_ydelta > 30 * 127)
                  device->accum_ydelta = 30 * 127;
               if(device->accum_ydelta < 30 * -128)
                  device->accum_ydelta = 30 * -128;

               device->button |= *((uint8 *)DeviceData[i] + 8);
               device->button_post_mask = *((uint8 *)DeviceData[i] + 8);
            }
            break;
         case INPUTDEVICE_DUALANALOG:
            {
               int stick, axis;
               InputDevice_DualAnalog *device = (InputDevice_DualAnalog*)Devices[i];
               uint8 *d8 = (uint8 *)DeviceData[i];

               device->buttons[0] = d8[0];
               device->buttons[1] = d8[1];

               for(stick = 0; stick < 2; stick++)
               {
                  for(axis = 0; axis < 2; axis++)
                  {
                     const uint8* aba = &d8[2] + stick * 8 + axis * 4;
                     int32 tmp;

                     //revert to 0.9.33, should be fixed on libretro side instead
                     //tmp = 32768 + MDFN_de16lsb(&aba[0]) - ((int32)MDFN_de16lsb(&aba[2]) * 32768 / 32767);

                     tmp = 32768 + MDFN_de32lsb((const uint8 *)DeviceData[i] + 
                           stick * 16 + axis * 8 + 4) - ((int32)MDFN_de32lsb((const uint8 *)DeviceData[i] + stick * 16 + axis * 8 + 8) * 32768 / 32767);
                     tmp >>= 8;

                     device->axes[stick][axis] = tmp;
                  }
               }

               //printf("%d %d %d %d\n", device->axes[0][0], device->axes[0][1], device->axes[1][0], device->axes[1][1]);
            }
            break;
         case INPUTDEVICE_DUALSHOCK:
            {
               int stick, axis;
               InputDevice_DualShock *device = (InputDevice_DualShock*)Devices[i];
               uint8 *d8 = (uint8 *)DeviceData[i];
               uint8* const rumb_dp = &d8[3 + 16];

               device->buttons[0] = d8[0];
               device->buttons[1] = d8[1];
               device->cur_ana_button_state = d8[2] & 0x01;

               for(stick = 0; stick < 2; stick++)
               {
                  for(axis = 0; axis < 2; axis++)
                  {
                     const uint8* aba = &d8[3] + stick * 8 + axis * 4;
                     int32 tmp;

                     //revert to 0.9.33, should be fixed on libretro side instead
                     //tmp = 32767 + MDFN_de16lsb(&aba[0]) - MDFN_de16lsb(&aba[2]);
                     //tmp = (tmp * 0x100) / 0xFFFF;

                     tmp = 32768 + MDFN_de32lsb((const uint8 *)DeviceData[i] + stick * 16 + axis * 8 + 4) - ((int32)MDFN_de32lsb((const uint8 *)DeviceData[i] + stick * 16 + axis * 8 + 8) * 32768 / 32767);
                     tmp >>= 8;
                     device->axes[stick][axis] = tmp;
                  }
               }

               //printf("%3d:%3d, %3d:%3d\n", axes[0][0], axes[0][1], axes[1][0], axes[1][1]);
            }
            break;
         case INPUTDEVICE_NEGCON:
            {
               InputDevice_neGcon *device = (InputDevice_neGcon*)Devices[i];
               uint8 *d8 = (uint8 *)DeviceData[i];

               device->buttons[0] = d8[0];
               device->buttons[1] = d8[1];

               device->twist = ((32768 + MDFN_de32lsb((const uint8 *)DeviceData[i] + 4) - (((int32)MDFN_de32lsb((const uint8 *)DeviceData[i] + 8) * 32768 + 16383) / 32767)) * 255 + 32767) / 65535;

               device->anabuttons[0] = (MDFN_de32lsb((const uint8 *)DeviceData[i] + 12) * 255 + 16383) / 32767; 
               device->anabuttons[1] = (MDFN_de32lsb((const uint8 *)DeviceData[i] + 16) * 255 + 16383) / 32767;
               device->anabuttons[2] = (MDFN_de32lsb((const uint8 *)DeviceData[i] + 20) * 255 + 16383) / 32767;
            }
            break;
         case INPUTDEVICE_GUNCON:
            {
               InputDevice_GunCon *device = (InputDevice_GunCon*)Devices[i];
               uint8 *d8 = (uint8 *)DeviceData[i];

               device->nom_x = (int16)MDFN_de16lsb(&d8[0]);
               device->nom_y = (int16)MDFN_de16lsb(&d8[2]);

               device->trigger_noclear = (bool)(d8[4] & 0x1);
               device->trigger_eff |= device->trigger_noclear;

               device->buttons = d8[4] >> 1;

               if(device->os_shot_counter > 0)	// FIXME if UpdateInput() is ever called more than once per video frame(at ~50 or ~60Hz).
                  device->os_shot_counter--;

               if((d8[4] & 0x8) && !device->prev_oss && device->os_shot_counter == 0)
                  device->os_shot_counter = 4;
               device->prev_oss = d8[4] & 0x8;
            }
            break;
         case INPUTDEVICE_JUSTIFIER:
            {
               InputDevice_Justifier *device = (InputDevice_Justifier*)Devices[i];
               uint8 *d8 = (uint8 *)DeviceData[i];

               device->nom_x = (int16)MDFN_de16lsb(&d8[0]);
               device->nom_y = (int16)MDFN_de16lsb(&d8[2]);

               device->trigger_noclear = (bool)(d8[4] & 0x1);
               device->trigger_eff |= device->trigger_noclear;

               device->buttons = (d8[4] >> 1) & 0x3;

               if(device->os_shot_counter > 0)	// FIXME if UpdateInput() is ever called more than once per video frame(at ~50 or ~60Hz).
                  device->os_shot_counter--;

               if((d8[4] & 0x8) && !device->prev_oss && device->os_shot_counter == 0)
                  device->os_shot_counter = 10;
               device->prev_oss = d8[4] & 0x8;
            }
            break;
      }
   }
}

void FrontIO_SetInput(unsigned int port, const char *type, void *ptr)
{
   delete Devices[port];
   Devices[port] = NULL;

   if(port < 2)
      irq10_pulse_ts[port] = PSX_EVENT_MAXTS;

   if(!strcmp(type, "gamepad") || !strcmp(type, "dancepad"))
   {
      Devices[port] = new InputDevice_Gamepad();
      DevicesType[port] = INPUTDEVICE_GAMEPAD;
   }
   else if(!strcmp(type, "dualanalog"))
   {
      Devices[port] = new InputDevice_DualAnalog(false);
      DevicesType[port] = INPUTDEVICE_DUALANALOG;
   }
   else if(!strcmp(type, "analogjoy"))
   {
      Devices[port] = new InputDevice_DualAnalog(true);
      DevicesType[port] = INPUTDEVICE_ANALOG_JOYSTICK;
   }
   else if(!strcmp(type, "dualshock"))
   {
      char name[256];
      snprintf(name, 256, _("DualShock on port %u"), port + 1);
      Devices[port] = new InputDevice_DualShock(std::string(name));
      DevicesType[port] = INPUTDEVICE_DUALSHOCK;
   }
   else if(!strcmp(type, "mouse"))
   {
      Devices[port] = new InputDevice_Mouse();
      DevicesType[port] = INPUTDEVICE_MOUSE;
   }
   else if(!strcmp(type, "negcon"))
   {
      Devices[port] = new InputDevice_neGcon();
      DevicesType[port] = INPUTDEVICE_NEGCON;
   }
   else if(!strcmp(type, "guncon"))
   {
      Devices[port] = new InputDevice_GunCon();
      DevicesType[port] = INPUTDEVICE_GUNCON;
   }
   else if(!strcmp(type, "justifier"))
   {
      Devices[port] = new InputDevice_Justifier();
      DevicesType[port] = INPUTDEVICE_JUSTIFIER;
   }
   else
      Devices[port] = new InputDevice();

   Devices[port]->SetAMCT(amct_enabled);
   Devices[port]->SetCrosshairsColor(chair_colors[port]);
   DeviceData[port] = ptr;

   FrontIO_MapDevicesToPorts();
}

uint64_t FrontIO_GetMemcardDirtyCount(unsigned int which)
{
   assert(which < 8);
   return(DevicesMC[which]->GetNVDirtyCount());
}

void FrontIO_LoadMemcard(unsigned int which)
{
   assert(which < 8);

   if(DevicesMC[which]->GetNVSize())
   {
      InputDevice_Memcard *device = (InputDevice_Memcard*)DevicesMC[which];
      device->WriteNV(device->card_data, 0, (1 << 17));
      device->dirty_count = 0;		// There's no need to rewrite the file if it's the same data.
   }
}

void FrontIO_LoadMemcard(unsigned int which, const char *path)
{
   assert(which < 8);

   if(DevicesMC[which]->GetNVSize())
   {
      FILE *fp = fopen(path, "rb");

      if (!fp)
         return;

      InputDevice_Memcard *device = (InputDevice_Memcard*)DevicesMC[which];
      fread(device->card_data, 1, 1 << 17, fp);
      fclose(fp);
      device->WriteNV(device->card_data, 0, (1 << 17));
      device->dirty_count = 0;		// There's no need to rewrite the file if it's the same data.
   }
}

void FrontIO_SaveMemcard(unsigned int which)
{
   assert(which < 8);

   if(DevicesMC[which]->GetNVSize() && DevicesMC[which]->GetNVDirtyCount())
   {
      InputDevice_Memcard *device = (InputDevice_Memcard*)DevicesMC[which];
      device->ReadNV(device->card_data, 0, (1 << 17));
      device->dirty_count = 0;		// There's no need to rewrite the file if it's the same data.
   }
}

void FrontIO_SaveMemcard(unsigned int which, const char *path)
{
   assert(which < 8);

   if(DevicesMC[which]->GetNVSize() && DevicesMC[which]->GetNVDirtyCount())
   {
      FILE *fp = fopen(path, "rb");

      if (!fp)
         return;

      InputDevice_Memcard *device = (InputDevice_Memcard*)DevicesMC[which];
      device->ReadNV(device->card_data, 0, (1 << 17));
      fwrite(device->card_data, 1, (1 << 17), fp);
      fclose(fp);
      device->dirty_count = 0;		// There's no need to rewrite the file if it's the same data.
   }
}

bool FrontIO_RequireNoFrameskip(void)
{
   unsigned i;

   for(i = 0; i < 8; i++)
      if(Devices[i]->RequireNoFrameskip())
         return(true);

   return(false);
}

void FrontIO_GPULineHook(const int32_t timestamp, const int32_t line_timestamp,
      bool vsync, uint32 *pixels, const MDFN_PixelFormat* const format,
      const unsigned width, const unsigned pix_clock_offset, const unsigned pix_clock,
      const unsigned pix_clock_divider)
{
   FrontIO_Update(timestamp);

   for(unsigned i = 0; i < 8; i++)
   {
      int32_t plts = Devices[i]->GPULineHook(line_timestamp, vsync, pixels, format, width, pix_clock_offset, pix_clock, pix_clock_divider);

      if(i < 2)
      {
         irq10_pulse_ts[i] = plts;

         if(irq10_pulse_ts[i] <= timestamp)
         {
            irq10_pulse_ts[i] = PSX_EVENT_MAXTS;
            IRQ_Assert(IRQ_PIO, true);
            IRQ_Assert(IRQ_PIO, false);
         }
      }
   }

   //
   // Draw crosshairs in a separate pass so the crosshairs won't mess up the color evaluation of later lightun GPULineHook()s.
   //
   if(pixels && pix_clock)
   {
      for(unsigned i = 0; i < 8; i++)
         Devices[i]->DrawCrosshairs(pixels, format, width, pix_clock);
   }

   PSX_SetEventNT(MDFN_IEN_PSX::PSX_EVENT_FIO, FrontIO_CalcNextEventTS(timestamp, 0x10000000));
}


static InputDeviceInfoStruct InputDeviceInfoPSXPort[] =
{
 // None
 {
  "none",
  "none",
  NULL,
  NULL,
  0,
  NULL 
 },

 // Gamepad(SCPH-1080)
 {
  "gamepad",
  "Digital Gamepad",
  "PlayStation digital gamepad; SCPH-1080.",
  NULL,
  sizeof(Device_Gamepad_IDII) / sizeof(InputDeviceInputInfoStruct),
  Device_Gamepad_IDII,
 },

 // Dual Shock Gamepad(SCPH-1200)
 {
  "dualshock",
  "DualShock",
  "DualShock gamepad; SCPH-1200.  Emulation in Mednafen includes the analog mode toggle button.  Rumble is emulated, but currently only supported on Linux, and MS Windows via the XInput API and XInput-compatible gamepads/joysticks.  If you're having trouble getting rumble to work on Linux, see if Mednafen is printing out error messages during startup regarding /dev/input/event*, and resolve the issue(s) as necessary.",
  NULL,
  sizeof(Device_DualShock_IDII) / sizeof(InputDeviceInputInfoStruct),
  Device_DualShock_IDII,
 },

 // Dual Analog Gamepad(SCPH-1180), forced to analog mode.
 {
  "dualanalog",
  "Dual Analog",
  "Dual Analog gamepad; SCPH-1180.  It is the predecessor/prototype to the more advanced DualShock.  Emulated in Mednafen as forced to analog mode, and without rumble.",
  NULL,
  sizeof(Device_DualAnalog_IDII) / sizeof(InputDeviceInputInfoStruct),
  Device_DualAnalog_IDII,
 },


 // Analog joystick(SCPH-1110), forced to analog mode - emulated through a tweak to dual analog gamepad emulation.
 {
  "analogjoy",
  "Analog Joystick",
  "Flight-game-oriented dual-joystick controller; SCPH-1110.   Emulated in Mednafen as forced to analog mode.",
  NULL,
  sizeof(Device_AnalogJoy_IDII) / sizeof(InputDeviceInputInfoStruct),
  Device_AnalogJoy_IDII,
 },

 {
  "mouse",
  "Mouse",
  NULL,
  NULL,
  sizeof(Device_Mouse_IDII) / sizeof(InputDeviceInputInfoStruct),
  Device_Mouse_IDII,
 },

 {
  "negcon",
  "neGcon",
  "Namco's unconventional twisty racing-game-oriented gamepad; NPC-101.",
  NULL,
  sizeof(Device_neGcon_IDII) / sizeof(InputDeviceInputInfoStruct),
  Device_neGcon_IDII,
 },

 {
  "guncon",
  "GunCon",
  "Namco's light gun; NPC-103.",
  NULL,
  sizeof(Device_GunCon_IDII) / sizeof(InputDeviceInputInfoStruct),
  Device_GunCon_IDII,
 },

 {
  "justifier",
  "Konami Justifier",
  "Konami's light gun; SLUH-00017.  Rumored to be wrought of the coagulated rage of all who tried to shoot The Dog.  If the game you want to play supports the \"GunCon\", you should use that instead. NOTE: Currently does not work properly when on any of ports 1B-1D and 2B-2D.",
  NULL,
  sizeof(Device_Justifier_IDII) / sizeof(InputDeviceInputInfoStruct),
  Device_Justifier_IDII,
 },

 {
  "dancepad",
  "Dance Pad",
  "Dingo Dingo Rodeo!",
  NULL,
  sizeof(Device_Dancepad_IDII) / sizeof(InputDeviceInputInfoStruct),
  Device_Dancepad_IDII,
 },

};

static const InputPortInfoStruct PortInfo[] =
{
 { "port1", "Virtual Port 1", sizeof(InputDeviceInfoPSXPort) / sizeof(InputDeviceInfoStruct), InputDeviceInfoPSXPort, "gamepad" },
 { "port2", "Virtual Port 2", sizeof(InputDeviceInfoPSXPort) / sizeof(InputDeviceInfoStruct), InputDeviceInfoPSXPort, "gamepad" },
 { "port3", "Virtual Port 3", sizeof(InputDeviceInfoPSXPort) / sizeof(InputDeviceInfoStruct), InputDeviceInfoPSXPort, "gamepad" },
 { "port4", "Virtual Port 4", sizeof(InputDeviceInfoPSXPort) / sizeof(InputDeviceInfoStruct), InputDeviceInfoPSXPort, "gamepad" },
 { "port5", "Virtual Port 5", sizeof(InputDeviceInfoPSXPort) / sizeof(InputDeviceInfoStruct), InputDeviceInfoPSXPort, "gamepad" },
 { "port6", "Virtual Port 6", sizeof(InputDeviceInfoPSXPort) / sizeof(InputDeviceInfoStruct), InputDeviceInfoPSXPort, "gamepad" },
 { "port7", "Virtual Port 7", sizeof(InputDeviceInfoPSXPort) / sizeof(InputDeviceInfoStruct), InputDeviceInfoPSXPort, "gamepad" },
 { "port8", "Virtual Port 8", sizeof(InputDeviceInfoPSXPort) / sizeof(InputDeviceInfoStruct), InputDeviceInfoPSXPort, "gamepad" },
};

InputInfoStruct FIO_InputInfo =
{
 sizeof(PortInfo) / sizeof(InputPortInfoStruct),
 PortInfo
};

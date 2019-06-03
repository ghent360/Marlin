/*
  mtwled.h
  header file for Makers Tool Works RGB LED I2C control
  Led Strip Driver for MTW Printers.
  I2C output
  I2C Device ID = 21
  Data Value of 1 is used for no selection = do nothing
  report the heat cool values at the same rate its reported to the UI
*/

#if (ARDUINO >= 100)
  # include "Arduino.h"
#else
  # include "WProgram.h"
#endif

#include "Wire.h"

#define MTWLED_H

union patterncode {  // access a pattern both as 32 bits and as array of 4 bytes.
  uint32_t value;    
  byte part[4]; 
};

void MTWLEDSetup();
void MTWLEDUpdate(byte pattern, byte red=0, byte green=0, byte blue=0, unsigned long timer=0, int control=-1);
void MTWLEDUpdate(patterncode pattern, unsigned long timer=0, int control=-1);
void MTWLEDLogic();
void MTWLEDTemp();
boolean MTWLEDEndstop(boolean force);
uint32_t MTWLEDConvert(byte pattern=0, byte red=0, byte green=0, byte blue=0);

// Pattern Selection Table for defaults that must not be changed
#define mtwled_nochange 	1	// Reserved for no change to LED Strip
#define mtwled_debug            255     // debugging use only
// see Configuration.h for the user customizable event pattern codes

/*
9 and 90-99 Reserved for heating and cooling values
Fixed values for heating and cooling cycle
Heating   will use a value of 10,20,30,40,50,60,70,80,90,100 % to change the
colors on the led strip the % is based on the ((current temp /requested temp-starting temp)rounded to the nearest 10th
*/


/* User docs for Configuration.h stored here in case they get lost
A pattern code is 4 bytes of data: the pattern ID plus one byte each for red, green, blue color values.
A simple way to experiment to find color values you like is to use the M242 command, the serial console
will display the individual values when it executes a M242 command. You can experiment with patterns and
RGB color values, then use a code returned by M242 for any default above. The M242 command has these parameters:
M242 P<pattern ID> R<red> E<green> B<blue> T<timer> C<control>
   pattern ID is the number of the pattern/animation to use
   R is a value from 0-127 for how red the color will be
   E is a value from 0-127 for how green the color will be
   B is a value from 0-127 for how blue the color will be
      Specifying colors is often optional, any color not given will be either 0 (none) or a default
      depending on the pattern selected.
   T is a timer in seconds for how long the pattern will override the default patterns from marlin
      If marlin events are enabled, the LEDs will resume automatic changed when the time has elapsed
   C is a control what patterns marlin sends automatically.
      C0 will enable all marlin LED events
      C1 will disable general status events (ready, holding temp, etc.)
      C2 will display endstop status
      C254 toggles serial debug output
      C255 will disable all marlin LED events
      The control parameter can be used by itself without specifying a pattern ID.
   Examples:
      M242 C1          Disable marlin's status events (handy to prevent it overriding while testing)
      M242 P10 B40     Sets pattern 10 (solid color) blue
      M242 P10 B50 R50 Sets pattern 10 (solid color) purple
      M242 P11 R50 T10 Sets a scanning pattern (cylon/KITT) red for at least 10 seconds
      M242 P13 E40 T10 Sets a chasing pattern green for at least 10 seconds
   Current patterns are:
      10 RGB	Solid color
      11 RGB 	Cylon
      12 RGB 	UFO PULSE
      13 XXX 	Color Chase
      14 XXX 	Color Cycle
      15 RGB 	Color Chase Single Led
      16 RGB 	Slow fill then solid
      17 RGB	Repeating Blink
*/

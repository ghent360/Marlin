/*
  mtwled.cpp
  Makers Tool Works RGB LED I2C control
  Contributed by OhmEye October 2014
*/

#include "Marlin.h"
#ifdef MTWLED

#if (ARDUINO >= 100)
  # include "Arduino.h"
#else
  # include "WProgram.h"
#endif

#include "mtwled.h"
#include "temperature.h"

patterncode MTWLED_lastpattern;
unsigned long MTWLED_timer;
int MTWLED_control;
boolean MTWLED_feedback=false;
boolean MTWLED_heated=false;
int MTWLED_mode=MTWLED_printmode; // select what type of events to send while printing... 0=temperature 1=coordinates

void MTWLEDSetup()
{
  MTWLED_lastpattern.value=0;
  MTWLED_timer=0;
  MTWLED_control=-1; // representing startup
  Wire.begin();
  patterncode pc;
  pc.value=MTWLEDConvert(mtwled_startup);
  MTWLEDUpdate(pc,2);
}

uint32_t MTWLEDConvert(byte pattern, byte red, byte green, byte blue)
{
  patterncode pc;
  pc.part[0]=pattern;
  pc.part[1]=red;
  pc.part[2]=green;
  pc.part[3]=blue;
  return pc.value;
}


void MTWLEDUpdate(byte pattern, byte red, byte green, byte blue, unsigned long timer, int control)
{
  patterncode pc;
  pc.part[0]=pattern;
  pc.part[1]=red;
  pc.part[2]=green;
  pc.part[3]=blue;
  MTWLEDUpdate(pc,timer,control);  
}

void MTWLEDUpdate(patterncode pattern, unsigned long timer, int control) // send pattern frame via I2C
{
  byte sout[]={250,pattern.part[0],constrain(pattern.part[1],0,127),constrain(pattern.part[2],0,127),constrain(pattern.part[3],0,127)}; // build frame
  
  if(control==2) { MTWLEDEndstop(true); return; }         // force endstop status display on C2
  if(control==254) { MTWLED_feedback=!MTWLED_feedback; return; }
  if(control==252) { MTWLED_mode ? MTWLED_mode=0 : MTWLED_mode=1; return; }
  if(control>=0) MTWLED_control=control;                  // handle exceptions/collisions/control
  if(pattern.part[0] < 1) return;
  if(pattern.value != MTWLED_lastpattern.value)           // don't sent sequential identical patterns
  {    
    Wire.beginTransmission(21);
     Wire.write(sout,5);                                  // send the 5 byte frame
    Wire.endTransmission();
    MTWLED_lastpattern=pattern;                           // update states
    if(timer) MTWLED_timer=millis()+(timer*1000);
  if(MTWLED_feedback)
    {
    SERIAL_PROTOCOL("LED P:");                           // print feedback to serial
    SERIAL_PROTOCOL((int)pattern.part[0]);
    SERIAL_PROTOCOL(" R:");
    SERIAL_PROTOCOL((int)pattern.part[1]);
    SERIAL_PROTOCOL(" E:");
    SERIAL_PROTOCOL((int)pattern.part[2]);
    SERIAL_PROTOCOL(" B:");
    SERIAL_PROTOCOL((int)pattern.part[3]);
    SERIAL_PROTOCOL(" C:");
    SERIAL_PROTOCOLLN((int)MTWLED_control);
//    SERIAL_PROTOCOL("MTWLED ");
//    SERIAL_PROTOCOLLN((uint32_t)pattern.value);
    }
  }
}

boolean MTWLEDEndstop(boolean force)
{
  boolean endx=0, endy=0, endz=0;

    #if defined(X_MAX_PIN) && X_MAX_PIN > -1
    if(force || current_position[X_AXIS]!=X_MAX_POS) endx += (READ(X_MAX_PIN)^X_MAX_ENDSTOP_INVERTING);
    #endif
    #if defined(Y_MAX_PIN) && Y_MAX_PIN > -1
    if(force || current_position[Y_AXIS]!=Y_MAX_POS) endy += (READ(Y_MAX_PIN)^Y_MAX_ENDSTOP_INVERTING);
    #endif
    #if defined(Z_MAX_PIN) && Z_MAX_PIN > -1
    if(force || current_position[Z_AXIS]!=Z_MAX_POS) endz += (READ(Z_MAX_PIN)^Z_MAX_ENDSTOP_INVERTING);
    #endif
  
    #if defined(X_MIN_PIN) && X_MIN_PIN > -1
    if(force || current_position[X_AXIS]!=0) endx += (READ(X_MIN_PIN)^X_MIN_ENDSTOP_INVERTING);
    #endif
    #if defined(Y_MIN_PIN) && Y_MIN_PIN > -1
    if(force || current_position[Y_AXIS]!=0) endy += (READ(Y_MIN_PIN)^Y_MIN_ENDSTOP_INVERTING);
    #endif
    #if defined(Z_MIN_PIN) && Z_MIN_PIN > -1
    if(force || current_position[Z_AXIS]!=0) endz += (READ(Z_MIN_PIN)^Z_MIN_ENDSTOP_INVERTING);
    #endif
  
  if(force || endx || endy || endz) {
      MTWLEDUpdate(2,endx,endy,endz,MTWLED_endstoptimer);
      if(endx || endy || endz) return true;
  }
  return false;
}

void MTWLEDLogic() // called from main loop
{
  patterncode pattern = MTWLED_lastpattern;
  
  if(MTWLED_control==1 || MTWLED_control==255) return;
  if(MTWLEDEndstop(false)) return;
//  if(pattern.value==mtwled_nochange) return;
  if(MTWLED_timer > millis()) return;
  if(MTWLED_control==-1) { // if this is first time display endstop status before clearing to ready
     MTWLEDEndstop(true);
     MTWLED_control=0;
     return;
  }

  if((thermalManager.degTargetHotend(active_extruder) == 0)) { // assume not printing since target temp is zero
    if((thermalManager.degHotend(active_extruder) > MTWLED_cool)) // heater is off but still warm
      pattern.value=MTWLEDConvert(mtwled_heateroff);
    else {
      pattern.value=MTWLEDConvert(mtwled_ready);
      MTWLED_heated=false;
    }
    MTWLEDUpdate(pattern);
  } else {
    if(MTWLED_heated)
      switch(MTWLED_mode) {
//        case 2: // display XYZ position colors separately needs pattern 8 implemented in controller
//          MTWLEDUpdate(8,(current_position[X_AXIS]/X_MAX_POS)*50+5,(current_position[Z_AXIS]/Z_MAX_POS)*100+5,(current_position[Y_AXIS]/Y_MAX_POS)*50+5,1);
//          break;
        case 1: // XYZ position used as RBG
          MTWLEDUpdate(10,(current_position[X_AXIS]/X_MAX_POS)*50+5,(current_position[Z_AXIS]/Z_MAX_POS)*100+5,(current_position[Y_AXIS]/Y_MAX_POS)*50+5,1);
          break;
        case 0:
        default: // show solid color based on XYZ=RGB color values
          if(abs(thermalManager.degTargetHotend(active_extruder) - thermalManager.degHotend(active_extruder)) > MTWLED_swing) {              // temp is not close to target
            if(thermalManager.isHeatingHotend(active_extruder)) pattern.value=MTWLEDConvert(mtwled_templow);    // heater is on so temp must be low
            if(thermalManager.isCoolingHotend(active_extruder)) pattern.value=MTWLEDConvert(mtwled_temphigh);   // heater is off so temp must be high
          } else {                                                                 // temp is close to target
            pattern.value=MTWLEDConvert(mtwled_temphit);                           // close to target temp, so consider us 'at temp'
          }
          MTWLEDUpdate(pattern);
          break;
      }
    }
}

void MTWLEDTemp() // called from inside heater function while heater is on to do the percentile display
{
	byte percent;
        if(MTWLED_heated) return;
        if(MTWLED_control==255) return;
        if((thermalManager.degTargetHotend(active_extruder) == 0)) return;
	  percent = ((thermalManager.degHotend(active_extruder) / (thermalManager.degTargetHotend(active_extruder))) * 100);
	  if(percent >= 100) {
            percent = 100;
            MTWLED_heated=true;
            if(MTWLED_feedback) {
              SERIAL_PROTOCOLLN("LED heated. Entering printmode");
            }
          }
	  MTWLEDUpdate(9,percent,MTWLED_heatmode,0);
}

#endif //MTWLED
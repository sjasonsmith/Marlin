#ifndef CONFIGURATION_ADV_H
#define CONFIGURATION_ADV_H

//===========================================================================
//=============================Thermal Settings  ============================
//===========================================================================

// Select one of these only to define how the bed temp is read.
//
//#define BED_LIMIT_SWITCHING
#ifdef BED_LIMIT_SWITCHING
  #define BED_HYSTERESIS 2 //only disable heating if T>target+BED_HYSTERESIS and enable heating if T>target-BED_HYSTERESIS
#endif
#define BED_CHECK_INTERVAL 5000 //ms

//// Heating sanity check:
// This waits for the watchperiod in milliseconds whenever an M104 or M109 increases the target temperature
// If the temperature has not increased at the end of that period, the target temperature is set to zero. 
// It can be reset with another M104/M109
//#define WATCHPERIOD 20000 //20 seconds

// Wait for Cooldown
// This defines if the M109 call should not block if it is cooling down.
// example: From a current temp of 220, you set M109 S200. 
// if CooldownNoWait is defined M109 will not wait for the cooldown to finish
#define CooldownNoWait true

#ifdef PIDTEMP
  // this adds an experimental additional term to the heatingpower, proportional to the extrusion speed.
  // if Kc is choosen well, the additional required power due to increased melting should be compensated.
  #define PID_ADD_EXTRUSION_RATE  
  #ifdef PID_ADD_EXTRUSION_RATE
    #define  DEFAULT_Kc (1) //heatingpower=Kc*(e_speed)
  #endif
#endif


//Filament detection. Uses two rotarty encoders and a pulse checker in de stepper loop. 
#define FILAMENT_DETECTION

//automatic temperature: The hot end target temperature is calculated by all the buffered lines of gcode.
//The maximum buffered steps/sec of the extruder motor are called "se".
//You enter the autotemp mode by a M109 S<mintemp> T<maxtemp> F<factor>
// the target temperature is set to mintemp+factor*se[steps/sec] and limited by mintemp and maxtemp
// you exit the value by any M109 without F*
// Also, if the temperature is set to a value <mintemp, it is not changed by autotemp.
#define AUTOTEMP
#ifdef AUTOTEMP
  #define AUTOTEMP_OLDWEIGHT 0.98
#endif


//These defines help to calibrate the AD595 sensor in case you get wrong temperature measurements.
//The measured temperature is defined as "actualTemp = (measuredTemp * TEMP_SENSOR_AD595_GAIN) + TEMP_SENSOR_AD595_OFFSET"
#define TEMP_SENSOR_AD595_OFFSET 0.0
#define TEMP_SENSOR_AD595_GAIN   1.0

//This is for controlling a fan to cool down the stepper drivers
//it will turn on when any driver is enabled
//and turn off after the set amount of seconds from last driver being disabled again
//#define CONTROLLERFAN_PIN 23 //Pin used for the fan to cool controller, comment out to disable this function
#define CONTROLLERFAN_SEC 60 //How many seconds, after all motors were disabled, the fan should run

//===========================================================================
//=============================Mechanical Settings===========================
//===========================================================================

// This defines the number of extruders
#define EXTRUDERS 2

//#define ENDSTOPS_ONLY_FOR_HOMING // If defined the endstops will only be used for homing

//#define Z_LATE_ENABLE // Enable Z the last moment. Needed if your Z driver overheats.

//homing hits the endstop, then retracts by this distance, before it tries to slowly bump again:
#define X_HOME_RETRACT_MM 5 
#define Y_HOME_RETRACT_MM 5 
#define Z_HOME_RETRACT_MM 1
 
#define AXIS_RELATIVE_MODES {false, false, false, false}

#define MAX_STEP_FREQUENCY 40000 // Max step frequency for Ultimaker (5000 pps / half step)

//By default pololu step drivers require an active high signal. However, some high power drivers require an active low signal as step.
#define INVERT_X_STEP_PIN false
#define INVERT_Y_STEP_PIN false
#define INVERT_Z_STEP_PIN false
#define INVERT_E_STEP_PIN false

//default stepper release if idle
#define DEFAULT_STEPPER_DEACTIVE_TIME 60

#define DEFAULT_MINIMUMFEEDRATE       0.0     // minimum feedrate
#define DEFAULT_MINTRAVELFEEDRATE     0.0

// minimum time in microseconds that a movement needs to take if the buffer is emptied.
#define DEFAULT_MINSEGMENTTIME        20000

// If defined the movements slow down when the look ahead buffer is only half full
#define SLOWDOWN

// Minimum planner junction speed. Sets the default minimum speed the planner plans for at the end
// of the buffer and all stops. This should not be much greater than zero and should only be changed
// if unwanted behavior is observed on a user's machine when running at very slow speeds.
#define MINIMUM_PLANNER_SPEED 0.05// (mm/sec)

//===========================================================================
//=============================Additional Features===========================
//===========================================================================


// extruder advance constant (s2/mm3)
//
// advance (steps) = STEPS_PER_CUBIC_MM_E * EXTUDER_ADVANCE_K * cubic mm per second ^ 2
//
// hooke's law says:		force = k * distance
// bernoulli's priniciple says:	v ^ 2 / 2 + g . h + pressure / density = constant
// so: v ^ 2 is proportional to number of steps we advance the extruder
//#define ADVANCE

#ifdef ADVANCE
  #define EXTRUDER_ADVANCE_K .0

  #define D_FILAMENT 2.85
  #define STEPS_MM_E 836
  #define EXTRUTION_AREA (0.25 * D_FILAMENT * D_FILAMENT * 3.14159)
  #define STEPS_PER_CUBIC_MM_E (axis_steps_per_unit[E_AXIS]/ EXTRUTION_AREA)

#endif // ADVANCE

const int dropsegments=5; //everything with less than this number of steps will be ignored as move and joined with the next movement

//===========================================================================
//=============================Buffers           ============================
//===========================================================================

// The number of linear motions that can be in the plan at any give time.  
// THE BLOCK_BUFFER_SIZE NEEDS TO BE A POWER OF 2, i.g. 8,16,32 because shifts and ors are used to do the ringbuffering.
#define BLOCK_BUFFER_SIZE 16 // maximize block buffer


//The ASCII buffer for recieving from the serial:
#define MAX_CMD_SIZE 96
#define BUFSIZE 32 //Was 4 (ADR 2014-04-03)


//===========================================================================
//=============================  Define Defines  ============================
//===========================================================================

#if TEMP_SENSOR_0 > 0
  #define THERMISTORHEATER_0 TEMP_SENSOR_0
  #define HEATER_0_USES_THERMISTOR
#endif
#if TEMP_SENSOR_1 > 0
  #define THERMISTORHEATER_1 TEMP_SENSOR_1
  #define HEATER_1_USES_THERMISTOR
#endif
#if TEMP_SENSOR_2 > 0
  #define THERMISTORHEATER_2 TEMP_SENSOR_2
  #define HEATER_2_USES_THERMISTOR
#endif
#if TEMP_SENSOR_BED > 0
  #define THERMISTORBED TEMP_SENSOR_BED
  #define BED_USES_THERMISTOR
#endif
#if TEMP_SENSOR_0 == -1
  #define HEATER_0_USES_AD595
#endif
#if TEMP_SENSOR_1 == -1
  #define HEATER_1_USES_AD595
#endif
#if TEMP_SENSOR_2 == -1
  #define HEATER_2_USES_AD595
#endif
#if TEMP_SENSOR_BED == -1
  #define BED_USES_AD595
#endif
#if TEMP_SENSOR_0 == -2
  #define HEATER_0_USES_MAX6675
#endif
#if TEMP_SENSOR_0 == 0
  #undef HEATER_0_MINTEMP
  #undef HEATER_0_MAXTEMP
#endif
#if TEMP_SENSOR_1 == 0
  #undef HEATER_1_MINTEMP
  #undef HEATER_1_MAXTEMP
#endif
#if TEMP_SENSOR_2 == 0
  #undef HEATER_2_MINTEMP
  #undef HEATER_2_MAXTEMP
#endif
#if TEMP_SENSOR_BED == 0
  #undef BED_MINTEMP
  #undef BED_MAXTEMP
#endif


#endif //__CONFIGURATION_ADV_H

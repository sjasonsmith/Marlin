#ifndef CONFIGURATION_H
#define CONFIGURATION_H

// This configurtion file contains the basic settings.
// Advanced settings can be found in Configuration_adv.h
// BASIC SETTINGS: select your board type, temperature sensor type, axis scaling, and endstop configuration

//User specified version info of THIS file to display in [Pronterface, etc] terminal window during startup.
//Implementation of an idea by Prof Braino to inform user that any changes made
//to THIS file by the user have been successfully uploaded into firmware.
#define STRING_VERSION_CONFIG_H __DATE__ " " __TIME__ //Personal revision number for changes to THIS file.
#define STRING_CONFIG_H_AUTHOR "Leapfrog 3D Printers" //Who made the changes.
#define LEAPFROG_FIRMWARE_VERSION "2.2"
#define LEAPFROG_MODEL "Xeed"

// This determines the communication speed of the printer
#define BAUDRATE 115200

// In steps that needs to be taken from the second extruder to the first extruder
// See M50 Gcode commando for more info 
#define EXTR2_X_OFFSET 28.00
#define EXTR2_Y_OFFSET 0.00

// The following define selects which electronics board you have. Please choose the one that matches your setup

#ifndef MOTHERBOARD
#define MOTHERBOARD 1
#endif

//===========================================================================
//=============================Thermal Settings  ============================
//===========================================================================
//
// 1 is 100k thermistor - best choice for EPCOS 100k (4.7k pullup)
#define TEMP_SENSOR_0 1
#define TEMP_SENSOR_1 1
#define TEMP_SENSOR_2 0
#define TEMP_SENSOR_BED 1

// Actual temperature must be close to target for this long before M109 returns success
#define TEMP_RESIDENCY_TIME 10	// (seconds)
#define TEMP_HYSTERESIS 3       // (degC) range of +/- temperatures considered "close" to the target one
#define TEMP_WINDOW     1       // (degC) Window around target to start the recidency timer x degC early.

// The minimal temperature defines the temperature below which the heater will not be enabled It is used
// to check that the wiring to the thermistor is not broken.
// Otherwise this would lead to the heater being powered on all the time.
#define HEATER_0_MINTEMP 5
#define HEATER_1_MINTEMP 5
#define HEATER_2_MINTEMP 5
#define BED_MINTEMP 5

// When temperature exceeds max temp, your heater will be switched off.
// This feature exists to protect your hotend from overheating accidentally, but *NOT* from thermistor short/failure!
// You should use MINTEMP for thermistor short/failure protection.
#define HEATER_0_MAXTEMP 300 
#define HEATER_1_MAXTEMP 300
#define HEATER_2_MAXTEMP 300
#define BED_MAXTEMP 100


// PID settings:
// Comment the following line to disable PID and enable bang-bang.
#define PIDTEMP
#define PID_MAX 255 // limits current to nozzle; 255=full current
#ifdef PIDTEMP
  //#define PID_DEBUG // Sends debug data to the serial port.
  //#define PID_OPENLOOP 1 // Puts PID in open loop. M104 sets the output power in %
  #define PID_INTEGRAL_DRIVE_MAX 255  //limit for the integral term
  #define K1 0.95 //smoothing factor withing the PID
  #define PID_dT ((16.0 * 8.0)/(F_CPU / 64.0 / 256.0)) //sampling period of the

// If you are using a preconfigured hotend then you can use one of the value sets by uncommenting it
// Xeed copied from HS new
    #define  DEFAULT_Kp 12.00
    #define  DEFAULT_Ki 0.20
    #define  DEFAULT_Kd 110.0


#endif // PIDTEMP

//this prevents dangerous Extruder moves, i.e. if the temperature is under the limit
//can be software-disabled for whatever purposes by
#define PREVENT_DANGEROUS_EXTRUDE
//if PREVENT_DANGEROUS_EXTRUDE is on, you can still disable (uncomment) very long bits of extrusion separately.
#define PREVENT_LENGTHY_EXTRUDE

#define EXTRUDE_MINTEMP 150
#define EXTRUDE_MAXLENGTH 2000 //prevent extrusion of very large distances.

//===========================================================================
//=============================Mechanical Settings===========================
//===========================================================================

// corse Endstop Settings
#define ENDSTOPPULLUPS // Comment this out (using // at the start of the line) to disable the endstop pullup resistors

#ifndef ENDSTOPPULLUPS
  // fine Enstop settings: Individual Pullups. will be ignord if ENDSTOPPULLUPS is defined
  #define ENDSTOPPULLUP_XMAX
  #define ENDSTOPPULLUP_YMAX
  #define ENDSTOPPULLUP_ZMAX
  #define ENDSTOPPULLUP_XMIN
  #define ENDSTOPPULLUP_YMIN
  #define ENDSTOPPULLUP_ZMIN
#endif

#ifdef ENDSTOPPULLUPS
  #define ENDSTOPPULLUP_XMAX
  #define ENDSTOPPULLUP_YMAX
  #define ENDSTOPPULLUP_ZMAX
  #define ENDSTOPPULLUP_XMIN
  #define ENDSTOPPULLUP_YMIN
  #define ENDSTOPPULLUP_ZMIN
#endif


#define ENDSTOPPULLUP_ZMIN
// The pullups are needed if you directly connect a mechanical endswitch between the signal and ground pins.
const bool X_ENDSTOPS_INVERTING = false; // set to true to invert the logic of the endstops.
const bool Y_ENDSTOPS_INVERTING = false; // set to true to invert the logic of the endstops.
const bool Z_ENDSTOPS_INVERTING = false; // set to true to invert the logic of the endstops.
//#define DISABLE_MAX_ENDSTOPS

// For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1
#define X_ENABLE_ON 0
#define Y_ENABLE_ON 0
#define Z_ENABLE_ON 0
#define E_ENABLE_ON 0 // For all extruders

// Disables axis when it's not being used.
#define DISABLE_X false
#define DISABLE_Y false
#define DISABLE_Z false
#define DISABLE_E false // For all extruders

#define INVERT_X_DIR false    // for Mendel set to false, for Orca set to true
#define INVERT_Y_DIR true    // for Mendel set to true, for Orca set to false
#define INVERT_Z_DIR true     // for Mendel set to false, for Orca set to true
#define INVERT_E0_DIR false  // right non inverted. 
#define INVERT_E1_DIR true    // left inverted. 
#define INVERT_E2_DIR false   // for direct drive extruder v9 set to true, for geared extruder set to false

// ENDSTOP SETTINGS:
// Sets direction of endstops when homing; 1=MAX, -1=MIN
#define X_HOME_DIR -1
#define Y_HOME_DIR 1
#define Z_HOME_DIR -1

#define min_software_endstops true  //If true, axis won't move to coordinates less than HOME_POS.
#define max_software_endstops true  //If true, axis won't move to coordinates greater than the defined lengths below.

// The position of the homing switches. Use MAX_LENGTH * -0.5 if the center should be 0, 0, 0
#define X_HOME_POS 0
#define Y_HOME_POS 220
#define Z_HOME_POS 0

// Travel limits after homing
#define X_MIN_POS 0
#define Y_MIN_POS 0
#define Z_MIN_POS 0
#define X_MAX_POS 280
#define Y_MAX_POS 220
#define Z_MAX_POS 230

#define MAX_ZOFFSET 2 // Maximum amount the Z-axis can go in - direction in mm. To allow moving in - but not too far. 

#define X_MAX_LENGTH (X_MAX_POS - X_MIN_POS)
#define Y_MAX_LENGTH (Y_MAX_POS - Y_MIN_POS)
#define Z_MAX_LENGTH (Z_MAX_POS - Z_MIN_POS)

//// MOVEMENT SETTINGS
#define NUM_AXIS 5 // The axis order in all axis related arrays is X, Y, Z, E
#define HOMING_FEEDRATE {50*60, 50*60, 4*60, 0, 0}  // set the homing speeds (mm/min)

// default settings

#define DEFAULT_AXIS_STEPS_PER_UNIT   {55.55555555, 55.5555555555, 6400/5, 198.60 , 198.60}  // default steps, X steps terug naar 1/2 (driver op 2000 ipv 4000)
#define DEFAULT_MAX_FEEDRATE          {200, 200, 10, 45, 45}    // (mm/sec)
#define DEFAULT_MAX_ACCELERATION      {500,500,300,10000, 10000}    // X, Y, Z, E maximum start speed for accelerated moves. E default values are good for skeinforge 40+, for older versions raise them a lot.

#define DEFAULT_ACCELERATION          500    // X, Y, Z and E max acceleration in mm/s^2 for printing moves
#define DEFAULT_RETRACT_ACCELERATION  7000   // X, Y, Z and E max acceleration in mm/s^2 for r retracts

// Jerk
#define DEFAULT_XYJERK                5.0    // (mm/sec)   std: 15.0
#define DEFAULT_ZJERK                 1     // (mm/sec)
#define DEFAULT_EJERK                 60.0    // (mm/sec)

// Offset of the extruders (uncomment if using more than one and relying on firmware to position when changing).
// The offset has to be X=0, Y=0 for the extruder 0 hotend (default extruder).
// For the other hotends it is their distance from the extruder 0 hotend.
#define EXTRUDER_OFFSET_X {0.0, -EXTR2_X_OFFSET} // (in mm) for each extruder, offset of the hotend on the X axis
#define EXTRUDER_OFFSET_Y {0.0, -EXTR2_Y_OFFSET}  // (in mm) for each extruder, offset of the hotend on the Y axis

//#define Z_STEPPER_SINGLE  0
//#define Z_TRIPLE_STEPPER_DRIVERS
//#define Z3_STEPPER_SINGLE
//===================================
//========================================
//=============================Additional Features===========================
//===========================================================================

// EEPROM
// the microcontroller can store settings in the EEPROM, e.g. max velocity...
// M500 - stores paramters in EEPROM
// M501 - reads parameters from EEPROM (if you need reset them after you changed them temporarily).
// M502 - reverts to the default "factory settings".  You still need to store them in EEPROM afterwards if you want to.
//define this to enable eeprom support
#define EEPROM_SETTINGS
//to disable EEPROM Serial responses and decrease program space by ~1700 byte: comment this out:
// please keep turned on if you can.
#define EEPROM_CHITCHAT


#define ENABLE_ZPROBE
#ifdef ENABLE_ZPROBE
#include "math3d.h"

/* 3-Punts coordinaten
static float ZP_COORDS[][3] =
{
  {5.0, 0.0, 0.0},           //z-probe point 1
  {255, 0.0, 0.0},            //z-probe point 2
  {127.5,  215.0 , 0.0}      //z-probe point 3
};*/

// "4punts"
static float ZP_COORDS[][5] =
{
   {0.0, 0.0, 0.0}            //z-probe point 1 Links Voor
  ,{280, 0.0, 0.0}            //z-probe point 2 Rechts Voor
  ,{280,  210, 0.0}            //z-probe point 3 Rechts Achter
  ,{0.0,  210, 0.0}            //z-probe point 4 Links Achter
  ,{ 165,  120,0.0}            //z-probe point 5 (Mid)
};

static Vector3d xbasis(1,0,0);
static Vector3d ybasis(0,1,0);
static Vector3d zbasis(0,0,1);

#endif
// Increase the FAN pwm frequency. Removes the PWM noise but increases heating in the FET/Arduino
#define FAST_PWM_FAN

#include "Configuration_adv.h"
#include "thermistortables.h"

#endif //__CONFIGURATION_H

#ifndef LANGUAGE_H
#define LANGUAGE_H


// Languages
// 1 English

#define LANGUAGE_CHOICE 1  // Pick your language from the list above

#define PROTOCOL_VERSION "1.0"

#define MACHINE_NAME "Xeed"
#define FIRMWARE_URL "http://www.lpfrg.com"

#define STRINGIFY_(n) #n
#define STRINGIFY(n) STRINGIFY_(n)

#if LANGUAGE_CHOICE == 1


// Serial Console Messages

	#define MSG_Enqueing "enqueing \""
	#define MSG_POWERUP "PowerUp"
	#define MSG_EXTERNAL_RESET " External Reset"
	#define MSG_BROWNOUT_RESET " Brown out Reset"
	#define MSG_WATCHDOG_RESET " Watchdog Reset"
	#define MSG_SOFTWARE_RESET " Software Reset"
	#define MSG_MARLIN "Marlin "
	#define MSG_AUTHOR " | Author: "
	#define MSG_CONFIGURATION_VER " Last Updated: "
	#define MSG_FREE_MEMORY " Free Memory: "
	#define MSG_PLANNER_BUFFER_BYTES "  PlannerBufferBytes: "
	#define MSG_OK "ok"
	#define MSG_FILE_SAVED "Done saving file."
	#define MSG_ERR_LINE_NO "Line Number is not Last Line Number+1, Last Line:"
	#define MSG_ERR_CHECKSUM_MISMATCH "checksum mismatch, Last Line:"
	#define MSG_ERR_NO_CHECKSUM "No Checksum with line number, Last Line:"
	#define MSG_ERR_NO_LINENUMBER_WITH_CHECKSUM "No Line Number with checksum, Last Line:"
	#define MSG_M104_INVALID_EXTRUDER "M104 Invalid extruder "
	#define MSG_M105_INVALID_EXTRUDER "M105 Invalid extruder "
	#define MSG_ERR_NO_THERMISTORS "No thermistors - no temp"
	#define MSG_M109_INVALID_EXTRUDER "M109 Invalid extruder "
	#define MSG_M115_REPORT "Leapfrog Firmware: " LEAPFROG_FIRMWARE_VERSION " Model: " LEAPFROG_MODEL " PROTOCOL_VERSION: " PROTOCOL_VERSION " FIRMWARE_NAME:Marlin V1;"
	#define MSG_COUNT_X " Count X:"
	#define MSG_ERR_KILLED "Printer halted. kill() called !!"
	#define MSG_ERR_STOPPED "Printer stopped due to errors. Fix the error and use M999 to restart!. (Temperature is reset. Set it before restarting)"
	#define MSG_RESEND "Resend: "
	#define MSG_UNKNOWN_COMMAND "Unknown command:\""
	#define MSG_ACTIVE_EXTRUDER "Active Extruder: "
	#define MSG_INVALID_EXTRUDER "Invalid extruder"
	#define MSG_X_MIN "x_min: "
	#define MSG_X_MAX "x_max: "
	#define MSG_Y_MIN "y_min: "
	#define MSG_Y_MAX "y_max: "
	#define MSG_Z_MIN "z_min: "
	#define MSG_Z_MAX "z_max: "
	#define MSG_M119_REPORT "Reporting endstop status"
	#define MSG_ENDSTOP_HIT "TRIGGERED"
	#define MSG_ENDSTOP_OPEN "open"

	#define MSG_STEPPER_TO_HIGH "Steprate to high : "
	#define MSG_ENDSTOPS_HIT "endstops hit: "
	#define MSG_ERR_COLD_EXTRUDE_STOP " cold extrusion prevented"
	#define MSG_ERR_LONG_EXTRUDE_STOP " too long extrusion prevented"

#endif
#endif // ifndef LANGUAGE_H

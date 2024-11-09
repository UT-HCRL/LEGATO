# Andrew Morgan
# Dynamixel register table

Universal_Control_Constants = {
 "TORQUE_ENABLE"							: 1,                   # Value for enabling the torque
 "TORQUE_DISABLE"							: 0,                   # Value for disabling the torque

 "CURRENT_CONTROL_MODE"				     	: 0,
 "VELOCITY_CONTROL_MODE"					: 1,
 "POSITION_CONTROL_MODE"					: 3,
 "EXTENDED_POSITION_CONTROL_MODE"			: 4,
 "CURRENT_BASED_POSITION_CONTROL_MODE"		: 5,
 "PWM_CONTROL_MODE"						    : 16
}

XM430_Settings = {
 "max_encoder_position_mode"                : 4095, #https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#minmax-position-limit48-52
 "current_limit_range"                      : 1193, #https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#current-limit38
 "velocity_limit_range"                     : 1023, #https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#velocity-limit44
}

XM520_Settings = {
 "max_encoder_position_mode"                : 4095, #https://emanual.robotis.com/docs/en/dxl/x/xm540-w270/#minmax-position-limit48-52
 "current_limit_range"                      : 2047, #https://emanual.robotis.com/docs/en/dxl/x/xm540-w270/#current-limit38
 "velocity_limit_range"                     : 1023, #https://emanual.robotis.com/docs/en/dxl/x/xm540-w270/#velocity-limit44
}

XC330_Settings = {
 "max_encoder_position_mode"                : 4095, #https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/#minmax-position-limit48-52
 "current_limit_range"                      : 910,  #https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/#current-limit38
 "velocity_limit_range"                     : 2047, #https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/#velocity-limit44
}


X_Series_Address = {
 "ADDR_MODEL_NUMBER" 			    : 0,		# 2 bytes (R)
 "ADDR_MODEL_INFORMATION" 		    : 2,		# 4 bytes (R)
 "ADDR_VERSION_OF_FIRMWARE"   	    : 6,		# 1 byte (R)
 "ADDR_ID" 					        : 7,		# 1 bytes (RW)
 "ADDR_BAUD_RATE"			        : 8,  		# 1 byte (RW)
 "ADDR_RETURN_DELAY_TIME"		    : 9, 		# 1 byte (RW)
 "ADDR_DRIVE_MODE"				    : 10, 		# 1 byte (RW)	(direction of rotation, takes 0 or 1)
 "ADDR_OPERATING_MODE"		        : 11,		# 1 byte (RW)
 "ADDR_SECONDARY_ID"			    : 12, 		# 1 byte (RW) (for grouping dynamixels)
 "ADDR_PROTOCOL_VERSION"		    : 13, 		# 1 byte (RW)
 "ADDR_HOMING_OFFSET"			    : 20, 		# 4 bytes (RW)
 "ADDR_MOVING_THRESHOLD"		    : 24, 		# 4 bytes (RW)
 "ADDR_TEMPERATURE_LIMIT"		    : 31, 		# 1 byte (RW)
 "ADDR_MAX_VOLTAGE_LIMIT"		    : 32, 		# 2 bytes (RW)
 "ADDR_MIN_VOLTAGE_LIMIT"		    : 34, 		# 2 bytes (RW)
 "ADDR_PWM_LIMIT"				    : 36, 		# 2 bytes (RW)
 "ADDR_CURRENT_LIMIT"			    : 38, 		# 2 bytes (RW)
 "ADDR_ACCELERATION_LIMIT"		    : 40, 		# 4 bytes (RW)
 "ADDR_VELOCITY_LIMIT"			    : 44, 		# 4 bytes (RW)
 "ADDR_MAX_POSITION_LIMIT"		    : 48, 		# 4 bytes (RW)
 "ADDR_MIX_POSITION_LIMIT"		    : 52, 		# 4 bytes (RW)
 "ADDR_SHUTDOWN"				    : 63, 		# 1 byte (RW)
 #-------^^^ Anything above this line requires torque disable to change -----------------#
 #-------⌄⌄⌄ Anything below this line does not require torque disable to change ---------#
 "ADDR_TORQUE_ENABLE"			    : 64, 		# 1 byte (RW)
 "ADDR_LED"					        : 65, 		# 1 byte (RW)
 "ADDR_STATUS_RETURN_LEVEL"	        : 68,		# 1 byte (RW)
 "ADDR_REGISTERED_INSTRUCTION"	    : 69, 		# 1 byte (R)
 "ADDR_HARDWARE_ERROR_STATUS"	    : 70, 		# 1 byte (R)
 "ADDR_VELOCITY_I_GAIN"		        : 76, 		# 2 bytes (RW)
 "ADDR_VELOCITY_P_GAIN"		        : 78, 		# 2 bytes (RW)
 "ADDR_POSITION_D_GAIN"		        : 80, 		# 2 bytes (RW)
 "ADDR_POSITION_I_GAIN"		        : 82, 		# 2 bytes (RW)
 "ADDR_POSITION_P_GAIN"		        : 84, 		# 2 bytes (RW)
 "ADDR_FEEDFORWARD_2ND_GAIN"	    : 88,		# 2 bytes (RW)
 "ADDR_FEEDFORWARD_1ST_GAIN"	    : 90, 		# 2 bytes (RW)
 "ADDR_BUS_WATCHDOG"			    : 98, 		# 1 byte (RW)
 "ADDR_GOAL_PWM"				    : 100,		# 2 bytes (RW)
 "ADDR_GOAL_CURRENT"			    : 102,		# 2 bytes (RW)
 "ADDR_GOAL_VELOCITY"			    : 104,		# 4 bytes (RW)
 "ADDR_PROFILE_ACCELERATION"	    : 108,		# 4 bytes (RW)
 "ADDR_PROFILE_VELOCITY"		    : 112,		# 4 bytes (RW)
 "ADDR_GOAL_POSITION"			    : 116,		# 4 bytes (RW)
 "ADDR_REALTIME_TICK"			    : 120,		# 2 bytes (R)
 "ADDR_MOVING"				        : 122,		# 1 byte (R)
 "ADDR_MOVING_STATUS"			    : 123,		# 1 byte (R)
 "ADDR_PRESENT_PWM"			        : 124,		# 2 bytes (RW)
 "ADDR_PRESENT_CURRENT"		        : 126,		# 2 bytes (R)
 "ADDR_PRESENT_VELOCITY"		    : 128,		# 4 bytes (R)
 "ADDR_PRESENT_POSITION"		    : 132,		# 4 bytes (R)
 "ADDR_VELOCITY_TRAJECTORY"	        : 136,		# 4 bytes (R)
 "ADDR_POSITION_TRAJECTORY"	        : 140,		# 4 bytes (R)
 "ADDR_PRESENT_INPUT_VOLTAGE"	    : 144,		# 2 bytes (R)
 "ADDR_PRESENT_TEMPERATURE"	        : 146,		# 1 byte (R)

} #End X-Series Dictionary

X_Series_Address_Length = {
 "LEN_MODEL_NUMBER" 			    : 2,		# 2 bytes (R)
 "LEN_MODEL_INFORMATION" 		    : 4,		# 4 bytes (R)
 "LEN_VERSION_OF_FIRMWARE"   	    : 1,		# 1 byte (R)
 "LEN_ID" 					        : 1,		# 1 bytes (RW)
 "LEN_BAUD_RATE"			        : 1,  		# 1 byte (RW)
 "LEN_RETURN_DELAY_TIME"		    : 1, 		# 1 byte (RW)
 "LEN_DRIVE_MODE"				    : 1, 		# 1 byte (RW)	(direction of rotation, takes 0 or 1)
 "LEN_OPERATING_MODE"		        : 1,		# 1 byte (RW)
 "LEN_SECONDARY_ID"			        : 1, 		# 1 byte (RW) (for grouping dynamixels)
 "LEN_PROTOCOL_VERSION"		        : 1, 		# 1 byte (RW)
 "LEN_HOMING_OFFSET"			    : 4, 		# 4 bytes (RW)
 "LEN_MOVING_THRESHOLD"		        : 4, 		# 4 bytes (RW)
 "LEN_TEMPERATURE_LIMIT"		    : 1, 		# 1 byte (RW)
 "LEN_MAX_VOLTAGE_LIMIT"		    : 2, 		# 2 bytes (RW)
 "LEN_MIN_VOLTAGE_LIMIT"		    : 2, 		# 2 bytes (RW)
 "LEN_PWM_LIMIT"				    : 2, 		# 2 bytes (RW)
 "LEN_CURRENT_LIMIT"			    : 2, 		# 2 bytes (RW)
 "LEN_ACCELERATION_LIMIT"		    : 4, 		# 4 bytes (RW)
 "LEN_VELOCITY_LIMIT"			    : 4, 		# 4 bytes (RW)
 "LEN_MAX_POSITION_LIMIT"		    : 4, 		# 4 bytes (RW)
 "LEN_MIX_POSITION_LIMIT"		    : 4, 		# 4 bytes (RW)
 "LEN_SHUTDOWN"				        : 1, 		# 1 byte (RW)
 "LEN_TORQUE_ENABLE"			    : 1, 		# 1 byte (RW)
 "LEN_LED"					        : 1, 		# 1 byte (RW)
 "LEN_STATUS_RETURN_LEVEL"	        : 1,		# 1 byte (RW)
 "LEN_REGISTERED_INSTRUCTION"	    : 1, 		# 1 byte (R)
 "LEN_HARDWARE_ERROR_STATUS"	    : 1, 		# 1 byte (R)
 "LEN_VELOCITY_I_GAIN"		        : 2, 		# 2 bytes (RW)
 "LEN_VELOCITY_P_GAIN"		        : 2, 		# 2 bytes (RW)
 "LEN_POSITION_D_GAIN"		        : 2, 		# 2 bytes (RW)
 "LEN_POSITION_I_GAIN"		        : 2, 		# 2 bytes (RW)
 "LEN_POSITION_P_GAIN"		        : 2, 		# 2 bytes (RW)
 "LEN_FEEDFORWARD_2ND_GAIN"	        : 2,		# 2 bytes (RW)
 "LEN_FEEDFORWARD_1ST_GAIN"	        : 2, 		# 2 bytes (RW)
 "LEN_BUS_WATCHDOG"	                : 1, 		# 1 byte (RW)
 "LEN_GOAL_PWM"				        : 2,		# 2 bytes (RW)
 "LEN_GOAL_CURRENT"			        : 2,		# 2 bytes (RW)
 "LEN_GOAL_VELOCITY"			    : 4,		# 4 bytes (RW)
 "LEN_PROFILE_ACCELERATION"	        : 4,		# 4 bytes (RW)
 "LEN_PROFILE_VELOCITY"		        : 4,		# 4 bytes (RW)
 "LEN_GOAL_POSITION"			    : 4,		# 4 bytes (RW)
 "LEN_REALTIME_TICK"			    : 2,		# 2 bytes (R)
 "LEN_MOVING"				        : 1,		# 1 byte (R)
 "LEN_MOVING_STATUS"			    : 1,		# 1 byte (R)
 "LEN_PRESENT_PWM"			        : 2,		# 2 bytes (RW)
 "LEN_PRESENT_CURRENT"		        : 2,		# 2 bytes (R)
 "LEN_PRESENT_VELOCITY"		        : 4,		# 4 bytes (R)
 "LEN_PRESENT_POSITION"		        : 4,		# 4 bytes (R)
 "LEN_VELOCITY_TRAJECTORY"	        : 4,		# 4 bytes (R)
 "LEN_POSITION_TRAJECTORY"	        : 4,		# 4 bytes (R)
 "LEN_PRESENT_INPUT_VOLTAGE"	    : 2,		# 2 bytes (R)
 "LEN_PRESENT_TEMPERATURE"	        : 1,		# 1 byte (R)
} #End X-Series Length

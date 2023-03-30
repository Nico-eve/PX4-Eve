uint64 timestamp		# time since system start (microseconds)

uint8 MISSION_NONE=0	#
uint8 MISSION_LANDINGLOOP=1	#
uint8 MISSION_MOVETONEST=2	#
uint8 mission_type
float64 lat			# latitude, in deg
float64 lon			# longitude, in deg
float32 alt			# altitude AMSL, in m
float32 yaw			# yaw (only for multirotors), in rad [-PI..PI), NaN = hold current yaw
uint32 state		# state of mission
float32 cruise_alt  # cruising altitude

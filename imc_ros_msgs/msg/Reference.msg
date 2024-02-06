
uint8 flags
# 0x01 	LOCATION    Use Location Reference
# 0x02 	SPEED       Use Speed Reference
# 0x04 	Z           Use Z Reference
# 0x08 	RADIUS      Use Radius Reference
# 0x10 	START_POINT Use this Reference as Start Position for PathControler
# 0x20 	DIRECT      Use Current Position as Start Position for PathControler
# 0x80 	MANDONE     Flag Maneuver Completion

DesiredSpeed speed

DesiredZ z
	
float64 lat
# Latitude Reference

float64 lon
# Longitude Reference


float32 radius
# Loitering radius (<=0 for no loitering)
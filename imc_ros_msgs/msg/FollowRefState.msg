
uint16 control_src
# The IMC identifier of the source system that is allowed to control the vehicle. 
# If the value ‘‘0xFFFF’’ is used, any system is allowed to command references.

uint8 control_ent
# The entity identifier of the entity that is allowed to control the vehicle. 
# If the value ‘‘0xFF’’ is used, any entity is allowed to command references.

Reference reference
# Reference to follow
	
uint8 state
# 1 	WAIT     Waiting for first reference
# 2 	GOTO     Going towards received reference
# 3 	LOITER   Loitering after arriving at the reference
# 4 	HOVER    Hovering after arriving at the reference 	
# 5 	ELEVATOR Moving in z after arriving at the target cylinder
# 6 	TIMEOUT  Controlling system timed out 

uint8 proximity
# 0x01 	FAR            Far from the destination
# 0x02 	XY_NEAR        Near in the horizontal plane
# 0x04 	Z_NEAR         Near in the vertical plane
# 0x08 	XY_UNREACHABLE Unreachable in the horizontal plane
# 0x10 	Z_UNREACHABLE  Unreachable in the vertical plane 	
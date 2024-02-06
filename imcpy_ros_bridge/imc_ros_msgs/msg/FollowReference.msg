
uint16 control_src
# The IMC identifier of the source system that is allowed to control the vehicle. 
# If the value ‘‘0xFFFF’’ is used, any system is allowed to command references.

uint8 control_ent
# The entity identifier of the entity that is allowed to control the vehicle. 
# If the value ‘‘0xFF’’ is used, any entity is allowed to command references.

float32 timeout 
# The ammount of time, in seconds, after which the maneuver will be terminated if no reference has been received. 
# In other words, the controlling entity should send reference updates in shorter periods than ‘timeout’.

float32 loiter_radius
# Whenever an intended reference is achieved, this maneuver will maintain the vehicle in vaticiny of that location. 
# The loiter radius is used to define the radius of this (xy) area.

float32 altitude_interval
# Similarly to Loiter Radius, this field is used to define the “z” distance considered to be inside the vacitiny of the target location. 
# An AUV may, for instance, be floating until it more than z units above the current reference, 
# in which case it actively changes its position in order to achieve the desired depth / altitude.
	
	
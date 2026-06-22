# Drives the firmware's servogauche/servodroit sit<->stand mechanism.
uint8 SIT = 0
uint8 STAND = 1

uint8 preset            # SIT or STAND — preferred for the recruiter demo / real firmware preset positions
float64 left_angle_rad  # direct angle override, used when preset is not applicable (e.g. flex_control's leg_position_controller)
float64 right_angle_rad

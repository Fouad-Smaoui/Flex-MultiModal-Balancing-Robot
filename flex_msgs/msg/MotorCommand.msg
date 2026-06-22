# PWM duty fraction, matching firmware/src/main.cpp's outputPID clamp range [-0.8, 0.8].
# Feeds the existing PWM output stage (MC33926 driver) directly, unchanged.
float64 left_duty
float64 right_duty

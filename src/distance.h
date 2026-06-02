#include <var.h>
#include <Arduino.h>

#define WHEEL_DIAMETER_M  0.15f
#define TICKS_PER_ENC_REV 4000.0f
#define MECH_RATIO        1.6f

const float DISTANCE_PER_TICK =
(
    PI * WHEEL_DIAMETER_M
) /
(
    TICKS_PER_ENC_REV * MECH_RATIO
);

float getDistance_L() 
{
    return encoderValue_L * DISTANCE_PER_TICK;
}

float getDistance_R()
{
    return encoderValue_R * DISTANCE_PER_TICK;
}

float getDistance()
{
    return (getDistance_L() + getDistance_R()) / 2.0f;
}

void resetDistance()
{
    encoderValue_L = 0;
    encoderValue_R = 0;
}
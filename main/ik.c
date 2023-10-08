#include <stdio.h>
#include <math.h>
#include "esp_dsp.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Values in milimeters
// https://github.com/michaelkubina/SpotMicroESP32/tree/b192facc57606a074d5a535afebaf3f2a5523ee7/kinematics
const float l1 = 10.0;
const float l2 = 60.5;
const float l3 = 111.1;
const float l4 = 118.5;
const float L = 207.5; // Spot body "length", distance between front and back shoulder
const float W = 78.0;  // Spot body "width", distance between left and right shoulder

/**
 * Calculates the inverse kinematic for one leg and for only one dimension (height)
 * therefore takes only height as input.
 * https://www.youtube.com/watch?v=wtf0RDwPl0c
 */
void leg_height_ik(float height, float leg_angles[3])
{
    // Computes angles using the law of cosines
    float upper_leg_angle = acosf((powf(l3, 2) + powf(height, 2) - powf(l4, 2)) / (2 * l3 * height)) * (180.0 / M_PI);
    float wrist_angle = acosf((powf(l3, 2) + powf(l4, 2) - powf(height, 2)) / (2 * l3 * l4)) * (180.0 / M_PI);

    // Hardcoded shoulder angle since we are doing only 1D
    // and this joint does not rotate on the same plan
    leg_angles[0] = 90.0;

    // We need to add 70 because in our calculations, upper leg angle
    // is 0 when in reality it is 70 degrees
    leg_angles[1] = upper_leg_angle + 70.0;
    // When a leg is completly closed, the wrist servo is calibrated to be 0 degree but
    // the actual leg links have an angle or 15.6 degrees, see it on this schematics:
    // https://github.com/michaelkubina/SpotMicroESP32/blob/b192facc57606a074d5a535afebaf3f2a5523ee7/kinematics/L3.png
    leg_angles[2] = wrist_angle + 15.6;
}
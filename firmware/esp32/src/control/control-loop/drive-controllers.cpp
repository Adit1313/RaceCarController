/*******************************************************************************
 * @file    drive-controller.c
 * @brief   Standard driving controllers that apply the torque from CRS directly
 ******************************************************************************/

#include "configuration.h"
#include "control.hpp"

void control_drive_step(float r_drive, float *u_throttle) {
    *u_throttle = CONFIG_DEFAULT_CONTROL_THROTTLE_GAIN * r_drive;
}

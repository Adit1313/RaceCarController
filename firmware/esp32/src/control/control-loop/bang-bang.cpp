/*******************************************************************************
 * @file    bang-bang.c
 * @brief   Bang-bang controller for the steering angle.
 ******************************************************************************/

#include "bang-bang.hpp"

#include <math.h>

#include "control.hpp"
#include "math-util.h"
#include "nvs-manager.h"

void control_steer_step_bangbang(float r_steer, float y_steer, float *u_steer) {

    if (fabs(r_steer - y_steer) < DEG2RAD(global_config.bb_deadband)) {
        *u_steer = 0.0f;
        return;
    }

    if (r_steer - y_steer > 0.0f) {
        *u_steer = +(global_config.bb_torque / 100.0f);
    } else if (r_steer - y_steer < 0.0f) {
        *u_steer = -(global_config.bb_torque / 100.0f);
    }
}

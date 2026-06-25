/*******************************************************************************
 * @file    bang-bang.h
 * @brief   Bang-bang controller for the steering angle and a feed-through for
 *          the torque reference.
 ******************************************************************************/

#pragma once

void control_steer_step_bangbang(float r_steer, float y_steer, float *u_steer);

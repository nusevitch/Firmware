/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*
 * @file aa241x_fw_control_params.h
 *
 * Definition of custom parameters for fixedwing controllers
 * being written for AA241x.
 *
 *  @author Adrien Perkins		<adrienp@stanford.edu>
 */
#pragma once

#ifndef AA241X_FW_CONTROL_PARAMS_H_
#define AA241X_FW_CONTROL_PARAMS_H_


#include <systemlib/param/param.h>

#ifdef __cplusplus
extern "C" {
#endif


/**
 * Struct of all of the custom parameters.
 *
 * Please make sure to add a variable for each of your newly defined
 * parameters here.
 */
struct aah_params {

    float example_high_param;
    float S_proportional_roll_gain;
    //Custom Gain Parameters that I have added
    float derivative_roll_gain;
    float S_proportional_pitch_gain;
    float derivative_pitch_gain;
    float S_proportional_course_gain;
    float integral_course_gain;
    float S_proportional_altitude_gain;
    float integral_altitude_gain;
    float proportional_sideslip_gain;
    float integral_sideslip_gain;
    float proportional_throttle_gain;
    float integral_throttle_gain;
    // Additional Pair of Gains for Turning Flight
    float T_proportional_pitch_gain;
    float T_proportional_altitude_gain;
    float T_proportional_roll_gain;
    float T_proportional_course_gain;

    //Allow for manual control of some params
    float man_throt;
    float man_pitch;
    float man_roll;
    float man_rudder;
    //Allow for Waypoint Following
    float K_Line_Follow;
    float Max_Line_Angle;
    float Enable_Waypoints;
    float Waypoint_Diff; //Switching Between Different Waypoint Arrays
    float Angle_Logic;
    float WPNorth;
    float WPEast;
    //Turning
    float K_Orbit;
    float Turn_Radius;
    float Enable_Orbit;
    //Enable Gearing of Rudder
    float T_Rudder_Prop;
    float S_Rudder_Prop;
    //Parameters that enable completion of the racecourse
    float Course_Radius;
    float Course_Offset;
    float S_race_throt;
    float T_race_throt;
    float Course_Straight_Throttle;
    float Course_Turn_Throttle;
    //Parameters that Enable Step Tuning
    float Step_Altitude;
    float Step_Course;
    float Manual_Inc;
    float Max_Roll_Angle;
    float Desired_Speed;
    float Desired_Alt;

    // TODO: add custom parameter variable names here......

};


/**
 * Struct of handles to all of the custom parameters.
 *
 *  Please make sure to add a variable for each of your newly
 *  defined parameters here.
 *
 *  NOTE: these variable names can be the same as the ones above
 *  (makes life easier if they are)
 */
struct aah_param_handles {

    param_t example_high_param;
    param_t S_proportional_roll_gain;

    // TODO: add custom parameter variable names here.......
    param_t derivative_roll_gain;
    param_t S_proportional_pitch_gain;
    param_t derivative_pitch_gain;
    param_t S_proportional_course_gain;
    param_t integral_course_gain;
    param_t S_proportional_altitude_gain;
    param_t integral_altitude_gain;
    param_t proportional_sideslip_gain;
    param_t integral_sideslip_gain;
    param_t proportional_throttle_gain;
    param_t integral_throttle_gain;
    // Additional Pair of Gains for Turning Flight
    param_t T_proportional_pitch_gain;
    param_t T_proportional_altitude_gain;
    param_t T_proportional_roll_gain;
    param_t T_proportional_course_gain;
    //Parameters to enable mixed control
    param_t man_throt;
    param_t man_pitch;
    param_t man_roll;
    param_t man_rudder;
    //Enable Waypoint Following
    param_t K_Line_Follow;
    param_t Max_Line_Angle;
    param_t Enable_Waypoints;
    param_t Waypoint_Diff;
    param_t Angle_Logic;
    param_t WPNorth;
    param_t WPEast;
    //Enable Orbit
    param_t K_Orbit;
    param_t Turn_Radius;
    param_t Enable_Orbit;
    //Enable Gearing
    param_t T_Rudder_Prop;
    param_t S_Rudder_Prop;
    //Enable Flying the Course
    param_t Course_Radius;
    param_t Course_Offset;
    param_t Course_Straight_Throttle;
    param_t Course_Turn_Throttle;
    //Parameters that Enable Step Tuning
    param_t Step_Altitude;
    param_t Step_Course;
    param_t race_throt; //Enables throttle to enter or leave race mode
    param_t Manual_Inc;
    param_t Max_Roll_Angle;
    param_t S_race_throt;
    param_t T_race_throt;
    param_t Desired_Speed;
    param_t Desired_Alt;
};

/**
 * Initialize all parameter handles and values
 *
 */
int aah_parameters_init(struct aah_param_handles *h);

/**
 * Update all parameters
 *
 */
int aah_parameters_update(const struct aah_param_handles *h, struct aah_params *p);

#ifdef __cplusplus
}
#endif




#endif /* AA241X_FW_CONTROL_PARAMS_H_ */

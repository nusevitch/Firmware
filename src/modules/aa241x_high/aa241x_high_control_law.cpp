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
 * @file aa241x_fw_control.cpp
 *
 * Secondary file to the fixedwing controller containing the
 * control law for the AA241x class.
 *
 *  @author Adrien Perkins		<adrienp@stanford.edu>
 *  @author YOUR NAME			<YOU@EMAIL.COM>
 */

#include <uORB/uORB.h>

// include header file
#include "aa241x_high_control_law.h"
#include "aa241x_high_aux.h"

// needed for variable names
using namespace aa241x_high;

/**
 * Main function in which your code should be written.
 *
 * This is the only function that is executed at a set interval,
 * feel free to add all the function you'd like, but make sure all
 * the code you'd like executed on a loop is in this function.
 */

float my_float_variable = 0.0f;		/**< example float variable */
float altitude_desired=0.0f;
float ground_course_desired=0.0f;
float desired_sideslip_angle= 0.0f;              //Desired Sideslip Angle
float groundspeed_desired = 0.0f;
//Can I declare variables here, or do I also need to put them in the header file?
float    integral_course_error = 0.0f;  //Initialize the Integral Terms to 0
float    integral_altitude_error =0.0f; //Initialize Altitude Error
float    integral_sideslip_error =0.0f; //Initialize Side Slip Error Hold Loop
float    integral_groundspeed_error= 0.0f;
float    Max_Roll_Angle=1.0f;  //Maximum roll angle in Radians, Corresponds to 59 degrees
float    Max_Pitch_Angle=0.5f; //Maximum Pitch Angle in Radians

void flight_control() {


    // An example of how to run a one time 'setup' for example to lock one's altitude and heading...
    if (hrt_absolute_time() - previous_loop_timestamp > 500000.0f) { // Run if more than 0.5 seconds have passes since last loop,
                                                                     //	should only occur on first engagement since this is 59Hz loop
        yaw_desired = yaw; 							// yaw_desired already defined in aa241x_high_aux.h

        //Inialize these values to their initial state (ie, maintain alltitude and heading
        altitude_desired = position_D_baro; 		// altitude_desired needs to be declared
        ground_course_desired = ground_course;    //Desired Course
        desired_sideslip_angle=0.0f;              //Desired Sideslip Angle
        groundspeed_desired=ground_speed;         //Current Throttle Control based Only on ground speed
        //Can I declare variables here, or do I also need to put them in the header file?
        integral_course_error=0.0f;  //Initialize the Integral Terms to 0
        integral_altitude_error=0.0f; //Initialize Altitude Error
        integral_sideslip_error=0.0f; //Initialize Side Slip Error Hold Loop
        integral_groundspeed_error=0.0f;
    }

float Dt=hrt_absolute_time() - previous_loop_timestamp; //Compute the loop time, right now assuming 60 Hz, can compute actual time
    // TODO: write all of your flight control here...


    // getting low data value example
    // float my_low_data = low_data.variable_name1;

    // setting high data value example
    high_data.variable_name1 = my_float_variable;


    // setting low data value example
    low_data.variable_name1 = my_float_variable;



    // // Make a really simple proportional roll stabilizer // //
    //

    //roll_desired = 0.0f; // roll_desired already exists in aa241x_high_aux so no need to repeat float declaration

    // Lateral Dyanamics

    // Compue the INtegral of the Error. Add anti windup here?
    integral_course_error=integral_course_error+(ground_course_desired - ground_course)*Dt;
    float proportionalCourseCorrection = aah_parameters.proportional_course_gain * (ground_course_desired - ground_course);
    float IntegralCourseCorrection = aah_parameters.integral_course_gain * (integral_course_error);  //how to get Derivative INt
    //float DerivativeCourseCorrection= aah_parameters.integral_course_gain * (ground_course_desired - ground_course);

    roll_desired = proportionalCourseCorrection+IntegralCourseCorrection; //Get Commanded Roll from Higher Loop
    //roll_rate_desired= 0.0f;//  ??????? How do I get this?

    // Do bounds checking for commanded roll
    //Also add anti-windup
    if (roll_desired > Max_Roll_Angle) {
        roll_desired = Max_Roll_Angle;
        integral_course_error=integral_course_error-(ground_course_desired - ground_course)*Dt;
    } else if ( roll_desired< -Max_Roll_Angle ) {
        roll_desired = -Max_Roll_Angle;
        integral_course_error=integral_course_error-(ground_course_desired - ground_course)*Dt;
    }


    // Now use your parameter gain and multiply by the error from desired
    float proportionalRollCorrection = aah_parameters.proportional_roll_gain * (roll - roll_desired);
    float derivativeRollCorrection= aah_parameters.derivative_roll_gain * (roll_rate);
    // Note the use of x.0f, this is important to specify that these are single and not double float values!

    float RollEffort=   -derivativeRollCorrection+ proportionalRollCorrection;


    // Altitude Hold Loop

    // Compute the Integral of the Error. Add anti windup here?
    integral_altitude_error=integral_altitude_error+(altitude_desired - position_D_baro)*Dt;
    float proportionalAltitudeCorrection = aah_parameters.proportional_altitude_gain * (altitude_desired - position_D_baro);  //Signs need to be switched here?
    float IntegralAltitudeCorrection = aah_parameters.integral_altitude_gain * (integral_altitude_error);  //how to get Derivative INt
    //float DerivativeCourseCorrection= aah_parameters.integral_course_gain * (ground_course_desired - ground_course);

    //Pitch Loop
    pitch_desired=  proportionalAltitudeCorrection+IntegralAltitudeCorrection;// This needs to come from the low priority loop
    //pitch_rate_desired= 0.0f; //Desired Pitch Rate  // Do I need to numerically take derivative to get this?

    // Do bounds checking for pitch_desired. Add some antiwindup here?
    if (pitch_desired > Max_Pitch_Angle) {
        pitch_desired = Max_Pitch_Angle;
        //Anti-Windup
        integral_altitude_error=integral_altitude_error-(altitude_desired - position_D_baro)*Dt;
    } else if ( pitch_desired< -Max_Pitch_Angle ) {
        pitch_desired = -Max_Pitch_Angle;
        //More AntiWindup
        integral_altitude_error=integral_altitude_error-(altitude_desired - position_D_baro)*Dt;
    }

    float proportionalPitchCorrection = aah_parameters.proportional_pitch_gain * (pitch - pitch_desired);
    float derivativePitchCorrection= aah_parameters.derivative_pitch_gain *(pitch_rate);

    float PitchEffort=  proportionalPitchCorrection- derivativePitchCorrection;

    //PI Controller for SideSlip Effort
    //SideSlip Angle Control Loop
    float sideslip=ground_course-yaw; //Yaw is artficial heading

    //might be issues here
    integral_sideslip_error=integral_sideslip_error+(desired_sideslip_angle - sideslip)*Dt;
    float proportionalSideSlipCorrection = aah_parameters.proportional_sideslip_gain * (desired_sideslip_angle - sideslip);  //Signs need to be switched here?
    float IntegralSideSlipCorrection = aah_parameters.integral_sideslip_gain * (integral_sideslip_error);  //how to get Derivative INt
    float YawEffort=proportionalSideSlipCorrection+IntegralSideSlipCorrection;


    integral_groundspeed_error=integral_groundspeed_error+(groundspeed_desired - ground_speed)*Dt;
    float proportionalthrottleCorrection = aah_parameters.proportional_throttle_gain * (groundspeed_desired - ground_speed);  //Signs need to be switched here?
    float IntegralthrottleCorrection = aah_parameters.integral_throttle_gain * (integral_groundspeed_error);  //how to get Derivative INt
    float throttle_effort=proportionalthrottleCorrection+IntegralthrottleCorrection;

    //Need to add some Anti-windup logic along with the bounds checking?

    // Do bounds checking to keep the roll correction within the -1..1 limits of the servo output
    if (RollEffort > 1.0f) {
        RollEffort = 1.0f;
        //Is antiwindup needed?  I don't think so, integrator is on ground course, not roll
        //integral_groundspeed_error=integral_groundspeed_error-(groundspeed_desired - ground_speed)*Dt; //Anti Windup
    } else if (RollEffort < -1.0f ) {
        RollEffort = -1.0f;
        //integral_groundspeed_error=integral_groundspeed_error-(groundspeed_desired - ground_speed)*Dt; //Anti Windup
    }

    // Do bounds checking to keep the roll correction within the -1..1 limits of the servo output
    if (PitchEffort > 1.0f) {
        PitchEffort = 1.0f;
    } else if (PitchEffort < -1.0f ) {
        PitchEffort = -1.0f;
    }

    // Do bounds checking to keep the throttle correction within the -1..1 limits of the servo output
    if (throttle_effort > 1.0f) {
        throttle_effort = 1.0f;
    } else if (throttle_effort < -1.0f ) {
        throttle_effort = -1.0f;
    }

    // Do bounds checking to keep the rudder correction within the -1..1 limits of the servo output
    if (YawEffort > 1.0f) {
        YawEffort = 1.0f;
        integral_sideslip_error=integral_sideslip_error-(desired_sideslip_angle - sideslip)*Dt; // Anti-Windup
    } else if (YawEffort < -1.0f ) {
        YawEffort = -1.0f;
        integral_sideslip_error=integral_sideslip_error-(desired_sideslip_angle - sideslip)*Dt; //Anti-Windup
    }

    // ENSURE THAT YOU SET THE SERVO OUTPUTS!!!
    // outputs should be set to values between -1..1 (except throttle is 0..1)
    // where zero is no actuation, and -1,1 are full throw in either the + or - directions

    // How to make the gains tuneable? From QR Ground Control?


    // Set output of roll servo to the control law output calculated above
    //roll_servo_out = RollEffort;   //PD Roll Control
    // as an example, just passing through manual control to everything but roll
    //pitch_servo_out = -man_pitch_in;  // WHy the negative sign?
    //pitch_servo_out = PitchEffort; // PD Control on Pitch
    //yaw_servo_out = man_yaw_in;

    //Set Roll Outputs
    if (aah_parameters.man_roll>0.5f){
        roll_servo_out=man_roll_in; //Is direction correct?
    } else {
        roll_servo_out = RollEffort; //throttle_effort;
    }

    //Set Pitch Outputs
    if (aah_parameters.man_pitch>0.5f){
        pitch_servo_out = -man_pitch_in;  //Negative Sign comes from testing, is this the right place to put it?
    } else {
        pitch_servo_out = -PitchEffort; //throttle_effort;  //throttle set to 0 for testing
    }

    //Set Yaw Outputs
    if (aah_parameters.man_rudder>0.5f){
        yaw_servo_out = man_yaw_in;
    } else {
        yaw_servo_out = -YawEffort; //Yaw Effort, Negative sign comes from testing
    }

    //Set Throttle Outputs
    if (aah_parameters.man_throt>0.5f){
        throttle_servo_out = man_throttle_in;
    } else {
        throttle_servo_out= throttle_effort; //throttle_effort;
    }

}

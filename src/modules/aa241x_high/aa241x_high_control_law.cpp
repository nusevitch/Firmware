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
 *  @author Nathan Usevitch		<usevitch@stanford.edu>
 */

#include <uORB/uORB.h>

// include header file
#include "aa241x_high_control_law.h"
#include "aa241x_high_aux.h"

// needed for variable names
using namespace aa241x_high;

// define global variables (can be seen by all files in aa241x_high directory unless static keyword used)

/**
 * Main function in which your code should be written.
 *
 * This is the only function that is executed at a set interval,
 * feel free to add all the function you'd like, but make sure all
 * the code you'd like executed on a loop is in this function.
 */

float my_float_variable = 0.0f;		/**< example float variable */
float altitude_desired=0.0f;
float Init_altitude_desired=0.0f;
float ground_course_desired=0.0f;
float Init_ground_course_desired=0.0f;
float desired_sideslip_angle= 0.0f;              //Desired Sideslip Angle
float groundspeed_desired = 0.0f;
//Can I declare variables here, or do I also need to put them in the header file?
float    integral_course_error = 0.0f;  //Initialize the Integral Terms to 0
float    integral_altitude_error =0.0f; //Initialize Altitude Error
float    integral_sideslip_error =0.0f; //Initialize Side Slip Error Hold Loop
float    integral_groundspeed_error= 0.0f;
float    Max_Roll_Angle=1.0f;  //Maximum roll angle in Radians, Corresponds to 59 degrees
float    Max_Pitch_Angle=0.5f; //Maximum Pitch Angle in Radians
int      WayPoint_Index=0;      //Keeps Track of which waypoint to follow
float    epy=0.0f;              //Lateral Deviance from Desired Line
float    qN=0.0f;               //North Component of Vectors pointing between waypoints
float    qE=0.0f;               //East Component of Vectors Pointing Between Waypoints
float   Xq=0.0f;
float   Xc=0.0f;
float   pN;
float   pE;
float Old_Manual_Inc=0.0f;    //Allows for the manual incrementing of Waypoints



// Code of how we would implement the racecourse code using
//float RaceCourse[][3]= {{50.0f, 150.0f, 0.0f},  // A zero in the last column is a straight line
//                       {50.0f, -200.0f, 0.0f},
//                       {50.0f, 200.0f, 1.0f},     // A 1 Indicates a turn, coordinates are the turns center
//                       {},
//                       {1001.0f, 1001.0f},
//                      };



//Straight Line
//float Waypoint1[][2]= {{0.0f, 150.0f},
//                     {0.0f, 50.0f},
//                     {0.0f, -150.0f  },
//                     {1001.0f, 1001.0f},
//                    };

//45 Degree Turn
//float Waypoint2[][2]= {{0.0f, 150.0f},
//                     {0.0f, 25.0f},
//                     {150.0f,  -125.0f},
//                     {1001.0f, 1001.0f},
//                    };
//15 Degree Turn
//float Waypoint3[][2]={{0.0f, 150.0f},
//                    {0.0f, 25.0f},
//                    {46.9f,  -150.0f},
//                    {1001.0f, 1001.0f},
//                   };

float WaypointCourse[][3]={{0.0f, 100.0f, 0.0f},
                           {-58.66f, 5.0f  ,   0.0f}, //First one
                           {-50.0f, 0.0f, 1.0f},
                           {50.0f, -10.0f, 0.0f},
                           {50.0f, 0.0f,  1.0f},
                           {0.0f, 100.0f, 0.0f},
                           {10.0f, 100.0f, 0.0f},
                         };

//This array is to help know when to increment turns
int OurTurnNum[]={0, 0, 1, 0, 2, 0, 0};


//These Waypoints Fly a simple course, allows for testing switching
//float Waypoint2[][2]= {{150.0f, 0.0f },   //Bottom Left
//                      {0.0f, 100.0f},   //Top
//                      {-100.0f, 100.0f},  //
//                      {0.0f,-100.0f},
//                      {1001.0f, 1001.0f}
//                      };    //

//These waypoints will complete the Pset
//float Waypoint3[][2]= {{-50.0f,  100.0f},   //Bottom Left
//                      {-50.0f, 0.0f},   //Top
//                      {0.0f, -75.0f},  //
//                      {100.0f, -75.0f},   //
//                      {150.0f, 0.0f },
//                      {100.0f, 100.0f },
//                      {50.0f, 100.0f},
//                      {-47.0f, 74.0f},
//                      {-134.0f, 24.0f},
//                      {1001.0f, 1001.0f},
//                     };


//There are many oddly relevant functions in lib/geo/geo.c
//waypoint_from_heading_and_distance();
float Straight_Line(float qN, float qE);              /* Prototype */
//A function to Follow a Straight Line

float Turn(float qN, float qE);
// Currently this is in a testing configuration, can be changed to waypoints when needed.
//float Turn(float Waypoint[][2], int &WayPoint_Index){
float Turn(float qN, float qE) {
    //Next two values enable waypoint following
    //qN=Waypoint[WayPoint_Index][1];
    //qE=Waypoint[WayPoint_Index][2];
   // float d= ((qN-position_N)^2.0f+(qN-position_N)^2.0f)^.5f; // I think we need to use pow for this
    float d= pow(pow(qN-position_N,2.0)+pow(qE-position_E,2.0),0.5);
    float phi=atan2f(position_E-qE,position_N-qN); //    aah_parameters.K_Orbit

    if (phi-ground_course>3.14159f){
            phi=phi-2.0f*3.14159f;
    }

    if (phi-ground_course<-3.14159f){
            Xq=phi+2.0f*3.14159f;
    }

    float Xc=phi+(3.14159f*0.5f+atanf(aah_parameters.K_Orbit*(d-aah_parameters.Turn_Radius)/aah_parameters.Turn_Radius));


//    //Adjust for the way Ground Course is measure (-pi,pi]
//    if (Xc>3.14159f){
//        Xc=Xc-2.0f*3.14159f;
//    }

//    if (Xc<-3.14159f){
//        Xc=Xc+2.0f*3.14159f;
//    }


    return Xc;
}


float Straight_Line(float qN, float qE){

    //Compute Xq based on the desired target point
    Xq=atan2f(qE-position_E,qN-position_N);


        if (Xq-ground_course>3.14159f){
            Xq=Xq-2.0f*3.14159f;
        }

        if (Xq-ground_course<-3.14159f){
            Xq=Xq+2.0f*3.14159f;
        }

    return Xq;
}

//  Also need to check to know where to switch waypoints... Need a manager to check and increment Waypoint_Index
//Another Parameter to Control the Mode???? Ie. Follow Waypoints?

void flight_control() {


    // An example of how to run a one time 'setup' for example to lock one's altitude and heading...
    if (hrt_absolute_time() - previous_loop_timestamp > 500000.0f) { // Run if more than 0.5 seconds have passes since last loop,
                                                                     //	should only occur on first engagement since this is 59Hz loop
        yaw_desired = yaw; 							// yaw_desired already defined in aa241x_high_aux.h

        //Inialize these values to their initial state (ie, maintain alltitude and heading
        Init_altitude_desired = position_D_baro; 		// altitude_desired needs to be declared
        Init_ground_course_desired = ground_course;    //Desired Course
        desired_sideslip_angle=0.0f;              //Desired Sideslip Angle
        groundspeed_desired=ground_speed;         //Current Throttle Control based Only on ground speed
        //Can I declare variables here, or do I also need to put them in the header file?
        integral_course_error=0.0f;  //Initialize the Integral Terms to 0
        integral_altitude_error=0.0f; //Initialize Altitude Error
        integral_sideslip_error=0.0f; //Initialize Side Slip Error Hold Loop
        integral_groundspeed_error=0.0f;
        WayPoint_Index=0;
        //This seems to look right....
        if( aah_parameters.Enable_Orbit>0.5f) {
            qN=aah_parameters.Turn_Radius*cosf(ground_course+3.14159f/2.0f)+position_N;
            qE=aah_parameters.Turn_Radius*sinf(ground_course+3.14159f/2.0f)+position_E;
        }

    }

    //What units is this in?
float Dt=(hrt_absolute_time() - previous_loop_timestamp)/1000000.0f; //Compute the loop time, right now assuming 60 Hz, can compute actual time
    // TODO: write all of your flight control here...



      ground_course_desired=Init_ground_course_desired+aah_parameters.Step_Course;
      altitude_desired=Init_altitude_desired+aah_parameters.Step_Altitude;

//    //The following lines should enable us to check a step response
//    if (abs(aah_parameters.Step_Altitude-Old_Step_Altitude)>0.5f){
//        altitude_desired=altitude_desired+aah_parameters.Step_Altitude;
//    }

//    if (abs(aah_parameters.Step_Course-Old_Step_Course)>0.5f){
//        ground_course_desired=ground_course_desired+aah_parameters.Step_Course;

//        //Some Logic to ensure that the desired groundcourse remains feasible.
//        if (ground_course_desired>3.14159f){
//            ground_course_desired=ground_course_desired-2.0f*3.14159f;
//        }

//        if (ground_course_desired<-3.14159f){
//            ground_course_desired=ground_course_desired+2.0f*3.14159f;
//        }

//    }
//    //Store the Old Value so that I can see if it checked
//    Old_Step_Altitude=aah_parameters.Step_Altitude;
//    Old_Step_Course=aah_parameters.Step_Course;

    // getting low data value example
    // float my_low_data = low_data.variable_name1;

    // setting high data value example
    high_data.variable_name1 = my_float_variable;

    // setting low data value example
    low_data.variable_name1 = my_float_variable;

    // // Make a really simple proportional roll stabilizer // //
    //

    //This probably unwise conditional will allw us to manually increment waypoints
    if (aah_parameters.Manual_Inc-Old_Manual_Inc>0.5f) {
        WayPoint_Index=WayPoint_Index+1;
    }

    if (aah_parameters.Manual_Inc-Old_Manual_Inc<-0.5f) {
        WayPoint_Index=WayPoint_Index-1;
        if (WayPoint_Index<-0.5f) {
            WayPoint_Index=0;
        }
    }

Old_Manual_Inc=aah_parameters.Manual_Inc;

    //roll_desired = 0.0f; // roll_desired already exists in aa241x_high_aux so no need to repeat float declaration

    // Lateral Dyanamics

    //Get Computed desired Ground Course From Vector Field, if enabled
    if( aah_parameters.Enable_Waypoints>0.5f) {

        if (WayPoint_Index==0){
            //If we've crossed the line, INcrement the Counter
            if (turn_num==0){
                WayPoint_Index=WayPoint_Index+1;  //Increment Waypoint INdex if we've crossed the starting line
            }
            //Go to the First Waypoint
            qN=WaypointCourse[0][0];
            qE=WaypointCourse[0][1];
            ground_course_desired=Straight_Line( qN, qE );

        } else {  //This Conditional Covers all points except the first one
            if (WaypointCourse[WayPoint_Index][2]<0.5f) {
                //Perform the Straight Line Stuff
                qN=WaypointCourse[WayPoint_Index][0];
                qE=WaypointCourse[WayPoint_Index][1];
                ground_course_desired=Straight_Line( qN, qE );

                //Check and see if I need to increment
                    //Compute the Vector from the Waypoint to the Current Position
                    pN=WaypointCourse[WayPoint_Index][0]-position_N;
                    pE=WaypointCourse[WayPoint_Index][1]-position_E;
                    //Compute the outward Normal For Comparison
                    //These Lines Compute the Vector and Rotate it by a positive 90 all at once, to
                    // Avoid redefining variables
                    float PEout=-(WaypointCourse[WayPoint_Index][0]- WaypointCourse[WayPoint_Index+1][0])/aah_parameters.Course_Radius;
                    float PNout=(WaypointCourse[WayPoint_Index][1]- WaypointCourse[WayPoint_Index+1][1])/aah_parameters.Course_Radius;
                    //Parameters to Include R, D

                //D is a distance Parameter that shows how far the offset plane is
                if (PNout*pN+PEout*pE> aah_parameters.Course_Offset) {
                    WayPoint_Index=WayPoint_Index+1; //Increment the INdex if we havecrossed the plane
                }

            } else {
                //Enter Turning Mode
               //Get the Coordinates of the Turn Center
                qN=WaypointCourse[WayPoint_Index][0];
                qE=WaypointCourse[WayPoint_Index][1];
                ground_course_desired=Turn(qN, qE);

                //A conditional to Increment the Index
                // If we've completed the turn, increment the index
                if (turn_num==OurTurnNum[WayPoint_Index]){
                    WayPoint_Index=WayPoint_Index+1;
                }
            }
        }
    }       //End of Enable Waypoints Parameter

    //Set index back to 0 if the limit is reached
    if (WayPoint_Index==6){
        WayPoint_Index=0;
    }

    if( aah_parameters.Enable_Orbit>0.5f) {
        ground_course_desired=Turn( qN, qE );  //Command a turn, right now should work
    }


    // Compue the Integral of the Error. Add anti windup here?
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
        //Anti Windup
        integral_groundspeed_error=integral_groundspeed_error-(groundspeed_desired - ground_speed)*Dt;
    } else if (throttle_effort < -1.0f ) {
        throttle_effort = -1.0f;
        //Anti Windup
        integral_groundspeed_error=integral_groundspeed_error-(groundspeed_desired - ground_speed)*Dt;
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
        yaw_servo_out = roll_servo_out*aah_parameters.Rudder_Prop; //Gear the rudder and Ailerons together
    }

    //Set Throttle Outputs
    if (aah_parameters.race_throt<0.5f)  {
        if (aah_parameters.man_throt>0.5f){
            throttle_servo_out = man_throttle_in;
        } else {
            throttle_servo_out= throttle_effort; //throttle_effort;
        }
    } else {  //If race_throt is high, then I will execute these parameters
        if (aah_parameters.Enable_Waypoints>0.5f){
            if (WaypointCourse[WayPoint_Index][2]<0.5f){
                throttle_servo_out = aah_parameters.Course_Straight_Throttle;
            } else {
                throttle_servo_out = aah_parameters.Course_Turn_Throttle;
            }
        } else {//This conditional will be entered if Enable Way is love but race_thot is high
            throttle_servo_out=aah_parameters.Course_Turn_Throttle;
        }
    }

}

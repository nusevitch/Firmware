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
 * @file aa241x_fw_control_params.c
 *
 * Definition of custom parameters for fixedwing controllers
 * being written for AA241x.
 *
 *  @author Adrien Perkins		<adrienp@stanford.edu>
 */

#include "aa241x_high_params.h"



/*
 *  controller parameters, use max. 15 characters for param name!
 *
 */

/**
 * This is an example parameter.  The name of the parameter in QGroundControl
 * will be AAH_EXAMPLE and will be in the AAH dropdown.  Make sure to always
 * start your parameters with AAH to have them all in one place.
 *
 * The default value of this float parameter will be 10.0.
 *
 * @unit meter 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_EXAMPLE, 10.0f);

/**
 * This is an example parameter.  The name of the parameter in QGroundControl
 * will be AAH_PROPROLLGAIN and will be in the AAH dropdown.  Make sure to always
 * start your parameters with AAH to have them all in one place.
 *
 * The default value of this float parameter will be 1.0.
 *
 * @unit none 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_SPROLLG, 1.0f);
/**
 * This is an example parameter.  The name of the parameter in QGroundControl
 * will be AAH_PROPROLLGAIN and will be in the AAH dropdown.  Make sure to always
 * start your parameters with AAH to have them all in one place.
 *
 * The default value of this float parameter will be 1.0.
 *
 * @unit none 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_DROLLG, 0.0f);
/**
 * This is an example parameter.  The name of the parameter in QGroundControl
 * will be AAH_PROPROLLGAIN and will be in the AAH dropdown.  Make sure to always
 * start your parameters with AAH to have them all in one place.
 *
 * The default value of this float parameter will be 1.0.
 *
 * @unit none 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_SPPITCHG, 1.0f);
/**
 * This is an example parameter.  The name of the parameter in QGroundControl
 * will be AAH_PROPROLLGAIN and will be in the AAH dropdown.  Make sure to always
 * start your parameters with AAH to have them all in one place.
 *
 * The default value of this float parameter will be 1.0.
 *
 * @unit none 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_DPITCHG, 0.0f);
/**
 * This is an example parameter.  The name of the parameter in QGroundControl
 * will be AAH_PROPROLLGAIN and will be in the AAH dropdown.  Make sure to always
 * start your parameters with AAH to have them all in one place.
 *
 * The default value of this float parameter will be 1.0.
 *
 * @unit none 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_SPCOURSEG, 0.0f);
/**
 * This is an example parameter.  The name of the parameter in QGroundControl
 * will be AAH_PROPROLLGAIN and will be in the AAH dropdown.  Make sure to always
 * start your parameters with AAH to have them all in one place.
 *
 * The default value of this float parameter will be 1.0.
 *
 * @unit none 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_ICOURSEG, 0.0f);
/**
 * This is an example parameter.  The name of the parameter in QGroundControl
 * will be AAH_PROPROLLGAIN and will be in the AAH dropdown.  Make sure to always
 * start your parameters with AAH to have them all in one place.
 *
 * The default value of this float parameter will be 1.0.
 *
 * @unit none 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_SPALTITUDEG, 0.0f);
/**
 * This is an example parameter.  The name of the parameter in QGroundControl
 * will be AAH_PROPROLLGAIN and will be in the AAH dropdown.  Make sure to always
 * start your parameters with AAH to have them all in one place.
 *
 * The default value of this float parameter will be 1.0.
 *
 * @unit none 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_IALTITUDEG, 0.0f);
/**
 * This is an example parameter.  The name of the parameter in QGroundControl
 * will be AAH_PROPROLLGAIN and will be in the AAH dropdown.  Make sure to always
 * start your parameters with AAH to have them all in one place.
 *
 * The default value of this float parameter will be 1.0.
 *
 * @unit none 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_PSIDESLIPG, 1.0f);
/**
 * This is an example parameter.  The name of the parameter in QGroundControl
 * will be AAH_PROPROLLGAIN and will be in the AAH dropdown.  Make sure to always
 * start your parameters with AAH to have them all in one place.
 *
 * The default value of this float parameter will be 1.0.
 *
 * @unit none 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_ISIDESLIPG, 0.0f);
/**
 * This is an example parameter.  The name of the parameter in QGroundControl
 * will be AAH_PROPROLLGAIN and will be in the AAH dropdown.  Make sure to always
 * start your parameters with AAH to have them all in one place.
 *
 * The default value of this float parameter will be 1.0.
 *
 * @unit none 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_PTHROTTLEG, 0.0f);
/**
 * This is an example parameter.  The name of the parameter in QGroundControl
 * will be AAH_PROPROLLGAIN and will be in the AAH dropdown.  Make sure to always
 * start your parameters with AAH to have them all in one place.
 *
 * The default value of this float parameter will be 1.0.
 *
 * @unit none 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_ITHROTTLEG, 0.0f);

// These parameters enable partial manual control
//If set to 0, autonomous control, if >.5, manual
/**
 * @unit none 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_MANTHROT, 0.0f);

/**
 * @unit none 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_MANPITCH, 0.0f);

/**
 * @unit none 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_MANROLL, 0.0f);
/**
 * @unit none 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_MANRUD, 0.0f);

/**
 * @unit none 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_FOLLOWWAY, 0.0f);  //Logical, Follow Waypoints or No?

/**
 * @unit none 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_KORBIT, 0.0f);  //Logical, Follow Waypoints or No?

/**
 * @unit none 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_ORBITR, 0.0f);  //Logical, Follow Waypoints or No?

/**
 * @unit none 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_ENAORBIT, 0.0f);  //Logical, Follow Waypoints or No?



/**
 * @unit none 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_TRUDPROP, 0.0f);  //Logical, Follow Waypoints or No?

/**
 * @unit none 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_SRUDPROP, 0.0f);  //Logical, Follow Waypoints or No?




/**
 * @unit none 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_ANGLOG, 0.0f);  //Logical, Follow Waypoints or No?

/**
 * @unit none 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_WPNORTH, 0.0f);  //Logical, Follow Waypoints or No?

/**
 * @unit none 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_WPEAST, 0.0f);  //Logical, Follow Waypoints or No?

/**
 * @unit none 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_CRADIUS, 0.0f);  //Logical, Follow Waypoints or No?

/**
 * @unit none 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_COFFSET, 0.0f);  //Logical, Follow Waypoints or No?

/**
 * @unit none 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_CSTRATHROT, 0.0f);  //Logical, Follow Waypoints or No?

/**
 * @unit none 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_CTURNTHROT, 0.0f);  //Logical, Follow Waypoints or No?

/**
 * @unit none 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_STEPALT, 0.0f);  //Logical, Follow Waypoints or No?

/**
 * @unit none 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_STEPCOURSE, 0.0f);  //Logical, Follow Waypoints or No?

/**
 * @unit none 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_SCONTHROT, 0.0f);  //Logical, Follow Waypoints or No?v

/**
 * @unit none 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_TCONTHROT, 0.0f);

/**
 * @unit none 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_MANINC, 0.0f);

//Turning Parameters
/**
 * @unit none 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_TPPITCHG, 0.0f);

/**
 * @unit none 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_TPALTITUDEG, 0.0f);

/**
 * @unit none 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_TPCOURSEG, 0.0f);

/**
 * @unit none 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_TPROLLG, 0.0f);

/**
 * @unit none 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_TMAXROLLA, 0.0f);

/**
 * @unit none 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_SMAXROLLA, 0.5f);


/**
* @unit none 						(the unit attribute (not required, just helps for sanity))
* @group AA241x High Params		(always include this)
*/
PARAM_DEFINE_FLOAT(AAH_SPEED, 0.0f);

/**
* @unit none 						(the unit attribute (not required, just helps for sanity))
* @group AA241x High Params		(always include this)
*/
PARAM_DEFINE_FLOAT(AAH_CALTITUDE, 0.0f);

//Feedforward
/**
* @unit none 						(the unit attribute (not required, just helps for sanity))
* @group AA241x High Params		(always include this)
*/
PARAM_DEFINE_FLOAT(AAH_FF_ON, 0.0f);

/**
* @unit none 						(the unit attribute (not required, just helps for sanity))
* @group AA241x High Params		(always include this)
*/
PARAM_DEFINE_FLOAT(AAH_FF_ROLL, 0.1955f);

/**
* @unit none 						(the unit attribute (not required, just helps for sanity))
* @group AA241x High Params		(always include this)
*/
PARAM_DEFINE_FLOAT(AAH_FF_PITCH, 0.3227f);

/**
* @unit none 						(the unit attribute (not required, just helps for sanity))
* @group AA241x High Params		(always include this)
*/
PARAM_DEFINE_FLOAT(AAH_CTRANSTHROT, 0.7f);

/**
* @unit none 						(the unit attribute (not required, just helps for sanity))
* @group AA241x High Params		(always include this)
*/
PARAM_DEFINE_FLOAT(AAH_WPGO_TO_ONE, 0.0f);

/**
* @unit none 						(the unit attribute (not required, just helps for sanity))
* @group AA241x High Params		(always include this)
*/
PARAM_DEFINE_FLOAT(AAH_FF_SPITCH, 0.0f);

/**
* @unit none 						(the unit attribute (not required, just helps for sanity))
* @group AA241x High Params		(always include this)
*/
PARAM_DEFINE_FLOAT(AAH_KLINE, 0.0f);


/**
* @unit none 						(the unit attribute (not required, just helps for sanity))
* @group AA241x High Params		(always include this)
*/
PARAM_DEFINE_FLOAT(AAH_KANGLE, 0.0f);

//NATHAN: The @unit none and @group AA241x High Params are necessary despite being in a comment
// TODO: define custom parameters here

/**
* @unit none 						(the unit attribute (not required, just helps for sanity))
* @group AA241x High Params		(always include this)
*/
PARAM_DEFINE_FLOAT(AAH_PREPCOURSE, 0.0f);

/**
* @unit none 						(the unit attribute (not required, just helps for sanity))
* @group AA241x High Params		(always include this)
*/
PARAM_DEFINE_FLOAT(AAH_MINPITCHD, 0.0f);






int aah_parameters_init(struct aah_param_handles *h)
{

    /* for each of your custom parameters, make sure to define a corresponding
     * variable in the aa_param_handles struct and the aa_params struct these
     * structs can be found in the aa241x_fw_control_params.h file
     *
     * NOTE: the string passed to param_find is the same as the name provided
     * in the above PARAM_DEFINE_FLOAT
     */
    h->example_high_param           = param_find("AAH_EXAMPLE");
    h->S_proportional_roll_gain       = param_find("AAH_SPROLLG");
    h->derivative_roll_gain         = param_find("AAH_DROLLG");
    h->S_proportional_pitch_gain      = param_find("AAH_SPPITCHG");
    h->derivative_pitch_gain        = param_find("AAH_DPITCHG");
    h->S_proportional_course_gain     = param_find("AAH_SPCOURSEG");
    h->integral_course_gain         = param_find("AAH_ICOURSEG");
    h->S_proportional_altitude_gain   = param_find("AAH_SPALTITUDEG");
    h->integral_altitude_gain       = param_find("AAH_IALTITUDEG");
    h->proportional_sideslip_gain   = param_find("AAH_PSIDESLIPG");
    h->integral_sideslip_gain       = param_find("AAH_ISIDESLIPG");
    h->proportional_throttle_gain   = param_find("AAH_PTHROTTLEG");
    h->integral_throttle_gain       = param_find("AAH_ITHROTTLEG");
    //Seperate Turning Gains
    h-> T_proportional_pitch_gain   = param_find("AAH_TPPITCHG");
    h-> T_proportional_altitude_gain= param_find("AAH_TPALTITUDEG");
    h-> T_proportional_roll_gain    = param_find("AAH_TPROLLG");
    h-> T_proportional_course_gain  = param_find("AAH_TPCOURSEG");
    //Parameters to Enable Partial Manual Control
    h->man_throt       = param_find("AAH_MANTHROT");
    h->man_pitch      = param_find("AAH_MANPITCH");
    h->man_roll       = param_find("AAH_MANROLL");
    h->man_rudder       = param_find("AAH_MANRUD");
    //Parameters for Line Following
    h->K_Line_Follow = param_find("AAH_KLINE");
    h->Max_Line_Angle = param_find("AAH_KANGLE");
    h-> Enable_Waypoints=param_find("AAH_FOLLOWWAY");
    h-> Angle_Logic = param_find("AAH_ANGLOG");
    h-> WPNorth= param_find("AAH_WPNORTH");
    h-> WPEast= param_find("AAH_WPEAST");
    h-> Go_to_Way= param_find("AAH_WPGO_TO_ONE");
    //Parameters for Orbit
    h-> K_Orbit   =param_find("AAH_KORBIT");
    h-> Turn_Radius  =param_find("AAH_ORBITR");
    h-> Enable_Orbit=param_find("AAH_ENAORBIT");
    h-> T_Rudder_Prop=param_find("AAH_TRUDPROP");
    h-> S_Rudder_Prop=param_find("AAH_SRUDPROP");
    //Enable Flying the Course
    h-> Course_Radius=param_find("AAH_CRADIUS");
    h-> Course_Offset=param_find("AAH_COFFSET");
    h-> Trans_race_throt=param_find("AAH_CTRANSTHROT");


    h-> Course_Straight_Throttle=param_find("AAH_CSTRATHROT");
    h-> Course_Turn_Throttle=param_find("AAH_CTURNTHROT");
    //Parameters that Enable Step Tuning
    h-> Step_Altitude=param_find("AAH_STEPALT");
    h-> Step_Course=param_find("AAH_STEPCOURSE");
    h-> S_Constant_Throttle=param_find("AAH_SCONTHROT");
    h-> T_Constant_Throttle=param_find("AAH_TCONTHROT");
    h-> Manual_Inc=param_find("AAH_MANINC");
    h-> Max_Roll_Angle=param_find("AAH_TMAXROLLA");
    h-> S_Max_Roll_Angle=param_find("AAH_SMAXROLLA");
    //
    h-> Desired_Speed=param_find("AAH_SPEED");
    h-> Desired_Alt=param_find("AAH_CALTITUDE");
    //Feedforward
    h-> FF_On=param_find("AAH_FF_ON");
    h-> FF_Roll=param_find("AAH_FF_ROLL");
    h-> FF_Pitch= param_find("AAH_FF_PITCH");
    h-> S_FF_Pitch= param_find("AAH_FF_SPITCH");

    h-> PrepCourse= param_find("AAH_PREPCOURSE");
    h-> Min_Pitch_Angle= param_find("AAH_MINPITCHD");
    // TODO: add the above line for each of your custom parameters........

    return OK;
}

int aah_parameters_update(const struct aah_param_handles *h, struct aah_params *p)
{

    // for each of your custom parameters, make sure to add this line with
    // the corresponding variable name
    param_get(h->example_high_param, &(p->example_high_param));
    param_get(h->S_proportional_roll_gain, &(p->S_proportional_roll_gain));
    // TODO: add the above line for each of your custom parameters.....
    param_get(h->derivative_roll_gain, &(p->derivative_roll_gain));
    param_get(h->S_proportional_pitch_gain, &(p->S_proportional_pitch_gain));
    param_get(h->derivative_pitch_gain, &(p->derivative_pitch_gain));
    param_get(h->S_proportional_course_gain, &(p->S_proportional_course_gain));
    param_get(h->integral_course_gain, &(p->integral_course_gain));
    param_get(h->S_proportional_altitude_gain, &(p->S_proportional_altitude_gain));
    param_get(h->integral_altitude_gain, &(p->integral_altitude_gain));
    param_get(h->proportional_sideslip_gain, &(p->proportional_sideslip_gain));
    param_get(h->integral_sideslip_gain, &(p->integral_sideslip_gain));
    param_get(h->proportional_throttle_gain, &(p->proportional_throttle_gain));
    param_get(h->integral_throttle_gain, &(p->integral_throttle_gain));
    // Enable the Turning Parameters
    param_get(h->T_proportional_pitch_gain, &(p->T_proportional_pitch_gain));
    param_get(h->T_proportional_course_gain, &(p->T_proportional_course_gain));
    param_get(h->T_proportional_altitude_gain, &(p->T_proportional_altitude_gain));
    param_get(h->T_proportional_roll_gain, &(p->T_proportional_roll_gain));
        //Enable Manual Flight
    param_get(h->man_throt, &(p->man_throt));
    param_get(h->man_pitch, &(p->man_pitch));
    param_get(h->man_roll, &(p->man_roll));
    param_get(h->man_rudder, &(p->man_rudder));
    //Enable Waypoints
    param_get(h->K_Line_Follow, &(p->K_Line_Follow));
    param_get(h->Max_Line_Angle, &(p->Max_Line_Angle));
    param_get(h->Enable_Waypoints, &(p->Enable_Waypoints));
    param_get(h->Waypoint_Diff, &(p->Waypoint_Diff));
    param_get(h->Angle_Logic, &(p->Angle_Logic));  //A debugging Angle Logic Parameter
//Allows us to manually set a waypoint to see if we can go towards it
    param_get(h->WPNorth, &(p->WPNorth));
    param_get(h->WPEast, &(p->WPEast));
    param_get(h->Go_to_Way, &(p->Go_to_Way));
    //Enable Orbit
    param_get(h->K_Orbit, &(p->K_Orbit));
    param_get(h->Turn_Radius, &(p->Turn_Radius));
    param_get(h->Enable_Orbit, &(p->Enable_Orbit));
    //Enable the Gearing
    param_get(h->T_Rudder_Prop, &(p->T_Rudder_Prop));
    param_get(h->S_Rudder_Prop, &(p->S_Rudder_Prop));
    //Enable Flying the Course
    param_get(h->Course_Radius, &(p->Course_Radius));
    param_get(h->Course_Offset, &(p->Course_Offset));
    param_get(h->Trans_race_throt, &(p->Trans_race_throt));

    param_get(h->Course_Straight_Throttle, &(p->Course_Straight_Throttle));
    param_get(h->Course_Turn_Throttle, &(p->Course_Turn_Throttle));
    //Parameters that Enable Step Tuning
    param_get(h->Step_Altitude, &(p->Step_Altitude));
    param_get(h->Step_Course, &(p->Step_Course));
    param_get(h->S_Constant_Throttle, &(p->S_Constant_Throttle));
    param_get(h->T_Constant_Throttle, &(p->T_Constant_Throttle));
    //Some Turning/ Debuggin Parameters
    param_get(h->Manual_Inc, & (p->Manual_Inc));
    param_get(h->Max_Roll_Angle, & (p->Max_Roll_Angle));
    param_get(h->S_Max_Roll_Angle, & (p->S_Max_Roll_Angle));
    param_get(h->Desired_Speed, & (p->Desired_Speed));
    param_get(h->Desired_Alt, & (p->Desired_Alt));
    //Feedforward
    param_get(h->FF_On, & (p->FF_On));
    param_get(h->FF_Roll, & (p->FF_Roll));
    param_get(h->FF_Pitch, & (p->FF_Pitch));
    param_get(h->S_FF_Pitch, & (p->S_FF_Pitch));
    param_get(h->PrepCourse, & (p->PrepCourse));
    param_get(h->Min_Pitch_Angle, & (p->Min_Pitch_Angle));





    return OK;
}

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
PARAM_DEFINE_FLOAT(AAH_PROPROLLGAIN, 1.0f);
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
PARAM_DEFINE_FLOAT(AAH_DROLLG, 1.0f);

PARAM_DEFINE_FLOAT(AAH_PPITCHG, 1.0f);
PARAM_DEFINE_FLOAT(AAH_DPITCHG, 1.0f);
PARAM_DEFINE_FLOAT(AAH_PCOURSEG, 1.0f);
PARAM_DEFINE_FLOAT(AAH_ICOURSEG, 1.0f);
PARAM_DEFINE_FLOAT(AAH_PALTITUDEG, 1.0f);
PARAM_DEFINE_FLOAT(AAH_IALTITUDEG, 1.0f);
PARAM_DEFINE_FLOAT(AAH_PSIDESLIPG, 1.0f);
PARAM_DEFINE_FLOAT(AAH_ISIDESLIPG, 1.0f);
PARAM_DEFINE_FLOAT(AAH_PTHROTTLEG, 1.0f);
PARAM_DEFINE_FLOAT(AAH_ITHROTTLEG, 1.0f);

//NATHAN: Do I need to include the @unit none and @group AA241x High Params for all of these?
// TODO: define custom parameters here


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
    h->proportional_roll_gain       = param_find("AAH_PROPROLLGAIN");
    h->derivative_roll_gain         = param_find("AAH_DROLLG");
    h->proportional_pitch_gain      = param_find("AAH_PPITCHG");
    h->derivative_pitch_gain        = param_find("AAH_DPITCHG");
    h->proportional_course_gain     = param_find("AAH_PCOURSEG");
    h->integral_course_gain         = param_find("AAH_ICOURSEG");
    h->proportional_altitude_gain   = param_find("AAH_PALTITUDEG");
    h->integral_altitude_gain       = param_find("AAH_IALTITUDEG");
    h->proportional_sideslip_gain   = param_find("AAH_PSIDESLIPG");
    h->integral_sideslip_gain       = param_find("AAH_ISIDESLIPG");
    h->proportional_throttle_gain   = param_find("AAH_PTHROTTLEG");
    h->integral_throttle_gain       = param_find("AAH_ITHROTTLEG");


    // TODO: add the above line for each of your custom parameters........

    return OK;
}

int aah_parameters_update(const struct aah_param_handles *h, struct aah_params *p)
{

    // for each of your custom parameters, make sure to add this line with
    // the corresponding variable name
    param_get(h->example_high_param, &(p->example_high_param));
    param_get(h->proportional_roll_gain, &(p->proportional_roll_gain));
    // TODO: add the above line for each of your custom parameters.....
    param_get(h->derivative_roll_gain, &(p->derivative_roll_gain));
    param_get(h->proportional_pitch_gain, &(p->proportional_pitch_gain));
    param_get(h->derivative_pitch_gain, &(p->derivative_pitch_gain));
    param_get(h->proportional_course_gain, &(p->proportional_course_gain));
    param_get(h->integral_course_gain, &(p->integral_course_gain));
    param_get(h->proportional_altitude_gain, &(p->proportional_altitude_gain));
    param_get(h->integral_altitude_gain, &(p->integral_altitude_gain));
    param_get(h->proportional_sideslip_gain, &(p->proportional_sideslip_gain));
    param_get(h->integral_sideslip_gain, &(p->integral_sideslip_gain));
    param_get(h->proportional_throttle_gain, &(p->proportional_throttle_gain));
    param_get(h->integral_throttle_gain, &(p->integral_throttle_gain));

    return OK;
}

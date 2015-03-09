/****************************************************************************
 *
f *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
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
 * AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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

/**
 * @file jager_test_params.c
 *
 * Parameters to assist in the Odroid Simulator testing.
 *
 * @author Adrien Perkins	<adrienp@stanford.edu>
 */

#include <nuttx/config.h>

#include <systemlib/param/param.h>


/*
 * Testing parameters, accessible via MAVLink
 *
 */

/**
 * Mission Scenario.
 *
 * Integer defining which scenario to run.
 * 0 - move cmds
 * 1 - rotate cmds
 *
 * @min 0
 * @max 1
 * @group Jager Testing Params
 */
PARAM_DEFINE_INT32(OSIM_SCENARIO, 0);

/**
 * Move Distance.
 *
 * Distance in [m] through which to move (basically edge size on box).
 *
 * @unit meters
 * @min 0
 * @group Jager Testing Params
 */
PARAM_DEFINE_FLOAT(OSIM_MOVE_DIST, 10.0f);


/**
 * Initial Rotation Direction.
 *
 * Initialize desired rotation direction:
 * -1 ccw
 * 1 cw
 *
 * @min -1
 * @group Jager Testing Params
 */
PARAM_DEFINE_INT32(OSIM_INIT_DIR, 1);

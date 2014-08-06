/*
 * hunt.cpp
 *
 *  Created on: Aug 5, 2014
 *      Author: adrienp
 */

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <fcntl.h>

#include <mavlink/mavlink_log.h>
#include <systemlib/err.h>

#include <uORB/uORB.h>

#include "navigator.h"
#include "hunt.h"

Hunt::Hunt(Navigator *navigator, const char *name) :
	NavigatorMode(navigator, name)
{
	/* just initializing some needed variables */
	_started = false;
	_hunt_state(HUNT_STATE_OFF);

	/* load initial params */
	updateParams();

	/* initial reset */
	on_inactive();
}

Hunt::~Hunt()
{
	// destructor, don't think need anything here
}

void
Hunt::on_inactive()
{
	// called when the hunt mode is made inactive
	// need to reset some of the paramters

	// maybe need to think about doing a suspended mode instead of off
	// would make the _started parameter not needed?
	/* put the hunt into off mode */
	_hunt_state(HUNT_STATE_OFF);
}

void
Hunt::on_activation()
{
	// called when we hunt mode gets activiated

	if (!_started) { // hunt not started, meaning this is the first time we have activated hunt
		// go to start position

		// set the state to moving to first position
		// XXX: maybe don't need state start, and just set it to state move here
		_hunt_state(HUNT_STATE_START);
	} else {
		// need to check if we are at the last location
		// if not at the last commanded location, go to last commanded location
	}

	// set the created mission item to a position setpoint?
	// also handle some of the states??
	set_next_item();

}

void
Hunt::on_active()
{
	// TODO: this is where all the stuff runs

	// XXX: not sure if or or and is best here, depends on when hunt state is
	// changed to wait
	if (_hunt_state == HUNT_STATE_WAIT || is_mission_item_reached()) {


	}


}

bool
Hunt::get_next_item()
{




	return false;
}

void
Hunt::set_next_item()
{

}





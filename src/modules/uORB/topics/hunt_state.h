/*
 * hunt_state.h
 *
 *  Created on: Aug 14, 2014
 *      Author: adrienp
 */

#ifndef HUNT_STATE_H_
#define HUNT_STATE_H_

#include <stdint.h>
#include "../uORB.h"

enum HUNT_STATE {
	HUNT_STATE_OFF = 0,		// hunt has been turned off
	HUNT_STATE_START,		// hunt just started, need to go to starting position
	HUNT_STATE_WAIT,		// waiting for next command from tracking
	HUNT_STATE_MOVE,		// executing a move command from tracking
	HUNT_STATE_ROTATE,		// executing a rotate command from tracking
};


struct hunt_state_s {
	uint64_t timestamp;

	uint8_t hunt_mode_state;
};


/* register this topic */
ORB_DECLARE(hunt_state);



#endif /* HUNT_STATE_H_ */

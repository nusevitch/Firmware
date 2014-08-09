/*
 * tracking_status.h
 *
 *  Created on: Jul 30, 2014
 *      Author: adrienp
 */

#ifndef TRACKING_STATUS_H_
#define TRACKING_STATUS_H_

#include <stdint.h>
#include "../uORB.h"

enum COMPUTER_STATUS {
	TRACKING_COMPUTER_UNKNOWN = 0,
	TRACKING_COMPUTER_STANDBY,
	TRACKING_COMPUTER_HUNT,
	TRACKING_COMPUTER_DONE
};

enum HUNT_STATE {
	HUNT_STATE_OFF = 0,		// hunt has been turned off
	HUNT_STATE_START,		// hunt just started, need to go to starting position
	HUNT_STATE_WAIT,		// waiting for next command from tracking
	HUNT_STATE_MOVE,		// executing a move command from tracking
	HUNT_STATE_ROTATE,		// executing a rotate command from tracking
};


struct tracking_status_s {
	uint64_t timestamp;

	uint8_t computer_status;
	uint8_t hunt_mode_state;
};


/* register this topic */
ORB_DECLARE(tracking_status);



#endif /* TRACKING_STATUS_H_ */

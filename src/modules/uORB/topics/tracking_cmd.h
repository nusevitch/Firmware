/*
 * tracking_status.h
 *
 *  Created on: August, 2014
 *      Author: adrienp
 */

#ifndef TRACKING_CMD_H_
#define TRACKING_CMD_H_

#include <stdint.h>
#include "../uORB.h"

/**
 * representation of MAVLINK tracking cmds
 */
enum HUNT_CMD {
	HUNT_CMD_MOVE = 1,
	HUNT_CMD_ROTATE
};


struct tracking_cmd_s {
	uint8_t cmd_type; 	/**< type of command */
	float paramf_1;		/**< north distance or angle or rotation, depending on command type */
	float paramf_2;		/**<east distance */
};


/* register this topic */
ORB_DECLARE(tracking_cmd);



#endif /* TRACKING_CMD_H_ */

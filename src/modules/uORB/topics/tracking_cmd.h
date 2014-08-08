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
	HUNT_CMD_TRAVEL = 1,
	HUNT_CMD_ROTATE,
	HUNT_CMD_FINISH
};


struct tracking_cmd_s {
	uint8_t cmd_type; 	/**< type of command */
	uint16_t cmd_id;		/**< id of the command, an incremental number sent from tracking (mainly for double checking */
	double north;		/**< north distance or angle or rotation, depending on command type */
	double east;		/**< east distance */
	float yaw_angle;		/**< a third parameter, possible desired yaw to maintain during flight... */
	float altitude;
};


/* register this topic */
ORB_DECLARE(tracking_cmd);



#endif /* TRACKING_CMD_H_ */

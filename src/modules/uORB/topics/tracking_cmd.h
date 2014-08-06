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

enum TRAC_CMD {
	TRACKING_CMD_MOVE = 0,
	TRACKING_CMD_ROTATE
};


struct tracking_cmd_s {
	uint8_t cmd_type;
	uint8_t parami_1;
	uint8_t parami_2;
	float paramf_1;
	float paramf_2;
};


/* register this topic */
ORB_DECLARE(tracking_cmd);



#endif /* TRACKING_CMD_H_ */

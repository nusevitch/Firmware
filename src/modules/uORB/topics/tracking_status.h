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

struct tracking_status_s {
	uint64_t timestamp_status;
	uint64_t timestamp_cmd;

	uint8_t status;
	uint8_t cmd_type;
	float cmd_dist;
	uint8_t cmd_direction;
};


/* register this topic */
ORB_DECLARE(tracking_status);



#endif /* TRACKING_STATUS_H_ */

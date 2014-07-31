/*
 * apnt_position.h
 *
 *  Created on: Jul 30, 2014
 *      Author: adrienp
 */

#ifndef APNT_POSITION_H_
#define APNT_POSITION_H_


#include <stdint.h>
#include "../uORB.h"

struct apnt_position_s {
	uint64_t timestamp;

	float lat;
	float lon;
};

/* register this topic */
ORB_DECLARE(apnt_position);

#endif /* APNT_POSITION_H_ */

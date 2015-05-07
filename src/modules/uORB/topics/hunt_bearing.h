/*
 * hunt_bearing.h
 *
 *  Created on: May 6, 2015
 *      Author: adrienp
 */

#ifndef HUNT_BEARING_H_
#define HUNT_BEARING_H_

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"

/**
 * @addtogroup topics
 * @{
 */

struct hunt_bearing_s
{
	uint64_t timestamp_us;
	double bearing;		/**< The measured bearing (in degrees?) */
	int32_t lat;		/**< the latitude of the measurement	 */
	int32_t lon;		/**< the longitude of the measurement	 */
	float alt;			/**< the altitude of the measurement	 */
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(hunt_bearing);


#endif /* HUNT_BEARING_H_ */

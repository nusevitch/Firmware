/*
 * hunt_rssi.h
 *
 *  Created on: May 6, 2015
 *      Author: adrienp
 */

#ifndef HUNT_RSSI_H_
#define HUNT_RSSI_H_

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"

/**
 * @addtogroup topics
 * @{
 */

struct hunt_rssi_s
{
	uint64_t timestamp_us;
	int32_t rssi;		/**< The measured rssi value */
	float heading;		/**< the heading of the measurement */
	int32_t lat;		/**< the latitude of the measurement	 */
	int32_t lon;		/**< the longitude of the measurement	 */
	float alt;			/**< the altitude of the measurement	 */
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(hunt_rssi);


#endif /* HUNT_RSSI_H_ */

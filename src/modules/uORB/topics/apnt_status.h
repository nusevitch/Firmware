/**
 * @file apnt_status.h
 * Currend status of the apnt system running on boards jager.
 *
 * @author Adrien Perkins <adrienp@stanford.edu>
 * July 2014
 * Stanford University GPS Lab
 */

#ifndef APNT_STATUS_H_
#define APNT_STATUS_H_

#include <stdint.h>
#include "../uORB.h"

struct apnt_status_s {
	uint64_t timestamp; /**< time in usec since boot. */

	uint8_t status; /**< simple system status of whether or not apnt is running well. */
	uint8_t connections; /**< enumerate which apnt towers connected to. */

	uint8_t num_sats; /**< the number of gps satellites currently in view */

	uint8_t gps_snr;
	uint8_t apnt_snr;


};

/* register this topic */
ORB_DECLARE(apnt_status);


#endif /* APNT_STATUS_H_ */

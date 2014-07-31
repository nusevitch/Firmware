/*
 * apnt_gps_status.h
 *
 *  Created on: Jul 30, 2014
 *      Author: adrienp
 */

#ifndef APNT_GPS_STATUS_H_
#define APNT_GPS_STATUS_H_

#include <stdint.h>
#include "../uORB.h"

struct apnt_gps_status_s {
	uint64_t timestamp;

	float lat;
	float lon;
	float alt;

	uint8_t prn[8];
	int16_t azimuth[8];
	uint8_t elevation[8];
	uint8_t snr[8];
};


/* register this topic */
ORB_DECLARE(apnt_gps_status);


#endif /* APNT_GPS_STATUS_H_ */

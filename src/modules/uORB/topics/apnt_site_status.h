/*
 * apnt_site_status.h
 *
 *  Created on: Jul 30, 2014
 *      Author: adrienp
 */

#ifndef APNT_SITE_STATUS_H_
#define APNT_SITE_STATUS_H_

#include <stdint.h>
#include "../uORB.h"

struct apnt_site_status_s {
	uint64_t timestamp;

	uint32_t id[4];
	float lat[4];
	float lon[4];
	uint16_t signal[4];
};


/* register this topic */
ORB_DECLARE(apnt_site_status);



#endif /* APNT_SITE_STATUS_H_ */

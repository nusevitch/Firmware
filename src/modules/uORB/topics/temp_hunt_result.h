/*
 * temp_hunt_result.h
 *
 *  Created on: Aug 16, 2014
 *      Author: adrienp
 */

#ifndef TEMP_HUNT_RESULT_H_
#define TEMP_HUNT_RESULT_H_


#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"

/**
 * @addtogroup topics
 * @{
 */

struct temp_hunt_result_s
{
	unsigned cmd_reached;		/**< Sequence of the mission item which has been reached */
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(temp_hunt_result);


#endif /* TEMP_HUNT_RESULT_H_ */

/*
 * hunt_mission_result.h
 *
 *  Created on: Aug 11, 2014
 *      Author: adrienp
 */

#ifndef HUNT_RESULT_H_
#define HUNT_RESULT_H_

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"

/**
 * @addtogroup topics
 * @{
 */

struct hunt_result_s
{
	unsigned cmd_reached;		/**< Sequence of the mission item which has been reached */
	unsigned cmd_current;		/**< Sequence of the current mission item				 */
	bool reached;				/**< true if mission has been reached					 */
	bool finished;				/**< true if mission has been completed					 */
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(hunt_result);



#endif /* HUNT_RESULT_H_ */

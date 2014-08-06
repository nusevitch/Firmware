/*
 * hunt.h
 *
 *  Created on: Aug 5, 2014
 *      Author: adrienp
 */

#ifndef HUNT_H_
#define HUNT_H_


#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>

#include <uORB/uORB.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/tracking_cmd.h>

#include "navigator_mode.h"
#include "mission_block.h"


class Navigator;

class Hunt : public MissionBlock
{
public:
	Hunt(Navigator *navigator, const char *name);

	~Hunt();

	virtual void on_inactive();

	virtual void on_activation();

	virtual void on_active();
private:

	/**
	 * get the next command from tracking via mavlink
	 */
	bool get_next_item();

	/**
	 * once the next item has been received,
	 * actually set it as a mission item and setpoint
	 */
	void set_next_item();

	/**
	 * states that the hunt script can be in
	 * will be used to help identify whether we
	 * should be polling for the next command or not
	 */
	enum HuntState {
		HUNT_STATE_OFF = 0,		// hunt has been turned off
		HUNT_STATE_START,		// hunt just started, need to go to starting position
		HUNT_STATE_WAIT,		// waiting for next command from tracking
		HUNT_STATE_MOVE,		// executing a move command from tracking
		HUNT_STATE_ROTATE,		// executing a rotate command from tracking
	} _hunt_state;

	/**
	 * boolean to simply hold whether or not a hunt has started
	 * will just change the behavior of vehicle when coming from off state
	 */
	bool _started;

	struct tracking_cmd_s _tracking_cmd;


};


#endif /* HUNT_H_ */

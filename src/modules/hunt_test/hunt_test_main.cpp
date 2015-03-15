/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file hunt_test_main.cpp
 *
 * Imitates what the odroid would be doing.
 *
 * @author Adrien Perkins	<adrienp@stanford.edu>
 *
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_accel.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/hunt_state.h>
#include <uORB/topics/hunt_result.h>
#include <uORB/topics/tracking_cmd.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/pid/pid.h>
#include <geo/geo.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h> 		/*< for conversion from local x,y to lat, lon */

#include <platforms/px4_defines.h>


extern "C" __EXPORT int hunt_test_main(int argc, char *argv[]);

class Hunt
{
public:
	Hunt();

	~Hunt();

	/**
	 * Start the main task.
	 *
	 * @return	OK on success.
	 */
	int		start();

	/**
	 * Task status
	 *
	 * @return	true if the mainloop is running
	 */
	bool		task_running() { return _task_running; }

private:
	bool		_task_should_exit;		/**< if true, attitude control task should exit */
	bool		_task_running;			/**< if true, task is running in its mainloop */
	int			_main_task;


	/**
	 * get the next command from tracking via mavlink
	 */
	bool get_next_cmd();

	/**
	 * once the next item has been received,
	 * actually set it as a mission item and setpoint
	 */
	void set_next_item();

	/*
	 * put the vehicle in a loiter mode as we wait for tracking to respond
	 */
	void set_waiting();

	/*
	 * update mission result to say that the most recent command has been confirmed finished
	 */
	void report_cmd_finished();

	/*
	 * update the current cmd id being flown
	 */
	void report_cmd_id();

	/*
	 * publish the mission result (which should get picked up by mavlink module to be sent to ground)
	 */
	void publish_hunt_result();

	/*
	 * report that a status change has occurred
	 */
	void report_status();

	/*
	 * publish the changed status to the corresponding orb message
	 */
	void publish_status();

	/*
	 * reset whether or not we have reached the hunt item
	 */
	void rest_hunt_item_reached();

	/*
	 * get an update of the local position
	 */
	void update_local_position();

	/*
	 * update the reference position used in converting from local position to global position
	 */
	void update_reference_position();

	/*
	 * set the mission lat, lon coordinates from the tracking cmds N, E position
	 */
	void set_mission_latlon();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main simulator loop
	 */
	void		task_main();

	/**
	 * boolean to simply hold whether or not a hunt has started
	 * will just change the behavior of vehicle when coming from off state
	 */
	bool _started;
	uint16_t _current_cmd_id;			/**< the current command being run */
	int _hunt_state;					/**< the current state */

	int _local_pos_sub; 		/**< subscription to the local position */
	int _global_pos_sub;
	int _tracking_command_sub; 	/**< subscription to the tracking command */

	orb_advert_t _hunt_result_pub;	/**< publish the current hunt result */
	orb_advert_t _hunt_state_pub;	/**< publish the current hunt state */

	struct vehicle_local_position_s _local_pos;	/**< local position struct */
	struct vehicle_global_position_s _global_pos;

	struct hunt_result_s _hunt_result;		/**< struct for the result of a cmd */
	struct hunt_state_s _hunt_state_s;		/**< struct for the state of the hunt mode */
	struct tracking_cmd_s _tracking_cmd;	/**< struct for the current tracking command */


	struct map_projection_reference_s _ref_pos;	// this is reference position

	/* rotation handling */
	int		_current_rotation_direction;	/**< -1 ccw, 1 cw and 0 no direction */
	float	_end_rotation_angle;			/**< the angle by which time rotation time should stop (same as initial...) */
	bool	_in_rotation;					/**< true if in a rotation already, false otherwise */
	bool	_allow_rotation_end;			/**< if true will start checking to see if we have reached the mission item */

	bool	_mission_reached;


	hrt_abstime _temp_time;
	hrt_abstime _test_time;
	hrt_abstime _ref_timestamp;	// timestamp the reference position was taken at
};

namespace hunt_sim
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

Hunt	*g_sim = nullptr;
}

Hunt::Hunt() :

	_task_should_exit(false),
	_task_running(false),
	_main_task(-1),

	/* hunt state/logic */
	_started(false),
	_current_cmd_id(0),
	_hunt_state(HUNT_STATE_OFF),

	/* subscriptions */
	_local_pos_sub(-1),
	_tracking_command_sub(-1),
	_global_pos_sub(-1),

	/* publications */
	_hunt_result_pub(-1),
	_hunt_state_pub(-1),

	/* rotation handling */
	_current_rotation_direction(0),
	_end_rotation_angle(0),
	_in_rotation(false),
	_allow_rotation_end(false),
	_mission_reached(false),

	/* time */
	_temp_time(hrt_absolute_time()),
	_test_time(hrt_absolute_time()),
	_ref_timestamp(0)
{
	/* initialize structs */
	_local_pos = {};
	_global_pos = {};
	_ref_pos = {};
	_tracking_cmd = {};
	_hunt_result = {};
	_hunt_state_s = {};


}

Hunt::~Hunt()
{
	if (_main_task != -1) {

		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				task_delete(_main_task);
				break;
			}
		} while (_main_task != -1);
	}

	hunt_sim::g_sim = nullptr;
}

bool
Hunt::get_next_cmd()
{
	bool updated = false;
	orb_check(_tracking_command_sub, &updated);

	if (updated) {
		// copy over the command
		orb_copy(ORB_ID(tracking_cmd), _tracking_command_sub, &_tracking_cmd);
		return true;
	}

	return false;
}

void
Hunt::set_next_item()
{

	// update the current cmd id to be that of the new cmd
	_current_cmd_id = _tracking_cmd.cmd_id;

	// XXX: IMPORTANT
	// TODO: should do a distance to home check on this to make sure we are not commanded to go insanely far

	// create a mission item from the tracking cmd
	switch (_tracking_cmd.cmd_type) {
	case HUNT_CMD_TRAVEL: {
		/* change the hunt state to move */
		_hunt_state = HUNT_STATE_MOVE;

		/* get desired north and east distances of travel */
		set_mission_latlon();

		break;
	}
	case HUNT_CMD_ROTATE: {

		/* change the hunt state to rotate */
		_hunt_state = HUNT_STATE_ROTATE;

		if (!_in_rotation) {
			_in_rotation = true;

		}

		_current_rotation_direction = _tracking_cmd.yaw_angle;

		/* if ending rotation */
		if (_allow_rotation_end) {

			break;
		}

		break;
	}
	default:
		break;

	}

	// report the new cmd id
	report_cmd_id();

	// status must have changed if at this point, so report it
	report_status();
}


void
Hunt::set_waiting()
{

	// _navigator->set_can_loiter_at_sp(pos_sp_triplet->current.type == SETPOINT_TYPE_LOITER); // this is what mission does

	// XXX: really not sure...
	// _navigator->set_can_loiter_at_sp(true);

}

void
Hunt::report_cmd_finished()
{
	_hunt_result.reached = true;
	_hunt_result.cmd_reached = _current_cmd_id;

	publish_hunt_result();
}

void
Hunt::report_cmd_id()
{
	// just make sure that reached and finished are both false
	_hunt_result.reached = false;
	_hunt_result.finished = false;
	_hunt_result.cmd_current = _current_cmd_id;
	publish_hunt_result();
}


void
Hunt::publish_hunt_result()
{
	/* lazily publish the mission result only once available */
	if (_hunt_result_pub > 0) {
		/* publish mission result */
		orb_publish(ORB_ID(hunt_result), _hunt_result_pub, &_hunt_result_pub);
	} else {
		/* advertise and publish */
		_hunt_result_pub = orb_advertise(ORB_ID(hunt_result), &_hunt_result_pub);
	}

	/* reset reached bool */
	_hunt_result.reached = false;
	_hunt_result.finished = false;
}

void
Hunt::report_status()
{
	// update the tracking status state to the current hunt state
	_hunt_state_s.hunt_mode_state = _hunt_state;

	// need to then publish the status change
	publish_status();
}


void
Hunt::publish_status()
{
	/* lazily publish the hunt state only once available */
	if (_hunt_state_pub > 0) {
		/* publish current hunt state */
		orb_publish(ORB_ID(hunt_state), _hunt_state_pub, &_hunt_state_s);
	} else {
		/* advertise and publish */
		_hunt_state_pub = orb_advertise(ORB_ID(hunt_state), &_hunt_state_s);
	}
}


void
Hunt::update_local_position()
{
	if (_local_pos_sub > 0) {
		orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);
	} else {
		_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
		orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);
	}

	if (_global_pos_sub > 0) {
		orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);
	} else {
		_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
		orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);
	}

}


void
Hunt::update_reference_position()
{
	// if we have a new local position, update the reference position
	if (_local_pos.ref_timestamp != _ref_timestamp) {

		/* update local projection reference */
		map_projection_init(&_ref_pos, _local_pos.ref_lat, _local_pos.ref_lon);
		// NOTE: these reference lat and lon points should never change...

		_ref_timestamp = _local_pos.ref_timestamp;
	}
}


void
Hunt::set_mission_latlon()
{
	// calculate the desired noth and east positions in the local frame
	float north_desired = _tracking_cmd.north + _local_pos.x;
	float east_desired = _tracking_cmd.east + _local_pos.y;

}




void
Hunt::task_main_trampoline(int argc, char *argv[])
{
	hunt_sim::g_sim->task_main();
}

void
Hunt::task_main()
{

	/* inform about start */
	warnx("Initializing..");
	fflush(stdout);

	/*
	 * do subscriptions
	 */
	_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_tracking_command_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));

	_hunt_state = HUNT_STATE_WAIT;

	/* wakeup source(s) */
	struct pollfd fds[1];

	/* Setup of loop */
	fds[0].fd = _local_pos_sub;
	fds[0].events = POLLIN;

	_task_running = true;

	while (!_task_should_exit) {

		/* wait for up to 500ms for data */
		int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 500);

		// though do all this stuff regardless of whether or not data was received


		// update our copy of the local position
		update_local_position();

		// update the reference position
		update_reference_position();

		// only check mission success when not in a waiting mode (to avoid always checking whether or not
		// we have reached the cmd when we are just waiting around)
		// TODO: use own is mission item reached.... trying to use the existing one may be the source of some problems
		if (_hunt_state == HUNT_STATE_MOVE && _mission_reached) {

			_test_time = hrt_absolute_time(); // update the test time, this is for a delay between

			// we finished with the cmd, so broadcast that
			report_cmd_finished();

			// change the state to waiting since we have reached the target
			_hunt_state = HUNT_STATE_WAIT;

			// have vehicle start waiting
			set_waiting();

			// state changed here, so report it
			report_status();

		} else if (_hunt_state == HUNT_STATE_ROTATE) {

			/* check to see if rotation finished */
			if (_allow_rotation_end && _mission_reached) {
				_allow_rotation_end = false;
				_in_rotation = false;

				report_cmd_finished();

				_hunt_state = HUNT_STATE_WAIT;
				set_waiting();
				report_cmd_finished();
			} else {
				/* if current location is w/in 5 deg of desired location, allow a stop */
				if (abs(_navigator->get_global_position()->yaw - _end_rotation_angle) <= math::radians(5.0)) {
					_allow_rotation_end = true;
				}

				// check to see if there is a new command, if not will just continue rotating
				if (get_next_cmd()) {
					// update the final angle
					_end_rotation_angle = _navigator->get_global_position()->yaw;
				}

				// just go ahead and keep rotating, etc.
				set_next_item();
			}
		}

		// XXX: not sure if or or and is best here, depends on when hunt state is
		// changed to wait
		if (_hunt_state == HUNT_STATE_WAIT) {
			// we have reached the desired point or are waiting

			if (get_next_cmd()) {
				// new command has come from tracking
				set_next_item();

				// set next item will handle the state change to a none wait state
			}
		}

		// always report the status here, just so there is a constant new mavlink message
		report_status();

		// XXX: IMPORTANT
		// TODO: figure out how to broadcast done with the command
	}

	warnx("exiting.\n");

	_main_task = -1;
	_task_running = false;
	_exit(0);
}

int
Hunt::start()
{
	ASSERT(_main_task == -1);

	/* start the task */
	_main_task = task_spawn_cmd("jager_test",
				       SCHED_DEFAULT,
				       SCHED_PRIORITY_MAX - 5,
				       2048,
				       (main_t)&Hunt::task_main_trampoline,
				       nullptr);

	if (_main_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int hunt_test_main(int argc, char *argv[])
{
	if (argc < 1)
		errx(1, "usage: hunt_test {start|stop|status}");

	if (!strcmp(argv[1], "start")) {

		if (hunt_sim::g_sim != nullptr)
			errx(1, "already running");

		hunt_sim::g_sim = new Hunt;

		if (hunt_sim::g_sim == nullptr)
			errx(1, "alloc failed");

		if (OK != hunt_sim::g_sim->start()) {
			delete hunt_sim::g_sim;
			hunt_sim::g_sim = nullptr;
			err(1, "start failed");
		}

		/* avoid memory fragmentation by not exiting start handler until the task has fully started */
		while (hunt_sim::g_sim == nullptr || !hunt_sim::g_sim->task_running()) {
			usleep(50000);
			printf(".");
			fflush(stdout);
		}
		printf("\n");

		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		if (hunt_sim::g_sim == nullptr)
			errx(1, "not running");

		delete hunt_sim::g_sim;
		hunt_sim::g_sim = nullptr;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (hunt_sim::g_sim) {
			errx(0, "running");

		} else {
			errx(1, "not running");
		}
	}

	warnx("unrecognized command");
	return 1;
}

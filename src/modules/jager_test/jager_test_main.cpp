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
 * @file jager_test_main.cpp
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

#include <platforms/px4_defines.h>

/**
 * Odroid simulator
 */
extern "C" __EXPORT int jager_test_main(int argc, char *argv[]);

class OdroidSimulator
{
public:
	/**
	 * Constructor
	 */
	OdroidSimulator();

	/**
	 * Destructor, also kills the main task.
	 */
	~OdroidSimulator();

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

	/* subscriptions */
	int _hunt_state_sub;	/**< subscription to the hunt state */
	int _hunt_result_sub;	/**< subscription to the hunt result */
	int _local_pos_sub;		/**< subscription to the local position data */
	int _params_sub;		/**< subscription to parameters */

	struct hunt_state_s					_hunt_state;	/**< struct for hunt state */
	struct hunt_result_s 				_hunt_result;	/**< struct for hunt result */
	struct vehicle_local_position_s		_local_pos;		/**< struct for local position */

	orb_advert_t	_hunt_cmd_pub;	/**< going to publish the hunt cmds */

	struct tracking_cmd_s	_hunt_cmd;	/**< the cmd that will be published */

	/* hunt stuff */
	int _cmd_id;					/**< the current tracking cmd id */
	hrt_abstime _last_cmd_time;		/**< timestamp the last cmd was sent */
	float _initial_angle;				/**< the heading when the cmd was send in rads (-pi .. pi) */
	float _final_angle;

	struct {
		param_t scenario;
		param_t move_dist;
		param_t initial_rot_dir;
	}		_params_handles;		/**< handles for interesting parameters */

	struct {
		int scenario;
		float move_dist;
		int initial_rot_dir;
	}		_params;



	/* testing */
	float _test_north[4];
	float _test_east[4];
	float _test_rotate_distance[4];		/**< the angle through which to let it turn before commanding new direction (in degs) */
	int _test_current_rotate_direction;

	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

	/**
	 * Check for changes in vehicle control mode.
	 */
	void		hunt_state_poll();

	/**
	 * Check for changes in manual inputs.
	 */
	void		hunt_result_poll();

	/**
	 * Check for airspeed updates.
	 */
	void		publish_hunt_cmd();

	/**
	 * Set the next command as a move cmd.
	 */
	void		set_next_move_cmd();

	/**
	 * Set the next command as a rotate cmd.
	 */
	void		set_next_rotate_cmd();

	/**
	 * cmd a change in the rotation direction.
	 */
	void		change_direction();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main simulator loop
	 */
	void		task_main();

};

namespace simulator
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

OdroidSimulator	*g_sim = nullptr;
}

OdroidSimulator::OdroidSimulator() :

	_task_should_exit(false),
	_task_running(false),
	_main_task(-1),

/* subscriptions */
	_hunt_state_sub(-1),
	_hunt_result_sub(-1),
	_local_pos_sub(-1),
	_params_sub(-1),

/* publications */
	_hunt_cmd_pub(-1),

/* hunt stuff */
	_cmd_id(-1),
	_last_cmd_time(-1),
	_initial_angle(-1),
	_final_angle(-1),

/* testing */
	_test_north{0},
	_test_east{0},
	_test_rotate_distance{0},
	_test_current_rotate_direction(0)
{
	/* safely initialize structs */
	_hunt_state = {};
	_hunt_result = {};
	_local_pos = {};
	_hunt_cmd = {};

	_params_handles.scenario		= param_find("OSIM_SCENARIO");
	_params_handles.move_dist		= param_find("OSIM_MOVE_DIST");
	_params_handles.initial_rot_dir = param_find("OSIM_INIT_DIR");
	parameters_update();


	// TESTING
	_test_north[0] = -_params.move_dist;
	_test_north[2] = _params.move_dist;

	_test_east[1] = _params.move_dist;
	_test_east[3] = -_params.move_dist;

	_test_rotate_distance[0] = 45.0;
	_test_rotate_distance[1] = 90.0;
	_test_rotate_distance[2] = 180.0;
	_test_rotate_distance[3] = 270.0;

	// TODO: define parameters for help and get them

}

OdroidSimulator::~OdroidSimulator()
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

	simulator::g_sim = nullptr;
}

int
OdroidSimulator::parameters_update()
{
	// TODO: update any desired parameters

	param_get(_params_handles.scenario, &_params.scenario);
	param_get(_params_handles.move_dist, &_params.move_dist);
	param_get(_params_handles.initial_rot_dir, &_params.initial_rot_dir);

	return OK;
}


void
OdroidSimulator::hunt_state_poll()
{
	/* Check if hunt state has changed */
	bool hunt_state_updated;
	orb_check(_hunt_state_sub, &hunt_state_updated);

	if (hunt_state_updated) {
		orb_copy(ORB_ID(hunt_state), _hunt_state_sub, &_hunt_state);
	}
}

void
OdroidSimulator::hunt_result_poll()
{
	/* Check if hunt result has changed */
	bool hunt_result_updated;
	orb_check(_hunt_result_sub, &hunt_result_updated);

	if (hunt_result_updated) {
		orb_copy(ORB_ID(hunt_result), _hunt_result_sub, &_hunt_result);
	}
}

void
OdroidSimulator::publish_hunt_cmd()
{
	/* publish or advertise as needed the hunt cmd */
	if (_hunt_cmd_pub > 0) {
		orb_publish(ORB_ID(tracking_cmd), _hunt_cmd_pub, &_hunt_cmd);
	} else {
		_hunt_cmd_pub = orb_advertise(ORB_ID(tracking_cmd), &_hunt_cmd);
	}
}

void
OdroidSimulator::set_next_move_cmd()
{
	// make sure at least 10 seconds have elapsed before sending the next move cmd
	if (hrt_absolute_time() - _last_cmd_time >= 1e7) {

		// increase cmd id
		_cmd_id++;

		// wrap around instead of trying to call invalid element
		if (_cmd_id >= 4) {
			_cmd_id = 0;
		}

		_hunt_cmd.timestamp = hrt_absolute_time();
		_hunt_cmd.cmd_id = _cmd_id;
		_hunt_cmd.cmd_type = HUNT_CMD_TRAVEL;
		_hunt_cmd.north = _test_north[_cmd_id];
		_hunt_cmd.east = _test_east[_cmd_id];
		_hunt_cmd.yaw_angle = 0.0;
		_hunt_cmd.altitude = 60.0;

		/* update time of last cmd */
		_last_cmd_time = hrt_absolute_time();
	}
}

void
OdroidSimulator::set_next_rotate_cmd()
{
	// increase cmd id
	_cmd_id++;

	// wrap around instead of trying to call invalid element
	if (_cmd_id >= 4) {
		_cmd_id = 0;
	}

	// set the initial rotation direction as user desired
	_test_current_rotate_direction = _params.initial_rot_dir;

	// double checking for safety as 0 will not be understood
	if (_test_current_rotate_direction == 0) {
		_test_current_rotate_direction = -1;
	}

	_hunt_cmd.timestamp = hrt_absolute_time();
	_hunt_cmd.cmd_id = _cmd_id;
	_hunt_cmd.cmd_type = HUNT_CMD_ROTATE;
	_hunt_cmd.yaw_angle = _test_current_rotate_direction;
	_hunt_cmd.altitude = 60.0;

	_last_cmd_time = hrt_absolute_time();
	_initial_angle = _local_pos.yaw;		// best guess this is -pi to pi
	_final_angle = _wrap_2pi(_initial_angle + math::radians(_test_rotate_distance[_cmd_id]));


}

void
OdroidSimulator::change_direction()
{

	// change the rotation direction
	_test_current_rotate_direction = -_test_current_rotate_direction;

	_hunt_cmd.timestamp = hrt_absolute_time();
	_hunt_cmd.cmd_id = _cmd_id;
	_hunt_cmd.cmd_type = HUNT_CMD_ROTATE;
	_hunt_cmd.yaw_angle = _test_current_rotate_direction;
	_hunt_cmd.altitude = 60.0;

	_last_cmd_time = hrt_absolute_time();
}




void
OdroidSimulator::task_main_trampoline(int argc, char *argv[])
{
	simulator::g_sim->task_main();
}

void
OdroidSimulator::task_main()
{

	/* inform about start */
	warnx("Initializing..");
	fflush(stdout);

	/*
	 * do subscriptions
	 */
	_hunt_state_sub = orb_subscribe(ORB_ID(hunt_state));
	_hunt_result_sub = orb_subscribe(ORB_ID(hunt_result));

	_hunt_state.hunt_mode_state = HUNT_STATE_OFF;

	/* get an initial update for all sensor and status data */
	hunt_state_poll();
	hunt_result_poll();

	/* wakeup source(s) */
	struct pollfd fds[2];

	/* Setup of loop */
	fds[0].fd = _params_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _local_pos_sub;
	fds[1].events = POLLIN;

	_task_running = true;

	while (!_task_should_exit) {

		static int loop_counter = 0;

		/* wait for up to 500ms for data */
		int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit, etc. */
		if (pret == 0)
			continue;

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		/* only update parameters if they changed */
		if (fds[0].revents & POLLIN) {
			/* read from param to clear updated flag */
			struct parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), _params_sub, &update);

			/* update parameters from storage */
			parameters_update();
		}

		/* only run simulator if local position has changed changed */
		if (fds[1].revents & POLLIN) {

			// poll for state and result updates
			hunt_result_poll();
			hunt_result_poll();


			// retrieve the local position information
			orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);

			// TODO: run simulation logic

			if (_hunt_state.hunt_mode_state == HUNT_STATE_OFF) {
				// not running in hunt mode yet
				continue;
			}

			if (_hunt_state.hunt_mode_state == HUNT_STATE_MOVE) {
				/* for now we don't want to interrupt mid move */
				continue;
			}

			if (_hunt_state.hunt_mode_state == HUNT_STATE_WAIT) {
				// TODO: create the next command to send

				if (_params.scenario == 0) { // run the move scenario
					set_next_move_cmd();
				} else if (_params.scenario == 1) { // run the rotate scenario
					set_next_rotate_cmd();
				} else {
					continue;
				}

				/* publish the cmd */
				publish_hunt_cmd();
			}

			if (_hunt_state.hunt_mode_state == HUNT_STATE_ROTATE) {
				// TODO: this is where we can switch up directions

				/* if within 5 degrees of target switch angle */
				if (abs(_local_pos.yaw - _final_angle) <= math::radians(5.0)) {

					/* set the cmd to change rotation direction */
					change_direction();

					/* publish the cmd */
					publish_hunt_cmd();
				}

			}
		}

		loop_counter++;
	}

	warnx("exiting.\n");

	_main_task = -1;
	_task_running = false;
	_exit(0);
}

int
OdroidSimulator::start()
{
	ASSERT(_main_task == -1);

	/* start the task */
	_main_task = task_spawn_cmd("jager_test",
				       SCHED_DEFAULT,
				       SCHED_PRIORITY_MAX - 5,
				       2048,
				       (main_t)&OdroidSimulator::task_main_trampoline,
				       nullptr);

	if (_main_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int jager_test_main(int argc, char *argv[])
{
	if (argc < 1)
		errx(1, "usage: jager_test {start|stop|status}");

	if (!strcmp(argv[1], "start")) {

		if (simulator::g_sim != nullptr)
			errx(1, "already running");

		simulator::g_sim = new OdroidSimulator;

		if (simulator::g_sim == nullptr)
			errx(1, "alloc failed");

		if (OK != simulator::g_sim->start()) {
			delete simulator::g_sim;
			simulator::g_sim = nullptr;
			err(1, "start failed");
		}

		/* avoid memory fragmentation by not exiting start handler until the task has fully started */
		while (simulator::g_sim == nullptr || !simulator::g_sim->task_running()) {
			usleep(50000);
			printf(".");
			fflush(stdout);
		}
		printf("\n");

		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		if (simulator::g_sim == nullptr)
			errx(1, "not running");

		delete simulator::g_sim;
		simulator::g_sim = nullptr;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (simulator::g_sim) {
			errx(0, "running");

		} else {
			errx(1, "not running");
		}
	}

	warnx("unrecognized command");
	return 1;
}

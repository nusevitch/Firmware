/**
 * @file everything_is_awesome.cpp
 * Simple application to be able to view the apnt status topic
 * from nsh shell.
 *
 * @author Adrien Perkins <adrienp@stanford.edu>
 * July 2014
 * Stanford University GPS Lab
 */


#include <nuttx/config.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

#include <uORB/uORB.h>
#include <uORB/topics/apnt_gps_status.h>
#include <uORB/topics/tracking_cmd.h>
#include <uORB/topics/tracking_status.h>
#include <uORB/topics/hunt_state.h>

__EXPORT int everything_is_awesome_main(int argc, char *argv[]);

int everything_is_awesome_main(int argc, char *argv[]) {
	printf("Watching APNT GPS status and Tracking CMDs\n");

	// just to count how many messages it received
	int count;

	/* subscribe to sensor_combined topic */
	int apnt_sub_fd = orb_subscribe(ORB_ID(apnt_gps_status));
	int tracking_cmd_sub_fd = orb_subscribe(ORB_ID(tracking_cmd));
	int tracking_status_sub_fd = orb_subscribe(ORB_ID(tracking_status));

	/* advertise hunt state topic */
	struct hunt_state_s hunt_state;
	memset(&hunt_state, 0, sizeof(hunt_state));
	int hunt_state_pub = orb_advertise(ORB_ID(hunt_state), &hunt_state);

	// only listen to the status once a second
	orb_set_interval(apnt_sub_fd, 1000);
	orb_set_interval(tracking_status_sub_fd, 1000);


	struct pollfd fds[] = {
			{ .fd = apnt_sub_fd,   .events = POLLIN },
			{ .fd = tracking_cmd_sub_fd,   .events = POLLIN },
			{ .fd = tracking_status_sub_fd,   .events = POLLIN },
	};

	int error_counter = 0;

	while (count < 10) {

		// wait 10 seconds
		int poll_ret = poll(fds, 1, 10000);

		if (poll_ret == 0) {
			printf("[everything_is_awesome] no apnt gps status change in last 5 seconds\n");
		} else if (poll_ret < 0) {
			if (error_counter < 10 || error_counter % 50 == 0) {
				printf("[everything_is_awesome] this means big problems\n");
			}
			error_counter++;
		} else {
			if (fds[0].revents & POLLIN) {
				struct apnt_gps_status_s apnt_gps_status;
				orb_copy(ORB_ID(apnt_gps_status), apnt_sub_fd, &apnt_gps_status);
				printf("[everything_is_awesome] APNT GPS Status: \n"
						"\t%llu\n\t%u\t%u\t%u\n", apnt_gps_status.timestamp,
						apnt_gps_status.prn[0], apnt_gps_status.elevation[0], apnt_gps_status.snr[0]);

			}
			if (fds[1].revents & POLLIN) {
				// just print out the tracking cmd for now
				struct tracking_cmd_s tracking_cmd;
				orb_copy(ORB_ID(tracking_cmd), tracking_cmd_sub_fd, &tracking_cmd);
				printf("[everything_is_awesome] TRACKING CMD: \n"
						"\t%llu\n\t%u\n", tracking_cmd.timestamp, tracking_cmd.cmd_id);
			}
			if (fds[2].revents & POLLIN) {
				// print out status
				struct tracking_status_s tracking_status;
				orb_copy(ORB_ID(tracking_status), tracking_status_sub_fd, &tracking_status);
				printf("[everything_is_awsome TRACKING STATUS: \n"
						"\t%llu\n\t%u\n", tracking_status.timestamp, tracking_status.computer_status);

				hunt_state.timestamp = tracking_status.timestamp;

				if (hunt_state.hunt_mode_state != HUNT_STATE_WAIT) {
					hunt_state.hunt_mode_state = HUNT_STATE_MOVE;
				} else {
					hunt_state.hunt_mode_state = HUNT_STATE_WAIT;
				}

				orb_publish(ORB_ID(hunt_state), hunt_state_pub, &hunt_state);

			}
		}

		// increase the counter
		count++;
	}


	return 0;
}




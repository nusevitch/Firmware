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

__EXPORT int everything_is_awesome_main(int argc, char *argv[]);

int everything_is_awesome_main(int argc, char *argv[]) {
	printf("Watching APNT status\n");

	/* subscribe to sensor_combined topic */
	int apnt_sub_fd = orb_subscribe(ORB_ID(apnt_gps_status));

	// only listen to the status once a second
	orb_set_interval(apnt_sub_fd, 1000);


	struct pollfd fds[] = {
			{ .fd = apnt_sub_fd,   .events = POLLIN },
	};

	int error_counter = 0;

	while (1) {

		int poll_ret = poll(fds, 1, 5000);

		if (poll_ret == 0) {
			printf("[everything_is_awesome] no apnt status change in last 5 seconds\n");
		} else if (poll_ret < 0) {
			if (error_counter < 10 || error_counter % 50 == 0) {
				printf("[everything_is_awesome] this means big problems\n");
			}
			error_counter++;
		} else {
			if (fds[0].revents & POLLIN) {
				struct apnt_gps_status_s apnt_status;
				orb_copy(ORB_ID(apnt_gps_status), apnt_sub_fd, &apnt_status);
				printf("[everything_is_awesome] APNT Status: \n"
						"\t%llu\n\t%u\t%u\t%u\n", apnt_status.timestamp,
						apnt_status.prn[0], apnt_status.elevation[0], apnt_status.snr[0]);
			}
		}
	}


	return 0;
}




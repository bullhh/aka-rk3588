/* SPDX-License-Identifier: Apache-2.0 */

#ifndef ROBOT_IVC_TRANSPORT_H_
#define ROBOT_IVC_TRANSPORT_H_

#include <axivc/axivc.h>

#include <stddef.h>

struct robot_ivc {
	axivc_channel_t *channel;
};

/* Wait until Starry VM 1 publishes the robot perception channel. */
int robot_ivc_subscribe(struct robot_ivc *ivc);

/* Return 0 for one message, -EAGAIN when the ring is currently empty. */
int robot_ivc_try_receive(struct robot_ivc *ivc, void *payload, size_t capacity,
			  size_t *length);

int robot_ivc_close(struct robot_ivc *ivc);

#endif /* ROBOT_IVC_TRANSPORT_H_ */

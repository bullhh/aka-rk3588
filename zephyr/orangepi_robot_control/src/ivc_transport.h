/* SPDX-License-Identifier: Apache-2.0 */

#ifndef ROBOT_IVC_TRANSPORT_H_
#define ROBOT_IVC_TRANSPORT_H_

#include <stddef.h>
#include <stdint.h>

struct robot_ivc {
	void *region;
	size_t size;
};

/* Wait until Starry VM 1 publishes the robot perception channel. */
int robot_ivc_subscribe(struct robot_ivc *ivc);

/* Return 0 for one message, -EAGAIN when the ring is currently empty. */
int robot_ivc_try_receive(struct robot_ivc *ivc, void *payload, size_t capacity,
			  size_t *length, uint64_t *sequence);

#endif /* ROBOT_IVC_TRANSPORT_H_ */

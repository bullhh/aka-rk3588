/* SPDX-License-Identifier: Apache-2.0 */

#include "ivc_transport.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <errno.h>
#include <stdint.h>

#define IVC_PUBLISHER_VM_ID 1U
#define IVC_CHANNEL_KEY 0x49564301U
#define IVC_SUBSCRIBE_RETRY_MS 100U

int robot_ivc_subscribe(struct robot_ivc *ivc)
{
	uint64_t attempts = 0U;

	if (ivc == NULL) {
		return -EINVAL;
	}

	for (;;) {
		++attempts;
		ivc->channel = axivc_subscribe(IVC_PUBLISHER_VM_ID,
						 IVC_CHANNEL_KEY, 0);
		if (ivc->channel != NULL) {
			printk("ZEPHYR_IVC_READY publisher=%u key=0x%x attempts=%llu "
			       "transport=ivc-sdk\n",
			       IVC_PUBLISHER_VM_ID, IVC_CHANNEL_KEY, attempts);
			return 0;
		}

		if (attempts == 1U || attempts % 30U == 0U) {
			printk("ZEPHYR_IVC_WAIT attempts=%llu publisher=%u key=0x%x\n",
			       attempts, IVC_PUBLISHER_VM_ID, IVC_CHANNEL_KEY);
		}
		k_msleep(IVC_SUBSCRIBE_RETRY_MS);
	}
}

int robot_ivc_try_receive(struct robot_ivc *ivc, void *payload, size_t capacity,
			  size_t *length)
{
	int status;

	if (ivc == NULL || ivc->channel == NULL || payload == NULL ||
	    length == NULL) {
		return -EINVAL;
	}

	status = axivc_recv(ivc->channel, payload, capacity, length, 0);
	/* The current SDK converts an empty non-blocking receive to ETIMEDOUT.
	 * Preserve the robot controller's established try-receive contract until
	 * the SDK timeout API is standardized in a separate change.
	 */
	if (status == -ETIMEDOUT) {
		return -EAGAIN;
	}
	return status;
}

int robot_ivc_close(struct robot_ivc *ivc)
{
	int status;

	if (ivc == NULL || ivc->channel == NULL) {
		return 0;
	}
	status = axivc_close(ivc->channel);
	ivc->channel = NULL;
	return status;
}

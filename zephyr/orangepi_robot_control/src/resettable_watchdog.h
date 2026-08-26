/* SPDX-License-Identifier: Apache-2.0 */

#ifndef RESETTABLE_WATCHDOG_H_
#define RESETTABLE_WATCHDOG_H_

#include <zephyr/kernel.h>

#include <stdint.h>

typedef void (*resettable_watchdog_handler_t)(void *context);

struct resettable_watchdog {
	struct k_timer timer;
	struct k_work work;
	resettable_watchdog_handler_t handler;
	void *context;
};

void resettable_watchdog_init(struct resettable_watchdog *watchdog,
			      resettable_watchdog_handler_t handler,
			      void *context);
void resettable_watchdog_restart(struct resettable_watchdog *watchdog,
				 uint32_t timeout_ms);
void resettable_watchdog_stop(struct resettable_watchdog *watchdog);

#endif /* RESETTABLE_WATCHDOG_H_ */

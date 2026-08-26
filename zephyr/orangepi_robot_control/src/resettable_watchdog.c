/* SPDX-License-Identifier: Apache-2.0 */

#include "resettable_watchdog.h"

#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(robot_watchdog_diag, LOG_LEVEL_DBG);

static void watchdog_work_handler(struct k_work *work)
{
	struct resettable_watchdog *watchdog =
		CONTAINER_OF(work, struct resettable_watchdog, work);
	const int64_t started_ms = k_uptime_get();

	LOG_DBG("ZEPHYR_DIAG component=watchdog event=work-enter uptime_ms=%lld",
		started_ms);
	watchdog->handler(watchdog->context);
	LOG_DBG("ZEPHYR_DIAG component=watchdog event=work-exit elapsed_ms=%lld",
		k_uptime_get() - started_ms);
}

static void watchdog_timer_expiry(struct k_timer *timer)
{
	struct resettable_watchdog *watchdog = k_timer_user_data_get(timer);

	/* UART and controller state changes are not IRQ-safe.  Move expiry
	 * handling to the system workqueue's thread context.
	 */
	const int submit_result = k_work_submit(&watchdog->work);
	LOG_DBG("ZEPHYR_DIAG component=watchdog event=timer-expiry submit=%d "
		"uptime_ms=%lld",
		submit_result, k_uptime_get());
}

void resettable_watchdog_init(struct resettable_watchdog *watchdog,
			      resettable_watchdog_handler_t handler,
			      void *context)
{
	watchdog->handler = handler;
	watchdog->context = context;
	k_work_init(&watchdog->work, watchdog_work_handler);
	k_timer_init(&watchdog->timer, watchdog_timer_expiry, NULL);
	k_timer_user_data_set(&watchdog->timer, watchdog);
}

void resettable_watchdog_restart(struct resettable_watchdog *watchdog,
				 uint32_t timeout_ms)
{
	k_timer_start(&watchdog->timer, K_MSEC(timeout_ms), K_NO_WAIT);
}

void resettable_watchdog_stop(struct resettable_watchdog *watchdog)
{
	k_timer_stop(&watchdog->timer);
}

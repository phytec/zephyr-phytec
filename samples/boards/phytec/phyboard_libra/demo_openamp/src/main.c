#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/gpio.h>

#include "rpmsg_platform.h"
#include "display.h"

LOG_MODULE_DECLARE(openamp_rsc_table);

K_THREAD_STACK_DEFINE(thread_mng_stack, APP_TASK_STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_tty_stack, APP_TTY_TASK_STACK_SIZE);

static struct k_thread thread_mng_data;
static struct k_thread thread_tty_data;

int main(void)
{
	LOG_INF("Starting application threads!");
	k_thread_create(&thread_mng_data, thread_mng_stack, APP_TASK_STACK_SIZE,
			rpmsg_mng_task,
			NULL, NULL, NULL, K_PRIO_COOP(8), 0, K_NO_WAIT);
	k_thread_create(&thread_tty_data, thread_tty_stack, APP_TTY_TASK_STACK_SIZE,
			app_rpmsg_tty,
			NULL, NULL, NULL, K_PRIO_COOP(7), 0, K_NO_WAIT);

	k_msleep(500);
	init_screen();
	return 0;
}

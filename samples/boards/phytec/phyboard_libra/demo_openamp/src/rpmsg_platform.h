#ifndef RPMSG_PLATFORM_H_
#define RPMSG_PLATFORM_H_

#include <zephyr/kernel.h>
#include <openamp/open_amp.h>
#include <stdbool.h>
#include <stddef.h>

#define APP_TASK_STACK_SIZE     (1024)
/* Add 1024 extra bytes for the TTY task stack for the "tx_buff" buffer. */
#define APP_TTY_TASK_STACK_SIZE (1536)

void rpmsg_tty_set_suspended(bool suspended);
bool rpmsg_tty_is_suspended(void);
void app_rpmsg_tty(void *arg1, void *arg2, void *arg3);
void rpmsg_mng_task(void *arg1, void *arg2, void *arg3);
int rpmsg_tty_send(const void *data, size_t len);

#endif /* RPMSG_PLATFORM_H_ */
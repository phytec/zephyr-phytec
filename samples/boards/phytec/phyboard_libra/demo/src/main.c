#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "ui.h"
#include "cubes.h"

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

	struct nunchuk_data_t actual_nunchuk = read_nunchuk();
	static enum main_state_t state = AWAKE;
	static enum main_state_t previous_state = AWAKE;
	static bool entry=false;
	while(1){
		actual_nunchuk = read_nunchuk();

		switch(state){
			case AWAKE:
				if (display(actual_nunchuk)==ID_SLEEP) {
					state=SLEEPING;
				};
				break;
			case SLEEPING:
				if(entry){
					int r = init_screen_cubes();
					if (r==-1){
						printk("ERROR INIT SCREEN CUBES");
						return -1;
					}	
				}
				if (cubes_screen(actual_nunchuk)==ID_WAKE){
					state=WAKING;
				}
				break;
			case WAKING:
				ui_resume();
				state = AWAKE;

				break;
			default:
				break;
		}
		entry = !(state==previous_state);
		previous_state=state;
		updateTime();
	}
	return 0;
}
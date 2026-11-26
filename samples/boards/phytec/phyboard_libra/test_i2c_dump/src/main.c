#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/sys_io.h>
#include "nunchuk.h"

int main(void)
{
	printk("COUCOU");
	while (1) {
		read_nunchuk();
		k_msleep(50);
	}

	return 0;
}
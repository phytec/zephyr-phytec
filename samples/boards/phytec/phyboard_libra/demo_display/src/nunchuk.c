// #include <zephyr/kernel.h>
// #include <zephyr/device.h>
// #include <zephyr/input/input_nunchuk.h>
// #include <zephyr/drivers/i2c.h>

// #include "nunchuk.h"

// const struct device *i2c_bus = DEVICE_DT_GET(DT_NODELABEL(i2c8));

// bool init_nunchuk(){
//     uint8_t init_seq_1[2] = {0xf0, 0x55};
//     uint8_t init_seq_2[2] = {0xfb, 0x00};
//     int ret;

//     if (!device_is_ready(i2c_bus)) {
//         printk("Device i2c not found. Aborting sample.");
//         return false;
//     }

//     if (!device_is_ready(i2c_bus)) {
//         printk("Device i2c not found. Aborting sample.");
//         return false;
//     }

//     ret = i2c_write(i2c_bus, init_seq_1, sizeof(init_seq_1), NUNCHUK_ADRESS);
//     if (ret < 0) {
//         printk("I2C write failed.\n");
//         return false;
//     }

//     k_msleep(1);

//     ret = i2c_write(i2c_bus, init_seq_2, sizeof(init_seq_2), NUNCHUK_ADRESS);
//     if (ret < 0) {
//         printk("I2C write failed.\n");
//         return false;
//     }
//     printk("Init OK");
//     return true;
// }

// struct nunchuk_data_t read_nunchuk(){
//     uint8_t value = 0;
//     uint8_t buffer[6];
//     bool z_pressed;
//     bool c_pressed;
//     struct nunchuk_data_t nunchuk_input;

//     i2c_write(i2c_bus, &value, 1, NUNCHUK_ADRESS);

//     k_msleep(10);

//     i2c_read(i2c_bus, buffer, 6, NUNCHUK_ADRESS);

//     z_pressed = buffer[INPUT_NUNCHUK_BTN_C_Z] & 1;
//     c_pressed = buffer[INPUT_NUNCHUK_BTN_C_Z] & 2;

//     // printk("Z : %d\n", z_pressed);
//     // printk("C : %d\n", c_pressed);
//     // printk("X joystick: %d\n",buffer[INPUT_NUNCHUK_STICK_X]);
//     // printk("Y joystick: %d\n\n",buffer[INPUT_NUNCHUK_STICK_Y]);
//     // printk("ACC X: %d\n\n",buffer[INPUT_NUNCHUK_ACC_X]);
//     // printk("ACC Y: %d\n\n",buffer[INPUT_NUNCHUK_ACC_Y]);
//     // printk("ACC Z: %d\n\n",buffer[INPUT_NUNCHUK_ACC_Z]);

//     nunchuk_input.joystick_x = buffer[INPUT_NUNCHUK_STICK_X];
//     nunchuk_input.joystick_y = buffer[INPUT_NUNCHUK_STICK_Y];
//     nunchuk_input.accel_x = buffer[INPUT_NUNCHUK_ACC_X];
//     nunchuk_input.accel_y = buffer[INPUT_NUNCHUK_ACC_Y];
//     nunchuk_input.accel_z = buffer[INPUT_NUNCHUK_ACC_Z];
//     nunchuk_input.btn_c = c_pressed;
//     nunchuk_input.btn_z = z_pressed;

//     return nunchuk_input;
// }
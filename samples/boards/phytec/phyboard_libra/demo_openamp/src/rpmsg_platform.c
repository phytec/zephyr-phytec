/*
 * Copyright (c) 2026, PHYTEC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/drivers/ipm.h>
#include <zephyr/logging/log.h>

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include <metal/sys.h>
#include <metal/io.h>
#include <resource_table.h>
#include <addr_translation.h>

#ifdef CONFIG_SHELL_BACKEND_RPMSG
#include <zephyr/shell/shell_rpmsg.h>
#endif

#include "rpmsg_platform.h"

LOG_MODULE_REGISTER(openamp_rsc_table);

#define SHM_DEVICE_NAME      "shm"

#define TTY_MAX_MSG_LEN      128
#define TTY_RX_QUEUE_LEN     4

#if !DT_HAS_CHOSEN(zephyr_ipc_shm)
#error "Sample requires definition of shared memory for rpmsg"
#endif

#if CONFIG_IPM_MAX_DATA_SIZE > 0
#define IPM_SEND(dev, w, id, d, s) ipm_send(dev, w, id, d, s)
#else
#define IPM_SEND(dev, w, id, d, s) ipm_send(dev, w, id, NULL, 0)
#endif

#define SHM_NODE       DT_CHOSEN(zephyr_ipc_shm)
#define SHM_START_ADDR DT_REG_ADDR(SHM_NODE)
#define SHM_SIZE       DT_REG_SIZE(SHM_NODE)

static const struct device *const ipm_handle =
    DEVICE_DT_GET(DT_CHOSEN(zephyr_ipc));

static metal_phys_addr_t shm_physmap = SHM_START_ADDR;
static metal_phys_addr_t rsc_tab_physmap;

static struct metal_io_region shm_io_data;
static struct metal_io_region rsc_io_data;

static struct metal_io_region *shm_io = &shm_io_data;
static struct metal_io_region *rsc_io = &rsc_io_data;

static struct rpmsg_virtio_device rvdev;

static void *rsc_table;
static struct rpmsg_device *rpdev;

static struct rpmsg_endpoint tty_ept;

static bool tty_ready;

/* Serializes sends and changes of the suspend state. */
K_MUTEX_DEFINE(tty_send_lock);

/* Non-zero while application transmissions are blocked. */
static atomic_t tty_suspended;

static K_SEM_DEFINE(data_sem, 0, 1);

/* Released once the RPMsg device is ready, so the TTY thread can start. */
static K_SEM_DEFINE(data_tty_sem, 0, 1);

/*
 * Messages are copied out of the RPMsg callback so the TTY thread never
 * holds a pointer to a shared OpenAMP buffer.
 */
struct tty_rx_msg {
    size_t len;
    uint8_t data[TTY_MAX_MSG_LEN];
};

K_MSGQ_DEFINE(tty_rx_queue,
              sizeof(struct tty_rx_msg),
              TTY_RX_QUEUE_LEN,
              4);

void rpmsg_tty_set_suspended(bool suspended)
{
    /* Taking the lock waits for any send in progress to complete. */
    k_mutex_lock(&tty_send_lock, K_FOREVER);

    atomic_set(&tty_suspended, suspended ? 1 : 0);

    k_mutex_unlock(&tty_send_lock);

    LOG_INF("RPMsg application TX: %s",
            suspended ? "SUSPENDED" : "ACTIVE");
}

bool rpmsg_tty_is_suspended(void)
{
    return atomic_get(&tty_suspended) != 0;
}

/*
 * When allow_suspended is true, the message is sent even during SUSPEND.
 * This is only meant for the SUSPEND acknowledgment.
 */
static int rpmsg_tty_send_internal(const void *data,
                                   size_t len,
                                   bool allow_suspended)
{
    int ret;

    if (data == NULL && len != 0U) {
        return -EINVAL;
    }

    k_mutex_lock(&tty_send_lock, K_FOREVER);

    if (!allow_suspended && atomic_get(&tty_suspended) != 0) {
        ret = -EAGAIN;
    } else if (!tty_ready ||
               tty_ept.dest_addr == RPMSG_ADDR_ANY) {
        ret = -ENOTCONN;
    } else {
        ret = rpmsg_send(&tty_ept, data, len);
    }

    k_mutex_unlock(&tty_send_lock);

    return ret;
}

/* Public send API: refused with -EAGAIN while suspended. */
int rpmsg_tty_send(const void *data, size_t len)
{
    return rpmsg_tty_send_internal(data, len, false);
}

static void platform_ipm_callback(const struct device *dev,
                                  void *context,
                                  uint32_t id,
                                  volatile void *data)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(context);
    ARG_UNUSED(data);

    LOG_DBG("%s: notification received, id=%u", __func__, id);

    k_sem_give(&data_sem);
}

static int rpmsg_recv_tty_callback(struct rpmsg_endpoint *ept,
                                   void *data,
                                   size_t len,
                                   uint32_t src,
                                   void *priv)
{
    struct tty_rx_msg msg;

    ARG_UNUSED(ept);
    ARG_UNUSED(src);
    ARG_UNUSED(priv);

    if (len > sizeof(msg.data)) {
        LOG_WRN("TTY message too long: %u bytes", (unsigned int)len);
        return RPMSG_SUCCESS;
    }

    msg.len = len;

    if (len > 0U) {
        memcpy(msg.data, data, len);
    }

    if (k_msgq_put(&tty_rx_queue, &msg, K_NO_WAIT) != 0) {
        LOG_WRN("TTY queue full: message dropped");
    }

    return RPMSG_SUCCESS;
}

static void receive_message(void)
{
    int status = k_sem_take(&data_sem, K_FOREVER);

    if (status == 0) {
        rproc_virtio_notified(rvdev.vdev, VRING1_ID);
    }
}

static void new_service_cb(struct rpmsg_device *rdev,
                           const char *name,
                           uint32_t src)
{
    ARG_UNUSED(rdev);
    ARG_UNUSED(src);

    LOG_ERR("%s: unexpected service: %s", __func__, name);
}

int mailbox_notify(void *priv, uint32_t id)
{
    ARG_UNUSED(priv);

    LOG_DBG("%s: notification id=%u", __func__, id);

    IPM_SEND(ipm_handle, 0, id, &id, 4);

    return 0;
}

int platform_init(void)
{
    int rsc_size;
    struct metal_init_params metal_params = METAL_INIT_DEFAULTS;
    int status;

    status = metal_init(&metal_params);

    if (status) {
        LOG_ERR("metal_init failed: %d", status);
        return -1;
    }

    metal_io_init(shm_io,
                  (void *)SHM_START_ADDR,
                  &shm_physmap,
                  SHM_SIZE,
                  -1,
                  0,
                  addr_translation_get_ops(shm_physmap));

    rsc_table_get(&rsc_table, &rsc_size);

    rsc_tab_physmap = (uintptr_t)rsc_table;

    metal_io_init(rsc_io,
                  rsc_table,
                  &rsc_tab_physmap,
                  rsc_size,
                  -1,
                  0,
                  NULL);

    if (!device_is_ready(ipm_handle)) {
        LOG_ERR("IPM device is not ready");
        metal_finish();
        return -1;
    }

    ipm_register_callback(ipm_handle, platform_ipm_callback, NULL);

    status = ipm_set_enabled(ipm_handle, 1);

    if (status) {
        LOG_ERR("ipm_set_enabled failed: %d", status);
        metal_finish();
        return -1;
    }

    return 0;
}

struct rpmsg_device *
platform_create_rpmsg_vdev(unsigned int vdev_index,
                           unsigned int role,
                           void (*rst_cb)(struct virtio_device *vdev),
                           rpmsg_ns_bind_cb ns_cb)
{
    struct fw_rsc_vdev_vring *vring_rsc;
    struct virtio_device *vdev;
    int ret;

    ARG_UNUSED(vdev_index);
    ARG_UNUSED(role);
    ARG_UNUSED(rst_cb);

    vdev = rproc_virtio_create_vdev(VIRTIO_DEV_DEVICE,
                                    VDEV_ID,
                                    rsc_table_to_vdev(rsc_table),
                                    rsc_io,
                                    NULL,
                                    mailbox_notify,
                                    NULL);

    if (!vdev) {
        LOG_ERR("Failed to create vdev");
        return NULL;
    }

    /* Block until the Linux side has finished its initialization. */
    rproc_virtio_wait_remote_ready(vdev);

    vring_rsc = rsc_table_get_vring0(rsc_table);

    ret = rproc_virtio_init_vring(vdev,
                                  0,
                                  vring_rsc->notifyid,
                                  (void *)vring_rsc->da,
                                  rsc_io,
                                  vring_rsc->num,
                                  vring_rsc->align);

    if (ret) {
        LOG_ERR("Failed to initialize vring 0: %d", ret);
        goto failed;
    }

    vring_rsc = rsc_table_get_vring1(rsc_table);

    ret = rproc_virtio_init_vring(vdev,
                                  1,
                                  vring_rsc->notifyid,
                                  (void *)vring_rsc->da,
                                  rsc_io,
                                  vring_rsc->num,
                                  vring_rsc->align);

    if (ret) {
        LOG_ERR("Failed to initialize vring 1: %d", ret);
        goto failed;
    }

    ret = rpmsg_init_vdev(&rvdev, vdev, ns_cb, shm_io, NULL);

    if (ret) {
        LOG_ERR("rpmsg_init_vdev failed: %d", ret);
        goto failed;
    }

    return rpmsg_virtio_get_rpmsg_device(&rvdev);

failed:
    rproc_virtio_remove_vdev(vdev);

    return NULL;
}

static void process_tty_message(const struct tty_rx_msg *rx)
{
    char cmd[TTY_MAX_MSG_LEN + 1];
    size_t cmd_len = rx->len;
    int ret;

    if (cmd_len > TTY_MAX_MSG_LEN) {
        cmd_len = TTY_MAX_MSG_LEN;
    }

    memcpy(cmd, rx->data, cmd_len);
    cmd[cmd_len] = '\0';

    /* Strip the trailing LF/CR added by echo on Linux. */
    while (cmd_len > 0U &&
           (cmd[cmd_len - 1U] == '\n' ||
            cmd[cmd_len - 1U] == '\r')) {
        cmd[--cmd_len] = '\0';
    }

    LOG_INF("[Linux TTY] message received: %s", cmd);

    if (strcmp(cmd, "SUSPEND") == 0) {
        if (rpmsg_tty_is_suspended()) {
            LOG_INF("Already suspended: no reply sent");
            return;
        }

        /* Block application sends before acknowledging. */
        rpmsg_tty_set_suspended(true);

        static const char ack[] = "Go to sleep...\n";

        /* The acknowledgment is the only message allowed while suspended. */
        ret = rpmsg_tty_send_internal(ack, sizeof(ack) - 1U, true);

        if (ret < 0) {
            LOG_ERR("SUSPEND ACK failed: %d", ret);
        }

        return;
    }

    if (strcmp(cmd, "RESUME") == 0) {
        rpmsg_tty_set_suspended(false);

        static const char ack[] = "Zephyr resumed\n";

        ret = rpmsg_tty_send(ack, sizeof(ack) - 1U);

        if (ret < 0) {
            LOG_ERR("RESUME ACK failed: %d", ret);
        }

        return;
    }

    if (rpmsg_tty_is_suspended()) {
        LOG_DBG("Message ignored while suspended");
        return;
    }

    {
        char tx_buff[TTY_MAX_MSG_LEN + 16];
        int prefix_len;

        prefix_len = snprintf(tx_buff,
                              sizeof(tx_buff),
                              "TTY 0x%04x: ",
                              (unsigned int)tty_ept.addr);

        if (prefix_len < 0 ||
            (size_t)prefix_len + rx->len > sizeof(tx_buff)) {
            LOG_ERR("Cannot build TTY echo");
            return;
        }

        /* Echo the original bytes, including any trailing newline. */
        memcpy(&tx_buff[prefix_len], rx->data, rx->len);

        ret = rpmsg_tty_send(tx_buff, (size_t)prefix_len + rx->len);

        if (ret < 0) {
            LOG_ERR("TTY echo failed: %d", ret);
        }
    }

    if (strcmp(cmd, "Hello") == 0) {
        static const char response[] = "Hallo !\n";

        ret = rpmsg_tty_send(response, sizeof(response) - 1U);

        if (ret < 0) {
            LOG_ERR("Failed to send Hallo: %d", ret);
        }
    }
}

void app_rpmsg_tty(void *arg1, void *arg2, void *arg3)
{
    int ret;

    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    /* rpdev is set by rpmsg_mng_task(); wait until it is ready. */
    k_sem_take(&data_tty_sem, K_FOREVER);

    LOG_INF("OpenAMP[remote] Linux TTY responder started");

    ret = rpmsg_create_ept(&tty_ept,
                           rpdev,
                           "rpmsg-tty",
                           RPMSG_ADDR_ANY,
                           RPMSG_ADDR_ANY,
                           rpmsg_recv_tty_callback,
                           NULL);

    if (ret) {
        LOG_ERR("[Linux TTY] Cannot create endpoint: %d", ret);
        return;
    }

    k_mutex_lock(&tty_send_lock, K_FOREVER);
    tty_ready = true;
    k_mutex_unlock(&tty_send_lock);

    LOG_INF("[Linux TTY] Endpoint created: addr=0x%x", tty_ept.addr);

    while (tty_ept.addr != RPMSG_ADDR_ANY) {
        struct tty_rx_msg rx;

        ret = k_msgq_get(&tty_rx_queue, &rx, K_FOREVER);

        if (ret == 0) {
            process_tty_message(&rx);
        }
    }

    k_mutex_lock(&tty_send_lock, K_FOREVER);
    tty_ready = false;
    k_mutex_unlock(&tty_send_lock);

    rpmsg_destroy_ept(&tty_ept);

    LOG_INF("OpenAMP Linux TTY responder ended");
}

void rpmsg_mng_task(void *arg1, void *arg2, void *arg3)
{
    int ret;

    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    LOG_INF("OpenAMP[remote] Linux responder demo started");

    ret = platform_init();

    if (ret) {
        LOG_ERR("Platform initialization failed");
        return;
    }

    rpdev = platform_create_rpmsg_vdev(0,
                                       VIRTIO_DEV_DEVICE,
                                       NULL,
                                       new_service_cb);

    if (!rpdev) {
        LOG_ERR("Failed to create RPMsg device");

        ipm_set_enabled(ipm_handle, 0);
        metal_finish();

        return;
    }

#ifdef CONFIG_SHELL_BACKEND_RPMSG
    (void)shell_backend_rpmsg_init_transport(rpdev);
#endif

    k_sem_give(&data_tty_sem);

    /*
     * Keep processing RX notifications at all times, even during SUSPEND:
     * Linux must still be able to send RESUME.
     */
    while (1) {
        receive_message();
    }
}

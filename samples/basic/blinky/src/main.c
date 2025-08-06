#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <stdio.h>
#include <string.h>

#define SLEEP_TIME_MS 1000
#define BUF_SIZE      8

#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

#define SPI0_NODE DT_NODELABEL(spi0)
static const struct device *spi_dev = DEVICE_DT_GET(SPI0_NODE);

static uint8_t huge_tx_buf[BUF_SIZE];
static uint8_t huge_rx_buf[BUF_SIZE];

static struct spi_buf tx_bufs_pool[1];
static struct spi_buf rx_bufs_pool[1];
static struct spi_buf_set tx_bufs;
static struct spi_buf_set rx_bufs;

static struct spi_config spi_cfg = {
    .frequency = 1000000U,
    .operation = SPI_WORD_SET(8)
               | SPI_TRANSFER_MSB
               | SPI_OP_MODE_MASTER
               | SPI_MODE_CPOL
               | SPI_MODE_CPHA,
};

#if CONFIG_SPI_ASYNC

#define ASYNC_STACK_SIZE 512
#define ASYNC_THREAD_PRIO 7

static struct k_poll_signal async_sig = K_POLL_SIGNAL_INITIALIZER(async_sig);
static struct k_poll_event async_evt = K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, &async_sig);
static K_SEM_DEFINE(async_sem, 0, 1);
static K_SEM_DEFINE(start_async, 0, 1);

static int async_result = 1;

void spi_async_wait_thread(void *p1, void *p2, void *p3) {
    struct k_poll_event *evt = p1;
    struct k_sem *result_sem = p2;

    while (1) {
        k_sem_take(&start_async, K_FOREVER);

        // Wait for signal (timeout as needed)
        int pret = k_poll(evt, 1, K_MSEC(2000));
        if (pret != 0) {
            printf("k_poll failed or timed out: %d\n", pret);
            async_result = -1;
        } else {
            async_result = evt->signal->result;
        }
        k_sem_give(result_sem);

        // Reinit for next call
        evt->signal->signaled = 0U;
        evt->state = K_POLL_STATE_NOT_READY;
    }
}

K_THREAD_STACK_DEFINE(async_stack, ASYNC_STACK_SIZE);
struct k_thread async_thread;
k_tid_t async_thread_id;

#endif // CONFIG_SPI_ASYNC

int main(void)
{
    int ret;
    bool led_state = true;

    for (size_t i = 0; i < BUF_SIZE; ++i) {
        huge_tx_buf[i] = i;
    }

    if (!gpio_is_ready_dt(&led)) {
        printf("LED device not ready\n");
        return 0;
    }
    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        printf("LED configure failed\n");
        return 0;
    }

    if (!device_is_ready(spi_dev)) {
        printf("SPI device not ready\n");
        return 0;
    }

    tx_bufs_pool[0].buf = huge_tx_buf;
    tx_bufs_pool[0].len = BUF_SIZE;
    rx_bufs_pool[0].buf = huge_rx_buf;
    rx_bufs_pool[0].len = BUF_SIZE;

    tx_bufs.buffers = tx_bufs_pool;
    tx_bufs.count = 1;
    rx_bufs.buffers = rx_bufs_pool;
    rx_bufs.count = 1;

#if CONFIG_SPI_ASYNC
    // Start the async wait thread (once)
    async_thread_id = k_thread_create(&async_thread, async_stack, ASYNC_STACK_SIZE,
                                      spi_async_wait_thread,
                                      &async_evt, &async_sem, NULL,
                                      ASYNC_THREAD_PRIO, 0, K_NO_WAIT);
#endif

    while (1) {

        ret = gpio_pin_toggle_dt(&led);
        if (ret < 0) {
            printf("LED toggle failed: %d\n", ret);
            continue;
        }
        led_state = !led_state;
        printf("LED state: %s\n", led_state ? "ON" : "OFF");

        memset(huge_rx_buf, 0, BUF_SIZE);

#if CONFIG_SPI_ASYNC
        k_poll_signal_reset(&async_sig);
        // async_sig.result = 0;
        async_evt.signal = &async_sig;
        async_evt.state = K_POLL_STATE_NOT_READY;

        k_sem_give(&start_async);

        ret = spi_transceive_signal(spi_dev, &spi_cfg, &tx_bufs, &rx_bufs, &async_sig);
        printf("spi_transceive_signal returned %d\n", ret);
        printf("DMA TX: ");
        for (int i = 0; i < BUF_SIZE; ++i) printf("%02X ", huge_tx_buf[i]);
        printf("\nDMA RX: ");
        for (int i = 0; i < BUF_SIZE; ++i) printf("%02X ", huge_rx_buf[i]);
        printf("\n");
        if (ret == -EINVAL || ret == -ENOTSUP) {
            printf("Spi config is invalid or not supported for this controller\n");
            break;
        } else if (ret) {
            printf("SPI signal transceive failed, code %d\n", ret);
            continue;
        }
        k_sem_take(&async_sem, K_FOREVER);

        if (async_result != 0) {
            printf("SPI async failed, result: %d\n", async_result);
            continue;
        }
        printf("SPI async operation completed successfully.\n");
#else
        ret = spi_transceive(spi_dev, &spi_cfg, &tx_bufs, &rx_bufs);
        printf("spi_transceive returned %d\n", ret);
        if (ret) {
            printf("SPI transceive failed: %d\n", ret);
            continue;
        }
        printf("SPI operation completed successfully.\n");
#endif

        printf("TX: ");
        for (int i = 0; i < BUF_SIZE; ++i) printf("%02X ", huge_tx_buf[i]);
        printf("\nRX: ");
        for (int i = 0; i < BUF_SIZE; ++i) printf("%02X ", huge_rx_buf[i]);
        printf("\n");

        k_msleep(SLEEP_TIME_MS);
    }

#if CONFIG_SPI_ASYNC
    k_thread_abort(async_thread_id);
#endif

    return 0;
}
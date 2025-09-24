#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

#define SLEEP_TIME_MS   1000
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

#define SPI0_NODE DT_NODELABEL(spi0)
static const struct device *spi_dev = DEVICE_DT_GET(SPI0_NODE);

#define BUF_SIZE 256
static uint8_t huge_tx_buf[BUF_SIZE];
static uint8_t huge_rx_buf[BUF_SIZE];

#define PRINT_BUF_SIZE(size) ((size * 5) + 1)
static uint8_t buffer_print_tx[PRINT_BUF_SIZE(BUF_SIZE)];
static uint8_t buffer_print_rx[PRINT_BUF_SIZE(BUF_SIZE)];

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

/* Helper: Format buffer to CSV style hex for inspection */
static void to_display_format(const uint8_t *src, size_t size, char *dst)
{
    size_t i;
    for (i = 0; i < size; i++) {
        sprintf(dst + 5 * i, "0x%02x,", src[i]);
    }
}

/* Helper: Print buffer as raw hex (16 bytes per line) */
static void print_buffer_hex(const char *label, const uint8_t *buf, size_t size)
{
    printf("%s:\n", label);
    for (size_t i = 0; i < size; ++i) {
        printf("%02X ", buf[i]);
        if ((i + 1) % 16 == 0)
            printf("\n");
    }
    if (size % 16 != 0)
        printf("\n");
}

/* Compare two buffers and print result */
static void spi_loopback_compare_bufs(const uint8_t *buf1, const uint8_t *buf2, size_t size,
                                      uint8_t *printbuf1, uint8_t *printbuf2)
{
    to_display_format(buf1, size, (char *)printbuf1);
    to_display_format(buf2, size, (char *)printbuf2);

    printf("TX buffer :\n%s\n", printbuf1);
    printf("RX buffer :\n%s\n", printbuf2);

    if (memcmp(buf1, buf2, size)) {
        printf("Buffer contents are different!\n");
    } else {
        printf("SPI: Data match for %zu bytes\n", size);
    }
}

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

#if IS_ENABLED(CONFIG_DMA)
    printf("DMA: ENABLED for SPI (if supported by hardware and DT)\n");
#else
    printf("DMA: DISABLED for SPI\n");
#endif

    while (1) {
        /* LED toggle */
        ret = gpio_pin_toggle_dt(&led);
        if (ret < 0) {
            printf("LED toggle failed\n");
            return 0;
        }
        led_state = !led_state;
        printf("LED state: %s\n", led_state ? "ON" : "OFF");

        memset(huge_rx_buf, 0, BUF_SIZE);

#if IS_ENABLED(CONFIG_DMA)
        /* Async transfer using spi_transceive_signal (best effort DMA showcase) */
        struct k_poll_signal spi_done_signal = K_POLL_SIGNAL_INITIALIZER(spi_done_signal);
        struct k_poll_event events[1] = {
            K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL,
                                    K_POLL_MODE_NOTIFY_ONLY,
                                    &spi_done_signal),
        };

        ret = spi_transceive_signal(spi_dev, &spi_cfg, &tx_bufs, &rx_bufs, &spi_done_signal);
        if (ret) {
            printf("SPI async transceive (DMA) failed, code %d\n", ret);
            continue;
        }

        /* Wait for async completion (DMA done), with timeout */
        ret = k_poll(events, 1, K_SECONDS(2)); // 2 second timeout
        if (ret == -EAGAIN) {
            printf("SPI async DMA transfer timed out! Falling back to blocking transfer.\n");
            ret = spi_transceive(spi_dev, &spi_cfg, &tx_bufs, &rx_bufs);
            if (ret) {
                printf("SPI blocking transfer failed, code %d\n", ret);
                continue;
            }
            printf("SPI blocking transfer complete (fallback)\n");
        } else if (ret) {
            printf("k_poll failed: %d\n", ret);
            continue;
        } else {
            printf("SPI async transfer complete (DMA path)\n");
        }
#else
        /* Blocking transfer (no DMA) */
        ret = spi_transceive(spi_dev, &spi_cfg, &tx_bufs, &rx_bufs);
        if (ret) {
            printf("SPI transceive failed, code %d\n", ret);
            continue;
        }
        printf("SPI blocking transfer complete (no DMA)\n");
#endif

        print_buffer_hex("TX Data (hex)", huge_tx_buf, BUF_SIZE);
        print_buffer_hex("RX Data (hex)", huge_rx_buf, BUF_SIZE);

        spi_loopback_compare_bufs(huge_tx_buf, huge_rx_buf, BUF_SIZE,
                                  buffer_print_tx, buffer_print_rx);

        k_msleep(SLEEP_TIME_MS);
    }
    return 0;
}
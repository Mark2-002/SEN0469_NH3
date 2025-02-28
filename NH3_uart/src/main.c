#include <zephyr/bluetooth/bluetooth.h> /* header files for the project */
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <time.h>

/* For modbus */
#include <zephyr/drivers/uart.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>

LOG_MODULE_REGISTER(TRANSMETER, LOG_LEVEL_DBG);

#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

/* change this to any other UART peripheral if desired */
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)

/* Get the UART device specified in the device tree */
static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

int err;
uint8_t mode_switch[9] = {0xFF, 0x01, 0x78, 0x03, 0x00, 0x00, 0x00, 0x00, 0x84};
uint8_t read_cmd[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
uint8_t rx_buf[9];
int NH3_Value;
/* Initialize UART */
void uart_init(void)
{
    struct uart_config uart_cfg = {
        .baudrate = 9600,
        .parity = UART_CFG_PARITY_NONE,
        .stop_bits = UART_CFG_STOP_BITS_1,
        .data_bits = UART_CFG_DATA_BITS_8,
        .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
    };
/* Configure the UART device with the settings */
    if (uart_configure(uart_dev, &uart_cfg) != 0)
    {
        LOG_ERR("Failed to configure UART");
    }
}

/* Send a command via UART */
void send_uart_command(const uint8_t *cmd, size_t length)
{
    for (size_t i = 0; i < length; i++)
    {
        uart_poll_out(uart_dev, cmd[i]);
        // k_usleep(100); // Fine-tuned delay
    }
}

/* Receive data from UART */
int receive_uart_data(uint8_t *buf, size_t length, k_timeout_t timeout)
{
    size_t received_len = 0;
    uint32_t start_time = k_uptime_get();

    while (received_len < length)
    {
        int ret = uart_poll_in(uart_dev, &buf[received_len]);
        if (ret == 0)
        {
            received_len++;
        }
        else if (ret == -1)
        {
            k_usleep(100); // Fine-tuned for responsive polling
            if (k_uptime_get() - start_time > timeout.ticks)
            {
                return -ETIMEDOUT;
            }
        }
        else
        {
            return ret;
        }
    }
    return 0;
}

int fetchData(){
    send_uart_command(read_cmd, sizeof(read_cmd));
    err = receive_uart_data(rx_buf, sizeof(rx_buf), K_MSEC(100));
    if (err == 0)
    {
        if (rx_buf[0] == 0xFF && rx_buf[1] == 0x86)
        {
            NH3_Value = (rx_buf[2] << 8) | rx_buf[3];
            LOG_INF("NH3 in Air: %d ppm", NH3_Value);
        }
        memset(rx_buf, 0, sizeof(rx_buf));
    }
    else
    {
        LOG_ERR("UART receive error: %d", err);
        k_msleep(2000);
    }
    return 0;
}

int main()
{
    if (!device_is_ready(uart_dev))
    {
        LOG_ERR("UART device not found!");
        return 0;
    }
    uart_init();
    send_uart_command(mode_switch, sizeof(mode_switch));
    LOG_INF("Mode Switching...");
    while (true)
    {
        fetchData();
    }
    return 0;
}

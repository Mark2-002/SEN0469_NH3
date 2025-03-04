#include <zephyr/bluetooth/bluetooth.h> /* header files for the project */
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <time.h>
#include <zephyr/sys/printk.h>

/* For modbus */
#include <zephyr/drivers/uart.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
/* FOR BLE */
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

LOG_MODULE_REGISTER(TRANSMETER, LOG_LEVEL_DBG);
#define DEVICE_NAME "NH3"
#define Device_ID 60
#define ADV_SIZE 8
// #define LED0_NODE DT_ALIAS(led0)
// #define LED1_NODE DT_ALIAS(led1)
// #define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

/* change this to any other UART peripheral if desired */
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)
#define LED0_NODE DT_ALIAS(led1) /* Get the UART device specified in the device tree */
static const struct gpio_dt_spec buzzer = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

int err;
struct sensor_value temp, hum;
const struct device *const sht = DEVICE_DT_GET_ANY(sensirion_sht4x);
struct bt_le_ext_adv *adv;
struct device *dev;
struct k_work work;
struct k_work work1;
struct k_timer timer;
struct k_timer timer1;
uint8_t mode_switch[9] = {0xFF, 0x01, 0x78, 0x03, 0x00, 0x00, 0x00, 0x00, 0x84};
uint8_t read_cmd[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
uint8_t rx_buf[9];

int NH3_Value;

struct bt_le_adv_param adv_param = BT_LE_ADV_PARAM_INIT(BT_LE_ADV_OPT_EXT_ADV | BT_LE_ADV_OPT_USE_IDENTITY | BT_LE_ADV_OPT_NO_2M | BT_LE_ADV_OPT_CODED,
                                                        0x30,
                                                        0x30, // 30 ms interval
                                                        NULL);
struct bt_le_ext_adv_start_param ext_adv_param = BT_LE_EXT_ADV_START_PARAM_INIT(0, 1);
static uint8_t mfg_data[ADV_SIZE];

static const struct bt_data ad[] = {
    BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data, ADV_SIZE),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

/* ---------------------Feching Sensor Data--------------------- */

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
/* This will return NH3 Value in the air in ppm */
int fetch_Nh3_Data()
{
    send_uart_command(read_cmd, sizeof(read_cmd));
    err = receive_uart_data(rx_buf, sizeof(rx_buf), K_MSEC(100));
    if (err == 0)
    {

        if (rx_buf[0] == 0xFF && rx_buf[1] == 0x86)
        {
            NH3_Value = (rx_buf[2] << 8) | rx_buf[3];
            printk("\nNH3 in Air: %d ppm", NH3_Value);
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

static void set_random_static_address(void) // Print a message indicating the start of the iBeacon Demo
{
    printk("::::::::::::::Starting Transmeter:::::::::::\n");
    bt_addr_le_t addr;
    bt_addr_le_from_str("FA:17:DF:DF:4B:60", "random", &addr);
    bt_id_create(&addr, NULL);
    printk("Created new address\n");
}

void adv_param_init(void)
{
    bt_le_ext_adv_create(&adv_param, NULL, &adv);
}

void start_adv(void)
{

    
    // k_msleep(100);
    sensor_sample_fetch(sht);
    sensor_channel_get(sht, SENSOR_CHAN_AMBIENT_TEMP, &temp);
    sensor_channel_get(sht, SENSOR_CHAN_HUMIDITY, &hum);
    printk("Temp: %d, Humidity: %d\n", temp.val1, hum.val1);
    // mfg_data[0] = 0x59;
    // mfg_data[1] = 0x00;
    mfg_data[0] = Device_ID;
    mfg_data[1] = temp.val1;
    mfg_data[2] = temp.val2 / 100000;
    mfg_data[3] = (temp.val2 / 10000) % 10;
    mfg_data[4] = hum.val1;
    mfg_data[5] = hum.val2 / 100000;
    mfg_data[6] = (hum.val2 / 10000) % 10;
    mfg_data[7] = NH3_Value;

    bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0);
    bt_le_ext_adv_start(adv, NULL);
    // gpio_pin_set_dt(&led,1);
    k_msleep(50);
    // gpio_pin_set_dt(&led,0);
    bt_le_ext_adv_stop(adv);
}

K_WORK_DEFINE(work, start_adv);
void exp_function(struct k_timer *timer_id)
{
    k_work_submit(&work);
}
K_TIMER_DEFINE(timer, exp_function, NULL);


void Alarm(){
    if (NH3_Value >= 3)
    {
        gpio_pin_set_dt(&buzzer, 0);
        k_msleep(500);
        gpio_pin_set_dt(&buzzer, 1);
    }
}

K_WORK_DEFINE(work1, Alarm);
void exp_function1   (struct k_timer *timer_id)
{
    k_work_submit(&work1);
}
K_TIMER_DEFINE(timer1, exp_function1, NULL);

int main()
{
    if (!device_is_ready(uart_dev))
    {
        LOG_ERR("UART device not found!");
        return 0;
    }
    set_random_static_address();
    bt_enable(NULL);
    uart_init();
    adv_param_init();
    send_uart_command(mode_switch, sizeof(mode_switch));
    LOG_INF("Mode Switching...");
    k_timer_start(&timer, K_SECONDS(1), K_SECONDS(10));
    k_timer_start(&timer1, K_SECONDS(1), K_SECONDS(1));
    gpio_pin_configure_dt(&buzzer, GPIO_OUTPUT_ACTIVE);
    gpio_pin_set_dt(&buzzer,1);
    while(1){
        // k_msleep(1500);
        send_uart_command(read_cmd, sizeof(read_cmd));
        err = receive_uart_data(rx_buf, sizeof(rx_buf), K_MSEC(100));
        if (err == 0)
        {

            if (rx_buf[0] == 0xFF && rx_buf[1] == 0x86)
            {
                NH3_Value = (rx_buf[2] << 8) | rx_buf[3];
                printk("\nNH3 in Air: %d ppm\n", NH3_Value);
            }
            
            memset(rx_buf, 0, sizeof(rx_buf));
        }
        else
        {
            LOG_ERR("UART receive error: %d", err);
            k_msleep(2000);
        }
       
    }

    return 0;
}

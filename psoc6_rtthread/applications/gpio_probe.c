/*
 * GPIO no-load probe commands.
 * Keep the L298P, battery, and motors disconnected during testing.
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <drivers/rt_drv_pwm.h>
#include <drivers/serial.h>
#include <finsh.h>
#include <board.h>
#include <stdlib.h>

#define CAR_DIR_A_PIN       P5_0
#define CAR_DIR_B_PIN       P11_2
#define CAR_PWM_DEV_NAME    "pwm0"
#define CAR_PWM_CHANNEL     7
#define CAR_PWM_PERIOD_NS   10000000U
#define BT_UART_DEV_NAME    "uart5"
#define BT_UART_BAUD_RATE   BAUD_RATE_9600
#define BT_DEFAULT_SPEED    30
#define BT_CMD_TIMEOUT_MS   1000
#define BT_THREAD_STACK     1024
#define BT_THREAD_PRIORITY  20
#define BT_THREAD_TICK      10
#define OPI_UART_DEV_NAME   "uart1"
#define OPI_UART_BAUD_RATE  BAUD_RATE_115200
#define OPI_THREAD_STACK    1024
#define OPI_THREAD_PRIORITY 21
#define OPI_THREAD_TICK     10
#define US_TRIG_PIN         GET_PIN(11, 4)
#define US_ECHO_PIN         GET_PIN(11, 5)
#define US_TRIG_PIN_NAME    "D6/P11_4"
#define US_ECHO_PIN_NAME    "D7/P11_5"
#define US_ECHO_IDLE_TIMEOUT_US  1000U
#define US_ECHO_TIMEOUT_US  30000U
#define US_DISTANCE_MIN_X100       300U
#define US_DISTANCE_MAX_X100       30000U
#define US_GUARD_OBSTACLE_X100     2000U
#define US_GUARD_CLEAR_X100        2500U
#define US_GUARD_TRIGGER_COUNT     2U
#define US_GUARD_CLEAR_COUNT       3U
#define US_GUARD_INTERVAL_MS       100
#define US_GUARD_THREAD_STACK      2048
#define US_GUARD_THREAD_PRIORITY   20
#define US_GUARD_THREAD_TICK       10
#define US_TIMEOUT_NONE            0U
#define US_TIMEOUT_WAIT_HIGH       1U
#define US_TIMEOUT_WAIT_LOW        2U
#define US_TIMEOUT_WAIT_IDLE       3U

static struct rt_device_pwm *car_pwm_dev = RT_NULL;
static rt_bool_t car_control_ready = RT_FALSE;
static rt_bool_t car_is_moving = RT_FALSE;
static rt_tick_t last_bt_cmd_tick = 0;
static int bt_speed = BT_DEFAULT_SPEED;
static rt_device_t bt_uart_dev = RT_NULL;
static rt_thread_t bt_thread = RT_NULL;
static rt_device_t opi_uart_dev = RT_NULL;
static rt_thread_t opi_thread = RT_NULL;
static volatile rt_bool_t us_guard_running = RT_FALSE;
static rt_thread_t us_guard_thread = RT_NULL;
static volatile rt_bool_t us_guard_blocked = RT_FALSE;
static rt_bool_t us_guard_last_valid = RT_FALSE;
static rt_uint32_t us_guard_last_distance_cm = 0;
static rt_uint8_t us_guard_obstacle_count = 0;
static rt_uint8_t us_guard_clear_count = 0;
static rt_bool_t us_guard_timeout_warned = RT_FALSE;

static void us_cycle_counter_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static rt_uint32_t us_cycle_per_us(void)
{
    rt_uint32_t cycle_per_us = SystemCoreClock / 1000000UL;

    return cycle_per_us == 0U ? 1U : cycle_per_us;
}

static rt_uint32_t us_elapsed_us(rt_uint32_t start_cycle)
{
    return (DWT->CYCCNT - start_cycle) / us_cycle_per_us();
}

static void gpio_probe_write(rt_base_t pin, const char *pin_name, rt_uint8_t level)
{
    rt_int8_t readback;

    rt_pin_mode(pin, PIN_MODE_OUTPUT);
    rt_pin_write(pin, level);
    readback = rt_pin_read(pin);
    rt_kprintf("%s target=%s read=%s(%d)\n",
               pin_name,
               level == PIN_HIGH ? "HIGH" : "LOW",
               readback == PIN_HIGH ? "HIGH" : "LOW",
               readback);
}

static rt_err_t car_control_init(void)
{
    rt_err_t result;

    rt_pin_mode(CAR_DIR_A_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(CAR_DIR_B_PIN, PIN_MODE_OUTPUT);

    if (car_pwm_dev == RT_NULL)
    {
        car_pwm_dev = (struct rt_device_pwm *)rt_device_find(CAR_PWM_DEV_NAME);
        if (car_pwm_dev == RT_NULL)
        {
            rt_kprintf("car pwm device %s not found\n", CAR_PWM_DEV_NAME);
            return -RT_ERROR;
        }
    }

    if (car_control_ready == RT_FALSE)
    {
        result = rt_pwm_set(car_pwm_dev, CAR_PWM_CHANNEL, CAR_PWM_PERIOD_NS, 0);
        if (result != RT_EOK)
        {
            rt_kprintf("car pwm init set failed: %d\n", result);
            return result;
        }

        result = rt_pwm_enable(car_pwm_dev, CAR_PWM_CHANNEL);
        if (result != RT_EOK)
        {
            rt_kprintf("car pwm enable failed: %d\n", result);
            return result;
        }

        car_control_ready = RT_TRUE;
    }

    return RT_EOK;
}

static int car_clamp_speed(int speed)
{
    if (speed < 0)
    {
        return 0;
    }

    if (speed > 100)
    {
        return 100;
    }

    return speed;
}

static int car_parse_speed(int argc, char **argv)
{
    if (argc < 2)
    {
        return 0;
    }

    return car_clamp_speed(atoi(argv[1]));
}

static rt_err_t car_do_motion(const char *action, int speed, rt_uint8_t dir_a, rt_uint8_t dir_b)
{
    rt_err_t result;
    rt_uint32_t pulse;

    result = car_control_init();
    if (result != RT_EOK)
    {
        return result;
    }

    speed = car_clamp_speed(speed);
    pulse = (rt_uint32_t)(((rt_uint64_t)CAR_PWM_PERIOD_NS * (rt_uint32_t)speed) / 100U);

    result = rt_pwm_set(car_pwm_dev, CAR_PWM_CHANNEL, CAR_PWM_PERIOD_NS, 0);
    if (result != RT_EOK)
    {
        rt_kprintf("car %s speed=%d pwm stop-before-dir failed: %d\n", action, speed, result);
        return result;
    }

    rt_pin_write(CAR_DIR_A_PIN, dir_a);
    rt_pin_write(CAR_DIR_B_PIN, dir_b);

    result = rt_pwm_set(car_pwm_dev, CAR_PWM_CHANNEL, CAR_PWM_PERIOD_NS, pulse);
    if (result != RT_EOK)
    {
        rt_kprintf("car %s speed=%d pwm set failed: %d\n", action, speed, result);
        return result;
    }

    car_is_moving = RT_TRUE;

    return RT_EOK;
}

static rt_err_t car_do_stop(void)
{
    rt_err_t result;

    result = car_control_init();
    if (result != RT_EOK)
    {
        return result;
    }

    result = rt_pwm_set(car_pwm_dev, CAR_PWM_CHANNEL, CAR_PWM_PERIOD_NS, 0);
    if (result != RT_EOK)
    {
        rt_kprintf("car stop speed=0 pwm set failed: %d\n", result);
        return result;
    }

    car_is_moving = RT_FALSE;

    return RT_EOK;
}

static void us_guard_update_interlock(rt_uint32_t distance_cm, rt_bool_t distance_valid)
{
    if (distance_valid == RT_FALSE)
    {
        us_guard_obstacle_count = 0;
        us_guard_clear_count = 0;
        return;
    }

    us_guard_last_distance_cm = distance_cm;

    if (distance_cm < US_GUARD_OBSTACLE_X100)
    {
        us_guard_clear_count = 0;

        if (us_guard_obstacle_count < US_GUARD_TRIGGER_COUNT)
        {
            us_guard_obstacle_count++;
        }

        if ((us_guard_obstacle_count >= US_GUARD_TRIGGER_COUNT) &&
            (us_guard_blocked == RT_FALSE))
        {
            us_guard_blocked = RT_TRUE;
            rt_kprintf("[us_guard] obstacle %u.%02u cm -> blocked and car_stop\n",
                       distance_cm / 100U,
                       distance_cm % 100U);
            car_do_stop();
        }

        return;
    }

    us_guard_obstacle_count = 0;

    if (distance_cm > US_GUARD_CLEAR_X100)
    {
        if (us_guard_clear_count < US_GUARD_CLEAR_COUNT)
        {
            us_guard_clear_count++;
        }

        if ((us_guard_clear_count >= US_GUARD_CLEAR_COUNT) &&
            (us_guard_blocked == RT_TRUE))
        {
            us_guard_blocked = RT_FALSE;
            rt_kprintf("[us_guard] clear %u.%02u cm -> unblock\n",
                       distance_cm / 100U,
                       distance_cm % 100U);
        }

        return;
    }

    us_guard_clear_count = 0;
}

static rt_err_t car_stop_forward_blocked_by_us_guard(void)
{
    car_do_stop();
    rt_kprintf("[safe] forward blocked by ultrasonic %u.%02u cm\n",
               us_guard_last_distance_cm / 100U,
               us_guard_last_distance_cm % 100U);

    return -RT_ERROR;
}

static rt_err_t car_do_forward(int speed)
{
    if ((us_guard_running == RT_TRUE) && (us_guard_blocked == RT_TRUE))
    {
        return car_stop_forward_blocked_by_us_guard();
    }

    return car_do_motion("forward", speed, PIN_HIGH, PIN_HIGH);
}

static rt_err_t car_do_back(int speed)
{
    return car_do_motion("back", speed, PIN_LOW, PIN_LOW);
}

static rt_err_t car_do_left(int speed)
{
    return car_do_motion("left", speed, PIN_LOW, PIN_HIGH);
}

static rt_err_t car_do_right(int speed)
{
    return car_do_motion("right", speed, PIN_HIGH, PIN_LOW);
}

static void bt_config_9600_8n1(struct serial_configure *config)
{
    *config = (struct serial_configure)RT_SERIAL_CONFIG_DEFAULT;
    config->baud_rate = BT_UART_BAUD_RATE;
    config->data_bits = DATA_BITS_8;
    config->stop_bits = STOP_BITS_1;
    config->parity = PARITY_NONE;
}

static void opi_config_115200_8n1(struct serial_configure *config)
{
    *config = (struct serial_configure)RT_SERIAL_CONFIG_DEFAULT;
    config->baud_rate = OPI_UART_BAUD_RATE;
    config->data_bits = DATA_BITS_8;
    config->stop_bits = STOP_BITS_1;
    config->parity = PARITY_NONE;
}

static char bt_visible_char(char ch)
{
    rt_uint8_t value = (rt_uint8_t)ch;

    if ((value >= 0x20) && (value <= 0x7E))
    {
        return ch;
    }

    return '.';
}

static void bt_mark_valid_cmd(void)
{
    last_bt_cmd_tick = rt_tick_get();
}

static void bt_set_speed(int speed)
{
    bt_speed = speed;
    bt_mark_valid_cmd();
    rt_kprintf("[bt] speed = %d%%\n", bt_speed);
}

static void opi_set_speed(int speed)
{
    bt_speed = speed;
    bt_mark_valid_cmd();
    rt_kprintf("[opi] speed = %d%%\n", bt_speed);
}

static void bt_check_timeout(void)
{
    rt_tick_t now;
    rt_tick_t timeout_tick;

    if (car_is_moving == RT_FALSE)
    {
        return;
    }

    timeout_tick = rt_tick_from_millisecond(BT_CMD_TIMEOUT_MS);
    now = rt_tick_get();

    if ((rt_tick_t)(now - last_bt_cmd_tick) > timeout_tick)
    {
        car_do_stop();
        rt_kprintf("[bt] timeout -> stop\n");
    }
}

static int car_stop(int argc, char **argv)
{
    rt_err_t result;

    (void)argc;
    (void)argv;

    result = car_do_stop();
    if (result != RT_EOK)
    {
        return -RT_ERROR;
    }

    rt_kprintf("car stop speed=0\n");
    return RT_EOK;
}
MSH_CMD_EXPORT(car_stop, Stop car by setting pwm pulse to 0);

static int car_forward(int argc, char **argv)
{
    int speed;
    rt_err_t result;

    if (argc < 2)
    {
        rt_kprintf("usage: car_forward <speed>\n");
        return -RT_ERROR;
    }

    speed = car_parse_speed(argc, argv);
    result = car_do_forward(speed);
    if (result != RT_EOK)
    {
        return -RT_ERROR;
    }

    rt_kprintf("car forward speed=%d\n", speed);
    return RT_EOK;
}
MSH_CMD_EXPORT(car_forward, Move car forward with speed 0-100);

static int car_back(int argc, char **argv)
{
    int speed;
    rt_err_t result;

    if (argc < 2)
    {
        rt_kprintf("usage: car_back <speed>\n");
        return -RT_ERROR;
    }

    speed = car_parse_speed(argc, argv);
    result = car_do_back(speed);
    if (result != RT_EOK)
    {
        return -RT_ERROR;
    }

    rt_kprintf("car back speed=%d\n", speed);
    return RT_EOK;
}
MSH_CMD_EXPORT(car_back, Move car back with speed 0-100);

static int car_left(int argc, char **argv)
{
    int speed;
    rt_err_t result;

    if (argc < 2)
    {
        rt_kprintf("usage: car_left <speed>\n");
        return -RT_ERROR;
    }

    speed = car_parse_speed(argc, argv);
    result = car_do_left(speed);
    if (result != RT_EOK)
    {
        return -RT_ERROR;
    }

    rt_kprintf("car left speed=%d\n", speed);
    return RT_EOK;
}
MSH_CMD_EXPORT(car_left, Turn car left with speed 0-100);

static int car_right(int argc, char **argv)
{
    int speed;
    rt_err_t result;

    if (argc < 2)
    {
        rt_kprintf("usage: car_right <speed>\n");
        return -RT_ERROR;
    }

    speed = car_parse_speed(argc, argv);
    result = car_do_right(speed);
    if (result != RT_EOK)
    {
        return -RT_ERROR;
    }

    rt_kprintf("car right speed=%d\n", speed);
    return RT_EOK;
}
MSH_CMD_EXPORT(car_right, Turn car right with speed 0-100);

static void bt_handle_char(char ch)
{
    switch (ch)
    {
    case 'F':
    case 'f':
        bt_mark_valid_cmd();
        rt_kprintf("[bt] F -> forward\n");
        car_do_forward(bt_speed);
        break;

    case 'B':
    case 'b':
        bt_mark_valid_cmd();
        rt_kprintf("[bt] B -> back\n");
        car_do_back(bt_speed);
        break;

    case 'L':
    case 'l':
        bt_mark_valid_cmd();
        rt_kprintf("[bt] L -> left\n");
        car_do_left(bt_speed);
        break;

    case 'R':
    case 'r':
        bt_mark_valid_cmd();
        rt_kprintf("[bt] R -> right\n");
        car_do_right(bt_speed);
        break;

    case 'S':
    case 's':
        bt_mark_valid_cmd();
        rt_kprintf("[bt] S -> stop\n");
        car_do_stop();
        break;

    case '1':
        bt_set_speed(30);
        break;

    case '2':
        bt_set_speed(40);
        break;

    case '3':
        bt_set_speed(50);
        break;

    case '4':
        bt_set_speed(60);
        break;

    case '5':
        bt_set_speed(70);
        break;

    case '\r':
    case '\n':
        break;

    default:
        break;
    }
}

static void bt_thread_entry(void *parameter)
{
    char ch;

    (void)parameter;

    while (1)
    {
        if (rt_device_read(bt_uart_dev, 0, &ch, 1) == 1)
        {
            rt_kprintf("[bt] recv: 0x%02X '%c'\n", (rt_uint8_t)ch, bt_visible_char(ch));
            bt_handle_char(ch);
        }
        else
        {
            rt_thread_mdelay(10);
        }

        bt_check_timeout();
    }
}

static void opi_handle_char(char ch)
{
    switch (ch)
    {
    case 'F':
    case 'f':
        bt_mark_valid_cmd();
        car_do_forward(bt_speed);
        break;

    case 'B':
    case 'b':
        bt_mark_valid_cmd();
        car_do_back(bt_speed);
        break;

    case 'L':
    case 'l':
        bt_mark_valid_cmd();
        car_do_left(bt_speed);
        break;

    case 'R':
    case 'r':
        bt_mark_valid_cmd();
        car_do_right(bt_speed);
        break;

    case 'S':
    case 's':
        bt_mark_valid_cmd();
        car_do_stop();
        break;

    case '1':
        opi_set_speed(30);
        break;

    case '2':
        opi_set_speed(40);
        break;

    case '3':
        opi_set_speed(50);
        break;

    case '4':
        opi_set_speed(60);
        break;

    case '5':
        opi_set_speed(70);
        break;

    case '\r':
    case '\n':
        break;

    default:
        break;
    }
}

static void opi_thread_entry(void *parameter)
{
    char ch;

    (void)parameter;

    while (1)
    {
        if (rt_device_read(opi_uart_dev, 0, &ch, 1) == 1)
        {
            rt_kprintf("[opi] recv: 0x%02X '%c'\n", (rt_uint8_t)ch, bt_visible_char(ch));
            opi_handle_char(ch);
        }
        else
        {
            rt_thread_mdelay(10);
        }

        bt_check_timeout();
    }
}

static int bt_start(int argc, char **argv)
{
    rt_err_t result;
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

    (void)argc;
    (void)argv;

    if (bt_thread != RT_NULL)
    {
        rt_kprintf("[bt] already started\n");
        return RT_EOK;
    }

    bt_uart_dev = rt_device_find(BT_UART_DEV_NAME);
    if (bt_uart_dev == RT_NULL)
    {
        rt_kprintf("[bt] find uart5 failed\n");
        return -RT_ERROR;
    }

    bt_config_9600_8n1(&config);
    result = rt_device_control(bt_uart_dev, RT_DEVICE_CTRL_CONFIG, &config);
    if (result != RT_EOK)
    {
        rt_kprintf("[bt] config uart5 failed: %d\n", result);
        return -RT_ERROR;
    }

    result = rt_device_open(bt_uart_dev, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
    if (result != RT_EOK)
    {
        rt_kprintf("[bt] open uart5 failed\n");
        return -RT_ERROR;
    }

    bt_thread = rt_thread_create("bt_car",
                                 bt_thread_entry,
                                 RT_NULL,
                                 BT_THREAD_STACK,
                                 BT_THREAD_PRIORITY,
                                 BT_THREAD_TICK);
    if (bt_thread == RT_NULL)
    {
        rt_kprintf("[bt] create thread failed\n");
        return -RT_ERROR;
    }

    rt_thread_startup(bt_thread);
    rt_kprintf("[bt] start uart5 9600\n");

    return RT_EOK;
}
MSH_CMD_EXPORT(bt_start, Start Bluetooth car control on uart5);

static int bt_send_test(int argc, char **argv)
{
    rt_device_t uart_dev;
    rt_err_t result;
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    const char *message = "HELLO_FROM_PSOC62\r\n";

    (void)argc;
    (void)argv;

    uart_dev = rt_device_find(BT_UART_DEV_NAME);
    if (uart_dev == RT_NULL)
    {
        rt_kprintf("[bt] find uart5 failed\n");
        return -RT_ERROR;
    }

    if ((uart_dev->open_flag & RT_DEVICE_OFLAG_OPEN) == 0)
    {
        bt_config_9600_8n1(&config);
        result = rt_device_control(uart_dev, RT_DEVICE_CTRL_CONFIG, &config);
        if (result != RT_EOK)
        {
            rt_kprintf("[bt] config uart5 failed: %d\n", result);
            return -RT_ERROR;
        }

        result = rt_device_open(uart_dev, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
        if (result != RT_EOK)
        {
            rt_kprintf("[bt] open uart5 failed\n");
            return -RT_ERROR;
        }
    }

    rt_device_write(uart_dev, 0, message, sizeof("HELLO_FROM_PSOC62\r\n") - 1);
    rt_kprintf("[bt] send test ok\n");

    return RT_EOK;
}
MSH_CMD_EXPORT(bt_send_test, Send Bluetooth UART test string on uart5);

static int opi_start(int argc, char **argv)
{
    rt_err_t result;
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

    (void)argc;
    (void)argv;

    if (opi_thread != RT_NULL)
    {
        rt_kprintf("[opi] already started\n");
        return RT_EOK;
    }

    opi_uart_dev = rt_device_find(OPI_UART_DEV_NAME);
    if (opi_uart_dev == RT_NULL)
    {
        rt_kprintf("[opi] find uart1 failed\n");
        return -RT_ERROR;
    }

    opi_config_115200_8n1(&config);
    result = rt_device_control(opi_uart_dev, RT_DEVICE_CTRL_CONFIG, &config);
    if (result != RT_EOK)
    {
        rt_kprintf("[opi] config uart1 failed: %d\n", result);
        return -RT_ERROR;
    }

    result = rt_device_open(opi_uart_dev, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
    if (result != RT_EOK)
    {
        rt_kprintf("[opi] open uart1 failed\n");
        return -RT_ERROR;
    }

    opi_thread = rt_thread_create("opi_car",
                                  opi_thread_entry,
                                  RT_NULL,
                                  OPI_THREAD_STACK,
                                  OPI_THREAD_PRIORITY,
                                  OPI_THREAD_TICK);
    if (opi_thread == RT_NULL)
    {
        rt_kprintf("[opi] create thread failed\n");
        return -RT_ERROR;
    }

    rt_thread_startup(opi_thread);
    rt_kprintf("[opi] start uart1 115200\n");

    return RT_EOK;
}
MSH_CMD_EXPORT(opi_start, Start Orange Pi ROS2 car control on uart1);

static void us_gpio_init(void)
{
    us_cycle_counter_init();
    rt_pin_mode(US_TRIG_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(US_TRIG_PIN, PIN_LOW);
    rt_pin_mode(US_ECHO_PIN, PIN_MODE_INPUT);
}

static rt_bool_t us_wait_echo_level(rt_uint8_t target_level, rt_uint32_t timeout_us)
{
    rt_uint32_t start_cycle = DWT->CYCCNT;

    while (rt_pin_read(US_ECHO_PIN) != target_level)
    {
        if (us_elapsed_us(start_cycle) >= timeout_us)
        {
            return RT_FALSE;
        }
    }

    return RT_TRUE;
}

static rt_err_t us_measure_distance_x100(rt_uint32_t *distance_x100,
                                         rt_uint32_t *echo_us,
                                         rt_uint8_t *timeout_stage)
{
    rt_uint32_t echo_start_cycle;
    rt_uint32_t echo_width_us = 0;

    if (distance_x100 == RT_NULL)
    {
        return -RT_ERROR;
    }

    if (timeout_stage != RT_NULL)
    {
        *timeout_stage = US_TIMEOUT_NONE;
    }

    us_gpio_init();

    if (rt_pin_read(US_ECHO_PIN) == PIN_HIGH)
    {
        if (us_wait_echo_level(PIN_LOW, US_ECHO_IDLE_TIMEOUT_US) == RT_FALSE)
        {
            if (timeout_stage != RT_NULL)
            {
                *timeout_stage = US_TIMEOUT_WAIT_IDLE;
            }

            return -RT_ETIMEOUT;
        }
    }

    rt_hw_us_delay(5);
    rt_pin_write(US_TRIG_PIN, PIN_HIGH);
    rt_hw_us_delay(12);
    rt_pin_write(US_TRIG_PIN, PIN_LOW);

    if (us_wait_echo_level(PIN_HIGH, US_ECHO_TIMEOUT_US) == RT_FALSE)
    {
        if (timeout_stage != RT_NULL)
        {
            *timeout_stage = US_TIMEOUT_WAIT_HIGH;
        }

        return -RT_ETIMEOUT;
    }

    echo_start_cycle = DWT->CYCCNT;
    while (rt_pin_read(US_ECHO_PIN) == PIN_HIGH)
    {
        echo_width_us = us_elapsed_us(echo_start_cycle);
        if (echo_width_us >= US_ECHO_TIMEOUT_US)
        {
            if (timeout_stage != RT_NULL)
            {
                *timeout_stage = US_TIMEOUT_WAIT_LOW;
            }

            return -RT_ETIMEOUT;
        }
    }
    echo_width_us = us_elapsed_us(echo_start_cycle);

    if (echo_us != RT_NULL)
    {
        *echo_us = echo_width_us;
    }

    *distance_x100 = (rt_uint32_t)((((rt_uint64_t)echo_width_us * 100U) + 29U) / 58U);

    return RT_EOK;
}

static rt_bool_t us_guard_distance_is_valid(rt_uint32_t distance_x100)
{
    return ((distance_x100 >= US_DISTANCE_MIN_X100) &&
            (distance_x100 <= US_DISTANCE_MAX_X100)) ? RT_TRUE : RT_FALSE;
}

static rt_bool_t us_guard_update_obstacle_count(rt_uint32_t distance_x100,
                                                rt_bool_t distance_valid,
                                                rt_uint8_t *obstacle_count)
{
    if (obstacle_count == RT_NULL)
    {
        return RT_FALSE;
    }

    if ((distance_valid == RT_FALSE) || (distance_x100 >= US_GUARD_OBSTACLE_X100))
    {
        *obstacle_count = 0;
        return RT_FALSE;
    }

    if (*obstacle_count < US_GUARD_TRIGGER_COUNT)
    {
        (*obstacle_count)++;
    }

    return (*obstacle_count >= US_GUARD_TRIGGER_COUNT) ? RT_TRUE : RT_FALSE;
}

static void us_guard_thread_entry(void *parameter)
{
    rt_err_t result;
    rt_uint32_t distance_x100 = 0;
    rt_uint8_t timeout_stage = US_TIMEOUT_NONE;
    rt_bool_t distance_valid;

    (void)parameter;

    while (us_guard_running == RT_TRUE)
    {
        result = us_measure_distance_x100(&distance_x100, RT_NULL, &timeout_stage);
        if (us_guard_running == RT_FALSE)
        {
            break;
        }

        if (result == RT_EOK)
        {
            us_guard_timeout_warned = RT_FALSE;
            us_guard_last_distance_cm = distance_x100;
            distance_valid = us_guard_distance_is_valid(distance_x100);
            us_guard_last_valid = distance_valid;
            us_guard_update_interlock(distance_x100, distance_valid);
        }
        else
        {
            us_guard_last_valid = RT_FALSE;
            us_guard_last_distance_cm = 0;
            us_guard_obstacle_count = 0;
            us_guard_clear_count = 0;

            if ((timeout_stage != US_TIMEOUT_NONE) &&
                (us_guard_timeout_warned == RT_FALSE))
            {
                rt_kprintf("[us_guard] ultrasonic timeout\n");
                us_guard_timeout_warned = RT_TRUE;
            }
        }

        rt_thread_mdelay(US_GUARD_INTERVAL_MS);
    }

    us_guard_thread = RT_NULL;
}

static int us_guard_start(int argc, char **argv)
{
    rt_err_t result;

    (void)argc;
    (void)argv;

    if ((us_guard_running == RT_TRUE) || (us_guard_thread != RT_NULL))
    {
        rt_kprintf("[us_guard] already running\n");
        return RT_EOK;
    }

    us_gpio_init();
    us_guard_blocked = RT_FALSE;
    us_guard_last_valid = RT_FALSE;
    us_guard_last_distance_cm = 0;
    us_guard_obstacle_count = 0;
    us_guard_clear_count = 0;
    us_guard_timeout_warned = RT_FALSE;
    us_guard_running = RT_TRUE;

    us_guard_thread = rt_thread_create("us_guard",
                                       us_guard_thread_entry,
                                       RT_NULL,
                                       US_GUARD_THREAD_STACK,
                                       US_GUARD_THREAD_PRIORITY,
                                       US_GUARD_THREAD_TICK);
    if (us_guard_thread == RT_NULL)
    {
        us_guard_running = RT_FALSE;
        rt_kprintf("[us_guard] create thread failed\n");
        return -RT_ERROR;
    }

    rt_kprintf("[us_guard] start\n");

    result = rt_thread_startup(us_guard_thread);
    if (result != RT_EOK)
    {
        us_guard_running = RT_FALSE;
        us_guard_thread = RT_NULL;
        rt_kprintf("[us_guard] startup thread failed: %d\n", result);
        return -RT_ERROR;
    }

    return RT_EOK;
}
MSH_CMD_EXPORT(us_guard_start, Start ultrasonic obstacle guard);

static int us_guard_stop(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    if ((us_guard_running == RT_FALSE) && (us_guard_thread == RT_NULL))
    {
        rt_kprintf("[us_guard] stopped\n");
        return RT_EOK;
    }

    us_guard_running = RT_FALSE;
    us_guard_blocked = RT_FALSE;
    us_guard_obstacle_count = 0;
    us_guard_clear_count = 0;
    rt_kprintf("[us_guard] stop\n");

    return RT_EOK;
}
MSH_CMD_EXPORT(us_guard_stop, Stop ultrasonic obstacle guard);

static int us_guard_state(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    if (us_guard_last_valid == RT_TRUE)
    {
        rt_kprintf("[us_guard] running=%d, blocked=%d, last_distance=%u.%02u cm, close_count=%u\n",
                   us_guard_running == RT_TRUE ? 1 : 0,
                   us_guard_blocked == RT_TRUE ? 1 : 0,
                   us_guard_last_distance_cm / 100U,
                   us_guard_last_distance_cm % 100U,
                   us_guard_obstacle_count);
    }
    else if (us_guard_last_distance_cm != 0)
    {
        rt_kprintf("[us_guard] running=%d, blocked=%d, last_distance=%u.%02u cm (invalid), close_count=%u\n",
                   us_guard_running == RT_TRUE ? 1 : 0,
                   us_guard_blocked == RT_TRUE ? 1 : 0,
                   us_guard_last_distance_cm / 100U,
                   us_guard_last_distance_cm % 100U,
                   us_guard_obstacle_count);
    }
    else
    {
        rt_kprintf("[us_guard] running=%d, blocked=%d, last_distance=invalid, close_count=%u\n",
                   us_guard_running == RT_TRUE ? 1 : 0,
                   us_guard_blocked == RT_TRUE ? 1 : 0,
                   us_guard_obstacle_count);
    }

    return RT_EOK;
}
MSH_CMD_EXPORT(us_guard_state, Show ultrasonic obstacle guard state);

static int us_test(int argc, char **argv)
{
    rt_uint32_t echo_us = 0;
    rt_uint32_t distance_x100;
    rt_uint8_t timeout_stage = US_TIMEOUT_NONE;
    rt_err_t result;

    (void)argc;
    (void)argv;

    rt_kprintf("[us] trig=%s echo=%s\n", US_TRIG_PIN_NAME, US_ECHO_PIN_NAME);

    result = us_measure_distance_x100(&distance_x100, &echo_us, &timeout_stage);
    if (result != RT_EOK)
    {
        if (timeout_stage == US_TIMEOUT_WAIT_HIGH)
        {
            rt_kprintf("[us] wait echo high timeout\n");
        }
        else if (timeout_stage == US_TIMEOUT_WAIT_LOW)
        {
            rt_kprintf("[us] wait echo low timeout\n");
        }
        else if (timeout_stage == US_TIMEOUT_WAIT_IDLE)
        {
            rt_kprintf("[us] wait echo idle timeout\n");
        }
        else
        {
            rt_kprintf("[us] measure failed: %d\n", result);
        }

        return -RT_ERROR;
    }

    rt_kprintf("[us] trig=%s echo=%s echo_us=%u, distance=%u.%02u cm\n",
               US_TRIG_PIN_NAME,
               US_ECHO_PIN_NAME,
               echo_us,
               distance_x100 / 100U,
               distance_x100 % 100U);

    return RT_EOK;
}
MSH_CMD_EXPORT(us_test, Test HC-SR04 ultrasonic distance once);

static void us_pin_test_print_echo(const char *stage)
{
    rt_int8_t echo_level = rt_pin_read(US_ECHO_PIN);

    rt_kprintf("[us_pin_test] %s echo=%d\n", stage, echo_level == PIN_HIGH ? 1 : 0);
}

static int us_pin_test(int argc, char **argv)
{
    int i;

    (void)argc;
    (void)argv;

    rt_kprintf("[us_pin_test] Trig = %s\n", US_TRIG_PIN_NAME);
    rt_kprintf("[us_pin_test] Echo = %s\n", US_ECHO_PIN_NAME);

    rt_pin_mode(US_ECHO_PIN, PIN_MODE_INPUT);
    rt_pin_mode(US_TRIG_PIN, PIN_MODE_OUTPUT);

    rt_pin_write(US_TRIG_PIN, PIN_LOW);
    rt_hw_us_delay(10);
    us_pin_test_print_echo("trig LOW");

    rt_pin_write(US_TRIG_PIN, PIN_HIGH);
    rt_thread_mdelay(1000);
    us_pin_test_print_echo("trig HIGH 1s");

    rt_pin_write(US_TRIG_PIN, PIN_LOW);
    rt_hw_us_delay(10);
    us_pin_test_print_echo("trig LOW");

    rt_kprintf("[us_pin_test] echo sample start\n");
    rt_pin_mode(US_ECHO_PIN, PIN_MODE_INPUT);
    for (i = 0; i < 20; i++)
    {
        rt_kprintf("[us_pin_test] echo[%02d]=%d\n",
                   i,
                   rt_pin_read(US_ECHO_PIN) == PIN_HIGH ? 1 : 0);
        rt_thread_mdelay(100);
    }

    return RT_EOK;
}
MSH_CMD_EXPORT(us_pin_test, Test ultrasonic Trig output and Echo input pins);

static void gpio_p11_3_high(void)
{
    gpio_probe_write(P11_3, "P11_3", PIN_HIGH);
}
MSH_CMD_EXPORT(gpio_p11_3_high, Set P11_3 high for GPIO no-load test);

static void gpio_p11_3_low(void)
{
    gpio_probe_write(P11_3, "P11_3", PIN_LOW);
}
MSH_CMD_EXPORT(gpio_p11_3_low, Set P11_3 low for GPIO no-load test);

static void gpio_p11_4_high(void)
{
    gpio_probe_write(P11_4, "P11_4", PIN_HIGH);
}
MSH_CMD_EXPORT(gpio_p11_4_high, Set P11_4 high for GPIO no-load test);

static void gpio_p11_4_low(void)
{
    gpio_probe_write(P11_4, "P11_4", PIN_LOW);
}
MSH_CMD_EXPORT(gpio_p11_4_low, Set P11_4 low for GPIO no-load test);

static void gpio_p11_2_high(void)
{
    gpio_probe_write(P11_2, "P11_2", PIN_HIGH);
}
MSH_CMD_EXPORT(gpio_p11_2_high, Set P11_2 high for GPIO no-load test);

static void gpio_p11_2_low(void)
{
    gpio_probe_write(P11_2, "P11_2", PIN_LOW);
}
MSH_CMD_EXPORT(gpio_p11_2_low, Set P11_2 low for GPIO no-load test);

static void gpio_p5_0_high(void)
{
    gpio_probe_write(P5_0, "P5_0", PIN_HIGH);
}
MSH_CMD_EXPORT(gpio_p5_0_high, Set P5_0 high for GPIO no-load test);

static void gpio_p5_0_low(void)
{
    gpio_probe_write(P5_0, "P5_0", PIN_LOW);
}
MSH_CMD_EXPORT(gpio_p5_0_low, Set P5_0 low for GPIO no-load test);

static void gpio_p5_6_high(void)
{
    gpio_probe_write(P5_6, "P5_6", PIN_HIGH);
}
MSH_CMD_EXPORT(gpio_p5_6_high, Set P5_6 high for GPIO no-load test);

static void gpio_p5_6_low(void)
{
    gpio_probe_write(P5_6, "P5_6", PIN_LOW);
}
MSH_CMD_EXPORT(gpio_p5_6_low, Set P5_6 low for GPIO no-load test);

static void gpio_p5_7_high(void)
{
    gpio_probe_write(P5_7, "P5_7", PIN_HIGH);
}
MSH_CMD_EXPORT(gpio_p5_7_high, Set P5_7 high for GPIO no-load test);

static void gpio_p5_7_low(void)
{
    gpio_probe_write(P5_7, "P5_7", PIN_LOW);
}
MSH_CMD_EXPORT(gpio_p5_7_low, Set P5_7 low for GPIO no-load test);

static void gpio_all_low(void)
{
    gpio_probe_write(P11_3, "P11_3", PIN_LOW);
    gpio_probe_write(P11_4, "P11_4", PIN_LOW);
    gpio_probe_write(P11_2, "P11_2", PIN_LOW);
    gpio_probe_write(P5_0, "P5_0", PIN_LOW);
    gpio_probe_write(P5_6, "P5_6", PIN_LOW);
    gpio_probe_write(P5_7, "P5_7", PIN_LOW);
}
MSH_CMD_EXPORT(gpio_all_low, Set all GPIO no-load test pins low);

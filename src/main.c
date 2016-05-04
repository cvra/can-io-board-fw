#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include "mpu60X0.h"
#include "mpu60X0_registers.h"
#include <serial-datagram/serial_datagram.h>
#include <cmp_mem_access/cmp_mem_access.h>
#include <cmp/cmp.h>
#include <arm-cortex-tools/fault.h>
#include <memstreams.h>
#include <stdarg.h>
#include <string.h>

BaseSequentialStream *stdout = NULL;

#define EXTI_EVENT_MPU6050_INT (1<<0)
event_source_t exti_events;

static void gpio_exti_callback(EXTDriver *extp, expchannel_t channel) {
    (void)extp;
    if (channel == GPIOB_PIN4) {
        chSysLockFromISR();
        chEvtBroadcastFlagsI(&exti_events, EXTI_EVENT_MPU6050_INT);
        chSysUnlockFromISR();
    }
}

static const EXTConfig extcfg = {{
    {EXT_CH_MODE_DISABLED, NULL}, // 0
    {EXT_CH_MODE_DISABLED, NULL}, // 1
    {EXT_CH_MODE_DISABLED, NULL}, // 2
    {EXT_CH_MODE_DISABLED, NULL}, // 3
    // PB4 = MPU_INT
    {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOB, gpio_exti_callback},
    {EXT_CH_MODE_DISABLED, NULL}, // 5
    {EXT_CH_MODE_DISABLED, NULL}, // 6
    {EXT_CH_MODE_DISABLED, NULL}, // 7
    {EXT_CH_MODE_DISABLED, NULL}, // 8
    {EXT_CH_MODE_DISABLED, NULL}, // 9
    {EXT_CH_MODE_DISABLED, NULL}, // 10
    {EXT_CH_MODE_DISABLED, NULL}, // 11
    {EXT_CH_MODE_DISABLED, NULL}, // 12
    {EXT_CH_MODE_DISABLED, NULL}, // 13
    {EXT_CH_MODE_DISABLED, NULL}, // 14
    {EXT_CH_MODE_DISABLED, NULL}, // 15
    {EXT_CH_MODE_DISABLED, NULL}, // 16
    {EXT_CH_MODE_DISABLED, NULL}, // 17
    {EXT_CH_MODE_DISABLED, NULL}, // 18
    {EXT_CH_MODE_DISABLED, NULL}, // 19
    {EXT_CH_MODE_DISABLED, NULL}, // 20
    {EXT_CH_MODE_DISABLED, NULL}, // 21
    {EXT_CH_MODE_DISABLED, NULL}  // 22
}};

void exti_setup(void)
{
    chEvtObjectInit(&exti_events);
    extStart(&EXTD1, &extcfg);
}

void send_fn(void *arg, const void *p, size_t len)
{
    (void)arg;
    if (len > 0) {
        chSequentialStreamWrite((BaseSequentialStream *)&SD2, (uint8_t *)p, len);
    }
}

static THD_WORKING_AREA(imu_thread, 256);
THD_FUNCTION(imu_thread_main, arg)
{
    (void) arg;
    // mpu pwr off
    palClearPad(GPIOB, GPIOB_PIN5);
    chThdSleepMilliseconds(100);
    // mpu pwr on
    palSetPad(GPIOB, GPIOB_PIN5);
    chThdSleepMilliseconds(100);

    I2CDriver *i2c_driver = &I2CD2;
    static const I2CConfig i2c_cfg = {
        .cr1 = 0,
        .cr2 = 0,
        // for 48MHz I2CC clock
        // 400kHz: PRESC 5, SCLDEL 0x3, SDADEL 0x3, SCLH 0x3, SCLL 0x3
        .timingr = (5<<28) | (0x3<<20) | (0x3<<16) | (0x3<<8) | (0x9<<0)
        // 100kHz: PRESC 0xb, SCLDEL 0x4, SDADEL 0x2, SCLH 0xf, SCLL 0x13
        // .timingr = (0xb<<28) | (0x4<<20) | (0x2<<16) | (0xf<<8) | (0x13<<0)
    };
    i2cStart(i2c_driver, &i2c_cfg);

    static mpu60X0_t mpu;
    mpu60X0_init_using_i2c(&mpu, i2c_driver, 0);

    i2cAcquireBus(i2c_driver);
    bool ok = false;
    if (mpu60X0_ping(&mpu)) {
        int config = MPU60X0_LOW_PASS_FILTER_6 | MPU60X0_SAMPLE_RATE_DIV(0);
        config |= MPU60X0_ACC_FULL_RANGE_2G;
        config |= MPU60X0_GYRO_FULL_RANGE_250DPS;
        mpu60X0_setup(&mpu, config);
        // check that the sensor still pings
        if (mpu60X0_ping(&mpu)) {
            ok = true;
        }
    }
    i2cReleaseBus(i2c_driver);

    if (!ok) {
        chSysHalt("mpu60x0 init failed");
    }
    chprintf(stdout, "IMU initialized\n");

    const eventmask_t mpu_interrupt_event = 1;
    static event_listener_t mpu_int_listener;
    chEvtRegisterMaskWithFlags(&exti_events, &mpu_int_listener,
                               (eventmask_t)mpu_interrupt_event,
                               (eventflags_t)EXTI_EVENT_MPU6050_INT);

    while (1) {
        chEvtWaitAnyTimeout(mpu_interrupt_event, OSAL_MS2ST(100));
        eventflags_t event_flag = chEvtGetAndClearFlags(&mpu_int_listener);
        if (event_flag & EXTI_EVENT_MPU6050_INT) {
            static float gyro[3], acc[3], temp;
            i2cAcquireBus(i2c_driver);
            mpu60X0_read(&mpu, gyro, acc, &temp);
            i2cReleaseBus(i2c_driver);

            static cmp_ctx_t ctx;
            static cmp_mem_access_t cma;
            static uint8_t dtgrm[100];
            cmp_mem_access_init(&ctx, &cma, dtgrm, sizeof(dtgrm));
            cmp_write_array(&ctx, 6);
            cmp_write_float(&ctx, gyro[0]);
            cmp_write_float(&ctx, gyro[1]);
            cmp_write_float(&ctx, gyro[2]);
            cmp_write_float(&ctx, acc[0]);
            cmp_write_float(&ctx, acc[1]);
            cmp_write_float(&ctx, acc[2]);
            size_t len = cmp_mem_access_get_pos(&cma);
            serial_datagram_send(dtgrm, len, send_fn, NULL);
        } else {
            chSysHalt("IMU timeout\n");
        }

        chThdSleepMilliseconds(10);
    }
}

char panic_log[500];
MemoryStream panic_log_stream;

void panic_log_init(void)
{
    memset(&panic_log[0], 0, sizeof(panic_log));
    // initialize stream object with buffer
    msObjectInit(&panic_log_stream, (uint8_t *)&panic_log[0],
        sizeof(panic_log) - 1, 0); // size - 1 for null terminated message
}

void fault_printf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    chvprintf((BaseSequentialStream *)&panic_log_stream, fmt, ap);
    va_end(ap);
}


#define BOOT_ARG_START_BOOTLOADER               0x00
#define BOOT_ARG_START_BOOTLOADER_NO_TIMEOUT    0x01
#define BOOT_ARG_START_APPLICATION              0x02
#define BOOT_ARG_START_ST_BOOTLOADER            0x03

#define BOOT_ARG_MAGIC_VALUE_LO 0x01234567
#define BOOT_ARG_MAGIC_VALUE_HI 0x0089abcd

void reboot_system(uint8_t arg)
{
    uint32_t *ram_start = (uint32_t *)0x20000000;

    ram_start[0] = BOOT_ARG_MAGIC_VALUE_LO;
    ram_start[1] = BOOT_ARG_MAGIC_VALUE_HI | (arg << 24);

    NVIC_SystemReset();
}


void panic_hook(const char *reason)
{
    (void)reason;
    reboot_system(BOOT_ARG_START_APPLICATION);
    // palSetPad(GPIOA, GPIOA_LED);
    // while (1);
}

static const SerialConfig debug_serial_config =
{
    921600,
    0,
    USART_CR2_STOP1_BITS | USART_CR2_LINEN,
    0
};

static const SerialConfig stream_serial_config =
{
    921600,
    0,
    USART_CR2_STOP1_BITS | USART_CR2_LINEN,
    0
};

int main(void)
{
    halInit();
    chSysInit();
    fault_init();
    panic_log_init();

    // UART1: TX=PB6  RX=PB7
    palSetPadMode(GPIOB, GPIOB_PIN6, PAL_MODE_ALTERNATE(7) |
                                     PAL_STM32_OTYPE_PUSHPULL |
                                     PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(GPIOB, GPIOB_PIN7, PAL_MODE_ALTERNATE(7) |
                                     PAL_STM32_PUDR_PULLUP);
    // UART2: TX=PB3
    palSetPadMode(GPIOB, GPIOB_PIN3, PAL_MODE_ALTERNATE(7) |
                                     PAL_STM32_OTYPE_PUSHPULL |
                                     PAL_STM32_OSPEED_HIGHEST);
    // I2C2: SCL=PA9 SDA=PA10
    palSetPadMode(GPIOA, GPIOA_PIN9, PAL_MODE_ALTERNATE(4) |
                                     PAL_STM32_OTYPE_OPENDRAIN |
                                     PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(GPIOA, GPIOA_PIN10, PAL_MODE_ALTERNATE(4) |
                                      PAL_STM32_OTYPE_OPENDRAIN |
                                      PAL_STM32_OSPEED_HIGHEST);
    // GPIOB, GPIOB_PIN4 MPU INT input
    palSetPadMode(GPIOB, GPIOB_PIN4, PAL_MODE_INPUT_PULLDOWN);
    // GPIOB, GPIOB_PIN5 MPU PWR output
    palSetPadMode(GPIOB, GPIOB_PIN5, PAL_MODE_OUTPUT_PUSHPULL |
                                     PAL_STM32_OSPEED_HIGHEST);

    exti_setup();

    sdStart(&SD1, &debug_serial_config);
    sdStart(&SD2, &stream_serial_config);

    stdout = (BaseSequentialStream *)&SD1;
    chprintf(stdout, "\nboot\n");

    chThdCreateStatic(imu_thread, sizeof(imu_thread), NORMALPRIO, imu_thread_main, NULL);
    while (1) {
        palSetPad(GPIOA, GPIOA_LED);
        chThdSleepMilliseconds(80);
        palClearPad(GPIOA, GPIOA_LED);
        chThdSleepMilliseconds(80);
        palSetPad(GPIOA, GPIOA_LED);
        chThdSleepMilliseconds(80);
        palClearPad(GPIOA, GPIOA_LED);
        chThdSleepMilliseconds(760);
    }
}

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>

#include <OMM_machine_common.h>
#include <spi_common.h>
#include <soft_spi.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>

/*
 * Uncomment if you want to use software SPI
*/
//#define USE_SOFTSPI

static uint32_t omm_timer_tick_val = 0;

/* Used for printf */
int _write(int file, char *ptr, int len)
{
    int i;

    if (file == STDOUT_FILENO || file == STDERR_FILENO) {
        for (i = 0; i < len; i++) {
            if (ptr[i] == '\n') {
                usart_send_blocking(USART1, '\r');
            }
            usart_send_blocking(USART1, ptr[i]);
        }
        return i;
    }
    errno = EIO;
    return -1;
}

static const struct rcc_clock_scale clock_72MHZ = {
    .pllmul = RCC_CFGR_PLLMUL_MUL9,
    .pllsrc = RCC_CFGR_PLLSRC_HSE_PREDIV,
    .hpre = RCC_CFGR_HPRE_DIV_NONE,
    .ppre1 = RCC_CFGR_PPRE1_DIV_2,
    .ppre2 = RCC_CFGR_PPRE2_DIV_NONE,
    .power_save = 0,
    .flash_waitstates = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2WS,
    .ahb_frequency  = 72000000,
    .apb1_frequency = 36000000,
    .apb2_frequency = 72000000,
};

static void rcc_clock_setup_hse(const struct rcc_clock_scale *clock)
{
    /* Enable internal high-speed oscillator. */
    rcc_osc_on(RCC_HSI);
    rcc_wait_for_osc_ready(RCC_HSI);
    /* Select HSI as SYSCLK source. */
    rcc_set_sysclk_source(RCC_CFGR_SW_HSI); /* XXX: se cayo */
    rcc_wait_for_sysclk_status(RCC_HSI);

    /* Enable external high-speed oscillator 8MHz. */
    rcc_osc_on(RCC_HSE);
    rcc_wait_for_osc_ready(RCC_HSE);

    rcc_osc_off(RCC_PLL);
    rcc_wait_for_osc_not_ready(RCC_PLL);
    rcc_set_pll_source(clock->pllsrc);
    rcc_set_pll_multiplier(clock->pllmul);
    /* Enable PLL oscillator and wait for it to stabilize. */
    rcc_osc_on(RCC_PLL);
    rcc_wait_for_osc_ready(RCC_PLL);
    /*
     * Set prescalers for AHB, ADC, ABP1, ABP2.
     * Do this before touching the PLL (TODO: why?).
     */
    rcc_set_hpre(clock->hpre);
    rcc_set_ppre2(clock->ppre2);
    rcc_set_ppre1(clock->ppre1);
    /* Configure flash settings. */
    flash_set_ws(clock->flash_waitstates);
    /* Select PLL as SYSCLK source. */
    rcc_set_sysclk_source(RCC_CFGR_SW_PLL); /* XXX: se cayo */
    /* Wait for PLL clock to be selected. */
    rcc_wait_for_sysclk_status(RCC_PLL);

    /* Set the peripheral clock frequencies used. */
    rcc_ahb_frequency  = clock->ahb_frequency;
    rcc_apb1_frequency = clock->apb1_frequency;
    rcc_apb2_frequency = clock->apb2_frequency;
}

static void MACH_STM32F3_clock_setup(void)
{
    #if(STM32_SYSTEM_CLOCK == 64000000)
    rcc_clock_setup_hsi(&rcc_hsi_8mhz[RCC_CLOCK_64MHZ]);
    #elif (STM32_SYSTEM_CLOCK == 72000000)
    rcc_clock_setup_hse(&clock_72MHZ);
    #else
    #error("Currently only 64MHZ HSI supported!")
    #endif
}

static void uart1_init(void)
{
    /* port pins and peripherals */
    rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_GPIOA);

    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
    gpio_set_af(GPIOA, GPIO_AF7, GPIO9);
    /* FIXME - if you're using rs485, you probably want a tx-enable pin here */

    /* Setup UART parameters. */
    usart_set_baudrate(USART1, 115200);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_set_mode(USART1, USART_MODE_TX);

    /* Only support 1 stop bit here, even though "none" should use 2 */
    /* Observe that ST has a different idea of counting databits */
    /* 9 bits only if parity is set, or stop bits > 1 */
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_databits(USART1, 8);

    usart_set_parity(USART1, USART_PARITY_NONE);

    usart_enable(USART1);
}

void enc28j60_irq_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOA);

    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO0);

    nvic_set_priority(NVIC_EXTI0_IRQ, 2);

    /* Enable EXTI0 interrupt. */
    nvic_enable_irq(NVIC_EXTI0_IRQ);

    /* Configure the EXTI subsystem. */
    exti_select_source(EXTI0, GPIOA);
    exti_set_trigger(EXTI0, EXTI_TRIGGER_FALLING);
    exti_enable_request(EXTI0);
}

#ifdef USE_SOFTSPI
static void lowlevel_softspi_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOE);
    gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO3);

    /* gpioB13 -> SCK */
    /* gpioB14 -> SO */
    /* gpioB15 -> SI */
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO13 );
    gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO14 );
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO15 );

    /* Start with spi communication disabled */
    gpio_set(GPIOE, GPIO3);
    gpio_set(GPIOB, GPIO13);
    gpio_set(GPIOB, GPIO15);
}

static void set_cs(uint8_t pin, uint8_t val)
{
    if(val)
    {
        gpio_set(GPIOE, GPIO3);
    }
    else
    {
        gpio_clear(GPIOE, GPIO3);
    }
}

static void set_mosi(uint8_t val)
{
    if(val)
        gpio_set(GPIOB, GPIO15);
    else
        gpio_clear(GPIOB, GPIO15);
}

static void set_clk(uint8_t val)
{
    if(val)
        gpio_set(GPIOB, GPIO13);
    else
        gpio_clear(GPIOB, GPIO13);
}

static uint8_t get_miso(void)
{
    if(gpio_get(GPIOB, GPIO14))
        return SOFT_SPI_HIGH;
    else
        return SOFT_SPI_LOW;
}
#else
static void lowlevel_hardspi_setup(void)
{
    rcc_periph_clock_enable(RCC_SPI2);
    /* For spi signal pins */
    rcc_periph_clock_enable(RCC_GPIOB);
    /* For spi mode select on the l3gd20 */
    rcc_periph_clock_enable(RCC_GPIOE);

    /* Setup GPIOE3 pin for spi mode l3gd20 select. */
    gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO3);
    /* Start with spi communication disabled */
    gpio_set(GPIOE, GPIO3);

    /* Setup GPIO pins for AF5 for SPI2 signals. */
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO13 | GPIO14 | GPIO15);
    gpio_set_af(GPIOB, GPIO_AF5, GPIO13 | GPIO14 | GPIO15);

    //spi initialization;
    spi_set_master_mode(SPI2);
    spi_set_baudrate_prescaler(SPI2, SPI_CR1_BR_FPCLK_DIV_8);
    spi_set_clock_polarity_0(SPI2);
    spi_set_clock_phase_0(SPI2);
    spi_set_full_duplex_mode(SPI2);
    spi_set_unidirectional_mode(SPI2); /* bidirectional but in 3-wire */
    spi_set_data_size(SPI2, SPI_CR2_DS_8BIT);
    spi_enable_software_slave_management(SPI2);
    spi_send_msb_first(SPI2);
    spi_set_nss_high(SPI2);
    spi_disable_ss_output(SPI2);
    spi_fifo_reception_threshold_8bit(SPI2);
    spi_disable_crc(SPI2);
    SPI_I2SCFGR(SPI2) &= ~SPI_I2SCFGR_I2SMOD;
    spi_enable(SPI2);
}

static void spi2_set_cs(spi_device_t *spi, uint8_t val)
{
    //printf("%s %u\n", __FUNCTION__, val);

    if(spi->pin > 0)
    {
        /*FIXME  ASSERT here */
        return;
    }

    if(val == SPI_COMMON_HIGH)
    {
        gpio_set(GPIOE, GPIO3);
    }
    else if(val == SPI_COMMON_LOW)
    {
        gpio_clear(GPIOE, GPIO3);
    }
}

static uint8_t spi2_transfer_byte(spi_device_t *spi, uint8_t byte)
{
    uint8_t ret_val = 0;
    
    spi2_set_cs(spi, SPI_COMMON_LOW);
    spi_send8(SPI2, byte);

    ret_val = spi_read8(SPI2);
    spi2_set_cs(spi, SPI_COMMON_HIGH);

    return ret_val;
}

static void spi2_transfer_msg(spi_device_t *spi, uint8_t *data_out, uint8_t *data_in, uint16_t len)
{
    int i = 0;

    spi2_set_cs(spi, SPI_COMMON_LOW);
    for(i=0;i<len;i++)
    {
        if(data_out == NULL)
            spi_send8(SPI2, 0x00);
        else
            spi_send8(SPI2, data_out[i]);

        if(data_in != NULL)
            data_in[i] = spi_read8(SPI2);
        else
            spi_read8(SPI2);
    }
    spi2_set_cs(spi, SPI_COMMON_HIGH);
}

static uint8_t spi2_transfer_byte_cs_off(spi_device_t *spi, uint8_t byte)
{
    spi_send8(SPI2, byte);
    return spi_read8(SPI2);
}

static void hardspi_setup(spi_master_t *spi_master)
{
    spi_master->SPI_set_cs = spi2_set_cs;
    spi_master->SPI_transfer_byte = spi2_transfer_byte;
    spi_master->SPI_transfer_msg = spi2_transfer_msg;
    spi_master->SPI_transfer_byte_cs_off = spi2_transfer_byte_cs_off;
    spi_master->platform_drv_info = NULL;
}
#endif

static void clktick_common_setup(void)
{
    #if 0
    /* 72MHz / 8 => 9000000 counts per second */
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);

    /* 9000000/9000 = 1000 overflows per second - every 1ms one interrupt */
    /* SysTick interrupt every N clock pulses: set reload to N-1 */
    systick_set_reload(0xFFFFFFFF);

    /* Start counting. */
    systick_counter_enable();
    #endif

    /* Enable TIM2 clock. */
    rcc_periph_clock_enable(RCC_TIM2);

    /* Reset TIM2 peripheral to defaults. */
    rcc_periph_reset_pulse(RST_TIM2);

    timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

    timer_set_prescaler(TIM2, ((rcc_apb1_frequency * 2) / 100000));

    /* Disable preload. */
    timer_disable_preload(TIM2);
    timer_continuous_mode(TIM2);

    /* count full range, as we'll update compare value continuously */
    timer_set_period(TIM2, 65535);

    timer_set_oc_value(TIM2, TIM_OC1, 65535);

    /* Counter enable. */
    timer_enable_counter(TIM2);

    /* Enable Channel 1 compare interrupt to recalculate compare values */
    timer_enable_irq(TIM2, TIM_DIER_CC1IE);

    nvic_set_priority(NVIC_TIM2_IRQ, 0);

    /* Enable TIM2 interrupt. */
    nvic_enable_irq(NVIC_TIM2_IRQ);
}

uint32_t OMM_get_clktick(void)
{
    return omm_timer_tick_val+timer_get_counter(TIM2);
}

void tim2_isr(void)
{
    timer_clear_flag(TIM2, TIM_SR_CC1IF);
    omm_timer_tick_val += 65535;
}

OMM_machine_t *machine_setup(void)
{
    static OMM_machine_t machine =
    {
            .name = "stm32_lwip"
    };

#ifdef USE_SOFTSPI
    static soft_spi_t soft_spi_enc28j60;
#endif
    static spi_master_t spi2_master;
    static spi_device_t spi_enc28j60;

    static OMM_platform_devices pdevs[] =
    {
            {"spi2", &spi2_master},
            {"enc28j60_spi", &spi_enc28j60},
            {NULL, NULL}
    };

    /* enable 16 bit priority, no subgroups */
    scb_set_priority_grouping(SCB_AIRCR_PRIGROUP_GROUP16_NOSUB);

    /* Clock setup */
    MACH_STM32F3_clock_setup();

    uart1_init();
   
#ifdef USE_SOFTSPI
    lowlevel_softspi_setup();

    soft_spi_enc28j60.pdev = &spi_enc28j60;
    soft_spi_enc28j60.set_mosi = set_mosi;
    soft_spi_enc28j60.get_miso = get_miso;
    soft_spi_enc28j60.set_clk = set_clk;
    soft_spi_enc28j60.set_cs = set_cs;

    /* Attach soft spi function to master */
    SOFT_SPI_master_new(&spi2_master, &soft_spi_enc28j60);
#else
    /* lowlevel initialization */
    lowlevel_hardspi_setup();

    /* spi common master abstraction layer */
    hardspi_setup(&spi2_master);
#endif

    /* Attach spi device to master */
    SPI_init_device(&spi_enc28j60, MODE_0_0, 0, &spi2_master);

    /* Enable systick */
    clktick_common_setup();

    /* exti interrupt pin */
    //enc28j60_irq_setup();

    machine.pdev_list = &pdevs[0];

    return &machine;
}

void main_set_link_callback(void);

void disable_ethernet_irq(void)
{
    /* Disable EXTI0 interrupt. */
    nvic_disable_irq(NVIC_EXTI0_IRQ);
}

void enable_ethernet_irq(void)
{
    /* Enable EXTI0 interrupt. */
    nvic_enable_irq(NVIC_EXTI0_IRQ);
}

void exti0_isr(void)
{
    exti_reset_request(EXTI0);
    printf("exti interrupt!\n");

    //main_set_link_callback();
}

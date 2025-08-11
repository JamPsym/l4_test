#include <stdint.h>
#include "stm32l432xx.h"
#include "stm32l4xx.h"
#include "system_stm32l4xx.h"

#include <stddef.h>

#include <stm32l4xx_ll_bus.h>
#include <stm32l4xx_ll_utils.h>
#include <stm32l4xx_ll_gpio.h>
#include <stm32l4xx_ll_tim.h>


static void led_pins(void);
static void led_pins_pwm(void);
static void encoder_init(void);
static void pwm_15_init(void);
static void toggle_all_pins(void);
static const uint32_t ll_high_pins[4] = {LL_GPIO_PIN_1,LL_GPIO_PIN_3,LL_GPIO_PIN_10,LL_GPIO_PIN_11}; //A1 A2 D0 D10
static const uint32_t ll_low_pins[4] = {LL_GPIO_PIN_7, LL_GPIO_PIN_6,LL_GPIO_PIN_5,LL_GPIO_PIN_4}; 

const uint8_t (*led_state)[4][4];

/* chessboard */
const uint8_t led_state_1[4][4] = {{1,0,1,0},{0,1,0,1},{1,0,1,0},{0,1,0,1}};
const uint8_t led_state_2[4][4] = {{0,1,0,1},{1,0,1,0},{0,1,0,1},{1,0,1,0}};

/* on-off 
const uint8_t led_state_1[4][4] = {{1,1,1,0},{1,1,1,0},{1,1,1,0},{1,1,1,0}};
const uint8_t led_state_2[4][4] = {{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};
*/

/* literki */
const uint8_t led_state_c[4][4] = { {0,1,1,1},
                                    {1,0,0,0},
                                    {1,0,0,0},
                                    {0,1,1,0}};
const uint8_t led_state_w[4][4] = { {1,0,0,1},
                                    {1,0,0,1},
                                    {1,1,1,1},
                                    {0,1,1,0}};
const uint8_t led_state_e[4][4] = { {1,1,1,1},
                                    {1,0,0,0},
                                    {1,1,1,0},
                                    {1,1,1,1}};
const uint8_t led_state_l[4][4] = { {1,0,0,0},
                                    {1,0,0,0},
                                    {1,0,0,0},
                                    {1,1,1,1}};

//uint8_t led_state_1[4][4] = {{1,1,1,1},{0,0,0,0}, {1,1,1,1}, {0,0,0,0}};

static uint8_t led_state_index;
const uint8_t (*led_state_array[])[4][4] = {&led_state_c, &led_state_w, &led_state_e, &led_state_l};
static const size_t led_state_len = sizeof(led_state_array)/sizeof(led_state_array[0]);

static void mux_leds(const uint8_t (*led_state)[4][4])
{
    static size_t row = 3;

    LL_GPIO_ResetOutputPin(GPIOA, ll_low_pins[row]);
    for(size_t i = 1; i<4; i++){ 
        size_t index = (row + i) & 0x3;
        LL_GPIO_SetOutputPin(GPIOA, ll_low_pins[index]);
    }

    for(size_t i = 0; i<4; i++) {
        (*led_state)[row][i] ? LL_GPIO_SetOutputPin(GPIOA, ll_high_pins[i]) : LL_GPIO_ResetOutputPin(GPIOA, ll_high_pins[i]);
    }

    row++;
    row &= 0x3;
}

int main(void)
{
    led_pins_pwm();

    SystemCoreClockUpdate();
    LL_Init1msTick(SystemCoreClock);

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);

    LL_TIM_InitTypeDef tim;
    LL_TIM_StructInit(&tim);
    tim.Autoreload = 250;
    tim.Prescaler = SystemCoreClock/1000 - 1;
    LL_TIM_Init(TIM6, &tim);
    LL_TIM_EnableIT_UPDATE(TIM6);
    LL_TIM_EnableCounter(TIM6);

    NVIC_EnableIRQ(TIM6_DAC_IRQn);

    encoder_init();

    pwm_15_init();
    //pwm test
    for(size_t i = 0; i<4; i++){ 
        LL_GPIO_ResetOutputPin(GPIOA, ll_low_pins[i]);
    }

    while(1)
    {
        //mux_leds(led_state);
        LL_mDelay(1);

        volatile int encoder = LL_TIM_GetCounter(TIM1);


    }
}

void TIM6_DAC_IRQHandler()
{
    led_state = led_state_array[led_state_index];

    led_state_index++;
    if(led_state_index == led_state_len)
        led_state_index = 0;
    LL_TIM_ClearFlag_UPDATE(TIM6);
}

static void toggle_all_pins(void){
    // 1 3 10 11
    for(size_t i = 0; i < sizeof(ll_high_pins)/sizeof(uint32_t); i++)
        LL_GPIO_TogglePin(GPIOA, ll_high_pins[i]);
}

static void led_pins(void)
{
    /* Enable clock for GPIOA */
    SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_GPIOAEN);

    /*
     * PA1, PA3, PA10, PA11 are high side drivers
     * PA4, PA5, PA6, PA7 are low side drivers
     */

    /* Configure pins as output */
    MODIFY_REG(GPIOA->MODER,
               (GPIO_MODER_MODE1 | GPIO_MODER_MODE3 | GPIO_MODER_MODE4 |
                GPIO_MODER_MODE5 | GPIO_MODER_MODE6 | GPIO_MODER_MODER7 |
                GPIO_MODER_MODE10 | GPIO_MODER_MODE11),
               (GPIO_MODER_MODE1_0 | GPIO_MODER_MODE3_0 | GPIO_MODER_MODE4_0 |
                GPIO_MODER_MODE5_0 | GPIO_MODER_MODE6_0 | GPIO_MODER_MODER7_0 |
                GPIO_MODER_MODE10_0 | GPIO_MODER_MODE11_0));

    /* Initialize low side drivers to low */
    CLEAR_BIT(GPIOA->ODR, GPIO_ODR_OD4);
    CLEAR_BIT(GPIOA->ODR, GPIO_ODR_OD5);
    CLEAR_BIT(GPIOA->ODR, GPIO_ODR_OD6);
    CLEAR_BIT(GPIOA->ODR, GPIO_ODR_OD7);

    /* Initialize high side drivers to high */
    SET_BIT(GPIOA->ODR, GPIO_ODR_OD1);
    SET_BIT(GPIOA->ODR, GPIO_ODR_OD3);
    SET_BIT(GPIOA->ODR, GPIO_ODR_OD10);
    SET_BIT(GPIOA->ODR, GPIO_ODR_OD11);
}

static void led_pins_pwm(void)
{
    /* Enable clock for GPIOA */
    SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_GPIOAEN);

    /*
     * PA1, PA3, PA10, PA11 are high side drivers
     * PA4, PA5, PA6, PA7 are low side drivers
     */

    /* Configure pins as output */
    MODIFY_REG(GPIOA->MODER,
               (GPIO_MODER_MODE1 | GPIO_MODER_MODE3 | GPIO_MODER_MODE4 |
                GPIO_MODER_MODE5 | GPIO_MODER_MODE6 | GPIO_MODER_MODER7 |
                GPIO_MODER_MODE10 | GPIO_MODER_MODE11),
               (GPIO_MODER_MODE1_0 | GPIO_MODER_MODE3_0 | GPIO_MODER_MODE4_0 |
                GPIO_MODER_MODE5_0 | GPIO_MODER_MODE6_0 | GPIO_MODER_MODER7_0 |
                GPIO_MODER_MODE10_0 | GPIO_MODER_MODE11_0));

    /* Initialize low side drivers to low */
    CLEAR_BIT(GPIOA->ODR, GPIO_ODR_OD4);
    CLEAR_BIT(GPIOA->ODR, GPIO_ODR_OD5);
    CLEAR_BIT(GPIOA->ODR, GPIO_ODR_OD6);
    CLEAR_BIT(GPIOA->ODR, GPIO_ODR_OD7);
}

void encoder_init()
{
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    // encoder config
    // CLK pin D9 - pwm1/1 PA8
    // DT pin D1 - pwm1/2 PA9
    LL_GPIO_InitTypeDef gpio_en;
    LL_GPIO_StructInit(&gpio_en);
    gpio_en.Pin = LL_GPIO_PIN_8 | LL_GPIO_PIN_9;
    gpio_en.Mode = LL_GPIO_MODE_ALTERNATE;
    gpio_en.Alternate = LL_GPIO_AF_1;
    LL_GPIO_Init(GPIOA, &gpio_en);

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

    LL_TIM_InitTypeDef tim1_init;
    LL_TIM_StructInit(&tim1_init);
    tim1_init.Autoreload = 1000;
    LL_TIM_Init(TIM1, &tim1_init);

    LL_TIM_ENCODER_InitTypeDef enc_init;
    LL_TIM_ENCODER_StructInit(&enc_init);
    enc_init.EncoderMode = LL_TIM_ENCODERMODE_X4_TI12;
    LL_TIM_ENCODER_Init(TIM1, &enc_init);

    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);

    LL_TIM_EnableCounter(TIM1);
}

void pwm_15_init()
{
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);

    LL_GPIO_InitTypeDef gpio_pwm;
    LL_GPIO_StructInit(&gpio_pwm);
    gpio_pwm.Alternate = LL_GPIO_AF_14;
    gpio_pwm.Pin = LL_GPIO_PIN_1 | LL_GPIO_PIN_3;
    gpio_pwm.Mode = LL_GPIO_MODE_ALTERNATE;
    LL_GPIO_Init(GPIOA, &gpio_pwm);

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM15);
    LL_TIM_InitTypeDef tim15_init;
    LL_TIM_StructInit(&tim15_init);
    tim15_init.Autoreload = 1000;
    tim15_init.Prescaler = SystemCoreClock/1000000 - 1;
    LL_TIM_Init(TIM15, &tim15_init);

    LL_TIM_OC_InitTypeDef pwm_init;
    LL_TIM_OC_StructInit(&pwm_init);
    pwm_init.CompareValue = 300;
    pwm_init.OCMode = LL_TIM_OCMODE_PWM1;
    pwm_init.OCNState = LL_TIM_OCSTATE_ENABLE;
    pwm_init.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
    LL_TIM_OC_Init(TIM15, LL_TIM_CHANNEL_CH1, &pwm_init);

    LL_TIM_OC_StructInit(&pwm_init);
    pwm_init.CompareValue = 300;
    pwm_init.OCMode = LL_TIM_OCMODE_PWM1;
    pwm_init.OCState = LL_TIM_OCSTATE_ENABLE;
    LL_TIM_OC_Init(TIM15, LL_TIM_CHANNEL_CH2, &pwm_init);

    //LL_TIM_OC_EnablePreload(TIM15, LL_TIM_CHANNEL_CH1);
    //LL_TIM_OC_EnablePreload(TIM15, LL_TIM_CHANNEL_CH2);
    //LL_TIM_EnableARRPreload(TIM15);
    //LL_TIM_EnableUpdateEvent(TIM15);

    LL_TIM_CC_EnableChannel(TIM15, LL_TIM_CHANNEL_CH1N);
    LL_TIM_CC_EnableChannel(TIM15, LL_TIM_CHANNEL_CH2);

    LL_TIM_EnableAllOutputs(TIM15);

    //LL_TIM_GenerateEvent_UPDATE(TIM15);
    LL_TIM_EnableCounter(TIM15);

    // tutaj dodatkowo: preload registers dla kanałów i dla ARR tysz, enable all outputs, i generate event lol
}

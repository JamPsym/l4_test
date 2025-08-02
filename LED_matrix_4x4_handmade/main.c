#include <stdint.h>
#include "cmsis_gcc.h"
#include "stm32l432xx.h"
#include "stm32l4xx.h"
#include "system_stm32l4xx.h"

#include <stddef.h>

#include <stm32l4xx_ll_bus.h>
#include <stm32l4xx_ll_utils.h>
#include <stm32l4xx_ll_gpio.h>
#include <stm32l4xx_ll_tim.h>


static void led_pins(void);
static void toggle_all_pins(void);
static const uint32_t ll_high_pins[4] = {LL_GPIO_PIN_1,LL_GPIO_PIN_3,LL_GPIO_PIN_10,LL_GPIO_PIN_11};
static const uint32_t ll_low_pins[4] = {LL_GPIO_PIN_4,LL_GPIO_PIN_5,LL_GPIO_PIN_6,LL_GPIO_PIN_7};

uint8_t (*led_state)[4][4];
uint8_t led_state_1[4][4] = {{1,0,1,0},{0,1,0,1},{1,0,1,0},{0,1,0,1}};
uint8_t led_state_2[4][4] = {{0,1,0,1},{1,0,1,0},{0,1,0,1},{1,0,1,0}};

static void mux_leds(uint8_t (*led_state)[4][4])
{
    static size_t row = 0;

    LL_GPIO_ResetOutputPin(GPIOA, ll_low_pins[row]);
    for(size_t i = 1; i<=3; i++){
        size_t index = (row + i) & 0x3;
        LL_GPIO_SetOutputPin(GPIOA, ll_low_pins[index]);
    }

    for(size_t i = 0; i<4; i++){
        (*led_state)[row][i] ? LL_GPIO_SetOutputPin(GPIOA, ll_high_pins[i]) : LL_GPIO_ResetOutputPin(GPIOA, ll_high_pins[i]);
    }

    row++;
    row &= 0x3;
}

int main(void)
{
    led_pins();

    SystemCoreClockUpdate();
    LL_Init1msTick(SystemCoreClock);

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);

    LL_TIM_InitTypeDef tim;
    LL_TIM_StructInit(&tim);
    tim.Autoreload = 150;
    tim.Prescaler = SystemCoreClock/1000 - 1;
    LL_TIM_Init(TIM6, &tim);
    LL_TIM_EnableIT_UPDATE(TIM6);
    LL_TIM_EnableCounter(TIM6);

    NVIC_EnableIRQ(TIM6_DAC_IRQn);

    while(1)
    {
        mux_leds(led_state);
        LL_mDelay(1);
    }
}

void TIM6_DAC_IRQHandler()
{
    static uint8_t flag;
    led_state = flag ? &led_state_1 : &led_state_2;

    flag = !flag;
    LL_TIM_ClearFlag_UPDATE(TIM6);
}

static void toggle_all_pins(void){
    // 1 3 10 11
    for(size_t i = 0; i < sizeof(ll_high_pins)/sizeof(uint32_t); i++)
        LL_GPIO_TogglePin(GPIOA, ll_high_pins[i]);
}

static void led_pins(void)
{
    SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_GPIOAEN);
    // PA7 gpio 1
    // PA6 gpio 2
    // PA5 gpio 3
    // PA4 gpio 4

    // PA1 PWM15/1N
    // PA3 PWM15/2 

    // PA10 PWM1/3
    // PA11 PWM1/4
    
    MODIFY_REG(GPIOA->MODER,
            (GPIO_MODER_MODE1 | GPIO_MODER_MODE3 | GPIO_MODER_MODE4 | GPIO_MODER_MODE5 | GPIO_MODER_MODE6 | GPIO_MODER_MODER7 | GPIO_MODER_MODE10 | GPIO_MODER_MODE11),
            (GPIO_MODER_MODE1_0 | GPIO_MODER_MODE3_0 | GPIO_MODER_MODE4_0 | GPIO_MODER_MODE5_0 | GPIO_MODER_MODE6_0 | GPIO_MODER_MODER7_0 | GPIO_MODER_MODE10_0 | GPIO_MODER_MODE11_0));

    CLEAR_BIT(GPIOA->ODR, GPIO_ODR_OD4);
    CLEAR_BIT(GPIOA->ODR, GPIO_ODR_OD5);
    CLEAR_BIT(GPIOA->ODR, GPIO_ODR_OD6);
    CLEAR_BIT(GPIOA->ODR, GPIO_ODR_OD7);

    SET_BIT(GPIOA->ODR, GPIO_ODR_OD1);
    SET_BIT(GPIOA->ODR, GPIO_ODR_OD3);
    SET_BIT(GPIOA->ODR, GPIO_ODR_OD10);
    SET_BIT(GPIOA->ODR, GPIO_ODR_OD11);
}


#include <string.h>
#include <stdint.h>
#include "stm32f446xx.h"
#include <math.h>
#include "system_stm32f4xx.h"

//#define SYS_CLOCK_FREQ 160000000  

#define IRQ_FREQ 15000
#define CPU_CLK_FREQ 160000000

//#define TICK_FREQ 
#define PI 3.141592
#define DAC_MAX 4095
#define AMP 1
#define OFFSET 1
#define FREQ 20
//#define ANGULAR_FREQUENCY 
#define POINTS 1000

volatile uint16_t current_dac_index = 0;
uint16_t sine_wave_table[POINTS];



void SystemClock_settings(void) {
    FLASH->ACR |= FLASH_ACR_LATENCY_5WS;
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));
    RCC->PLLCFGR = (8 << RCC_PLLCFGR_PLLM_Pos) | (160 << RCC_PLLCFGR_PLLN_Pos) |
                    (0 << RCC_PLLCFGR_PLLP_Pos) | (RCC_PLLCFGR_PLLSRC_HSE);
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));
    RCC->CFGR |= (RCC_CFGR_HPRE_DIV1) | (RCC_CFGR_PPRE1_DIV4) | (RCC_CFGR_PPRE2_DIV2);
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}


void init_GPIO_A_AND_C(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;  

    GPIOA->MODER |= (3 << (4 * 2));  

    GPIOC->MODER &= ~(3 << (4 * 2));  
    GPIOC->MODER |= (1 << (4 * 2));   
}

void Make_DAC_Output(void) {
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;
    DAC->CR |= DAC_CR_EN1;
    for (int i = 0; i < POINTS; i++) {
        sine_wave_table[i] = (uint16_t)((OFFSET + AMP * sinf((2.0f * PI * i ) / POINTS)) * (DAC_MAX/ 3.3));
    }
}

void SysTick_Handler(void) {
    GPIOC->ODR ^= (1 << 4);  
    DAC->DHR12R1 = sine_wave_table[current_dac_index];
    current_dac_index = (current_dac_index + 1) % POINTS;
}



int main(void) {
    #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
        SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));  
    #endif 

    SystemClock_settings();
    init_GPIO_A_AND_C();
    Make_DAC_Output();
    SysTick_Config(CPU_CLK_FREQ / (POINTS*FREQ));
    
    while (1) {

    }
}

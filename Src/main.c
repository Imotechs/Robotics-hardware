
#include <string.h>
#include <stdint.h>
#include "stm32f446xx.h"
#include <math.h>

#define CLK_FREQ 160000000 // CPU Frequency: 160 MHz
#define IRQ_CLK 15000      // Timer IRQ Frequency: 15 kHz
#define DURATION 1000       // Number of samples for sine wave
volatile uint16_t Dac_index = 0;
uint16_t Wave_formation[DURATION];

#define PI 3.14159265359
#define DAC_MAX 4095        // 12-bit DAC resolution (0-4095)
#define AMP 2.0             // Amplitude in volts
#define FREQ 20             //  Desired sine wave frequency in Hz 
#define OFFSET 1            // Offset for sine wave (Half of 3.3V for centered signal)

void SystemClock_settings(void) {
    // Set the Flash latency to 5 wait states for high-speed operation (160 MHz)
    FLASH->ACR |= FLASH_ACR_LATENCY_5WS;
    
    // Enable the High-Speed External (HSE) oscillator
    RCC->CR |= RCC_CR_HSEON;  
    // Wait until the HSE oscillator is ready
    while (!(RCC->CR & RCC_CR_HSERDY));  

    // PLL multiplier (PLL_N) set to 160, PLL divider (PLL_M) set to 8, and PLL source set to HSE (High-Speed External oscillator)
   RCC->PLLCFGR = (8 << RCC_PLLCFGR_PLLM_Pos) |  // PLLM = 6
               (160 << RCC_PLLCFGR_PLLN_Pos) | // PLLN = 160
               (0 << RCC_PLLCFGR_PLLP_Pos)  |  // PLLP = 2 (0 corresponds to /2)
               (RCC_PLLCFGR_PLLSRC_HSE);      // Select HSE as PLL source
                                        //PLLP can be 2, 4, 6, or 8, and it's encoded as:
                                        // 0 → PLLP = 2
                                        // 1 → PLLP = 4
                                        // 2 → PLLP = 6
                                        // 3 → PLLP = 8

    // Enable the PLL (Phase-Locked Loop)
    RCC->CR |= RCC_CR_PLLON;
    // Wait until the PLL is ready
    while (!(RCC->CR & RCC_CR_PLLRDY));

    // Configure the system clock prescalers:
    // - PPRE1: AHB1 peripheral clock divided by 4
    // - PPRE2: AHB2 peripheral clock divided by 2
    // - HPRE: AHB clock (system clock) divided by 1 (no division)
    RCC->CFGR |= (RCC_CFGR_PPRE1_DIV4) | (RCC_CFGR_PPRE2_DIV2) | (RCC_CFGR_HPRE_DIV1);

    // Select PLL as the system clock source
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    // Wait until the system clock switching to PLL is complete
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}


void init_GPIOA_C(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;  

    GPIOA->MODER |= (3 << (4 * 2));       

    GPIOC->MODER &= ~(3 << (4 * 2));  
    GPIOC->MODER |= (1 << (4 * 2));   
}

void make_DAC_Output(void) {
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;    
    DAC->CR |= DAC_CR_EN1;                

    for (int i = 0; i < DURATION; i++) {
        Wave_formation[i] = (uint16_t)((OFFSET + AMP * sinf(2 * PI * FREQ * i / DURATION)) * DAC_MAX / 3.3);
    }
}

void init_TIM6(void) {  
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;  // Enable TIM6 clock  
//TIM6->PSC = 0;                  // Prescaler = 80, Timer Clock = 1 MHz
TIM6->ARR = (CLK_FREQ / (FREQ * DURATION)) - 1;  
TIM6->DIER |= TIM_DIER_UIE;      // Enable Update Interrupt
TIM6->CR1 |= TIM_CR1_CEN;        // Start Timer
NVIC_EnableIRQ(TIM6_DAC_IRQn);   // Enable TIM6 Interrupt
} 

void TIM6_DAC_IRQHandler(void) {
    if (TIM6->SR & TIM_SR_UIF) {  // Check update flagddddddd'[[[[[]]]]]
        TIM6->SR &= ~TIM_SR_UIF;  // Clear the update interrupt flag

        GPIOC->ODR ^= (1 << 4);  
        DAC->DHR12R1 = Wave_formation[Dac_index];  
        Dac_index = (Dac_index + 1) % DURATION;  
    }
}

int main(void) {
    #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
        SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));  
    #endif

    SystemClock_settings();
    init_GPIOA_C();
    make_DAC_Output();
    init_TIM6();  // Use TIM6 instead of SysTick

    while (1) {

    }
}
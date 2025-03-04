#include "stm32f446xx.h"

#define HCLK_FREQ 100000000  // 100 MHz
#define SYSTICK_FREQ 5000    // 5 kHz
#define LED_TOGGLE_FREQ 1    // 1 Hz

void SystemClock_Config(void);
void SysTick_Handler(void);

int main(void) {
    // Configure the system clock
    SystemClock_Config();

    // Configure SysTick to generate an interrupt every 1/SYSTICK_FREQ seconds
    SysTick_Config(HCLK_FREQ / SYSTICK_FREQ);

    // Enable GPIOA clock (for LED)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // Configure PA5 as output (LED)
    GPIOA->MODER &= ~(3 << (5 * 2));  // Clear mode bits for PA5
    GPIOA->MODER |= (1 << (5 * 2));   // Set PA5 to output mode

    while (1) {
        // Main loop
    }
}

void SystemClock_Config(void) {
    // Enable HSE (High-Speed External clock)
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));

    // Configure PLL
    RCC->PLLCFGR = (8 << RCC_PLLCFGR_PLLM_Pos) |  // PLLM = 8
                   (200 << RCC_PLLCFGR_PLLN_Pos) | // PLLN = 200
                   (0 << RCC_PLLCFGR_PLLP_Pos) |  // PLLP = 2 (PLLP = 0 means divide by 2)
                   (4 << RCC_PLLCFGR_PLLQ_Pos);   // PLLQ = 4

    // Enable PLL
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    // Set FLASH latency
    FLASH->ACR = FLASH_ACR_LATENCY_3WS;  // 3 wait states for 100 MHz

    // Configure AHB, APB1, and APB2 prescalers
    RCC->CFGR = (0 << RCC_CFGR_HPRE_Pos) |  // AHB prescaler = 1 (no division)
                (4 << RCC_CFGR_PPRE1_Pos) | // APB1 prescaler = 2 (divide by 2)
                (0 << RCC_CFGR_PPRE2_Pos);  // APB2 prescaler = 1 (no division)

    // Switch to PLL as system clock source
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

void SysTick_Handler(void) {
    static uint32_t tick_count = 0;
    tick_count++;

    // Toggle LED every 5000 ticks (1 Hz)
    if (tick_count >= SYSTICK_FREQ / LED_TOGGLE_FREQ) {
        tick_count = 0;
        GPIOA->ODR ^= GPIO_ODR_OD5;  // Toggle PA5 (assuming LED is connected to PA5)
    }
}
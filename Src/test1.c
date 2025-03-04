#include <stdint.h>
// STM32F446RETx microcontroller specific addresses
#define GPIOD_BASE_ADDRESS  0x40020C00
#define GPIOC_BASE_ADDRESS  0x40020800
#define RCC_BASE_ADDRESS    0x40023800
#define RCC_AHB1ENR (*(volatile uint32_t *)(RCC_BASE_ADDRESS + 0x30))

// Define LED and button ports and pins for STM32F446RET6
#define LED_START_PIN  4
#define LED_COUNT      8

#define RED_BUTTON_PIN  13
#define BLUE_BUTTON_PIN 2

// GPIO structure
typedef struct {
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDR;
    volatile uint32_t PUPDR;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t LCKR;
    volatile uint32_t AFR[2];
} GPIO_TypeDef;

#define GPIOC  ((GPIO_TypeDef *) GPIOC_BASE_ADDRESS)
#define GPIOD  ((GPIO_TypeDef *) GPIOD_BASE_ADDRESS)

// Simple debounce function
void debounce(void) {
    for (volatile int i = 0; i < 10000; i++); // Short delay for debouncing
}

void delay(void) {
    for (volatile int i = 0; i < 500000; i++); // Short delay
}

// Initialize peripherals (GPIO pins and clocks)
void init_peripherals(void) {
    // Enable clock for GPIOC and GPIOD
    RCC_AHB1ENR |= (1 << 2) | (1 << 3);  
    (void)RCC_AHB1ENR; // Ensure the clock write completes

    // Configure LED pins as outputs

    GPIOC->MODER |= (0x5555 << (LED_START_PIN * 2));   // Set pins to output mode

    // Configure button pins as inputs with pull-up resistors
    GPIOC->MODER &= ~(3 << (RED_BUTTON_PIN * 2));  // Set to input mode
    GPIOD->MODER &= ~(3 << (BLUE_BUTTON_PIN * 2));  // Set to input mode
}

// Toggle LEDs based on button presses
int main(void) {
    uint32_t red_button_state = 1;
    uint32_t blue_button_state = 1;
    uint8_t led_count = 0; // Track the number of LEDs currently on

    // Initialize peripherals
    init_peripherals();

    // Initially turn off all LEDs
    GPIOC->ODR &= ~(0xFF << LED_START_PIN);  // Clear all LED bits on GPIOC
    GPIOD->ODR &= ~(0xFF << LED_START_PIN);  // Clear all LED bits on GPIOD

    while (1) {
        // Handle Red Button (Decrease lit LEDs)
        if (!(GPIOC->IDR & (1 << RED_BUTTON_PIN))) { // Check if red button is pressed

        
            debounce(); // Debounce the button
            if (!(GPIOC->IDR & (1 << RED_BUTTON_PIN))) { // Confirm press
                  
                if (red_button_state == 1) {
                    if (led_count > 0) {
                        led_count--;
                        GPIOC->ODR &= ~(1 << (LED_START_PIN + led_count)); // Turn off last LED on GPIOC
                        GPIOD->ODR &= ~(1 << (LED_START_PIN + led_count)); // Turn off last LED on GPIOD
                    }
                    red_button_state = 0; // Prevent re-triggering until released
                }

            }
        } 
        else {
            red_button_state = 1; // Reset when button is released
        }

        // Handle Blue Button (Increase lit LEDs)
        if (!(GPIOD->IDR & (1 << BLUE_BUTTON_PIN))) { // Check if blue button is pressed
            debounce(); // Debounce the button
            if (!(GPIOD->IDR & (1 << BLUE_BUTTON_PIN))) { // Confirm press
                if (blue_button_state == 1) {
                    if (led_count < LED_COUNT) {
                        led_count++;
                        GPIOC->ODR |= (1 << (LED_START_PIN + led_count - 1)); // Turn on next LED on GPIOC
                        GPIOD->ODR |= (1 << (LED_START_PIN + led_count - 1)); // Turn on next LED on GPIOD
                    }
                    if (led_count == 9) {
                    led_parade();
                    led_count = 0; // Reset count after parade
                }
                    blue_button_state = 0; // Prevent re-triggering until released
                }
            }
        } else {
            blue_button_state = 1; // Reset when button is released
        }
    }
}

void led_parade(void) {
    GPIOC->ODR &= ~((1 << 4) | (1 << 5) | (1 << 6) | (1 << 7) | (1 << 8) | (1 << 9) | (1 << 10) | (1 << 11));  
    for (int i=0,i<=3;i++){
        for (int i = 0; i < LED_COUNT; i++) {
                GPIOC->ODR |= (1 << (LED_START_PIN + i));
                delay();
            }
            delay()
            for (int i = LED_COUNT - 1; i >= 0; i--) {
                GPIOC->ODR &= ~(1 << (LED_START_PIN + i));
                delay();
            }
    }
   
}

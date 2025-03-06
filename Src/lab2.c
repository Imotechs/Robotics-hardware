#include "stm32f446xx.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#define RX_BUFFER_SIZE 12  // Fixed packet length for commands

volatile uint8_t rx_buffer[RX_BUFFER_SIZE];
volatile uint8_t rx_index = 0;
volatile uint8_t rx_complete = 0;
//volatile uint8_t report_status = 0;

volatile char tx_buffer[RX_BUFFER_SIZE] = {0};  // Buffer to hold data to send
volatile int tx_index = 0;  // Index to track current position in the buffer

void GPIO_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;  // Enable GPIOC clock
    GPIOC->MODER &= ~(0xFFFF << 8);       // Clear PC4 - PC11 mode bits
    GPIOC->MODER |= (0x5555 << 8);        // Set PC4 - PC11 as output
    GPIOC->OTYPER &= ~(0xFF << 4);        // Push-pull output
    GPIOC->OSPEEDR |= (0x55 << (4 * 2));  // Medium speed
    GPIOC->PUPDR &= ~(0xFF << (4 * 2));   // No pull-up/pull-down
    //GPIOC->ODR &= ~(0xFF << 4);           // Turn off all LEDs initially
}

void UART_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // Configure PA2 (TX) and PA3 (RX) as Alternate Function (AF7)
    GPIOA->MODER |= (GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1);
    GPIOA->OTYPER &= ~(GPIO_OTYPER_OT2 | GPIO_OTYPER_OT3); 
    GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR2 | GPIO_OSPEEDER_OSPEEDR3);
    GPIOA->PUPDR |= (GPIO_PUPDR_PUPD3_0); // Pull-up on RX
    GPIOA->AFR[0] |= (7 << GPIO_AFRL_AFSEL2_Pos) | (7 << GPIO_AFRL_AFSEL3_Pos);

    USART2->BRR = (16000000 / 9600);  // Baud rate
    USART2->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;
    USART2->CR1 |= USART_CR1_RXNEIE;  // Enable RX interrupt

    NVIC_SetPriority(USART2_IRQn, 1);
    NVIC_EnableIRQ(USART2_IRQn);

    __enable_irq();  // Enable global interrupts
}

void USART2_IRQHandler(void) {
    if (USART2->SR & USART_SR_RXNE) {
        char data = USART2->DR;

        if (data == '\n' || data == '\r') {  // End of command
            rx_complete = 1;
            rx_buffer[rx_index] = '\0';  // Null-terminate the string
            rx_index = 0;  // Reset index
        } else {
            if (rx_index < RX_BUFFER_SIZE - 1) {
                rx_buffer[rx_index++] = data;
            }
        }
    }

    if (USART2->SR & USART_SR_TXE) {
        if (tx_index < RX_BUFFER_SIZE && tx_buffer[tx_index] != 0) {
            USART2->DR = tx_buffer[tx_index++];
        } else {
            USART2->CR1 &= ~USART_CR1_TXEIE;  // Disable TX interrupt when done
        }
    }
}
void Delay(void) {
    for (volatile int i = 0; i < 100000; i++); // Short delay
}

void UART_SendString(char *str) {
    for (int i = 0; i < RX_BUFFER_SIZE; i++) {
        tx_buffer[i] = 0;  // Clear buffer
    }

    for (int i = 0; i < RX_BUFFER_SIZE - 1 && str[i] != '\0'; i++) {
        tx_buffer[i] = str[i];
    }

    tx_index = 0;  // Reset the index
    USART2->CR1 |= USART_CR1_TXEIE;  // Start transmission
}


void ProcessCommand(void) {
    if (strcmp((char*)rx_buffer, "STATUS") == 0) {  
        uint8_t status = (GPIOC->ODR >> 4) & 0xFF;  // Extract PC4-PC11 status
        char buffer[12];  // 8 bits + newline + null terminator

        // Convert status to a binary string
        UART_SendString("STATUS:");
        for (int i = 0; i < 8; i++) {
            buffer[i] = (status & (1 << (7 - i))) ? '1' : '0';  // Extract bit
        }
        buffer[8] = '\n';   // Newline for readability
        buffer[9] = '\0';   // Null terminator 
        Delay();
        UART_SendString(buffer);
    } 
    else if (strcmp((const char*)rx_buffer, "ALL") == 0) {  
        GPIOC->ODR |= (0xFF << 4);  // Turn ON all LEDs
        UART_SendString("\r\nLEDs ON\r\n"); 
    }
    else if (strcmp((char *)rx_buffer, "NONE") == 0) {  
        GPIOC->ODR &= ~(0xFF << 4);  // Turn OFF all LEDs
        UART_SendString("\r\nLEDs OFF\r\n");  
    }
    else if (strncmp((char *)rx_buffer, "ON", 2) == 0) {  
    // Loop through characters in rx_buffer starting from index 2 (after "ON")
    for (int i = 2; rx_buffer[i] != '\0'; i++) {
        if (rx_buffer[i] >= '1' && rx_buffer[i] <= '8') {
            int led_bit = rx_buffer[i] - '1' + 4; // Map 1-8 to PC4-PC11
            GPIOC->ODR |= (1 << led_bit); // Turn ON the specific LED
        }
    }
    UART_SendString("\r ON: ");
    Delay();
    UART_SendString((char*)rx_buffer + 2);  // Send the LEDs that were turned ON
}
else if (strncmp((char *)rx_buffer, "OFF", 3) == 0) {  
    // Loop through characters in rx_buffer starting from index 3 (after "OFF")
    for (int i = 3; rx_buffer[i] != '\0'; i++) {
        if (rx_buffer[i] >= '1' && rx_buffer[i] <= '8') {
            int led_bit = rx_buffer[i] - '1' + 4; // Map 1-8 to PC4-PC11
            GPIOC->ODR &= ~(1 << led_bit);  // Turn OFF the specific LED
        }
    }
    UART_SendString("\r\nOFF: ");
    Delay();
    UART_SendString((char*)rx_buffer + 3);  // Send the LEDs that were turned OFF
}

    else {  // Invalid command
        UART_SendString("\r\nInvalid\r\n");
    }

    rx_complete = 0;  // Reset the flag to indicate that the command has been processed
}


int main(void) {
    GPIO_Init();
    UART_Init();
    UART_SendString("\r\nReady\r\n");

    while (1) {
        if (rx_complete) {
            ProcessCommand();
            rx_complete = 0;
        }
    }
}

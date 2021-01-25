#ifndef SH_REG
#define SH_REG

#include "stdint.h"

#define SER_PIN						(2)
#define SER_PORT					GPIOA
#define SER_CLOCK_BIT			RCC_APB2ENR_IOPAEN
		
#define RCLK_PIN					(5)
#define RCLK_PORT					GPIOA
#define RCLK_CLOCK_BIT		RCC_APB2ENR_IOPAEN
		
#define SRCLK_PIN					(6)
#define SRCLK_PORT				GPIOA
#define SRCLK_CLOCK_BIT		RCC_APB2ENR_IOPAEN

void init_shift_register_pins(void);
void display_4_digit_number(uint16_t number);

void test(void);


#endif

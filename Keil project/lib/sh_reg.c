#include "stm32f10x.h"
#include "sh_reg.h"


#define SER_BIT						(1<<SER_PIN)
#define RCLK_BIT					(1<<RCLK_PIN)
#define SRCLK_BIT					(1<<SRCLK_PIN)
		
#define SER_UP						SER_PORT->ODR |= SER_BIT
#define RCLK_UP						RCLK_PORT->ODR |= RCLK_BIT
#define SRCLK_UP					SRCLK_PORT->ODR |= SRCLK_BIT
		
#define SER_DOWN 					SER_PORT->ODR &= (uint16_t)~SER_BIT
#define RCLK_DOWN 				RCLK_PORT->ODR &= (uint16_t)~RCLK_BIT
#define SRCLK_DOWN 				SRCLK_PORT->ODR &= (uint16_t)~SRCLK_BIT


#define SHORT_DELAY	2

static uint8_t digits[] = {
	
	160,
	187,
	98, 
	42, 
	57, 
	44,
	36,
	186,
	32, 
	40
};


void delay(uint16_t value);
void push_bit(uint8_t b);
void push_byte(uint8_t d);
void update_register(void);
void display_digit(uint16_t digit, uint16_t position);


void init_shift_register_pins(void)
{
	//enable clock
	RCC->APB2ENR |= SER_CLOCK_BIT;
	RCC->APB2ENR |= RCLK_CLOCK_BIT;
	RCC->APB2ENR |= SRCLK_CLOCK_BIT;
	
	//configure pin as output push-pull
	
	//configure SER_PIN
	if(SER_PIN <= 7)
	{
		GPIOA->CRL |= (1 << (SER_PIN*4)) | (1 << (SER_PIN*4 + 1));
		//GPIOA->CRL &= ~(uint32_t)(1 << (SER_PIN*4 + 2));
	} 
	else
	{
		GPIOA->CRH |= (1 << ((SER_PIN-8)*4)) | (1 << ((SER_PIN-8)*4 + 1));
		//GPIOA->CRH &= ~(uint32_t)(1 << ((SER_PIN-8)*4 + 2));
	}
	
	
	//configure RCLK_PIN
	if(RCLK_PIN <= 7)
	{
		GPIOA->CRL |= (1 << (RCLK_PIN*4)) | (1 << (RCLK_PIN*4 + 1));
		//GPIOA->CRL &= ~(uint32_t)(1 << (RCLK_PIN*4 + 2));
	} 
	else
	{
		GPIOA->CRH |= (1 << ((RCLK_PIN-8)*4)) | (1 << ((RCLK_PIN-8)*4 + 1));
		//GPIOA->CRH &= ~(uint32_t)(1 << ((RCLK_PIN-8)*4 + 2));
	}
	
	//configure SRCLK_PIN
	if(SRCLK_PIN <= 7)
	{
		GPIOA->CRL |= (1 << (SRCLK_PIN*4)) | (1 << (SRCLK_PIN*4 + 1));
		//GPIOA->CRL &= ~(uint32_t)(1 << (SRCLK_PIN*4 + 2));
	} 
	else
	{
		GPIOA->CRH |= (1 << ((SRCLK_PIN-8)*4)) | (1 << ((SRCLK_PIN-8)*4 + 1));
		//GPIOA->CRH &= ~(uint32_t)(1 << ((SRCLK_PIN-8)*4 + 2));
	}
	
} 

inline void delay(uint16_t value)
{
	for(uint16_t i = 0; i < value; i++);
}


inline void push_bit(uint8_t b)
{
		//set data line
		b ? (SER_UP) : (SER_DOWN);		
		delay(SHORT_DELAY);
		
		//strobe SRCLK
		SRCLK_UP;											
		delay(SHORT_DELAY);
		SRCLK_DOWN;	
	
}

inline void update_register()
{
	//strobe RCLK
	RCLK_UP;
	delay(SHORT_DELAY);
	RCLK_DOWN;
	
}


void push_byte(uint8_t b)
{
	for(int i = 0; i < 8; i++)
	{
		push_bit(b & 0x1);
		b >>= 1;
	}
}

void display_digit(uint16_t digit, uint16_t position)
{
		push_byte((uint8_t)(1 << (3 + position)));
		push_byte(digits[digit]);
		update_register();
	
}

void display_4_digit_number(uint16_t number)
{
	
	for(uint16_t i = 0; i < 5; i++)
	{
		display_digit(number%10, i);
		number = number / 10;
	}
	
}


void test(void)
{
		static uint16_t count = 0;
		static uint16_t n = 0;
	
		count++;
		if(!(count%100)) n++;
		
	
		display_4_digit_number(n);

}

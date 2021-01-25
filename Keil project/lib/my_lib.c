#include "my_lib.h"


//UART
#define UART_WAITING_TIME 2
static uint16_t init_uart_flag = 0;

void init_UART1(uint16_t speed)
{

	//configure clock
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;		// enable GPIOA clock
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;		// enable AF clock
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;	// enable USART clock
	
	//configure TX pin
	init_pin(GPIOA, 9, OUTPUT_50Mhz, ALTERNATE_PUSH_PULL);
	
	//configure RX pin
	init_pin(GPIOA, 10, INPUT, PULL_DOWN);
	
	//configure UART1
	USART1->BRR = speed;
	USART1->CR1 |= USART_CR1_TE;
	USART1->CR1 |= USART_CR1_RE;
	USART1->CR1 |= USART_CR1_UE;
	
	init_uart_flag = 1;
}

void print_msg(const char* msg, ...)
{
	
	if(!init_uart_flag)
		init_UART1(UART_SPEED_9600);
	
	char buff[50];
	
	va_list args;
	va_start(args, msg);
	vsprintf(buff, msg, args);
	
	uint32_t len = strlen(buff);
	
	for(uint32_t i = 0; i < len; i++)
	{
		USART1->DR = buff[i];
		while(!(USART1->SR & USART_SR_TXE));
	}
	
	
}

uint16_t receive_msg(const char* t_msg, char* r_msg, uint16_t length)
{
	//init uart if needed
	if(!init_uart_flag)
		init_UART1(UART_SPEED_9600);
	
	//print message
	print_msg(t_msg);
	
	//clear RXNE flag
	USART1->SR &= ~USART_SR_RXNE;
	
	//wait until first byte will be received
	while(!(USART1->SR & USART_SR_RXNE));
	
	//bytes counter for prevent buffer overflow
	uint16_t counter = 0;
	
	//write first received byte
	r_msg[counter] = (char)(USART1->DR & 0xFF);
	counter++;
	
	//if system timer was off - turn it on
	if(!sys_timer_status_flag)
		start_system_timer();
	
	//delay_tick - get_sys_tick() is a time that should pass for end receiving, in this case it's UART_WAITING_TIME ms
	uint32_t delay_tick = get_sys_tick()+ UART_WAITING_TIME;
	
	//receiving bytes
	while(1)
	{
		//read the data register if RXNE flag has been set by hardware
		if(USART1->SR & USART_SR_RXNE)
		{
			//buffer overflow detection
			if(counter == length-1)
			{
				//if system timer was on when we start - do not turn it off
				//if system timer was off when we start - turn it off and set sys_tick to 0
				if(!sys_timer_status_flag)
				{
					stop_system_timer();
					set_sys_tick_to_zero();
				}	
				
				//end string
				r_msg[counter] = '\0';
				//buffer overflow exit
				return 1;
			}
				
			//write next byte
			r_msg[counter] = (char)(USART1->DR & 0xFF);
			counter++;
			//add UART_WAITING_TIME ms to delay for prevent premature exit when receiving is proceed
			delay_tick = get_sys_tick()+ UART_WAITING_TIME;
		} 
		
		//if it's last more than UART_WAITING_TIME ms since last byte received stop receiving
		if(get_sys_tick() > delay_tick)
		{
			//if system timer was on when we start - do not turn it off
			//if system timer was off when we start - turn it off and set sys_tick to 0
			if(!sys_timer_status_flag)
			{
				stop_system_timer();
				set_sys_tick_to_zero();
			}				
			
			//end string
			r_msg[counter] = '\0';
			//no buffer overflow exit
			return 0;
		}
		
	}
	
}
//UART



//DELAY
void init_tim2_1us(void);
void TIM2_IRQHandler(void);
static uint32_t us_tick = 0;
static uint16_t init_delay_flag = 0;

void init_tim2_1us(void)
{
	//enable timer2 clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	
	//config timer
	TIM2->PSC = 0;
	TIM2->ARR = 71;
	TIM2->CR1 |= TIM_CR1_URS;
	TIM2->DIER |= TIM_DIER_UIE;
	TIM2->EGR |= TIM_EGR_UG;
	
	NVIC_EnableIRQ(TIM2_IRQn);
	
	init_delay_flag = 1;
}


//void TIM2_IRQHandler(void)
//{
//	TIM2->SR &= ~TIM_SR_UIF;
//	us_tick++;
//	
//}

void delay_ms(uint32_t delay_value)
{
	if(!init_delay_flag)
		init_tim2_1us();
	
	us_tick = 0;
	
	TIM2->CR1 |= TIM_CR1_CEN;
	while(us_tick < delay_value*1000);
	
	TIM2->CR1 &= ~TIM_CR1_CEN;
	
}

void delay_us(uint32_t delay_value)
{
	if(!init_delay_flag)
		init_tim2_1us();
	
	us_tick = 0;
	
	TIM2->CR1 |= TIM_CR1_CEN;
	while(us_tick < delay_value);
	
	TIM2->CR1 &= ~TIM_CR1_CEN;
	
}


//DELAY


//LED13
void init_led_pin(void);
static uint16_t init_led_flag = 0;


void init_led_pin(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	
	GPIOC->CRH |= GPIO_CRH_MODE13_0 | GPIO_CRH_MODE13_1;
	GPIOC->CRH &= ~GPIO_CRH_CNF13_0;
	
}


void led_turn_off(void)
{
	if(!init_led_flag)
	{		
		init_led_pin();
		init_led_flag = 1;
	}
	
	GPIOC->ODR |= 1 << 13;
	
}

void led_turn_on(void)
{
	if(!init_led_flag)
	{		
		init_led_pin();
		init_led_flag = 1;
	}
	
	GPIOC->ODR &= (uint16_t)(~(1 << 13));
	
}

void led_toggle(void)
{
	if(!init_led_flag)
	{		
		init_led_pin();
		init_led_flag = 1;
	}
	
	GPIOC->ODR ^= (1 << 13);
	
}

//LED13


//INIT PIN
void init_pin(GPIO_TypeDef *port, uint32_t pin, uint32_t mode, uint32_t cnf)
{
	//enable clock
	if(port == GPIOA)
		RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	else if (port == GPIOB)
		RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	else if (port == GPIOC)
		RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	else if (port == GPIOD)
		RCC->APB2ENR |= RCC_APB2ENR_IOPDEN;
	
	//set pullup or pulldown if mode==input
	if((mode == INPUT) && (cnf & 0x2))
	{
		uint16_t temp = (cnf & 0x4) >> 2;
		cnf &= (uint16_t)(~(0x4));
		
		if(temp)
			port->ODR |= (1 << pin);
		else
			port->ODR &= ~(1 << pin);
	}
	
	//set mode and cnf bits
	if(pin <= 7)
	{
		port->CRL &= ~(uint32_t)(0xF << pin*4);
		port->CRL |= (uint32_t)((mode << pin*4) | (cnf << (pin*4+2)));
	}
	else
	{
		port->CRH &= ~(uint32_t)(0xF << (pin-8)*4);
		port->CRH |= (uint32_t)((mode << (pin-8)*4) | (cnf << ((pin-8)*4+2)));
	}
	
}


//INIT PIN


//SYSTEM_TIMER
static uint32_t sys_tick = 0;
uint16_t sys_flag = 0;
uint16_t sys_timer_status_flag = 0;

void TIM3_IRQHandler(void);


void start_system_timer(void)
{
	//enable timer3 clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	
	//config timer
	TIM3->PSC = (1000*SYSTEM_TIMER_MULTIPLIER)-1;
	TIM3->ARR = 71;
	TIM3->CR1 |= TIM_CR1_URS;
	TIM3->DIER |= TIM_DIER_UIE;
	TIM3->EGR |= TIM_EGR_UG;
	
	TIM3->CR1 |= TIM_CR1_CEN;
	
	NVIC_EnableIRQ(TIM3_IRQn);
	
	sys_timer_status_flag = 1;
	
}

void stop_system_timer(void)
{
	TIM3->CR1 &= ~TIM_CR1_CEN;
	sys_timer_status_flag = 0;
}


void TIM3_IRQHandler(void)
{
	TIM3->SR &= ~TIM_SR_UIF;
	
	sys_tick++;
	sys_flag = 1;
	
}
uint32_t get_sys_tick(void)
{
	//reading and modifying 64bit variable is critical
	__disable_irq();
	uint32_t temp = sys_tick;
	__enable_irq();
	
	return temp;
}

void set_sys_tick_to_zero(void)
{
	__disable_irq();
	sys_tick = 0;
	__enable_irq();
}
//SYSTEM_TIMER


//ADC
void ADC1_2_IRQHandler(void);
static uint16_t init_ADC_flag = 0;

void init_ADC(void)
{
	//set ADC divider (12 MHz)
	RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6;
	//enable ADC clock
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

	//turn adc on and wait until it starts
	ADC1->CR2 |= ADC_CR2_ADON;
	
	silly_delay_ms(1);
	
	ADC1->CR2 |= ADC_CR2_ADON;
	
	silly_delay_ms(1);
	
	ADC1->CR2 |= ADC_CR2_CAL;
	
	silly_delay_ms(2);
	
	ADC1->SR &= (uint16_t)~ADC_SR_EOC;
	
	init_ADC_flag = 1;
}

uint16_t ADC_single_conversion(uint16_t channel)
{
	//init ADC if needed
	if(!init_ADC_flag)
		init_ADC();
	
	//enable alternative function clock
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
	
	//init pin
	if(channel < 8)
	{
		RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
		init_pin(GPIOA, channel, INPUT, FLOATING);
	} 
	else 
	{
		RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
		init_pin(GPIOB, channel-8, INPUT, FLOATING);
	}

		
	//set sample time
	ADC1->SMPR2 |= (uint32_t)(0x7 << (channel*3));
	
	//use chanel 5 for 1 element of sequence
	ADC1->SQR3 |= channel;
	
	
	ADC1->CR2 |= ADC_CR2_ADON;
	while(!(ADC1->SR & ADC_SR_EOC));
	//uint32_t sample = ADC1->DR;
	return (uint16_t)(ADC1->DR);
}
//ADC

//PWM
static uint16_t init_timer4_pwm_status_flag = 0;

void init_timer4_pwm(uint16_t channel, uint32_t freq, uint16_t duty_cycle)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

	//configure pin
	init_pin(GPIOB, channel+5, OUTPUT_50Mhz, ALTERNATE_PUSH_PULL);
	
	//configure timer frequency
	TIM4->PSC = (uint16_t)(72000000 / (freq * 100));
	TIM4->ARR = 100;
	
	
	//configure duty cycle
	switch(channel)
	{
		case 1: TIM4->CCR1 = duty_cycle;
						break;
		case 2: TIM4->CCR2 = duty_cycle;
						break;
		case 3: TIM4->CCR3 = duty_cycle;
						break;
		case 4: TIM4->CCR4 = duty_cycle;
						break;
	}
		
	
	if(channel < 3)
	{
		TIM4->CCMR1 |= 0x6 << (4 + (channel - 1)*8);		//PWM mode 1
		TIM4->CCMR1 |= 1 << (3 + (channel - 1)*8);												//ENABLE preload register
	}
	else
	{
		TIM4->CCMR2 |= 0x6 << (4 + (channel - 3)*8);
		TIM4->CCMR2 |= 1 << (3 + (channel - 3)*8);
	}
		
	
	TIM4->CR1 |= TIM_CR1_ARPE;														//enable auto reload preload register
	TIM4->EGR |= TIM_EGR_UG;															//update generation
	TIM4->CR1 |= TIM_CR1_CEN;															//ENABLE timer
	TIM4->CCER |= 1 << ((channel-1)*4);										//enable channel 
	
	init_timer4_pwm_status_flag |= (1 << channel);
}

void pwm_set_duty_cycle(uint16_t channel, uint16_t duty_cycle)
{
	if(!(init_timer4_pwm_status_flag & (1 << channel)))
		init_timer4_pwm(channel, 60, duty_cycle);
	
	switch(channel)
	{
		case 1: TIM4->CCR1 = duty_cycle;
						break;
		case 2: TIM4->CCR2 = duty_cycle;
						break;
		case 3: TIM4->CCR3 = duty_cycle;
						break;
		case 4: TIM4->CCR4 = duty_cycle;
						break;
	}
	
}
//PWM


//GENEGAR PURPOSE FUNCTION
uint16_t map(uint16_t val, uint16_t s_begin, uint16_t s_end, uint16_t d_begin, uint16_t d_end)
{
	return (uint16_t)((float)val/(float)(s_end - s_begin)*(d_end - d_begin)+d_begin);	
}

void silly_delay_ms(uint16_t d_value)
{
	int t = d_value * 3600;
	for(int i = 0; i < t; i++);
	
}
//GENEGAR PURPOSE FUNCTION

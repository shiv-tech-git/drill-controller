#include "stm32f10x.h"                  // Device header
#include "my_lib.h"
#include "sh_reg.h"


//Optical sensor pin interrupt
void pin_interrupt_init(void);
void EXTI0_IRQHandler(void);
//Optical sensor pin interrupt


//Measuring angular speed
void TIM2_IRQHandler(void);
void init_tim2(void);

static uint16_t a_vel = 0;
//Measuring angular speed


//PID controller
uint16_t moving_average(uint16_t value);
int16_t pid(int16_t target_val, int16_t current_val);
void get_coef(void);

static double kp = 2;
static double ki = 0.05;
static double kd = -0.2;
//PID controller


int main()
{

	init_shift_register_pins();
	init_timer4_pwm(3, 1000, 0);
	init_UART1(UART_SPEED_115200);

	pin_interrupt_init();

	start_system_timer();
	
	init_tim2();
	
	
	uint16_t target_a_vel = 0;
	int pwm_val = 0;
	
	
	
		while(1)
		{
			
			target_a_vel = map(ADC_single_conversion(8), 0, 4096, 0, 20)*5;
			print_msg("%d \r", target_a_vel);
			display_4_digit_number(target_a_vel);
			
			
			if(sys_flag)
			{
				sys_flag = 0;
				pwm_val = pid(target_a_vel, a_vel);
				pwm_set_duty_cycle(3, pwm_val);
				print_msg("$%d, %d, %d;", target_a_vel, a_vel, pwm_val);
				
			}
			
		}
		
return 0;
}


//Optical sensor pin interrupt
void pin_interrupt_init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN;
	
	init_pin(GPIOA, 0, INPUT, PULL_UP);
	
	AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI0_PA;
	
	EXTI->IMR |= 1 << 0;
	
	EXTI->FTSR |= 1 << 0;
	
	NVIC_EnableIRQ(EXTI0_IRQn);
	
}

void EXTI0_IRQHandler(void)
{
	EXTI->PR |= 1 << 0;
	a_vel = (uint16_t)(500000/(TIM2->CNT));
	
	TIM2->EGR |= TIM_EGR_UG;
}
//Optical sensor pin interrupt





//Measuring angular speed
void init_tim2(void)
{
	//enable timer2 clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	
	//config timer
	TIM2->PSC = 72;
	TIM2->ARR = 65000;
	TIM2->CR1 |= TIM_CR1_URS;
	TIM2->DIER |= TIM_DIER_UIE;
	TIM2->EGR |= TIM_EGR_UG;
	
	NVIC_EnableIRQ(TIM2_IRQn);
	
	TIM2->CR1 |= TIM_CR1_CEN;
	
}

void TIM2_IRQHandler(void)
{
	TIM2->SR &= ~TIM_SR_UIF;
	a_vel = 0;
	
}
//Measuring angular speed




//PID controller
int16_t pid(int16_t target_val, int16_t current_val)
{
	static int i = 0;
	static int prev_val = 0;
	
	int16_t diff = target_val - current_val;
	i += diff;
	
	int16_t r = (int16_t)(diff*kp + i*ki + (current_val - prev_val)*kd);
	
	if(r > 100)
		return 100;
	else if(r < 0)
		return 0;
	else 
		return r;
}


uint16_t moving_average(uint16_t value)
{
	static uint16_t window[10];
	static uint16_t pointer = 0;
	static uint16_t summ = 0;
	
	summ = summ - window[pointer] + value;
	window[pointer] = value;
	pointer = (pointer+1)%5;
	
	return summ/5;
}

void get_coef(void)
{
	char msg[10];
	
	receive_msg("p coeff\r", msg, 10);
	print_msg("p = %f \r", atof(msg));
	receive_msg("i coeff\r", msg, 10);
	print_msg("i = %f \r", atof(msg));
	
}
//PID controller

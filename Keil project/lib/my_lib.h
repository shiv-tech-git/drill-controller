#ifndef MY_LIB
#define MY_LIB

#include "stm32f10x.h"
#include "string.h"
#include "stdint.h"
#include "stdlib.h"
#include "stdarg.h"

//GENERAL PURPOSE FUNCTIONS #######################################################
																																								//#
/*functions from stdlib.h ---------------------------------------------------		//#
																																						|		//#
int atoi(const char *);																											|		//#
int abs(int);																																|		//#
																																						|		//#
#define RAND_MAX 0x7fffffff																									|		//#
int rand(void);																															|		//#
void srand(unsigned int);																										|		//#
																																						|		//#
functions from stdlib.h*///--------------------------------------------------		//#

uint16_t map(uint16_t val, uint16_t s_begin, uint16_t s_end,\
							uint16_t d_begin, uint16_t d_end);

void silly_delay_ms(uint16_t d_value);
																																								//#
//GENERAL PURPOSE FUNCTIONS #######################################################


//UART ############################################################################
																																								//#
#define UART_DEFAULT_SPEED					UART_SPEED_9600															//#
#define UART_SPEED_9600							(uint16_t)0x1d4C														//#
#define UART_SPEED_19200						(uint16_t)0xEA4															//#
#define UART_SPEED_38400						(uint16_t)0x753															//#
#define UART_SPEED_57600						(uint16_t)0x4E2															//#
#define UART_SPEED_115200						(uint16_t)0x271															//#
																																								//#
void init_UART1(uint16_t speed);																						    //#
void print_msg(const char* msg, ...);																		        //#
uint16_t receive_msg(const char* t_msg, char* r_msg, uint16_t length);					//#
																																								//#
//UART ############################################################################



//PIN INITIALIZATION ##############################################################
																																								//#
#define INPUT												(uint16_t)(0x0)															//#																																						//#
#define ANALOG											(uint16_t)(0x0)															//#
#define FLOATING										(uint16_t)(0x1)															//#
#define PULL_UP											(uint16_t)(0x6)															//#
#define PULL_DOWN										(uint16_t)(0x2)															//#
																																								//#
#define OUTPUT_2Mhz									(uint16_t)(0x2)															//#
#define OUTPUT_10Mhz								(uint16_t)(0x1)															//#
#define OUTPUT_50Mhz								(uint16_t)(0x3)															//#
#define PUSH_PULL										(uint16_t)(0x0)															//#
#define OPEN_DRAIN									(uint16_t)(0x1)															//#
#define ALTERNATE_PUSH_PULL					(uint16_t)(0x2)															//#
#define ALTERNATE_OPEN_DRAIN				(uint16_t)(0x3)															//#
																																								//#
void init_pin(GPIO_TypeDef *port, uint32_t pin, uint32_t mode, uint32_t cnf);		//#
																																								//#
//PIN INITIALIZATION ##############################################################



//DELAY############################################################################
																																								//#
void delay_ms(uint32_t delay_value);																						//#
void delay_us(uint32_t delay_value);																						//#
																																								//#
//DELAY ###########################################################################



//LED #############################################################################
																																								//#
void led_turn_on(void);																													//#
void led_turn_off(void);																												//#
void led_toggle(void);																													//#
																																								//#
//LED #############################################################################



//SYSTEM TIMER ####################################################################
																																								//#
#define SYSTEM_TIMER_MULTIPLIER			10																					//#	
																																								//#
extern uint16_t sys_flag;																												//#
extern uint16_t sys_timer_status_flag;																					//#
																																								//#
uint32_t get_sys_tick(void);																										//#
void set_sys_tick_to_zero(void);																								//#
void start_system_timer(void);																									//#
void stop_system_timer(void);																										//#
																																								//#
//SYSTEM TIMER ####################################################################


//ADC #############################################################################
																																								//#																																		//#
void init_ADC(void);																														//#
uint16_t ADC_single_conversion(uint16_t channel);																//#
																																								//#
//ADC #############################################################################

void init_timer4_pwm(uint16_t channel, uint32_t freq, uint16_t duty_cycle);
void pwm_set_duty_cycle(uint16_t channel, uint16_t duty_cycle);


#endif

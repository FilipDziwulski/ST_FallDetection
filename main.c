#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "stm32f4xx_it.h"
#include "my_printf.h"
#include "imu.h"
#include <math.h>
#include "defs.h"

__IO uint32_t g_timing_delay;
__IO uint32_t g_ticks = 0; // increments every millisecond

//free fall vector sum threshold
#define FREE_FALL_THRESHOLD		0.3 //g's

#define DECIMAL_PORTION(number)    		abs((((int)(number * 100.0)) - (((int)number)*100)))

static volatile imuDof_S dof = {0};


void timing_delay_decrement(void);
void delay_ms(uint32_t t);
void init_Peripherals(void);
void process_Button(void);
void printDOF(void);
uint8_t isIMUinFreefall(void);
uint32_t get_ticks();


int main(void)
{
	//initializes peripherals
    init_Peripherals();

    while(1) {

    	//update DOF from IMU
    	imu_getDOF(&dof);
    	//output DOF over serial
    	printDOF();

    	//check for freeFall
    	if(TRUE == isIMUinFreefall())
    	{
    		my_printf("Catch me! I'm Falling!!!\r\n\n");
    	}


    	delay_ms(1); //main loop runs at 1kHz
    }
}

void printDOF(void)
{
	//my_printf does not support float, so data printed over serial is split apart and printed

	my_printf("acc.x: %d.%d g\r\n", (int)dof.acc.x, DECIMAL_PORTION(dof.acc.x));
	my_printf("acc.y: %d.%d g\r\n", (int)dof.acc.y, DECIMAL_PORTION(dof.acc.y));
	my_printf("acc.z: %d.%d g\r\n", (int)dof.acc.z, DECIMAL_PORTION(dof.acc.z));

	my_printf("mag.x: %d.%d gauss\r\n", (int)dof.mag.x, DECIMAL_PORTION(dof.mag.x));
	my_printf("mag.y: %d.%d gauss\r\n", (int)dof.mag.y, DECIMAL_PORTION(dof.mag.y));
	my_printf("mag.z: %d.%d gauss\r\n", (int)dof.mag.z, DECIMAL_PORTION(dof.mag.z));

	my_printf("rot.x: %d.%d deg/s\r\n", (int)dof.rot.x, DECIMAL_PORTION(dof.rot.x));
	my_printf("rot.y: %d.%d deg/s\r\n", (int)dof.rot.y, DECIMAL_PORTION(dof.rot.y));
	my_printf("rot.z: %d.%d deg/s\r\n", (int)dof.rot.z, DECIMAL_PORTION(dof.rot.z));
	my_printf("\n");
}

uint8_t isIMUinFreefall(void)
{
	//calculate vector sum of components of acceleration
	float vectorSum = sqrt((dof.acc.x * dof.acc.x) + (dof.acc.y * dof.acc.y) + (dof.acc.z * dof.acc.z));

	if( ((float) FREE_FALL_THRESHOLD) > vectorSum)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

void process_Button(void)
{
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)) {
		GPIO_SetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 |  GPIO_Pin_15);
	}
    else {
		GPIO_ResetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 |  GPIO_Pin_15);
	}
}

void init_LED()
{
    GPIO_InitTypeDef gpio; // LEDS on GPIOD

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    gpio.GPIO_Pin   = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 |  GPIO_Pin_15;
	gpio.GPIO_Mode  = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;

	GPIO_Init(GPIOD, &gpio);
}

void init_blue_push_button()
{
    GPIO_InitTypeDef gpio; // push button on GPIOA

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	gpio.GPIO_Pin   = GPIO_Pin_0;
	gpio.GPIO_Mode  = GPIO_Mode_IN;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd  = GPIO_PuPd_DOWN;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;

	GPIO_Init(GPIOA, &gpio);
}

void init_UART4()
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;

    usart.USART_BaudRate = 115200;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    /* Enable GPIO clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    /* Enable UART clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

    /* Connect PXx to USARTx_Tx*/
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);

    /* Connect PXx to USARTx_Rx*/
    GPIO_PinAFConfig( GPIOC, GPIO_PinSource11, GPIO_AF_UART4);
 
    /* Configure USART Tx as alternate function  */
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &gpio);

    /* USART configuration */
    USART_Init(UART4, &usart);

    /* Enable USART */
    USART_Cmd(UART4, ENABLE);
}

void init_Peripherals(void)
{
	/*!< At this stage the microcontroller clock setting is already configured,
	   this is done through SystemInit() function which is called from startup
	   file (startup_stm32f4xx.s) before to branch to application main.
	   To reconfigure the default setting of SystemInit() function, refer to
	   system_stm32f4xx.c file
	 */

	// Enable Usage Fault, Bus Fault, and MMU Fault, else it will default to HardFault handler
	//SCB->SHCSR |= 0x00070000;
	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq(&RCC_Clocks);
	SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000); // tick every 1 ms, used by delay_ms()

    init_UART4();
    init_LED();
    init_blue_push_button();
    init_IMU();
}

void delay_ms(uint32_t t)
{
    g_timing_delay = t;

    while (g_timing_delay != 0);
}

void timing_delay_decrement(void)
{
    if (g_timing_delay != 0x00) { 
        g_timing_delay--;
    }
}

uint32_t get_ticks()
{
    return g_ticks;
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: my_printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif


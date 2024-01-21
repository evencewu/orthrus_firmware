#include "bsp_delay.h"

__IO u32 TimingDelay;

void SysTick_Init(void)
{

	if (SysTick_Config(SystemCoreClock / 1000))
	{ 
		while (1);
	}
}


void Delay_ms(__IO u32 nTime)
{ 
	TimingDelay = nTime;	

	while(TimingDelay != 0);
}


void SysTick_Delay_Ms( __IO uint32_t ms)
{
	uint32_t i;	
	SysTick_Config(SystemCoreClock/1000);
	
	for(i=0;i<ms;i++)
	{

		while( !((SysTick->CTRL)&(1<<16)) );
	}

	SysTick->CTRL &=~ SysTick_CTRL_ENABLE_Msk;
}

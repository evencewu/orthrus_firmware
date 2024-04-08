#include "RST.h"

void rst_setup(void)
{
    /* Setup NRST as GPIO out and pull it high */
    GPIO_InitTypeDef gpio;
 
    RCC_AHB1PeriphClockCmd(ESC_RCC_APB1PERIPH_GPIOX_RSTN, ENABLE);

    gpio.GPIO_Pin   = ESC_GPIO_Pin_RSTN; 
    gpio.GPIO_Mode  = GPIO_Mode_OUT;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(ESC_GPIOX_RSTN, &gpio);
    
    rst_high();
}

void rst_high(void)
{
    /* Set RSTN line high */
    GPIO_SetBits(ESC_GPIOX_RSTN, ESC_GPIO_Pin_RSTN);
}

void rst_low(void)
{    /* Set RSTN line low */
    GPIO_ResetBits(ESC_GPIOX_RSTN, ESC_GPIO_Pin_RSTN);
}
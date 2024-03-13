#include "bsp_tim.h"

static void TIMx_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    // 设置中断组为0
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    // 设置中断来源
    NVIC_InitStructure.NVIC_IRQChannel = BASIC_TIM_IRQn;
    // 设置抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    // 设置子优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

static void TIM_Mode_Config(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    // 开启TIMx_CLK,x[6,7]
    RCC_APB1PeriphClockCmd(BASIC_TIM_CLK, ENABLE);

    /* 累计 TIM_Period个后产生一个更新或者中断*/
    // 当定时器从0计数到4999，即为5000次，为一个定时周期
    TIM_TimeBaseStructure.TIM_Period = 1000 - 1;

    // 定时器时钟源TIMxCLK = 2 * PCLK1
    //         PCLK1 = HCLK / 4
    //         =>
    //  TIMxCLK=HCLK/2=SystemCoreClock/2=84000000Hz
    //  设定定时器频率为=TIMxCLK/(TIM_Prescaler+1)=100000Hz
    TIM_TimeBaseStructure.TIM_Prescaler = 840 - 1;

    //callback 频率 100000Hz/(TIM_Period+1) = 1000hz

    // 初始化定时器TIMx, x[2,3,4,5]
    TIM_TimeBaseInit(BASIC_TIM, &TIM_TimeBaseStructure);

    // 清除定时器更新中断标志位
    TIM_ClearFlag(BASIC_TIM, TIM_FLAG_Update);

    // 开启定时器更新中断
    TIM_ITConfig(BASIC_TIM, TIM_IT_Update, ENABLE);

    // 使能定时器
    TIM_Cmd(BASIC_TIM, ENABLE);
}

void TIMx_Configuration()
{
    TIMx_NVIC_Configuration();
    TIM_Mode_Config();
}

void TIM6_DAC_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
        modfiy_cmd(&cmd_leg[3], 1, 0, 0, 0, 0, 0);
        unitreeA1_tx(3);
    }
}

// int tim3_flag;
//
// void TIM1_UP_TIM10_IRQHandler(void)
//{
//
//     TIM_ClearFlag(TIM1, TIM_FLAG_Update); // 进入中断先清除更新标志
//     if (tim3_flag == 0)
//     {
//         GPIO_SetBits(GPIOB, GPIO_Pin_14);
//         tim3_flag = 1;
//     }
//     else
//     {
//         GPIO_ResetBits(GPIOB, GPIO_Pin_14);
//         tim3_flag = 0;
//     }
// }

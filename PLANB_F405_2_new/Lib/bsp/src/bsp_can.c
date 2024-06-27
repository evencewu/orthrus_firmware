#include "bsp_can.h"

__IO uint32_t flag;

CanTxMsg Can1_TxMessage; // 发送缓冲区
CanRxMsg Can1_RxMessage; // 接收缓冲区

uint8_t can1_rx_flag = 0;

static void CAN1_GPIO_Config(void)
{

    GPIO_InitTypeDef GPIO_InitStructure;

    /* 使能GPIO时钟*/

    RCC_APB1PeriphClockCmd(CAN1_CLK, ENABLE);
    RCC_AHB1PeriphClockCmd(CAN1_TX_GPIO_CLK | CAN1_RX_GPIO_CLK, ENABLE);

    /* 引脚源*/
    GPIO_PinAFConfig(CAN1_TX_GPIO_PORT, CAN1_RX_SOURCE, CAN1_AF_PORT);
    GPIO_PinAFConfig(CAN1_RX_GPIO_PORT, CAN1_TX_SOURCE, CAN1_AF_PORT);
    /* 配置 CAN TX 引脚 */
    GPIO_InitStructure.GPIO_Pin = CAN1_TX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(CAN1_TX_GPIO_PORT, &GPIO_InitStructure);
    /* 配置 CAN RX 引脚 */
    GPIO_InitStructure.GPIO_Pin = CAN1_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(CAN1_RX_GPIO_PORT, &GPIO_InitStructure);
}

static void CAN1_Mode_Config(void)
{
    CAN_InitTypeDef CAN_InitStructure;
    /************************CAN通信参数设置************************/
    /* Enable CAN clock */
    RCC_APB1PeriphClockCmd(CAN1_CLK, ENABLE);

    /*CAN寄存器初始化*/
    CAN_DeInit(CAN1);
    CAN_StructInit(&CAN_InitStructure);

    /*CAN单元初始化*/
    CAN_InitStructure.CAN_TTCM = DISABLE;         // MCR-TTCM 关闭时间触发通信模式使能
    CAN_InitStructure.CAN_ABOM = DISABLE;         // MCR-ABOM 使能自动离线管理
    CAN_InitStructure.CAN_AWUM = DISABLE;         // MCR-AWUM 使用自动唤醒模式
    CAN_InitStructure.CAN_NART = DISABLE;         // MCR-NART 禁止报文自动重传
    CAN_InitStructure.CAN_RFLM = DISABLE;         // MCR-RFLM 接收FIFO 不锁定
                                                  //  溢出时新报文会覆盖原有报文
    CAN_InitStructure.CAN_TXFP = DISABLE;         // MCR-TXFP 发送FIFO优先级取决于报文标示符
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal; // 正常工作模式
    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;      // BTR-SJW 重新同步跳跃宽度 2个时间单元

    /* ss=1 bs1=5 bs2=3 位时间宽度为(1+5+3) 波特率即为时钟周期tq*(1+3+5) */
    CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq; // BTR-TS1 时间段1 占用了5个时间单元
    CAN_InitStructure.CAN_BS2 = CAN_BS2_3tq; // BTR-TS1 时间段2 占用了3个时间单元

    /* CAN Baudrate = 1 MBps (1MBps已为stm32的CAN最高速率) (CAN 时钟频率为 APB 1 = 45 MHz) */
    ////BTR-BRP 波特率分频器定义了时间单元的时间长度 45/(1+5+3)/5=1 Mbps
    CAN_InitStructure.CAN_Prescaler = 6;
    CAN_Init(CAN1, &CAN_InitStructure);
}

static void CAN1_Filter_Config(void)
{
    CAN_FilterInitTypeDef CAN_FilterInitStructure;

    /*CAN筛选器初始化*/
    CAN_FilterInitStructure.CAN_FilterNumber = 0; // 筛选器组0
    // 工作在掩码模式
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    // 筛选器位宽为单个32位。
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;

    /* 使能筛选器，按照标志符的内容进行比对筛选，
    扩展ID不是如下的就抛弃掉，是的话，会存入FIFO0。 */
    // 要筛选的ID高位，第0位保留，第1位为RTR标志，第2位为IDE标志，从第3位开始是EXID
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0;

    // 要筛选的ID低位
    CAN_FilterInitStructure.CAN_FilterIdLow = 0;
    // 筛选器高16位每位必须匹配
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0;
    // 筛选器低16位每位必须匹配
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0;
    // 筛选器被关联到FIFO0
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    // 使能筛选器
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);
    /*CAN通信中断使能*/
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}

static void CAN1_NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    /* Configure one bit for preemption priority */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    /*中断设置*/

    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX_IRQ; // CAN RX中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void CAN1_Config(void)
{
    CAN1_GPIO_Config();
    CAN1_Mode_Config();

    CAN1_NVIC_Config();
    CAN1_Filter_Config();
}

// CAN2 ============================================

CanTxMsg Can2_TxMessage; // 发送缓冲区
CanRxMsg Can2_RxMessage; // 接收缓冲区

// 负载均衡
int can2_balance_flag_[20];

// 负载均衡分类缓冲区
CanRxMsg Can2_RxBalance[20];

static void CAN2_GPIO_Config(void)
{

    GPIO_InitTypeDef GPIO_InitStructure;

    /* 使能GPIO时钟*/

    RCC_APB1PeriphClockCmd(CAN2_CLK, ENABLE);
    RCC_AHB1PeriphClockCmd(CAN2_TX_GPIO_CLK | CAN2_RX_GPIO_CLK, ENABLE);

    /* 引脚源*/
    GPIO_PinAFConfig(CAN2_TX_GPIO_PORT, CAN2_RX_SOURCE, CAN2_AF_PORT);
    GPIO_PinAFConfig(CAN2_RX_GPIO_PORT, CAN2_TX_SOURCE, CAN2_AF_PORT);
    /* 配置 CAN TX 引脚 */
    GPIO_InitStructure.GPIO_Pin = CAN2_TX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(CAN2_TX_GPIO_PORT, &GPIO_InitStructure);
    /* 配置 CAN RX 引脚 */
    GPIO_InitStructure.GPIO_Pin = CAN2_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(CAN2_RX_GPIO_PORT, &GPIO_InitStructure);
}

static void CAN2_Mode_Config(void)
{
    CAN_InitTypeDef CAN_InitStructure;
    /************************CAN通信参数设置************************/
    /* Enable CAN clock */

    RCC_APB1PeriphClockCmd(CAN1_CLK, ENABLE); // 再开启CAN2时钟
    RCC_APB1PeriphClockCmd(CAN2_CLK, ENABLE);

    /*CAN寄存器初始化*/
    CAN_DeInit(CAN2);
    CAN_StructInit(&CAN_InitStructure);

    /*CAN单元初始化*/
    CAN_InitStructure.CAN_TTCM = DISABLE;         // MCR-TTCM 关闭时间触发通信模式使能
    CAN_InitStructure.CAN_ABOM = DISABLE;         // MCR-ABOM 使能自动离线管理
    CAN_InitStructure.CAN_AWUM = DISABLE;         // MCR-AWUM 使用自动唤醒模式
    CAN_InitStructure.CAN_NART = DISABLE;         // MCR-NART 禁止报文自动重传
    CAN_InitStructure.CAN_RFLM = DISABLE;         // MCR-RFLM 接收FIFO 不锁定
                                                  //  溢出时新报文会覆盖原有报文
    CAN_InitStructure.CAN_TXFP = DISABLE;         // MCR-TXFP 发送FIFO优先级取决于报文标示符
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal; // 正常工作模式
    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;      // BTR-SJW 重新同步跳跃宽度 2个时间单元

    /* ss=1 bs1=5 bs2=3 位时间宽度为(1+5+3) 波特率即为时钟周期tq*(1+3+5) */
    CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq; // BTR-TS1 时间段1 占用了5个时间单元
    CAN_InitStructure.CAN_BS2 = CAN_BS2_3tq; // BTR-TS1 时间段2 占用了3个时间单元

    /* CAN Baudrate = 1 MBps (1MBps已为stm32的CAN最高速率) (CAN 时钟频率为 APB 1 = 45 MHz) */
    ////BTR-BRP 波特率分频器定义了时间单元的时间长度 45/(1+5+3)/5=1 Mbps
    CAN_InitStructure.CAN_Prescaler = 6;
    CAN_Init(CAN2, &CAN_InitStructure);
}

static void CAN2_Filter_Config(void)
{
    CAN_FilterInitTypeDef CAN_FilterInitStructure;

    /*CAN筛选器初始化*/
    CAN_FilterInitStructure.CAN_FilterNumber = 14; // 筛选器组0
    // 工作在掩码模式
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    // 筛选器位宽为单个32位。
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;

    /* 使能筛选器，按照标志符的内容进行比对筛选，
    扩展ID不是如下的就抛弃掉，是的话，会存入FIFO0。 */
    // 要筛选的ID高位，第0位保留，第1位为RTR标志，第2位为IDE标志，从第3位开始是EXID
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0;

    // 要筛选的ID低位
    CAN_FilterInitStructure.CAN_FilterIdLow = 0;
    // 筛选器高16位每位必须匹配
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0;
    // 筛选器低16位每位必须匹配
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0;
    // 筛选器被关联到FIFO0
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    // 使能筛选器
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);
    /*CAN通信中断使能*/
    CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);
}

static void CAN2_NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    /* Configure one bit for preemption priority */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    /*中断设置*/

    NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX_IRQ; // CAN RX中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void CAN2_Config(void)
{
    CAN2_GPIO_Config();
    CAN2_Mode_Config();

    CAN2_NVIC_Config();
    CAN2_Filter_Config();

    for (int i = 0; i < 20; ++i)
    {
        can2_balance_flag_[i] = 0;
    }
}

/********************************************************************/
void CAN1_RX0_IRQHandler(void)
{
    CAN_Receive(CAN1, CAN_FIFO0, &Can1_RxMessage);
}

void CAN2_RX0_IRQHandler(void)
{
    CAN_Receive(CAN2, CAN_FIFO0, &Can2_RxMessage);
    if ((int)Can2_RxMessage.StdId < 20)
    {
        can2_balance_flag_[(int)Can2_RxMessage.StdId] = 1;

        Can2_RxBalance[(int)Can2_RxMessage.StdId].StdId = Can2_RxMessage.StdId;
        Can2_RxBalance[(int)Can2_RxMessage.StdId].ExtId = Can2_RxMessage.ExtId;
        Can2_RxBalance[(int)Can2_RxMessage.StdId].IDE = Can2_RxMessage.IDE;
        Can2_RxBalance[(int)Can2_RxMessage.StdId].RTR = Can2_RxMessage.RTR;
        Can2_RxBalance[(int)Can2_RxMessage.StdId].DLC = Can2_RxMessage.DLC;
        Can2_RxBalance[(int)Can2_RxMessage.StdId].Data[0] = Can2_RxMessage.Data[0];
        Can2_RxBalance[(int)Can2_RxMessage.StdId].Data[1] = Can2_RxMessage.Data[1];
        Can2_RxBalance[(int)Can2_RxMessage.StdId].Data[2] = Can2_RxMessage.Data[2];
        Can2_RxBalance[(int)Can2_RxMessage.StdId].Data[3] = Can2_RxMessage.Data[3];
        Can2_RxBalance[(int)Can2_RxMessage.StdId].Data[4] = Can2_RxMessage.Data[4];
        Can2_RxBalance[(int)Can2_RxMessage.StdId].Data[5] = Can2_RxMessage.Data[5];
        Can2_RxBalance[(int)Can2_RxMessage.StdId].Data[6] = Can2_RxMessage.Data[6];
        Can2_RxBalance[(int)Can2_RxMessage.StdId].Data[7] = Can2_RxMessage.Data[7];
    }
}
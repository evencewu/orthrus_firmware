#ifndef __RX_TASK_H
#define __RX_TASK_H

#include "bsp_can.h"
#include "main.h"
#include "struct_typedef.h"

void bmi088_Task(void const * argument);
void Float_to_Byte(float a,float b,unsigned char byte[]);   //参考https://blog.csdn.net/ls667/article/details/50811519博客



#endif

#include "bsp_can.h"
#include "main.h"
#include "struct_typedef.h"

void Float_to_Byte(float a,float b,unsigned char byte[]);

uint8_t byte_0[8];
uint8_t byte_1[8];



//定义共用体，参考https://blog.csdn.net/ls667/article/details/50811519博客
typedef union     
{
	float fdata;
	unsigned long ldata;
}FloatLongType;


/*将浮点数f转化为4个字节数据存放在byte[4]中*/
void Float_to_Byte(float a,float b,unsigned char byte[])   //参考https://blog.csdn.net/ls667/article/details/50811519博客
{
	FloatLongType fl,f2;
	fl.fdata=a;f2.fdata=b;
	byte[0]=(unsigned char)fl.ldata;
	byte[1]=(unsigned char)(fl.ldata>>8);
	byte[2]=(unsigned char)(fl.ldata>>16);
	byte[3]=(unsigned char)(fl.ldata>>24);
    byte[4]=(unsigned char)f2.ldata;
	byte[5]=(unsigned char)(f2.ldata>>8);
	byte[6]=(unsigned char)(f2.ldata>>16);
	byte[7]=(unsigned char)(f2.ldata>>24);
	
}





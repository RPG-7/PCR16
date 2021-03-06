#include "misc.h"


void delay_ms(u16 ms)
{    
   u16 i=0;  
   while(ms--)
   {
      i=20000;  //自己定义
      while(i--) ;    
   }
}

s16 FUN_MIN16S(s16 x, s16 y)
{
	if(x > y)	{
		return y;
	}
		return x;
}
//自己写的log函数
float MYLOG(float a)
{
   int N = 10;//取5+1项计算
   int k,nk;
   float x,xx,y;
   x = (a-1)/(a+1);
   xx = x*x;
   nk = 2*N+1;
   y = 1.0/nk;
   for(k=N;k>0;k--)
   {
     nk = nk - 2;
     y = 1.0/nk+xx*y;
   }
	x *= (float)2.0;
   return (float)(x*y);
}
//热敏电阻和温度转换公式 1/T1 =ln(Rt/Rp)/Bx+1/T2
/*#define Rp				120
#define Bx				3910*/
#define	Ka				273.15f
#define TEMP_25			25.0f
#define T2_cent			(1/(Ka+TEMP_25))//0.003354f//298.15f
float CalculateTemperature(u32 Rt,const u32 Rp,const u32 Bx)
{
	float temp;

	temp = (Rt*1.0)/Rp;
	temp = MYLOG(temp);//ln(Rt/Rp)
	temp/=Bx;//ln(Rt/Rp)/B
	temp+=T2_cent;
	temp = 1/(temp);
	temp-=Ka;

	return temp;
}

INT16U CRC16_Sum(INT8U *dat, INT8U len)
{
	INT16U crc=0,j;
	for(j=0; j<len; j++)
	{
		crc = crc + dat[j];
	}
	return crc;
}

/*********************************************************************
 *  FUNCTION: FloatTo4uint8Format							          *
 *																	  *
 *  PARAMETERS:  							                          *
 *																	  *
 *  DESCRIPTION: ?Float?????char??              			  *
 *																 	  *
 *  RETURNS: ?													   	  *
 *																	  *
 *********************************************************************/
u16 FloatTo4uint8Format(u8* Dest, const float Source)
{
    u16 Len,i;
    u32 temp = Source;
    Len = sizeof(float);
    //temp = *(u32*)&Source;

    for (i = 0; i < Len; i++)
    {
        *Dest++ = (u8)(temp >> (8*(Len-i-1)));
    }

    return Len;
}

s32 floatToInt(float f)
{
    s32 i = 0;
    if(f>0) //
      i = (s32)((f*10 + 5)/10);
    else if(f<0) //
      i = (s32)((f*10 - 5)/10);
    else i = 0;

    return i;
}
//冒泡排序
u8 BubbleSort(u8 *pbuf,u32 size)
{
	u8 i,j,flag;
	u8 x,y;

	if(size==0)	{
			return 0;
	}
	i=0;
	/*---------------- 冒泡排序,由小到大排序 -----------------*/
	do{
		flag=0;
		for (j=0;j<size-i-1;j++)
		{
			if (pbuf[j] > pbuf[j+1])
			{
				x = pbuf[j];
				pbuf[j]   = pbuf[j+1];
				pbuf[j+1] = x;
				flag = 1;
			}
			if (pbuf[j] > pbuf[j+1])
			{
				y = pbuf[j];
				pbuf[j] = pbuf[j+1];
				pbuf[j+1] = y;
				flag = 1;
			}
		}
		i++;
	}while((i<size) && flag);
	//////////////////////////////////////////////////////
	return 1;
}

//BCD码转十进制函数，输入BCD，返回十进制
u8 BCD_Decimal(u8 bcd)
{
	u8 Decimal;

	Decimal=bcd>>4;
	return(Decimal=Decimal*10+(bcd&=0x0F));
}
//实现split以某个字符分割一个字符串
//		src 源字符串的首地址(buf的地址) strtok函数会破坏源字符串完整
//		separator 指定的分割字符
//		dest 接收子字符串的数组
//		num 分割后子字符串的个数
void split(char *src,const char *separator,char **dest,u16 *num) 
{
	char *pNext;
	u16 count;

	count = 0;
	if (src == NULL || strlen(src) == 0) //如果传入的地址为空或长度为0，直接终止 
		return;
	if (separator == NULL || strlen(separator) == 0) //如未指定分割的字符串，直接终止 
		return;
	pNext = (char *)strtok(src,separator); //必须使用(char *)进行强制类型转换(虽然不写有的编译器中不会出现指针错误)
	while(pNext != NULL) {
		*dest++ = pNext;
		++count;
		pNext = (char *)strtok(NULL,separator);  //必须使用(char *)进行强制类型转换
	}
	*num = count;
} 

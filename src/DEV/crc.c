#include "crc.h"


float OutData[4]={ 0 };//放置将要传递的虚拟示波器四个通道的数


//==================================================================
//获取循环冗余校验码
//入口：缓冲区：*Buf  深度：CRC_CNT
//返回：无
//
//==================================================================
unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT)
{
    unsigned short CRC_Temp;
    unsigned char i,j;
    CRC_Temp = 0xffff;

    for (i=0;i<CRC_CNT; i++){      
        CRC_Temp ^= Buf[i];
        for (j=0;j<8;j++) {
            if (CRC_Temp & 0x01)
                CRC_Temp = (CRC_Temp >>1 ) ^ 0xa001;
            else
                CRC_Temp = CRC_Temp >> 1;
        }
    }
    return(CRC_Temp);
}




//==================================================================
//四通道数据
//入口：无
//返回：无
//
//==================================================================
void OutPut_Data(void)
{
  int temp[4];
  unsigned int temp1[4] = {0};
  unsigned char databuf[10] = {0};
  unsigned char i;
  short CRC16 = 0;
  for(i=0;i<4;i++)
   {
    
    temp[i]  = (int)OutData[i];
    temp1[i] = (unsigned int)temp[i];
    
   }
   
  for(i=0;i<4;i++) 
  {
    databuf[i*2]   = (unsigned char)(temp1[i]%256);
    databuf[i*2+1] = (unsigned char)(temp1[i]/256);
  }
  
  CRC16 = CRC_CHECK(databuf,8);
  databuf[8] = CRC16%256;
  databuf[9] = CRC16/256;
  
  for(i=0;i<10;i++)
  //UART_SendData(UART0,*databuf);
	UART_printf("%d",databuf);
}




//==================================================================
//发送数据
//入口：无
//返回：无
//
//==================================================================
void sendData(void)
{

 OutData[0]=1000;
 OutData[1]=500;
 OutData[2]=250;
 OutData[3]=125;
 OutPut_Data();
 
}



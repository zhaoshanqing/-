/*Voice.cpp——语音识别库文件。pps
*/
#include "utility/PinMap.h"
#include "Voice.h"

 typedef VoiceClass Voice;　//起别名


void VoiceClass::Input(){  //设置输入方式
  if(I==1) {　　　//如果I等于１
    LD_WriteReg(0x1c, 0x0b);//
    Serial.println("MIC");　　//打印出MIC
     }
  if(I==2) { //如果I等于２
    LD_WriteReg(0x1c, 0x23);
    Serial.println("MONO");//打印出MoNo    
  }
}

void VoiceClass::MI(){
  I=1;
}

void VoiceClass::MO(){
  I=2;
}

void VoiceClass::Initialise(uint8 mode,uint8 board){//initialise
  if(mode==MIC) //mode==MIC(88)
    MI(); //I取值为1
  else if(mode==MONO) //mode==MONO(99)
       MO();  //I取值为2
  begin(4);        //使用数字口4,作为输出引脚
  if(board)
     RSTB(9);//VoiceRecognitionV1.0 Arduino shield 使用数字口9，作为输出引脚
  else
     RSTB(6);    //VoiceRecognitionV2.0 Gadgeteer Compatible module 使用数字口6，作为输出引脚
  setDataMode(0x08);
  LD_reset();//reset
}

uint8_t VoiceClass::begin(uint8_t chipSelectPin){

chipSelectPin_ = chipSelectPin;
  // set pin modes
  pinMode(chipSelectPin_, OUTPUT);//设置chipSelectPin_引脚作为输出
  chipSelectHigh();
  pinMode(SPI_MISO_PIN, INPUT);//设置外部串行总线的主机输入从机输出引脚为输入
  pinMode(SPI_MOSI_PIN, OUTPUT);//设置外部串行总线的主机输出从机输入引脚为输出
  pinMode(SPI_SCK_PIN, OUTPUT);//设置外部串行总线的时钟引脚为输出

#ifndef SOFTWARE_SPI
  // SS must be in output mode even it is not chip select
  pinMode(SS_PIN, OUTPUT);
  digitalWrite(SS_PIN, HIGH); // disable any SPI device using hardware SS pin
  // Enable SPI, Master, clock rate f_osc/128
  SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR1) | (1 << SPR0);//set  SPE MSTR  SPR1 SPR0 bits for 1 
  // clear double speed
  SPSR &= ~(1 << SPI2X); 
#endif  // SOFTWARE_SPI
}

void VoiceClass::chipSelectHigh(void) {
  digitalWrite(chipSelectPin_, HIGH);//chipSelectPin_引脚设置为高电平
}

//------------------------------------------------------------------------------
void VoiceClass::chipSelectLow(void) {
  digitalWrite(chipSelectPin_, LOW);//chipSelectPin引脚设置为低电平
}

void VoiceClass::setDataMode(uint8_t mode)
{
  SPCR = (SPCR & ~SPI_MODE_MASK) | mode; /*设置时钟极性(3　bit)和时钟相位(2 bit),为0,mode:四种工作方式*/
}

uint8_t VoiceClass::RSTB(uint8_t RSTB)
{
  RSTB_=RSTB;
  pinMode(RSTB_,OUTPUT);//RSTB引脚设置为输出
  
}

void VoiceClass::LD_WriteReg(unsigned char address,unsigned char value)//将value写进address
{

  address_=address; //赋值
  value_=value;　//赋值
	
  chipSelectLow();//chipSelectpin_设置输出为低电平
  delay(10);//延迟10毫秒
  Voice.transfer(0x04);//当SPIF位为１的时候，返回0x04
  Voice.transfer(address_);//当spif有效的时候，返回address_
  Voice.transfer(value_);//当SPIF有效的时候，返回value_
  chipSelectHigh(); //chipSelectpin_设置输出为高电平
 }

unsigned char VoiceClass::LD_ReadReg(unsigned char address)
{ 
   address_=address; //赋值

  chipSelectLow(); //设置chipSelectpin_设置为低电平
  delay(10);//延迟10毫秒
  Voice.transfer(0x05);//当spif的值为１时，返回0x05
  Voice.transfer(address_);//
  result = Voice.transfer(0x00);  
  chipSelectHigh();
  return(result);
 }

void VoiceClass::LD_reset()
{

  digitalWrite(RSTB_,HIGH);
  delay(1);
  digitalWrite(RSTB_,LOW);
  delay(1);
  digitalWrite(RSTB_,HIGH);
  delay(1);
  chipSelectLow();
  delay(1);
  chipSelectHigh();
  delay(1);
}


void VoiceClass::LD_Init_Common()
{

  LD_ReadReg(0x06);//读出0x06地址下数据  
  LD_WriteReg(0x17, 0x35); //0x35写进0x17地址里面
  delay(10);　　　//延迟10毫秒
  LD_ReadReg(0x06);  //读出0x06地址下的数据

  LD_WriteReg(0x89, 0x03);  //将0x03写进0x89地址里面
  delay(5);
  LD_WriteReg(0xcf, 0x43);  
  delay(5);
  LD_WriteReg(0xcb, 0x02);

  LD_WriteReg(0x11, LD_PLL_11);  
  LD_WriteReg(0x1e,0x00);
  LD_WriteReg(0x19, LD_PLL_ASR_19); 
  LD_WriteReg(0x1b, LD_PLL_ASR_1B);	
  LD_WriteReg(0x1d, LD_PLL_ASR_1D);

  delay(10);
  LD_WriteReg(0xcd, 0x04);
  LD_WriteReg(0x17, 0x4c); 
  delay(5);
  LD_WriteReg(0xb9, 0x00);
  LD_WriteReg(0xcf, 0x4f); 
}


void VoiceClass::LD_Init_ASR()
{
  nLD_Mode=LD_MODE_ASR_RUN;//LD_MODE_ASR_RUN
  LD_Init_Common();

  LD_WriteReg(0xbd, 0x00);
  LD_WriteReg(0x17, 0x48);
  delay(10);

  LD_WriteReg(0x3c, 0x80);  
  LD_WriteReg(0x3e, 0x07);
  LD_WriteReg(0x38, 0xff);  
  LD_WriteReg(0x3a, 0x07);
  LD_WriteReg(0x40, 0);   
  LD_WriteReg(0x42, 8);
  LD_WriteReg(0x44, 0); 
  LD_WriteReg(0x46, 8); 
  delay(1);
}

void VoiceClass::ProcessInt0()
{ 
  uint8 nAsrResCount=0;
  //detachInterrupt(0) ;
  ucRegVal = LD_ReadReg(0x2b);
	if(nLD_Mode == LD_MODE_ASR_RUN)
	{
		LD_WriteReg(0x29,0) ;
		LD_WriteReg(0x02,0) ;	
		if((ucRegVal & 0x10) &&
			LD_ReadReg(0xb2)==0x21 && 
			LD_ReadReg(0xbf)==0x35)
		{
            
			nAsrResCount = LD_ReadReg(0xba);
			if(nAsrResCount>0 && nAsrResCount<4) 
			{
                                Serial.print( "ASR_FOUN ONE: ");
				nAsrStatus=LD_ASR_FOUNDOK;
			}
			else
		        {
                                 Serial.println( "ASR_FOUND ZERO");
                                 Serial.println( " ");
				nAsrStatus=LD_ASR_FOUNDZERO;
			}	
		}
		else
		{
                        Serial.println( "ASR_FOUND ZERO");
                        Serial.println( " ");
			nAsrStatus=LD_ASR_FOUNDZERO;
		}
			
		LD_WriteReg(0x2b, 0);
    LD_WriteReg(0x1C,0);

		return;
	}
	
  delay(10);
  //detachInterrupt(1); 
}
unsigned char VoiceClass::LD_Check_ASRBusyFlag_b2()  //检测忙碌标志位0xb2,是否为0x21,如果是，则flag置为１，否则置为０
{ 
  uint8 j;
  uint8 flag = 0;
  for (j=0; j<10; j++)
	{
	  if (LD_ReadReg(0xb2) == 0x21)
		{
			flag = 1;
			break;
		}
	  delay(10);		
	}
  return flag;
}

void VoiceClass::LD_AsrStart()
{
  LD_Init_ASR();
}

unsigned char VoiceClass::LD_AsrRun()
{
  LD_WriteReg(0x35, MIC_VOL);//MIC_VOL  0X55

  LD_WriteReg(0x1c, 0x09);

  LD_WriteReg(0xbd, 0x20);
  LD_WriteReg(0x08, 0x01);
  delay( 1);
  LD_WriteReg(0x08, 0x00);
  delay( 1);
  if(LD_Check_ASRBusyFlag_b2() == 0) /*flag==0*/
	{
		return 0; /*如果flag=1,则程序结束*/
	}

  LD_WriteReg(0xb2, 0xff);

  LD_WriteReg(0x37, 0x06);
  delay( 5 );
  //Serial.println( LD_ReadReg(0xbf),BYTE);
  //	LD_WriteReg(0x1c, 0x0b);
  Input();
  LD_WriteReg(0x29, 0x10);
  LD_WriteReg(0xbd, 0x00);
  //detachInterrupt(1) ;
  return 1;
}

unsigned char VoiceClass::RunASR(int x,int y,char (*p)[30])
{
  int i=0;
  int r=0;
  for (i=0; i<5; i++)	//防止由于硬件原因导致LD3320芯片工作不正常，所以一共尝试5次启动ASR识别流程
	{
		LD_AsrStart();

		delay(100);
		if (LD_AsrAddFixed(x,y,p)==0)
		{

			LD_reset();			//	LD3320芯片内部出现不正常，立即重启LD3320芯片
			delay(100);			//	并从初始化开始重新ASR识别流程
		continue;	
		}
		delay(10);
		if (LD_AsrRun() == 0)
		{
			LD_reset();			//	LD3320芯片内部出现不正常，立即重启LD3320芯片
			delay(100);			//	并从初始化开始重新ASR识别流程
                 continue;
		}
		r=1;
		break;					//	ASR流程启动成功，退出当前for循环。开始等待LD3320送出的中断信号
	}
  return r;
}

unsigned char VoiceClass::LD_AsrAddFixed(int x,int y,char (*p)[30])
{
  int k,flag;
  int nAsrAddLength;

	flag = 1;
	for (k=0; k<x; k++)
	{
			
		if(LD_Check_ASRBusyFlag_b2() == 0) //如果不忙的话，则跳出循环
		{
			flag = 0;
			break;
		}
		
		LD_WriteReg(0xc1, k);
		LD_WriteReg(0xc3, 0 );
		LD_WriteReg(0x08, 0x04);
  
		delay(1);　//延迟１毫秒
		LD_WriteReg(0x08, 0x00);
		delay(1);//延迟1毫秒

		for (nAsrAddLength=0; nAsrAddLength<y; nAsrAddLength++)
		{
			if (p[k][nAsrAddLength] == 0)
				break;
			LD_WriteReg(0x5, p[k][nAsrAddLength]);
 
		}
		LD_WriteReg(0xb9, nAsrAddLength);
		LD_WriteReg(0xb2, 0xff);
 
		LD_WriteReg(0x37, 0x04);
	}
  //Serial.println( LD_ReadReg(0xbf),BYTE);  
  return flag;
}

unsigned char VoiceClass::LD_GetResult()
{
  return LD_ReadReg(0xc5 );　//读出0xc5里面的数据
}




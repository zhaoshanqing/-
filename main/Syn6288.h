#include <util/delay.h>
#include "Syn6288.h"
#include <stdio.h>
#include <arduino.h>
#ifndef Syn6288_h
#define Syn6288_h
#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>            /*pgmspace.h是将数据保存在程序存储器中的相关函数，有了这个头文件，
                  你可以定义一个数组，把这个数组保存在ROM中，而不是RAM中，调用时需要特殊的命令，
                  具体可以参考GCC自带的文档。现在版本的AVR Studio只要添加avr/io.h就可以了，
                  但是需要在设置中选择你所使用的单片机型号，编译器自动选择相应的头文件。*/
#include <util/delay.h>        //延迟工具包
#include <stddef.h>             //stddef .h 头文件定义了各种变量类型和宏。这些定义中的大部分也出现在其它头文件中.定义宏

#define HEADLEN      5        //头部长度为5
//#define LEN_OFFSET    2

//#define uint8_t    unsigned int
class Syn6288
{
  public:

//private:
uint8_t music;
uint8_t TEXTLEN;
uint8_t pi;
void play(uint8_t *text,uint8_t TEXTLEN,uint8_t music);
//void play(uint8_t *text,uint8_t music);
void Slaveboudset(uint16_t boudr);  //设置波特率
void stop();           //清除所有时钟选择位
void restore();         //复位
void inquire();       //查询
void Pause();         //暂停，中止
void sleep();         //睡眠

};
#endif

uint8_t head[5] = {0xfd,0x00,0x00,0x01,0x00};//合成播放命令
uint8_t sound[5] = {0xfd,0x00,0x00,0x01,0x01};//合成提示音
uint16_t boud[5]={0xFD,0x00,0x03,0x31,0x00}; //设置波特率
/*发送文本合成命令，music为背景音乐设置*/
void Syn6288::play(uint8_t *text,uint8_t TEXTLEN,uint8_t music)//输出语音指令
  {
    pi=0;
    delay(100);
  head[2]=TEXTLEN+3;
   switch(music)
        {
          case 0: head[4]=0x01;break;
          case 1: head[4]=0x09;break;
          case 2: head[4]=0x11;break;
          case 3: head[4]=0x19;break;
          case 4: head[4]=0x21;break;
          case 5: head[4]=0x29;break; 
          case 6: head[4]=0x31;break;
          case 7: head[4]=0x39;break;
          case 8: head[4]=0x41;break;
          case 9: head[4]=0x49;break; 
          case 10: head[4]=0x51;break;
          case 11: head[4]=0x59;break;
          case 12: head[4]=0x61;break;
          case 13: head[4]=0x69;break;
          case 14: head[4]=0x71;break;
          case 15: head[4]=0x79;break;
        } 
   for(int i=0;i<5;i++)
      {
        pi^=head[i];
        Serial.write(head[i]);
        delay(2);
      }
   for(int j=0;j<TEXTLEN;j++)
      {
        pi^=text[j];
        Serial.write(text[j]);
        delay(2);
      }
    Serial.write(pi);
    delay(300*TEXTLEN);
  }



void Syn6288::Slaveboudset(uint16_t boudr)  //波特率设置  
{                                             
  uint8_t p;
    p=0;
switch(boudr)
  {

   case 9600:   boud[4]=0x00;
                   break;
   case 19200:  boud[4]=0x01;
                   break;
  }
  for(int z=0;z<HEADLEN;z++)    //HEADLEN=5，头部长度为5
    {
      p^=boud[z];
      Serial.write(boud[z]);
      delay(2);
    }               
  Serial.write(p);
  p=0;
  for(int z=0;z<HEADLEN;z++)
    {
      p^=boud[z];
      Serial.write(boud[z]);
      delay(2);
    }               
  Serial.write(p);
  delay(200);
}
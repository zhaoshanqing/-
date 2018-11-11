/*
识别关键词的个数为SUM；
n为数组中对应关键词的序列号，例如数组sRecog中的第一个关键词为“kai deng”则对应的序列号为0；
Voice.Initialise();初始化设置输入方式MIC/MONO。
*/
#include <TimerOne.h>
#include <avr/wdt.h>
#include <Voice.h>
#include <Syn6288.h>
//#include <DHT.h>

//#define DHTTYPE DHT11   // DHT 11 

Syn6288 syn;
uint8  nAsrStatus=0;
#define SUM 20
 
#define DHT11_PIN 6          //DHT11传感器插数字口6
#define light_sensor_pin A2  //光线传感器插模拟口2
#define light_threshold 200  //光线传感器阈值
//#define gas_sensor_pin A1    //气体传感器插模拟口1
//#define gas_threshold 700    //气体传感器报警阈值
#define relay  8              //继电器插数字口8,灯１
       //***
#define D5  5　　　　　　　　　//灯２

#define state_led 7          //状态指示灯插数字口7





//DHT dht(DHT11_PIN, DHTTYPE);

char sRecog[SUM][30] = {"xiao jian guo","an yi dian ","kai deng","guan deng","xu yao kai deng ma","liang yi dian","xiao jian","xiao","jian guo","","kai","guan","liang","liang yi","an","an yi","yi dian"};
//                            0               1             2         3             4                   5               6         7       8      9   10    11      12        13      14    15      16小坚果，开灯，关灯，需要开灯吗

//uint8_t text1[]={0xC1,0xE3,0xD2,0xBB,0xB6,0xFE,0xC8,0xFD,0xCB,0xC4,0xCE,0xE5,0xC1,0xF9,0xC6,0xDF,0xB0,0xCB,0xBE,0xC5,0xCA,0xAE};//零一二三四五六七八九十
uint8_t text2[]={0xC4,0xFA,0xBA,0xC3};// 您好
//uint8_t text3[]={0xB5,0xB1,0xC7,0xB0,0xCE,0xC2,0xB6,0xC8,0xCE,0xAA};//当前温度为
//uint8_t text4[]={0xCA,0xAA,0xB6,0xC8,0xCE,0xAA,0xB0,0xD9,0xB7,0xD6,0xD6,0xAE};//湿度为百分之
//uint8_t text6[]="soundn";//报警音
//uint8_t text7[]={0xC7,0xEB,0xD7,0xA2,0xD2,0xE2,0xA3,0xAC,0xD3,0xD0,0xCE,0xA3,0xCF,0xD5,0xC6,0xF8,0xCC,0xE5,0xD0,0xB9,0xC2,0xA9};//请注意，有危险气体泄漏
uint8_t text7[]={0xBA,0xC3,0xB5,0xC4};//好的
uint8_t text8[]={0xD5,0xFD,0xD4,0xDA,0xBF,0xAA,0xB5,0xC6};//正在开灯
uint8_t text9[]={0xD5,0xFD,0xD4,0xDA,0xB9,0xD8,0xB5,0xC6};//正在关灯
uint8_t text10[]={0xC7,0xEB,0xCE,0xCA,0xC4,0xE3,0xD4,0xDA,0xCB,0xB5,0xCA,0xB2,0xC3,0xB4,0xA3,0xBF};//请问你在说什么？
uint8_t text11[]={0xB9,0xE2,0xCF,0xDF,0xB2,0xBB,0xD7,0xE3,0x20,0xA3,0xAC,0xD5,0xFD,0xD4,0xDA,0xBF,0xAA,0xB5,0xC6};//光线不足，正在开灯
uint8_t text12[]={0xCF,0xD6,0xD4,0xDA,0xB9,0xE2,0xCF,0xDF,0xC3,0xF7,0xC1,0xC1,0xA3,0xAC,0xB2,0xBB,0xD0,0xE8,0xD2,0xAA,0xBF,0xAA,0xB5,0xC6};//光线明亮，不需要开灯


//uint8_t text[2];
uint8_t flag;  //标志位，收到小坚果口令后将置为1，动作执行完后清零

void finally(uint8_t n) //n为sRecog数组中对应关键词的序列号
{
  switch(n)
  {
        case 0: syn.play(text2,sizeof(text2),0);flag=1;break; // 您好
    /*case 1:       //现在温度怎么样
             if(flag==1)  //收到小米渣的出发命令后
             {
          flag=0;
          uint8_t h = dht.readHumidity();
                                  uint8_t t = dht.readTemperature();  //读取温度
          //Serial.print("Temperature:");
          //Serial.print(DHT.temperature);
          //Serial.print("   humidity:");
          //Serial.println(DHT.humidity);
          uint8_t pig1 = t/10;  //温度数值十位
          uint8_t pig2 = t%10;  //温度数值个位
          uint8_t pig3 = h/10;    //湿度数值十位
          uint8_t pig4 = h%10;    //湿度数值个位
          syn.play(text3,sizeof(text3),0);  //当前温度为    
          if(pig1>0){
          text[0]=text1[pig1*2];
          text[1]=text1[pig1*2+1];
          syn.play(text,sizeof(text),0);    //播放温度数值十位
          text[0]=text1[20];
          text[1]=text1[21];
          syn.play(text,sizeof(text),0);    //播放语音十
          }
          if(pig2>0){
          text[0]=text1[pig2*2];
          text[1]=text1[pig2*2+1];
          syn.play(text,sizeof(text),0);    //播放温度数值个位  
          }         
          syn.play(text5,sizeof(text5),0);  //播放摄氏度
          syn.play(text4,sizeof(text4),0);  //播放湿度为百分之
          
          if(pig3>0){
          text[0]=text1[pig3*2];
          text[1]=text1[pig3*2+1];
          syn.play(text,sizeof(text),0);  //播放湿度数值十位
          text[0]=text1[20];
          text[1]=text1[21];
          syn.play(text,sizeof(text),0); //播放语音十
          }
          if(pig4>0){
          text[0]=text1[pig4*2];
          text[1]=text1[pig4*2+1];
          syn.play(text,sizeof(text),0);  //播放湿度数值个位  
          }         
             }
      break;*/
    case 2: //开灯
             if(flag==1)
             {
               syn.play(text8,sizeof(text8),0); //播放正在开灯
              digitalWrite(D5,HIGH);
               digitalWrite(relay,LOW);  //高电平驱动
               flag=0;  
             }
      break;  
        case 3: //关灯
       if(flag==1)
       {
        syn.play(text9,sizeof(text9),0); //播放正在关灯
        digitalWrite(relay,LOW);
        digitalWrite(D5,LOW);
        flag=0;
      }
      break;
        case 4: //需要开灯吗？
      if(flag==1)
       {
        int shine=analogRead(light_sensor_pin); //读取光照度模拟量
        //Serial.print("Light:");
        //Serial.println(shine);
        if(shine < light_threshold) //小于光照度阀值时
        {
          syn.play(text11,sizeof(text11),0); //光线不足，正在开灯
          digitalWrite(relay,LOW);
          digitalWrite(D5,HIGH);
        }
        else 
        {
          syn.play(text12,sizeof(text12),0); //光线明亮，不需要开灯
          digitalWrite(relay,LOW);
          digitalWrite(D5,LOW);
        }
        flag=0;
       }
            break;
    case 5: //亮一点？ 
        if(flag==1)
             {
               syn.play(text7,sizeof(text7),0); //播放好的
               digitalWrite(D5,HIGH);  //高电平驱动
                digitalWrite(relay,HIGH);
                delay(100);
               flag=0;  
             }
      break;  
     case 1: //暗一点？
        if(flag==1)
             {
               syn.play(text7,sizeof(text7),0); //播放好的
               digitalWrite(D5,HIGH);  //高电平驱动
            digitalWrite(relay,LOW);
            delay(100);
               flag=0;  
             }
      break;  
      
    default:syn.play(text10,sizeof(text10),0);break;  //非正常命令，请问你在说什么     
  }
}

void ExtInt0Handler ()
{
  Voice.ProcessInt0();  //芯片送出中断信号
        
}
void timerIsr()
{
  wdt_reset();
}
void setup()
{
  Serial.begin(9600);
  Voice.Initialise(MIC,VoiceRecognitionV1);//Initialise mode MIC or MONO,default is MIC
                                           //VoiceRecognitionV1 is VoiceRecognitionV1.0 shield
                                           //VoiceRecognitionV2 is VoiceRecognitionV2.1 module  
  attachInterrupt(0,ExtInt0Handler,LOW);
 
  pinMode(D5,OUTPUT);
  digitalWrite(D5,LOW);
  
   pinMode(relay,OUTPUT);
  digitalWrite(relay,LOW);


  pinMode(state_led,OUTPUT);
  digitalWrite(state_led,LOW);
  Timer1.initialize(90000);
  Timer1.attachInterrupt(timerIsr); // attach the service routine here
  wdt_enable(WDTO_120MS);
}
void loop()
{  
    static uint8_t nAsrRes=0;
    static unsigned long time = millis();
  if(millis()-time>100)
  {  
       /*int gas=analogRead(gas_sensor_pin);
       if(gas > gas_threshold)
       {
      syn.play(text6,sizeof(text6),0);
      syn.play(text7,sizeof(text7),0);
       }*/
    switch(nAsrStatus)
    {
      case LD_ASR_RUNING:
      case LD_ASR_ERROR:break;
      case LD_ASR_NONE:
      {
        nAsrStatus=LD_ASR_RUNING;
                if (Voice.RunASR(SUM,30,sRecog)==0) //识别不正确
        {
          nAsrStatus= LD_ASR_ERROR;
                    Serial.println( "ASR_ERROR"); 
        }

                Serial.println( "ASR_RUNING.....");
        digitalWrite(state_led,HIGH);
        break;
      }
      case LD_ASR_FOUNDOK:
      {
        digitalWrite(state_led,LOW);
        nAsrRes =Voice. LD_GetResult();// 一次ASR识别流程结束，去取ASR识别结果            
                finally(nAsrRes);
        nAsrStatus = LD_ASR_NONE;
        break;
      }
      case LD_ASR_FOUNDZERO:
      default:
      { 
        digitalWrite(state_led,LOW);
        nAsrStatus = LD_ASR_NONE;
        break;
      }
      }// switch

  }//if
}
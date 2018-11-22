/*

 *  Interrupt and PWM utilities for 16 bit Timer1 on ATmega168/328
 
 */
#ifndef TIMERONE_cpp
#define TIMERONE_cpp

#ifndef TIMERONE_h
#define TIMERONE_h

#include <avr/io.h>
#include <avr/interrupt.h>

#define RESOLUTION 65536    // Timer1 is 16 bit，分辨率resolution,2^16=65536

class TimerOne
{
  public:
  
    // properties
    unsigned int pwmPeriod;
    unsigned char clockSelectBits;   //时钟选择位
  char oldSREG;         // To hold Status Register while ints disabled  在INT禁用时保存状态寄存器

    // methods
    void initialize(long microseconds=1000000);//设置初始化，周期=1s
    void start();
    void stop();
    void restart();
  void resume();     //继续
  unsigned long read();
    void pwm(char pin, int duty, long microseconds=-1);
    void disablePwm(char pin);
    void attachInterrupt(void (*isr)(), long microseconds=-1);  //联系中断  
    void detachInterrupt();      //拆分中断
    void setPeriod(long microseconds);
    void setPwmDuty(char pin, int duty);//设置pwm的作用
    void (*isrCallback)();
};

extern TimerOne Timer1;
#endif

TimerOne Timer1;             

ISR(TIMER1_OVF_vect)          // interrupt service
{
  Timer1.isrCallback();    //中断服务回调函数就相当于一个中断处理函数，由系统在符合你设定的条件时自动调用
}


void TimerOne::initialize(long microseconds)
{
  TCCR1A = 0;                 // clear control register A 
  TCCR1B = _BV(WGM13);        // 设置模式8：相位和频率校正PWM，停止计时器
  setPeriod(microseconds);
}


void TimerOne::setPeriod(long microseconds)   // AR modified for atomic accessAr修改用于原子访问
{
  
  long cycles = (F_CPU / 2000000) * microseconds;                                // 中断位于底部，所以将微秒除以2。
  if(cycles < RESOLUTION)              clockSelectBits = _BV(CS10);              // no prescale, full xtal  无分频，全xtal
  else if((cycles >>= 3) < RESOLUTION) clockSelectBits = _BV(CS11);              // prescale by /8
  else if((cycles >>= 3) < RESOLUTION) clockSelectBits = _BV(CS11) | _BV(CS10);  // prescale by /64
  else if((cycles >>= 2) < RESOLUTION) clockSelectBits = _BV(CS12);              // prescale by /256
  else if((cycles >>= 2) < RESOLUTION) clockSelectBits = _BV(CS12) | _BV(CS10);  // prescale by /1024
  else        cycles = RESOLUTION - 1, clockSelectBits = _BV(CS12) | _BV(CS10);  // 请求超出了范围，设置为最大值。
  
  oldSREG = SREG;       
  cli();              //禁用16位寄存器访问的中断
  ICR1 = pwmPeriod = cycles;                                          // ICR1 is TOP in p & f correct pwm mode在p&f正确的PWM模式中，icr 1是最高的
  SREG = oldSREG;
  
  TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
  TCCR1B |= clockSelectBits;                                          // reset clock select register, and starts the clock重置时钟选择寄存器，并启动时钟
}

void TimerOne::setPwmDuty(char pin, int duty)
{
  unsigned long dutyCycle = pwmPeriod;
  
  dutyCycle *= duty;
  dutyCycle >>= 10;
  
  oldSREG = SREG;
  cli();
  if(pin == 1 || pin == 9)       OCR1A = dutyCycle;
  else if(pin == 2 || pin == 10) OCR1B = dutyCycle;
  SREG = oldSREG;
}

void TimerOne::pwm(char pin, int duty, long microseconds)  // expects duty cycle to be 10 bit (1024)期望占空比为10位(1024)
{
  if(microseconds > 0) setPeriod(microseconds);
  if(pin == 1 || pin == 9) {
    DDRB |= _BV(PORTB1);                                   // 为PWM输出引脚设置数据方向寄存器
    TCCR1A |= _BV(COM1A1);                                 // 激活输出引脚
  }
  else if(pin == 2 || pin == 10) {
    DDRB |= _BV(PORTB2);
    TCCR1A |= _BV(COM1B1);
  }
  setPwmDuty(pin, duty);
  resume();     // Lex - make sure the clock is running. Lex-确保时钟在运转
          // and the first one is in the middle of a cycle第一个是在循环的中间
}

void TimerOne::disablePwm(char pin)
{
  if(pin == 1 || pin == 9)       TCCR1A &= ~_BV(COM1A1);   // clear the bit that enables pwm on PB1清除在pb1上启用pwm的位
  else if(pin == 2 || pin == 10) TCCR1A &= ~_BV(COM1B1);   // clear the bit that enables pwm on PB2清除在pb2上启用pwm的位
}

void TimerOne::attachInterrupt(void (*isr)(), long microseconds)
{
  if(microseconds > 0) setPeriod(microseconds);
  isrCallback = isr;                                       // register the user's callback with the real ISR将用户的回调注册到实际的ISR中
  TIMSK1 = _BV(TOIE1);                                     // sets the timer overflow interrupt 设置计时器溢出中断
//  sei();
  resume();                       
}

void TimerOne::detachInterrupt()
{
  TIMSK1 &= ~_BV(TOIE1);                                   // clears the timer overflow interrupt enable bit 清除计时器溢出中断启用位
                              // timer continues to count without calling the isr计时器继续计数，而不调用ISR。
}

void TimerOne::resume()       // AR suggested
{ 
  TCCR1B |= clockSelectBits;
}

void TimerOne::restart()    // Depricated - Public interface to start at zero分解-公共接口从零开始
{
  start();        
}

void TimerOne::start()  // AR addition, renamed by Lex to reflect it's actual role     Ar加法，改名为lex，以反映它的实际作用
{
  unsigned int tcnt1;
  
  TIMSK1 &= ~_BV(TOIE1);        // AR added    AR加
  GTCCR |= _BV(PSRSYNC);      // AR added - reset prescaler (NB: shared with all 16 bit timers);增加-重设定分频(NB:与所有16位定时器共享)

  oldSREG = SREG;       // AR - save status register  AR -保存状态寄存器
  cli();            // AR - Disable interrupts  AR、-禁止中断
  TCNT1 = 0;                  
  SREG = oldSREG;             // AR - Restore status register  AR-恢复状态寄存器
  resume();
  do {  // Nothing -- wait until timer moved on from zero - otherwise get a phantom interrupt什么都没有——等到定时器从0移动到0——否则就会得到一个幻象中断
  oldSREG = SREG;
  cli();
  tcnt1 = TCNT1;
  SREG = oldSREG;
  } while (tcnt1==0); 
 
//  TIFR1 = 0xff;                 // AR - Clear interrupt flags
//  TIMSK1 = _BV(TOIE1);              // sets the timer overflow interrupt enable bit
}

void TimerOne::stop()
{
  TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));          // 清除所有时钟选择位
}

unsigned long TimerOne::read()    //returns the value of the timer in microseconds  返回的计时器的知识微秒
{                 //rember! phase and freq correct mode counts up to then down again  牢记!相位和频率正确的模式从上到下计数
    unsigned long tmp;        // AR amended to hold more than 65536 (could be nearly double this)   AR修正后持有超过65536(可能是这个的两倍)
    unsigned int tcnt1;       // AR added  AR增加

  oldSREG= SREG;
    cli();              
    tmp=TCNT1;              
  SREG = oldSREG;

  char scale=0;
  switch (clockSelectBits)
  {
  case 1:// no prescalse
    scale=0;
    break;
  case 2:// x8 prescale
    scale=3;
    break;
  case 3:// x64
    scale=6;
    break;
  case 4:// x256
    scale=8;
    break;
  case 5:// x1024
    scale=10;
    break;
  }
  
  do {  // Nothing -- max delay here is ~1023 cycles.  AR modified  没有，这里的最大延迟是~1023个周期。基于“增大化现实”技术的修改
    oldSREG = SREG;
    cli();
    tcnt1 = TCNT1;
    SREG = oldSREG;
  } while (tcnt1==tmp); //if the timer has not ticked yet  如果计时器还没有计时

  //if we are counting down add the top value to how far we have counted down如果我们在倒数，就把上面的值加到倒数的地方
  tmp = (  (tcnt1>tmp) ? (tmp) : (long)(ICR1-tcnt1)+(long)ICR1  );    // AR amended to add casts and reuse previous TCNT1   AR经过修正以添加转换和重用以前的TCNT1
  return ((tmp*1000L)/(F_CPU /1000L))<<scale;
}

#endif
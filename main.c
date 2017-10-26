#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include <stdio.h>
#include <math.h>
#include <inttypes.h>
#include "accelerometers/accelerometers.h"
#include "stm32f4xx_usart.h"
#include "misc.h"

#define BT_BAUD 38400
#define MAX_STRLEN 8

static void init_systick();
static volatile uint32_t msTicks; // counts 1 ms timeTicks
static __IO uint32_t TimingDelay;
//void UARTSend(const unisgned chat * pucBuffer, unsigned long ulCount);
 volatile char received_string[MAX_STRLEN+1];
 volatile uint16_t incoming;
int nw = 0;
float pitch_acc;
float roll_acc;
int step = 0;

//Button Press:
static volatile uint32_t ispressed=0;
uint32_t rval=0;
int req_bit=0;
int s_flag=0;


struct Gestures{
  float gesture_array[3];
  uint32_t gesture_symbol;
};

struct Gestures gesture[3]={
  {0,1,0,116}, //should be 108
  {0,1,0,114},
  {0,1,0,115}
};



char HC_05_RX_buffer[5];
uint8_t HC_counter=0;
char test_pitch = 0;

void SysTick_Handler(void)
{
  msTicks++;

      req_bit= (GPIOA->IDR & 1);
  if(req_bit==1 && ispressed==0)
  {
    while(rval<250)
    {
      req_bit= (GPIOA->IDR & 1);
      if(req_bit==1 && ispressed==0)
      {
        rval=rval+1;
      }
      else
      {
        rval=0;
        break;
      
      }
    if(rval==250)
      { 
      printf("\nButton Pressed");
      ispressed=1;
      rval=0;
      }
    }
  }
}

void init_systick(void)
{
  SystemCoreClockUpdate();                      /* Get Core Clock Frequency   */
  if (SysTick_Config(SystemCoreClock / 1000)) { /* SysTick 1 msec interrupts  */
    while (1);                                  /* Capture error              */
  }
}


struct AccelData{
  float s_roll; //The roll
  float s_pitch; //The pitch
};

struct AccelData accel_data_window[5] = {
  {0,0},
  {0,0},
  {0,0},
  {0,0},
  {0,0}
}; // This is an array of structure. We use an array of 5 to average



struct TimedTask{
  void (*function) (void);
  uint32_t time_interval;  //This will give the time interval after which the function is called
  uint32_t last_call;  //This will give the last call of the function
};

struct TimedTask timed_task[5]={
  {0,0,0},
  {0,0,0},
  {0,0,0},
  {0,0,0},
  {0,0,0}
}; // This is an array of structure. We use an array of 5

void add_timed_task(void (*fptr)(void), float a) //Not sure if I can simply use any name for function here
{
  int i = 0;
  for(i=0;i<=4;i++)
    {
      if(timed_task[i].function == fptr)    //If the address of the struct is the same as the address of the function passed
  {
    timed_task[i].time_interval = (uint32_t)(a*1000);  //Then for that sector of struct we change the time interval, we typecast here
  }
    }

}

void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll)
{
  *roll = (180.0/M_PI)*atan2(acc_y, acc_z);
  *pitch = (180.0/M_PI)*atan2(-acc_x, sqrt(acc_y*acc_y+acc_z*acc_z));
}

//Assume you are able to call functions to calculate roll and pitch

void rot_detect()
{
        float a[3]; // array of 3 floats into which accelerometer data will be read
      read_accelerometers(a); // read data from accelerometers (X, Y, and Z axes)
            float pitch, roll;
      calc_pitch_roll(a[0], a[1], a[2], &pitch, &roll); 
      accel_data_window[nw].s_roll = roll;
      accel_data_window[nw].s_pitch = pitch;

}



void calc_acceleration()
{int prev;
    if (nw == 5)
  {
    nw = 0;
  }
  prev = nw-1;
  if (nw == 0)
  {
    prev = 4;
  }
  rot_detect();
  pitch_acc = (accel_data_window[prev].s_pitch - accel_data_window[nw].s_pitch); //This is gonna be called every mili second
  roll_acc = (accel_data_window[prev].s_roll - accel_data_window[nw].s_roll);
  nw++;
}




void gesture_detect()
{
  char c;
  int f=0;
  for(f=0;f<3;f++)
  {
    if(gesture[f].gesture_array[0] == 1 && gesture[f].gesture_array[1] == 1)
    {
      c = (char)gesture[f].gesture_symbol;
      UU_PutString(USART2,&c);
    }
    gesture[f].gesture_array[0] = 0;
  }
}



void motion_detect()
{
  if(accel_data_window[nw].s_pitch  > 50)  //Considering about 45 degree movement in quarter of a second
  {
    gesture[0].gesture_array[0] = 1; 
  }
  if(accel_data_window[nw].s_pitch  < -50)
  {
    gesture[1].gesture_array[0] = 1;
    //printf("\na2\n");
  }

    if(accel_data_window[nw].s_roll < -40)
  {
    gesture[2].gesture_array[0] = 1;
  }


}



void init_USART2(uint32_t baudrate)
{

GPIO_InitTypeDef GPIO_InitStruct;
USART_InitTypeDef USART_InitStruct;
NVIC_InitTypeDef NVIC_InitStructure;

RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //This is USART 2 that will be used for receiving
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 


GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;  //2(TX) 3(RX) USART 2

GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
 GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;   // this defines the IO speed and has nothing to do with the baudrate!
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;     // this defines the output type as push pull mode (as opposed to open drain)
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;     // this activates the pullup resistors on the IO pins
  
  GPIO_Init(GPIOA, &GPIO_InitStruct); 

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2); //
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);


   USART_InitStruct.USART_BaudRate = baudrate;        // the baudrate is set to the value we passed into this init function
  USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
  USART_InitStruct.USART_StopBits = USART_StopBits_1;   // we want 1 stop bit (standard)
  USART_InitStruct.USART_Parity = USART_Parity_No;    // we don't want a parity bit (standard)
  USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
  USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
  USART_Init(USART2, &USART_InitStruct);
  USART_HalfDuplexCmd(USART2,DISABLE);


 USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // enable the USART2 receive interrupt
  
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;    // we want to configure the USART2 interrupts
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the USART2 interrupts
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;     // this sets the subpriority inside the group
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;      // the USART2 interrupts are globally enabled
  NVIC_Init(&NVIC_InitStructure);              // the properties are passed to the NVIC_Init function which takes care of the low level stuff
  
  // finally this enables the complete USART2 peripheral
  USART_Cmd(USART2, ENABLE);



}





void USART2_IRQHandler(void)
{
  if( USART_GetITStatus(USART2, USART_IT_RXNE) ){
    //USART_ClearITPendingBit(USART2, USART_IT_RXNE);
     static uint8_t cnt = 0;
    
    uint16_t t = USART_ReceiveData(USART2); // the character from the USART1 data register is saved in t
    incoming = t - 80;
    if(t)
      printf("\nhello\n");
    if((t != '\n') && (cnt < MAX_STRLEN) ){   //
      received_string[cnt++] = t;
      
    }
    else{ // otherwise reset the character counter and print the received string
      
      UU_PutString(USART2,received_string);
      printf("If condition false, or end of loop reached\n");
      received_string[cnt]='\0';
      cnt = 0;
    }
    USART_ClearITPendingBit(USART2, USART_IT_RXNE);
  }
}


  





/**
*@brief Method used to wait a certain amount of time
*@param nCount the time you want to wait
*/


void UU_PutChar(USART_TypeDef* USARTx, uint8_t ch)
{
  while(!(USARTx->SR & USART_SR_TXE)); 
  USARTx->DR = ch; 
}

void UU_PutString(USART_TypeDef* USARTx, uint8_t * str)
{
  while(*str != 0

    )
  {
    UU_PutChar(USARTx, *str);
    str++;
  }
}





void main(void) {
    SystemInit();
  initialise_monitor_handles();
  init_systick();
  init_accelerometers();
  char s[128];
  char t;
  init_USART2(BT_BAUD);


  timed_task[2].function = &gesture_detect;
  timed_task[1].function = &motion_detect;
  timed_task[0].function = &calc_acceleration;

  add_timed_task(&calc_acceleration,0.005);
  add_timed_task(&motion_detect,0.005);
  add_timed_task(&gesture_detect,0.01);

  while(1){
for(step =0; step < 3; step++)
{
      if(msTicks >= (timed_task[step].time_interval)+(timed_task[step].last_call))
  {
    timed_task[step].function();
    timed_task[step].last_call = msTicks;
  }


  if(ispressed==1)
    {
      req_bit= (GPIOA->IDR & 1);
      if(req_bit==0 && ispressed==1)
      {
        printf("\nButton Released");
        ispressed=0;
        s_flag=1;
        
      }
      if(s_flag==1)
      {
        char small_f[]="q";
        //write_serial_usb_bytes(small_f,strlen(small_f));
        UU_PutString(USART2,small_f);
        s_flag=0;
      }
    }
}
  }



}

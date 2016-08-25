#include "stm32f37x.h"
#include "stmlib/stmlib.h"

int main() {

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE, ENABLE);
   
  GPIO_InitTypeDef gpio_init;
  gpio_init.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
  gpio_init.GPIO_Mode = GPIO_Mode_OUT;
  gpio_init.GPIO_OType = GPIO_OType_PP;
  gpio_init.GPIO_Speed = GPIO_Speed_2MHz;
  gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOE, &gpio_init);
 
  NVIC_InitTypeDef nvicStructure;
  nvicStructure.NVIC_IRQChannel = TIM2_IRQn;
  nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
  nvicStructure.NVIC_IRQChannelSubPriority = 1;
  nvicStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvicStructure);
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = 200;
  TIM_TimeBaseStructure.TIM_Period = 10;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM2, ENABLE);

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC1, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  DAC_InitTypeDef DAC_InitStructure;
  DAC_StructInit(&DAC_InitStructure);
  DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
  DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
  DAC_Init(DAC1, DAC_Channel_1, &DAC_InitStructure);
  DAC_Cmd(DAC1, DAC_Channel_1, ENABLE);
  
  while(1) {
    for (int i=0; i<1000000; i++) { }
    GPIO_WriteBit(GPIOE, GPIO_Pin_8, static_cast<BitAction>(1));
    for (int i=0; i<1000000; i++) { }
       GPIO_WriteBit(GPIOE, GPIO_Pin_8, static_cast<BitAction>(0));
  }
}

bool led_state;
uint16_t sample;

extern "C" {

  void TIM2_IRQHandler() {
     if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
         TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
         sample += 100;
         DAC_SetChannel1Data(DAC1, DAC_Align_12b_L, sample);
    }
  }

}

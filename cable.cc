#include "stm32f30x.h"
#include "stmlib/stmlib.h"
#include "stmlib/dsp/dsp.h"

#include <cmath>

const int kSampleRate = 96000;

// x between -1 and +1
float FasterSine(float x) {
  return 4.0f * (x - x * fabsf(x));
}

// x between -1 and +1
float FastSine(float x) {
  float y = FasterSine(x);
  y = 0.225f * (y * fabsf(y) - y) + y;
  return y;
}

uint8_t led_state;
float phase = 0.0f;

void Init() {

  // GPIO: LEDs (PE8, PE9)
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE, ENABLE);
  GPIO_InitTypeDef gpio_init;
  gpio_init.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
  gpio_init.GPIO_Mode = GPIO_Mode_OUT;
  gpio_init.GPIO_OType = GPIO_OType_PP;
  gpio_init.GPIO_Speed = GPIO_Speed_10MHz;
  gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOE, &gpio_init);

  // GPIO: DAC output pin (PA4)
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  GPIO_StructInit(&gpio_init);
  gpio_init.GPIO_Pin =  GPIO_Pin_4;
  gpio_init.GPIO_Mode = GPIO_Mode_AN;
  GPIO_Init(GPIOA, &gpio_init);

  // GPIO: ADC input pin (PC1)
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
  GPIO_StructInit(&gpio_init);
  gpio_init.GPIO_Pin =  GPIO_Pin_1;
  gpio_init.GPIO_Mode = GPIO_Mode_AN;
  GPIO_Init(GPIOC, &gpio_init);

  // Interrupt configuration
  NVIC_InitTypeDef nvicStructure;
  nvicStructure.NVIC_IRQChannel = TIM2_IRQn;
  nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
  nvicStructure.NVIC_IRQChannelSubPriority = 1;
  nvicStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvicStructure);

  // Timer
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_Period = F_CPU / kSampleRate - 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM2, ENABLE);

  // DAC
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC1, ENABLE);
  DAC_InitTypeDef DAC_InitStructure;
  DAC_StructInit(&DAC_InitStructure);
  DAC_InitStructure.DAC_Buffer_Switch = DAC_BufferSwitch_Enable;
  DAC_InitStructure.DAC_Trigger = DAC_Trigger_T2_TRGO;
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
  DAC_Init(DAC1, DAC_Channel_1, &DAC_InitStructure);
  DAC_Cmd(DAC1, DAC_Channel_1, ENABLE);

  // ADC
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE);
  RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div2);

  ADC_DeInit(ADC1);
  ADC_InitTypeDef adc_init;
  ADC_StructInit(&adc_init);

  ADC_CommonInitTypeDef adc_common;
  adc_common.ADC_Mode = ADC_Mode_Independent;
  adc_common.ADC_Clock = ADC_Clock_AsynClkMode;                    
  adc_common.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;             
  adc_common.ADC_DMAMode = ADC_DMAMode_OneShot;                  
  adc_common.ADC_TwoSamplingDelay = 0;
  ADC_CommonInit(ADC1, &adc_common);

  adc_init.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Enable;
  adc_init.ADC_Resolution = ADC_Resolution_12b;
  adc_init.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;
  adc_init.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
  adc_init.ADC_DataAlign = ADC_DataAlign_Left;
  adc_init.ADC_OverrunMode = ADC_OverrunMode_Disable;
  adc_init.ADC_AutoInjMode = ADC_AutoInjec_Disable;
  adc_init.ADC_NbrOfRegChannel = 1;
  ADC_Init(ADC1, &adc_init);

  ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 1, ADC_SampleTime_601Cycles5);
  ADC_Cmd(ADC1, ENABLE);
}

int main() {

  // Peripheral initialization
  Init();

  // Main loop
  while(1) {

    GPIO_WriteBit(GPIOE, GPIO_Pin_8, static_cast<BitAction>(led_state < 50));
    led_state++;

    ADC_StartConversion(ADC1);
    while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
  }
}

extern "C" {

  void TIM2_IRQHandler() {
     if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
         TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

         uint16_t adc = ADC_GetConversionValue(ADC1);
         float cv = static_cast<float>(adc) / 65535.0f;
         // cv += 0.1f;

         float phase_increment = 440.0f * cv * 4.0f / kSampleRate;

         phase += phase_increment;
         if (phase > 1.0f) phase--;

         float s = FastSine(phase * 2.0f - 1.0f);
         s = (s + 1.0f) * 0.5f;

         uint16_t sample = static_cast<uint16_t>(s * 60000) + 2000;
         DAC_SetChannel1Data(DAC1, DAC_Align_12b_L, sample);
    }
  }
}

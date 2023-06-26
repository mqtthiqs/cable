#include "stm32f30x.h"
#include "stmlib/stmlib.h"
#include "stmlib/dsp/units.h"

#include <cmath>

const int kSampleRate = 32000;

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

/* extern */
const float stmlib::lut_pitch_ratio_high[] = {
   6.151958251e-04,  6.517772725e-04,  6.905339660e-04,  7.315952524e-04,
   7.750981699e-04,  8.211879055e-04,  8.700182794e-04,  9.217522585e-04,
   9.765625000e-04,  1.034631928e-03,  1.096154344e-03,  1.161335073e-03,
   1.230391650e-03,  1.303554545e-03,  1.381067932e-03,  1.463190505e-03,
   1.550196340e-03,  1.642375811e-03,  1.740036559e-03,  1.843504517e-03,
   1.953125000e-03,  2.069263856e-03,  2.192308688e-03,  2.322670146e-03,
   2.460783301e-03,  2.607109090e-03,  2.762135864e-03,  2.926381010e-03,
   3.100392680e-03,  3.284751622e-03,  3.480073118e-03,  3.687009034e-03,
   3.906250000e-03,  4.138527712e-03,  4.384617376e-03,  4.645340293e-03,
   4.921566601e-03,  5.214218180e-03,  5.524271728e-03,  5.852762019e-03,
   6.200785359e-03,  6.569503244e-03,  6.960146235e-03,  7.374018068e-03,
   7.812500000e-03,  8.277055425e-03,  8.769234752e-03,  9.290680586e-03,
   9.843133202e-03,  1.042843636e-02,  1.104854346e-02,  1.170552404e-02,
   1.240157072e-02,  1.313900649e-02,  1.392029247e-02,  1.474803614e-02,
   1.562500000e-02,  1.655411085e-02,  1.753846950e-02,  1.858136117e-02,
   1.968626640e-02,  2.085687272e-02,  2.209708691e-02,  2.341104808e-02,
   2.480314144e-02,  2.627801298e-02,  2.784058494e-02,  2.949607227e-02,
   3.125000000e-02,  3.310822170e-02,  3.507693901e-02,  3.716272234e-02,
   3.937253281e-02,  4.171374544e-02,  4.419417382e-02,  4.682209615e-02,
   4.960628287e-02,  5.255602595e-02,  5.568116988e-02,  5.899214454e-02,
   6.250000000e-02,  6.621644340e-02,  7.015387802e-02,  7.432544469e-02,
   7.874506562e-02,  8.342749089e-02,  8.838834765e-02,  9.364419230e-02,
   9.921256575e-02,  1.051120519e-01,  1.113623398e-01,  1.179842891e-01,
   1.250000000e-01,  1.324328868e-01,  1.403077560e-01,  1.486508894e-01,
   1.574901312e-01,  1.668549818e-01,  1.767766953e-01,  1.872883846e-01,
   1.984251315e-01,  2.102241038e-01,  2.227246795e-01,  2.359685782e-01,
   2.500000000e-01,  2.648657736e-01,  2.806155121e-01,  2.973017788e-01,
   3.149802625e-01,  3.337099635e-01,  3.535533906e-01,  3.745767692e-01,
   3.968502630e-01,  4.204482076e-01,  4.454493591e-01,  4.719371563e-01,
   5.000000000e-01,  5.297315472e-01,  5.612310242e-01,  5.946035575e-01,
   6.299605249e-01,  6.674199271e-01,  7.071067812e-01,  7.491535384e-01,
   7.937005260e-01,  8.408964153e-01,  8.908987181e-01,  9.438743127e-01,
   1.000000000e+00,  1.059463094e+00,  1.122462048e+00,  1.189207115e+00,
   1.259921050e+00,  1.334839854e+00,  1.414213562e+00,  1.498307077e+00,
   1.587401052e+00,  1.681792831e+00,  1.781797436e+00,  1.887748625e+00,
   2.000000000e+00,  2.118926189e+00,  2.244924097e+00,  2.378414230e+00,
   2.519842100e+00,  2.669679708e+00,  2.828427125e+00,  2.996614154e+00,
   3.174802104e+00,  3.363585661e+00,  3.563594873e+00,  3.775497251e+00,
   4.000000000e+00,  4.237852377e+00,  4.489848193e+00,  4.756828460e+00,
   5.039684200e+00,  5.339359417e+00,  5.656854249e+00,  5.993228308e+00,
   6.349604208e+00,  6.727171322e+00,  7.127189745e+00,  7.550994501e+00,
   8.000000000e+00,  8.475704755e+00,  8.979696386e+00,  9.513656920e+00,
   1.007936840e+01,  1.067871883e+01,  1.131370850e+01,  1.198645662e+01,
   1.269920842e+01,  1.345434264e+01,  1.425437949e+01,  1.510198900e+01,
   1.600000000e+01,  1.695140951e+01,  1.795939277e+01,  1.902731384e+01,
   2.015873680e+01,  2.135743767e+01,  2.262741700e+01,  2.397291323e+01,
   2.539841683e+01,  2.690868529e+01,  2.850875898e+01,  3.020397801e+01,
   3.200000000e+01,  3.390281902e+01,  3.591878555e+01,  3.805462768e+01,
   4.031747360e+01,  4.271487533e+01,  4.525483400e+01,  4.794582646e+01,
   5.079683366e+01,  5.381737058e+01,  5.701751796e+01,  6.040795601e+01,
   6.400000000e+01,  6.780563804e+01,  7.183757109e+01,  7.610925536e+01,
   8.063494719e+01,  8.542975067e+01,  9.050966799e+01,  9.589165292e+01,
   1.015936673e+02,  1.076347412e+02,  1.140350359e+02,  1.208159120e+02,
   1.280000000e+02,  1.356112761e+02,  1.436751422e+02,  1.522185107e+02,
   1.612698944e+02,  1.708595013e+02,  1.810193360e+02,  1.917833058e+02,
   2.031873347e+02,  2.152694823e+02,  2.280700718e+02,  2.416318240e+02,
   2.560000000e+02,  2.712225522e+02,  2.873502844e+02,  3.044370214e+02,
   3.225397888e+02,  3.417190027e+02,  3.620386720e+02,  3.835666117e+02,
   4.063746693e+02,  4.305389646e+02,  4.561401437e+02,  4.832636481e+02,
   5.120000000e+02,  5.424451043e+02,  5.747005687e+02,  6.088740429e+02,
   6.450795775e+02,  6.834380053e+02,  7.240773439e+02,  7.671332234e+02,
   8.127493386e+02,  8.610779292e+02,  9.122802874e+02,  9.665272962e+02,
   1.024000000e+03,  1.084890209e+03,  1.149401137e+03,  1.217748086e+03,
   1.290159155e+03,  1.366876011e+03,  1.448154688e+03,  1.534266447e+03,
};

/* extern */
const float stmlib::lut_pitch_ratio_low[] = {
   1.000000000e+00,  1.000225659e+00,  1.000451370e+00,  1.000677131e+00,
   1.000902943e+00,  1.001128806e+00,  1.001354720e+00,  1.001580685e+00,
   1.001806701e+00,  1.002032768e+00,  1.002258886e+00,  1.002485055e+00,
   1.002711275e+00,  1.002937546e+00,  1.003163868e+00,  1.003390242e+00,
   1.003616666e+00,  1.003843141e+00,  1.004069668e+00,  1.004296246e+00,
   1.004522874e+00,  1.004749554e+00,  1.004976285e+00,  1.005203068e+00,
   1.005429901e+00,  1.005656786e+00,  1.005883722e+00,  1.006110709e+00,
   1.006337747e+00,  1.006564836e+00,  1.006791977e+00,  1.007019169e+00,
   1.007246412e+00,  1.007473707e+00,  1.007701053e+00,  1.007928450e+00,
   1.008155898e+00,  1.008383398e+00,  1.008610949e+00,  1.008838551e+00,
   1.009066205e+00,  1.009293910e+00,  1.009521667e+00,  1.009749475e+00,
   1.009977334e+00,  1.010205245e+00,  1.010433207e+00,  1.010661221e+00,
   1.010889286e+00,  1.011117403e+00,  1.011345571e+00,  1.011573790e+00,
   1.011802061e+00,  1.012030384e+00,  1.012258758e+00,  1.012487183e+00,
   1.012715661e+00,  1.012944189e+00,  1.013172770e+00,  1.013401401e+00,
   1.013630085e+00,  1.013858820e+00,  1.014087607e+00,  1.014316445e+00,
   1.014545335e+00,  1.014774277e+00,  1.015003270e+00,  1.015232315e+00,
   1.015461411e+00,  1.015690560e+00,  1.015919760e+00,  1.016149011e+00,
   1.016378315e+00,  1.016607670e+00,  1.016837077e+00,  1.017066536e+00,
   1.017296046e+00,  1.017525609e+00,  1.017755223e+00,  1.017984889e+00,
   1.018214607e+00,  1.018444376e+00,  1.018674198e+00,  1.018904071e+00,
   1.019133996e+00,  1.019363973e+00,  1.019594002e+00,  1.019824083e+00,
   1.020054216e+00,  1.020284401e+00,  1.020514637e+00,  1.020744926e+00,
   1.020975266e+00,  1.021205659e+00,  1.021436104e+00,  1.021666600e+00,
   1.021897149e+00,  1.022127749e+00,  1.022358402e+00,  1.022589107e+00,
   1.022819863e+00,  1.023050672e+00,  1.023281533e+00,  1.023512446e+00,
   1.023743411e+00,  1.023974428e+00,  1.024205498e+00,  1.024436619e+00,
   1.024667793e+00,  1.024899019e+00,  1.025130297e+00,  1.025361627e+00,
   1.025593009e+00,  1.025824444e+00,  1.026055931e+00,  1.026287470e+00,
   1.026519061e+00,  1.026750705e+00,  1.026982401e+00,  1.027214149e+00,
   1.027445949e+00,  1.027677802e+00,  1.027909707e+00,  1.028141664e+00,
   1.028373674e+00,  1.028605736e+00,  1.028837851e+00,  1.029070017e+00,
   1.029302237e+00,  1.029534508e+00,  1.029766832e+00,  1.029999209e+00,
   1.030231638e+00,  1.030464119e+00,  1.030696653e+00,  1.030929239e+00,
   1.031161878e+00,  1.031394569e+00,  1.031627313e+00,  1.031860109e+00,
   1.032092958e+00,  1.032325859e+00,  1.032558813e+00,  1.032791820e+00,
   1.033024879e+00,  1.033257991e+00,  1.033491155e+00,  1.033724372e+00,
   1.033957641e+00,  1.034190964e+00,  1.034424338e+00,  1.034657766e+00,
   1.034891246e+00,  1.035124779e+00,  1.035358364e+00,  1.035592003e+00,
   1.035825694e+00,  1.036059437e+00,  1.036293234e+00,  1.036527083e+00,
   1.036760985e+00,  1.036994940e+00,  1.037228947e+00,  1.037463008e+00,
   1.037697121e+00,  1.037931287e+00,  1.038165506e+00,  1.038399777e+00,
   1.038634102e+00,  1.038868479e+00,  1.039102910e+00,  1.039337393e+00,
   1.039571929e+00,  1.039806518e+00,  1.040041160e+00,  1.040275855e+00,
   1.040510603e+00,  1.040745404e+00,  1.040980258e+00,  1.041215165e+00,
   1.041450125e+00,  1.041685138e+00,  1.041920204e+00,  1.042155323e+00,
   1.042390495e+00,  1.042625720e+00,  1.042860998e+00,  1.043096329e+00,
   1.043331714e+00,  1.043567151e+00,  1.043802642e+00,  1.044038185e+00,
   1.044273782e+00,  1.044509433e+00,  1.044745136e+00,  1.044980892e+00,
   1.045216702e+00,  1.045452565e+00,  1.045688481e+00,  1.045924450e+00,
   1.046160473e+00,  1.046396549e+00,  1.046632678e+00,  1.046868860e+00,
   1.047105096e+00,  1.047341385e+00,  1.047577727e+00,  1.047814123e+00,
   1.048050572e+00,  1.048287074e+00,  1.048523630e+00,  1.048760239e+00,
   1.048996902e+00,  1.049233618e+00,  1.049470387e+00,  1.049707210e+00,
   1.049944086e+00,  1.050181015e+00,  1.050417999e+00,  1.050655035e+00,
   1.050892125e+00,  1.051129269e+00,  1.051366466e+00,  1.051603717e+00,
   1.051841021e+00,  1.052078378e+00,  1.052315790e+00,  1.052553255e+00,
   1.052790773e+00,  1.053028345e+00,  1.053265971e+00,  1.053503650e+00,
   1.053741383e+00,  1.053979169e+00,  1.054217010e+00,  1.054454903e+00,
   1.054692851e+00,  1.054930852e+00,  1.055168907e+00,  1.055407016e+00,
   1.055645178e+00,  1.055883395e+00,  1.056121664e+00,  1.056359988e+00,
   1.056598366e+00,  1.056836797e+00,  1.057075282e+00,  1.057313821e+00,
   1.057552413e+00,  1.057791060e+00,  1.058029760e+00,  1.058268515e+00,
   1.058507323e+00,  1.058746185e+00,  1.058985101e+00,  1.059224071e+00,
};

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

  // GPIO: ADC input pin (PA0 & PA5)
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  GPIO_StructInit(&gpio_init);
  gpio_init.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_5;
  gpio_init.GPIO_Mode = GPIO_Mode_AN;
  GPIO_Init(GPIOA, &gpio_init);

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
  // RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC34, ENABLE);
  RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div2);
  // RCC_ADCCLKConfig(RCC_ADC34PLLCLK_Div2);

  ADC_InitTypeDef adc_init;
  ADC_StructInit(&adc_init);

  ADC_CommonInitTypeDef adc_common;
  adc_common.ADC_Mode = ADC_Mode_Independent;
  adc_common.ADC_Clock = ADC_Clock_AsynClkMode;                    
  adc_common.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;             
  adc_common.ADC_DMAMode = ADC_DMAMode_OneShot;                  
  adc_common.ADC_TwoSamplingDelay = 0;
  ADC_CommonInit(ADC1, &adc_common);

  ADC_CommonInit(ADC2, &adc_common);

  adc_init.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Disable;
  adc_init.ADC_Resolution = ADC_Resolution_12b;
  adc_init.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;
  adc_init.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
  adc_init.ADC_DataAlign = ADC_DataAlign_Left;
  adc_init.ADC_OverrunMode = ADC_OverrunMode_Disable;
  adc_init.ADC_AutoInjMode = ADC_AutoInjec_Disable;
  adc_init.ADC_NbrOfRegChannel = 1;
  ADC_Init(ADC1, &adc_init);

  adc_init.ADC_NbrOfRegChannel = 1;
  ADC_Init(ADC2, &adc_init);

  ADC_SelectCalibrationMode(ADC1, ADC_CalibrationMode_Single);
  ADC_StartCalibration(ADC1);
  while(ADC_GetCalibrationStatus(ADC1) != RESET);
  // ADC_SelectCalibrationMode(ADC2, ADC_CalibrationMode_Single);
  // ADC_StartCalibration(ADC2);
  // while(ADC_GetCalibrationStatus(ADC2) != RESET);

  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_2Cycles5);

  ADC_RegularChannelConfig(ADC2, ADC_Channel_2, 1, ADC_SampleTime_2Cycles5);
  // ADC_RegularChannelConfig(ADC2, ADC_Channel_2, 2, ADC_SampleTime_2Cycles5);

  ADC_Cmd(ADC1, ENABLE);
  ADC_Cmd(ADC2, ENABLE);
}

int main() {

  // Peripheral initialization
  Init();

  // Main loop
  while(1) {
    GPIO_WriteBit(GPIOE, GPIO_Pin_8, static_cast<BitAction>(led_state < 50));
    led_state++;
  }
}

extern "C" {

  void TIM2_IRQHandler() {
     if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
         TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

         float cv = static_cast<float>(ADC_GetConversionValue(ADC1)) / 65535.0f;
         cv = (cv * 2.0f) - 1.0f;

         float pot = static_cast<float>(ADC_GetConversionValue(ADC2)) / 65535.0f;

         float note = pot * 12.0f * 6.0f - 24.0f + cv * 12.0f * 1.5f;
         float freq = stmlib::SemitonesToRatio(note) * 440.0f;

         float phase_increment = freq / kSampleRate;

         phase += phase_increment;
         if (phase > 1.0f) phase--;

         float osc = FastSine(phase * 2.0f - 1.0f);

         // mix input and sine
         float sample = osc;

         // clip
         if (sample < -1.0f) sample = -1.0f;
         if (sample > 1.0f) sample = 1.0f;

         // scale and write to dac
         float s = (sample + 1.0f) * 0.5f;
         DAC_SetChannel1Data(DAC1, DAC_Align_12b_L, static_cast<uint16_t>(s * 65535.0f));

         // start conversion
         ADC_StartConversion(ADC1);
         ADC_StartConversion(ADC2);
    }
  }
}

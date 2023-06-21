#include "stm32f0xx.h"

uint16_t dr_ADC[2];
int i = 0;

void ADC1_IRQHandler()
{
	ADC1->ISR |= ADC_ISR_EOC; //сброс флага прерывания ацп

	if(i == 2)
		i = 0;
	dr_ADC[i] = ADC1->DR;
	i++;

}
void TIM3_IRQHandler()
{
	TIM3->SR &= ~TIM_SR_UIF;
	TIM3->CCR3 = dr_ADC[0];
	TIM3->PSC = dr_ADC[1];

	GPIOC->ODR ^= GPIO_ODR_9;
}
void tim15_TRG0()
{
	RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
	TIM15->ARR = 4000;    //20 Hz
	TIM15->PSC = 100;
	TIM15->CR2 |= TIM_CR2_MMS_1;	// update tim15_trgO
	TIM15->CR1 |= TIM_CR1_CEN;
}

void init_ADC()
{
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA->MODER |= GPIO_MODER_MODER2;   // pa2 on AF
	GPIOA->MODER |= GPIO_MODER_MODER6;  // pa6 on AF

	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;  // ADC clock selection

	ADC1->CFGR1 |= ADC_CFGR1_CONT; // ADC continuous conversion mode
	ADC1->CFGR1 |= ADC_CFGR1_OVRMOD; //Регистр ADC_DR перезаписывается последним результатом преобразования при обнаружении переполнения.
	ADC1->SMPR  |= 0x07;	//239.5 adc cycles
	ADC1->CHSELR = 0x0044; //01000100     2 and 6 channels on

	//TIM15 TRGO
	ADC1->CFGR1 &= ~ADC_CFGR1_CONT;
	ADC1->CFGR1 |= ADC_CFGR1_DISCEN;
	ADC1->CFGR1 |= ADC_CFGR1_EXTEN_0; //Detection on rising edge
	ADC1->CFGR1 |= ADC_CFGR1_EXTSEL_2; // TIM15_TRGO

	ADC1->IER |= (0x1<<2); //ADC_IER_EOCIE // interrupt every time, when
	ADC1->CR |= ADC_CR_ADEN; // Write 1 to enable the ADC

	NVIC_EnableIRQ(ADC1_IRQn);
	NVIC_SetPriority(ADC1_IRQn, 1);

	while((ADC1->ISR & ADC_ISR_ADRDY) != ADC_ISR_ADRDY);
	ADC1->CR |= ADC_CR_ADSTART;
}

void init_tim3_pwm_ch3()
{
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	GPIOC->MODER |= GPIO_MODER_MODER8_1; //AF


	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	TIM3->ARR = 1000;
	TIM3->PSC = 100;

	TIM3->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;	//pwm on
	TIM3->CCER |= TIM_CCER_CC3E;

	TIM3->DIER |= TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM3_IRQn);
	NVIC_SetPriority(TIM3_IRQn, 2);

	TIM3->CR1 |= TIM_CR1_CEN;
}





int main(void)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	GPIOC->MODER |= GPIO_MODER_MODER9_0; //LED

	tim15_TRG0();
	init_ADC();
	init_tim3_pwm_ch3();

  while (1)
  {

  }
}

/*
 * lib.cpp
 *
 *  Created on: 19. 12. 2016
 *      Author: Patrik Bakyta
 */

#include "stm32l1xx.h"
#include <stm32l1xx_gpio.h>
#include <stm32l1xx_tim.h>
#include <stm32l1xx_rcc.h>
#include <stm32l1xx_usart.h>
#include <stdlib.h>
#include <misc.h>
#include <lib.h>

uint8_t ADC_Values[2] = {128, 128}; // hodnoty z ADC v defaultnej polohe

int *int_pointer; // pointer na pole INT
char *char_pointer; // pointer na pole CHAR

int default_values[6] = {128,128,128,128,128,128}; // x_min, x_stred, x_max, y...

int q = 0; // pocitadlo strednej hodnoty

extern bool map; // premenna zapnuteho/vypnuteho mapovania

void initSYSTEMCLOCK(void) {

	RCC_HSICmd(ENABLE);
	while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);

	RCC_SYSCLKConfig(RCC_CFGR_SW_HSI);
	SystemCoreClockUpdate();

	//uint32_t SystemClockValue = SystemCoreClock;

	return;

}

void initTIMERwithINTERRUPT(uint16_t TimerVal) {

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_TimeBaseInitTypeDef timerInitStructure;
	timerInitStructure.TIM_Prescaler = 16000-1;
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStructure.TIM_Period = TimerVal-1;
	timerInitStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseInit(TIM2, &timerInitStructure);
	TIM_Cmd(TIM2, ENABLE);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); // povolenie update eventu

    NVIC_InitTypeDef nvicStructure;
    nvicStructure.NVIC_IRQChannel = TIM2_IRQn;
    nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
    nvicStructure.NVIC_IRQChannelSubPriority = 1;
    nvicStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicStructure);

    return;
}

extern "C" void TIM2_IRQHandler(void) {

	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {

		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

		GPIO_ToggleBits(GPIOA,GPIO_Pin_5);

		// premapovanie hodnot
		if (map==true) {
			int_pointer = remap(ADC_Values);
		}

		// najprv sa posle x potom y
		for (int j=0; j<2; j++) {

			// konverzia hodnoty z ADC na pole charov, funkcia vracia smernik
			if (map==true) {
				char_pointer = INTconversionCHAR(*(int_pointer+j));
			} else {
				char_pointer = INTconversionCHAR(*(ADC_Values+j));
			}

			int i = *(char_pointer); // na 1. mieste je pocet cifier
			while (i>0) {
				while (USART_GetFlagStatus(USART2,USART_FLAG_TXE)==0) {};
				USART_SendData(USART2,*(char_pointer+i));
				i--;
			}

			// najprv space a pootm new line
			while (USART_GetFlagStatus(USART2,USART_FLAG_TXE)==0) {};
			if (j==0) {
				USART_SendData(USART2,32);
			} else {
				USART_SendData(USART2,13);
			}

		}

		/*
		// priame posielanie hodnot
		while (USART_GetFlagStatus(USART1,USART_FLAG_TXE)==0) {};
		USART_SendData(USART1,ADC_Values[0]);
		while (USART_GetFlagStatus(USART1,USART_FLAG_TXE)==0) {};
		USART_SendData(USART1,ADC_Values[1]);
		*/

	}

	return;
}

int *remap(uint8_t ADC_Values[2]) {

	// funkcia vracia smernik na nove pole

	static int int_array[2];
	int rozsah;

	if (q<10) {
		q++;
	}

	for (int j=0; j<2; j++) {

		// nastavenie defaultnych strednych hodnot
		if (q==10) {
			default_values[j*3+1] = ADC_Values[j];
			if (j==1) {
				q = 11;
			}
		}

		// nastavenie novych minimalnych/maximalnych hodnot
		if (ADC_Values[j]<default_values[j*3+0]) {
			default_values[j*3+0] = ADC_Values[j];
		} else if (ADC_Values[j]>default_values[j*3+2]) {
			default_values[j*3+2] = ADC_Values[j];
		}

		// premapovanie
		if (ADC_Values[j]>default_values[j*3+1]) {
			rozsah = default_values[j*3+2]-default_values[j*3+1]; // max-stred
		} else if (ADC_Values[j]<default_values[j*3+1]) {
			rozsah = default_values[j*3+1]-default_values[j*3+0]; // stred-min
		}

		int_array[j] = 128+(ADC_Values[j]-default_values[j*3+1])*(128/(double)rozsah);

		// pre istotu obmedzenie
		if (int_array[j]>255) {
			int_array[j] = 255;
		} else if (int_array[j]<0) {
			int_array[j] = 0;
		}

	}

	return int_array;
}

char *INTconversionCHAR(uint8_t value) {

	// funkcia vracia smernik na pole

	int j = 1; // index pola, zacina na 1
	static char char_array[4]; // pole, na 1. mieste pocet cifier hodnoty z ADC (1-3)

	do {
		*(char_array+j) = (char)(value % 10) + '0'; // konverzia z INT na CHAR
		value /= 10;
		j++;
	} while (value);

	*(char_array) = j-1; // teraz uz vieme pocet cifier, zapis na 1. miesto v poli

	return char_array;
}
void initUSART(void) {

	 GPIO_InitTypeDef GPIO_InitStruct;    // this is for the GPIO pins used as TX and RX
	 USART_InitTypeDef USART_InitStruct;  // this is for the USART1 initilization
	 NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)

	 RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	 GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; // Pins 2 (TX) and 3 (RX) are used
	 GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 			 // the pins are configured as alternate function so the USART peripheral has access to them
	 GPIO_InitStruct.GPIO_Speed = GPIO_Speed_40MHz;		 // this defines the IO speed and has nothing to do with the baudrate!
	 GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;		 // this defines the output type as push pull mode (as opposed to open drain)
	 GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;			 // this activates the pullup resistors on the IO pins
	 GPIO_Init(GPIOA, &GPIO_InitStruct);				 // now all the values are passed to the GPIO_Init() function which sets the GPIO registers

	 GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	 GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	 USART_InitStruct.USART_BaudRate = 9600;				 // the baudrate is set to the value we passed into this init function
	 USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	 USART_InitStruct.USART_StopBits = USART_StopBits_1;	 // we want 1 stop bit (standard)
	 USART_InitStruct.USART_Parity = USART_Parity_No;		 // we don't want a parity bit (standard)
	 USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	 USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	 USART_Init(USART2, &USART_InitStruct);					 // again all the properties are passed to the USART_Init function which takes care of all the bit setting

	 USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt

	 NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;		  // we want to configure the USART1 interrupts
	 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8; // this sets the priority group of the USART1 interrupts
	 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  // this sets the subpriority inside the group
	 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // the USART1 interrupts are globally enabled
	 NVIC_Init(&NVIC_InitStructure);							  // the properties are passed to the NVIC_Init function which takes care of the low level stuff

	 USART_Cmd(USART2, ENABLE);

	 return;
}

void initADCwithDMA(void) {

    ADC_InitTypeDef       ADC_InitStruct;
    ADC_CommonInitTypeDef ADC_CommonInitStruct;
    DMA_InitTypeDef       DMA_InitStruct;
    GPIO_InitTypeDef      GPIO_InitStruct;

    // Enable DMA2 clocks
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    // Enable GPIOA clocks
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    // Enable ADC1 clocks
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    // DMA1 Stream0 channel0 configuration
    DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR; // ADC1's data register
    DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)&ADC_Values;
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStruct.DMA_BufferSize = 2;
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // Reads 8 bit values
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; // Stores 8 bit values
    DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStruct.DMA_Priority = DMA_Priority_High;
    DMA_Init(DMA1_Channel1, &DMA_InitStruct);
    DMA_Cmd(DMA1_Channel1, ENABLE);

    // Configure GPIO pins
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; // PA0 - Channel 0, PA1 - Channel 1
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN; // The pins are configured in analog mode
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; // We don't need any pull up or pull down
    GPIO_Init(GPIOC, &GPIO_InitStruct); // Initialize GPIOA pins with the configuration

    // ADC Common Init
    ADC_CommonInitStruct.ADC_Prescaler = ADC_Prescaler_Div2;
    ADC_CommonInit(&ADC_CommonInitStruct);

    // ADC1 Init
    ADC_DeInit(ADC1);
    ADC_InitStruct.ADC_Resolution = ADC_Resolution_8b; // Input voltage is converted into a 8bit int (max 255)
    ADC_InitStruct.ADC_ScanConvMode = ENABLE; // The scan is configured in multiple channels
    ADC_InitStruct.ADC_ContinuousConvMode = ENABLE; // Continuous conversion: input signal is sampled more than once
    ADC_InitStruct.ADC_ExternalTrigConv = 0;
    ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right; // Data converted will be shifted to right
    ADC_InitStruct.ADC_NbrOfConversion = 2;
    ADC_Init(ADC1, &ADC_InitStruct); // Initialize ADC with the configuration

    // Select the channels to be read from
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_96Cycles); // PA0
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_96Cycles); // PA1

    // Enable DMA request after last transfer (Single-ADC mode)
    ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
    // Enable ADC1 DMA
    ADC_DMACmd(ADC1, ENABLE);
    // Enable ADC1
    ADC_Cmd(ADC1, ENABLE);
    // Start ADC1 Software Conversion
    ADC_SoftwareStartConv(ADC1);

    return;
}

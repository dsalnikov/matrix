/*
 * exchange.c
 *
 *  Created on: 25 èþë. 2016 ã.
 *      Author: Dev
 */

#include "exchange.h"
#include "stm32f0xx_spi.h"
#include "stm32f0xx_usart.h"
#include "stm32f0xx_dma.h"

#include <stdio.h>

uint8_t gamma[18*2];


static void init_pins() {
	GPIO_InitTypeDef gpio;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	/* init mosi pin */
	gpio.GPIO_Pin = MOSI_PIN | DCK_PIN;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	gpio.GPIO_Speed = GPIO_Speed_Level_3;

	GPIO_Init(GPIOB, &gpio);
	GPIO_PinAFConfig(GPIOB, MOSI_PIN_SOURCE, GPIO_AF_0);
	GPIO_PinAFConfig(GPIOB, DCK_PIN_SOURCE, GPIO_AF_0);

	/* init cs, en pin */
	gpio.GPIO_Pin = LAT_B_PIN | SELBK_PIN | RSTB_PIN;
	gpio.GPIO_Mode = GPIO_Mode_OUT;

	GPIO_Init(GPIOB, &gpio);

	/* init row select ping */
	gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |
			GPIO_Pin_7 | GPIO_Pin_6 | GPIO_Pin_5 | GPIO_Pin_4;
	gpio.GPIO_Mode = GPIO_Mode_OUT;

	GPIO_Init(GPIOA, &gpio);

	/* init uart pins */
	gpio.GPIO_Pin = RX_PIN | TX_PIN;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOA, &gpio);

	GPIO_PinAFConfig(GPIOC, RX_PIN_SOURCE, GPIO_AF_1);
	GPIO_PinAFConfig(GPIOC, TX_PIN_SOURCE, GPIO_AF_1);

	/* init pwm timer pin */
	gpio.GPIO_Pin = PWM_PIN;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &gpio);

	GPIO_PinAFConfig(GPIOB, PWM_PIN_SOURCE, GPIO_AF_2);

}

static void init_spi() {
	SPI_InitTypeDef spi;
	DMA_InitTypeDef dma;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	spi.SPI_CPOL = SPI_CPOL_Low;
	spi.SPI_CPHA = SPI_CPHA_1Edge;
	spi.SPI_DataSize = SPI_DataSize_8b;
	spi.SPI_Direction = SPI_Direction_1Line_Tx;
	spi.SPI_FirstBit = SPI_FirstBit_MSB;
	spi.SPI_Mode = SPI_Mode_Master;
	spi.SPI_NSS = SPI_NSS_Soft;
	spi.SPI_CRCPolynomial = 1;

	SPI_Init(SPI1, &spi);
	SPI_Cmd(SPI1, ENABLE);

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	dma.DMA_BufferSize = 3*8;
	dma.DMA_DIR = DMA_DIR_PeripheralDST;
	dma.DMA_MemoryBaseAddr = (uint32_t)screen;
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma.DMA_Mode = DMA_Mode_Normal;
	dma.DMA_PeripheralBaseAddr = (uint32_t)&SPI1->DR;
	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma.DMA_Priority = DMA_Priority_High;
	dma.DMA_M2M = DMA_M2M_Disable;

	DMA_Init(DMA_CHANNEL, &dma);

	NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

	DMA_ITConfig(DMA_CHANNEL, DMA_IT_TC, ENABLE);

	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);
}

static void init_uart(uint32_t baud) {
	USART_InitTypeDef uart;

	RCC_APB1PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	uart.USART_BaudRate = baud;
	uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	uart.USART_Parity = USART_Parity_No;
	uart.USART_StopBits = USART_StopBits_1;
	uart.USART_WordLength = USART_WordLength_8b;

	USART_Init(USART1, &uart);
	USART_Cmd(USART1, ENABLE);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART1, USART_IT_RTO, ENABLE);

	NVIC_InitTypeDef nvic;
	nvic.NVIC_IRQChannel = USART1_IRQn;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	nvic.NVIC_IRQChannelPriority = 0;

	NVIC_Init(&nvic);
}

static void init_timer(uint32_t freq) {
	uint32_t period = 48000000U / freq;

	assert_param(period >= 2);

	TIM_TimeBaseInitTypeDef tim;
	TIM_OCInitTypeDef tim_oc;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	tim.TIM_ClockDivision = TIM_CKD_DIV1;
	tim.TIM_CounterMode = TIM_CounterMode_Up;
	tim.TIM_Period = period-1;
	tim.TIM_Prescaler = 0;

	TIM_TimeBaseInit(TIM1, &tim);

	TIM_OCStructInit(&tim_oc);

	tim_oc.TIM_OCMode = TIM_OCMode_PWM1;
	tim_oc.TIM_OutputState = TIM_OutputState_Enable;
	tim_oc.TIM_OutputNState = TIM_OutputNState_Enable;
	tim_oc.TIM_OCPolarity = TIM_OCPolarity_High;
	tim_oc.TIM_Pulse = period >> 1;

	TIM_OC2Init(TIM1, &tim_oc);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM1,ENABLE);

	TIM_CtrlPWMOutputs(TIM1, ENABLE);

	TIM_Cmd(TIM1, ENABLE);
}

static void init_crc() {
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);

	CRC_PolynomialSizeSelect(CRC_PolSize_16);
	CRC->INIT = 0;
	CRC->POL = CRC_POLYNOMIAL;
}

static uint8_t usart_buffer[512];
static uint16_t usart_buffer_p = 0;

uint16_t crc16(uint8_t *data, uint16_t len) {
	CRC->CR |= CRC_CR_RESET;

	while (len--) {
	    CRC->DR = *data++;
	}

	return CRC->DR;
}

void handle_uart(uint8_t *data, uint16_t len) {
	if (crc16(data, len))
		return;

	switch (data[0]) {
		case 0:

		break;

		default:
			fprintf(stderr, "Unhandled USART command.\n");
	}
}

void USART1_IRQHandler() {
	if (USART_GetITStatus(USART1, USART_IT_RXNE)) {
		usart_buffer[usart_buffer_p++] = USART1->RDR;

		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}

	if (USART_GetITStatus(USART1, USART_IT_RTO)) {
		handle_uart(usart_buffer, usart_buffer_p);
		usart_buffer_p = 0;
		USART_ClearITPendingBit(USART1, USART_IT_RTO);
	}
}

void DMA1_Channel2_3_IRQHandler() {
	static uint8_t line_num = 0;

	if (DMA_GetITStatus(DMA1_IT_GL3)) {

		printf("dma isr: %x\n", DMA1->ISR);

		reset_all_lines();

		LAT_B_HIGH;
		LAT_B_LOW;


		set_active_line(line_num++);

		SELBK_HIGH;

		DMA_ClearFlag(DMA1_FLAG_TC3);
		DMA_ClearITPendingBit(DMA1_IT_TC3);

		//printf("dma isr: %x\n", DMA1->ISR);
		//request next line update
		if (line_num == 7)
			line_num = 0;

		send_line(line_num);


	}
}

void reset_all_lines() {
	//disable all lines
	GPIO_ResetBits(GPIOA, 0x00FF);
}

void set_active_line(uint8_t n) {
	uint16_t pin_nums[] = {GPIO_Pin_0, GPIO_Pin_1, GPIO_Pin_2, GPIO_Pin_3, GPIO_Pin_7, GPIO_Pin_6, GPIO_Pin_5, GPIO_Pin_4};

	GPIO_SetBits(GPIOA, pin_nums[n]);
}

void exchange_init() {
	fill_screen(128, 1);
	memset(gamma, 0, 18*2);

	printf("Initialising pins ...\n");
	init_pins();

	printf("Initialising SPI ...\n");
	init_spi();

	//printf("Initialising USART ...\n");
	//init_uart(115200);

	printf("Initialising PWM timer ...\n");
	init_timer(12000000);

	//printf("Initialising CRC unit ...\n");
	//init_crc();

	printf("Setting pins to default state ...\n");

	RSTB_HIGH;
	SELBK_LOW;
	LAT_B_HIGH;
}

void update_screen() {
	SELBK_HIGH;
	send_line(0);
}

void spi_send(uint16_t data) {
	SPI_SendData8(SPI1, data);
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)==RESET);
}



void send_line(uint8_t n) {
#if 1


	for (uint16_t i=0; i<24*2; i++) {
		spi_send(screen[i]);
	}
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY)==SET);
	LAT_B_HIGH;
	for(uint8_t i=0;i<255;i++);
	LAT_B_LOW;

	SELBK_HIGH;

#else
	DMA_InitTypeDef dma;

	printf("drawing line %d\n", n);

	DMA_ClearFlag(DMA1_FLAG_GL3);

	DMA_DeInit(DMA_CHANNEL);

	dma.DMA_BufferSize = 8*3;
	dma.DMA_DIR = DMA_DIR_PeripheralDST;
	dma.DMA_MemoryBaseAddr = (uint32_t)&screen[n*8*3];
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma.DMA_Mode = DMA_Mode_Normal;
	dma.DMA_PeripheralBaseAddr = (uint32_t)&SPI1->DR;
	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma.DMA_Priority = DMA_Priority_High;
	dma.DMA_M2M = DMA_M2M_Disable;

	DMA_Init(DMA_CHANNEL, &dma);

	NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

	DMA_ITConfig(DMA_CHANNEL, DMA_IT_TC, ENABLE);

	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);

	DMA_Cmd(DMA_CHANNEL, ENABLE);
#endif
}

void fill_screen(uint8_t c, uint8_t stribe){
	for (uint16_t i=0; i<SCREEN_SIZE; i+=stribe) {
		screen[i] = c;
	}
}

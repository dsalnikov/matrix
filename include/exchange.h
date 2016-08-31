#ifndef __EXCHANGE_H__
#define __EXCHANGE_H__

#include "stm32f0xx.h"


#define DCK_PIN GPIO_Pin_3
#define DCK_PIN_SOURCE GPIO_PinSource3

#define MOSI_PIN GPIO_Pin_5
#define MOSI_PIN_SOURCE GPIO_PinSource5

#define DMA_CHANNEL DMA1_Channel3

#define LAT_B_PIN GPIO_Pin_4

#define LAT_B_HIGH GPIO_SetBits(GPIOB, LAT_B_PIN)
#define LAT_B_LOW 	GPIO_ResetBits(GPIOB, LAT_B_PIN)

#define SELBK_PIN GPIO_Pin_6

#define SELBK_HIGH GPIO_SetBits(GPIOB, SELBK_PIN)
#define SELBK_LOW 	GPIO_ResetBits(GPIOB, SELBK_PIN)

#define RSTB_PIN GPIO_Pin_7

#define RSTB_HIGH 	GPIO_SetBits(GPIOB, RSTB_PIN)
#define RSTB_LOW 	GPIO_ResetBits(GPIOB, RSTB_PIN)

#define RX_PIN GPIO_Pin_10
#define TX_PIN GPIO_Pin_9
#define RX_PIN_SOURCE GPIO_PinSource10
#define TX_PIN_SOURCE GPIO_PinSource9

#define PWM_PIN GPIO_Pin_0
#define PWM_PIN_SOURCE GPIO_PinSource0


//crc16 0x8005 / 0xA001 / 0xC002
#define CRC_POLYNOMIAL 0x8005


#define SCREEN_SIZE 8*16*3


uint16_t crc16(uint8_t *data, uint16_t len);

void exchange_init();
void send_line(uint8_t n);
void update_screen();
void fill_screen(uint8_t c);

static volatile uint8_t screen[SCREEN_SIZE];
void handle_uart(uint8_t *data, uint16_t len);

void reset_all_lines();
void set_active_line(uint8_t n);

#endif // __EXCHANGE_H__

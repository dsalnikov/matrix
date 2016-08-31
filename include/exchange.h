#ifndef __EXCHANGE_H__
#define __EXCHANGE_H__

#include "stm32f0xx.h"



#define EXCHANGE_MOSI_PIN GPIO_Pin_0
#define EXCHANGE_MOSI_PIN_SOURCE GPIO_PinSource0

#define EXCHANGE_DMA_CHANNEL DMA1_Channel3

#define EXCHANGE_LAT_B_PIN GPIO_Pin_0

#define EXCHANGE_LAT_B_HIGH GPIO_SetBits(GPIOC, EXCHANGE_LAT_B_PIN)
#define EXCHANGE_LAT_B_LOW 	GPIO_ResetBits(GPIOC, EXCHANGE_LAT_B_PIN)

#define EXCHANGE_EN_PIN GPIO_Pin_0

#define EXCHANGE_EN_HIGH 	GPIO_SetBits(GPIOC, EXCHANGE_EN_PIN)
#define EXCHANGE_EN_LOW 	GPIO_ResetBits(GPIOC, EXCHANGE_EN_PIN)

#define EXCHANGE_SELBK_PIN GPIO_Pin_0

#define EXCHANGE_SELBK_HIGH GPIO_SetBits(GPIOC, EXCHANGE_SELBK_PIN)
#define EXCHANGE_SELBK_LOW 	GPIO_ResetBits(GPIOC, EXCHANGE_SELBK_PIN)

#define EXCHANGE_RSTB_PIN GPIO_Pin_0

#define EXCHANGE_RSTB_HIGH 	GPIO_SetBits(GPIOC, EXCHANGE_RSTB_PIN)
#define EXCHANGE_RSTB_LOW 	GPIO_ResetBits(GPIOC, EXCHANGE_RSTB_PIN)

#define EXCHANGE_RX_PIN GPIO_Pin_0
#define EXCHANGE_TX_PIN GPIO_Pin_0
#define EXCHANGE_RX_PIN_SOURCE GPIO_PinSource0
#define EXCHANGE_TX_PIN_SOURCE GPIO_PinSource0

#define EXCHANGE_PWM_PIN GPIO_Pin_0
#define EXCHANGE_PWM_PIN_SOURCE GPIO_PinSource0


//crc16 0x8005 / 0xA001 / 0xC002
#define EXCHANGE_CRC_POLYNOMIAL 0x8005


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

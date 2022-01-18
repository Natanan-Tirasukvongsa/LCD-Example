/*
 * ST7735.h
 *
 *  Created on: Aug 21, 2021
 *      Author: AlphaP
 */

#ifndef INC_ST7735_H_
#define INC_ST7735_H_
#include "stm32h7xx_hal.h"
//ประกาศตัวแปรต่าง ๆ

//เก็บข้อมูล port pin ต่าง ๆ
typedef struct
{
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *RstPort,*CSPort,*DCPort;
	uint32_t  RstPin;
	uint32_t  CSPin;
	uint32_t  DCPin;

}LCDHandle;
//ขนาดจอ 128 * 128*RGB
//สามารถส่งข้อมูลให้จอได้ทันที
//ข้อเสียคือ ถ้าไม่มีการแก้ไขภาพในจอ ภาพซ้ำ ๆ เดิม ๆ จะถูกเขียนซ้ำไปซ้ำมา
//ทำให้เปลือง bandwid ของจอ
#define LCD_BUFFER_SIZE 3*128*128
extern uint8_t Framememory[LCD_BUFFER_SIZE];
extern uint8_t owl[LCD_BUFFER_SIZE];
extern uint8_t man[LCD_BUFFER_SIZE];
extern uint8_t dog[LCD_BUFFER_SIZE];
extern uint8_t N1[3072];
extern uint8_t fan[LCD_BUFFER_SIZE];
extern uint8_t fan_off[6075];
extern uint8_t fan_on[6075];
extern uint8_t speed1[3072];
extern uint8_t speed2[3072];
extern uint8_t speed3[3072];
extern uint8_t n0[4800];
extern uint8_t n15[4800];
extern uint8_t n30[4800];
extern uint8_t n45[4800];
extern uint8_t n60[4800];

void LCD_init(LCDHandle *lcd);
void LCD_flush(LCDHandle *lcd);
uint8_t* LCDBufferAddr();

void LCD_show(uint8_t RTC_ON, uint8_t fan_mode);
void LCD_timer(uint8_t timer);
void LCD_fan(uint8_t RTC_ON);
void LCD_speed(uint8_t speed);

#endif /* INC_ST7735_H_ */

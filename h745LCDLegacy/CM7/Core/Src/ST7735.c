/*
 * ST7735.C
 *
 *  Created on: Aug 21, 2021
 *      Author: AlphaP
 */

#include "ST7735.h"

//อธิบายการทำงานของโปรแกรม
uint8_t Framememory[LCD_BUFFER_SIZE]={0};
uint8_t LCDSTARTUPSeq[]= //send commandand parameter
{
		//reset การทำงานของจอทุกครั้งที่เริ่มการทำงาน
		0x01,						//SW (software) Reset (command)
		//เปิดการทำงานของจอ
		0x11,						//Sleep Out (command)
		//แสดงผลการทำงาน
		0x29,						//display on (command)
		//memory data access control เป็นการเขียนจอในรูปแบบไหน
		//parameter 0b MY = 0, MX = 1, MV = 0, ML = 0, RGB = 1, MH = 0, D1 = 0, D0 = 0
		0x36, 0b01001000,			//seting Scan Order (command, parameter)
		//คำสั่งบอกระยะการเขียนภาพใน memory  x-axis
		//จุดเริ่มต้นและจุดสิ้นสุดในการเขียนจอภาพ (128 pixles)
		0x2a,0x00,0x00,0x00,127,	//Set C Area (command, parameter)
		//คำสั่งบอกระยะการเขียนภาพใน memory y-axis
		//จุดเริ่มต้นและจุดสิ้นสุดในการเขียนจอภาพ (128 pixles)
		0x2b,0x00,0x00,0x00,127,	//Set R Area (command, parameter)
		//เข้าสู่โหมดเตรียมพร้อมในการเขียนข้อมูล
		0x2c						//Write Memory
};

//การสั่งขา D/C หรือ A0 เพื่อส่ง command หรือ parameter
//เปลี่ยนแปลง data command ให้เป็น high เพื่อส่ง parameter
//เปลี่ยนแปลง data command ให้เป็น low เพื่อส่ง command
//ส่งข้อมูลภาพไปในจอใช้ขา data command high
void LCD_init(LCDHandle *lcd){
	//reset LCD
	HAL_GPIO_WritePin(lcd->RstPort, lcd->RstPin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(lcd->RstPort, lcd->RstPin, GPIO_PIN_SET);

	//select SPI CS
	HAL_GPIO_WritePin(lcd->CSPort, lcd->CSPin, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(lcd->CSPort, lcd->CSPin, GPIO_PIN_RESET);

	//write reset , sleep out, display on, scan order
	//write command 0x01, 0x11, 0x29, 0x36
	//4 = 4 commands
	HAL_GPIO_WritePin(lcd->DCPort, lcd->DCPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(lcd->hspi, LCDSTARTUPSeq, 4, 100);

	//write parameter of scan order
	//write parameter 0b01001000
	//&LCDSTARTUPSeq[4] 0ffset ไป 4 ตำแหน่ง
	//1 = 1 parameter
	HAL_GPIO_WritePin(lcd->DCPort, lcd->DCPin, GPIO_PIN_SET);
	HAL_SPI_Transmit(lcd->hspi, &LCDSTARTUPSeq[4], 1, 100);

	//write set C area
	//write command 0x2a
	HAL_GPIO_WritePin(lcd->DCPort, lcd->DCPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(lcd->hspi, &LCDSTARTUPSeq[5], 1, 100);

	//write set C area parameter
	//write parameter 0x00,0x00,0x00,127
	HAL_GPIO_WritePin(lcd->DCPort, lcd->DCPin, GPIO_PIN_SET);
	HAL_SPI_Transmit(lcd->hspi, &LCDSTARTUPSeq[6], 4, 100);

	//write set R area
	//write command 0x2b
	HAL_GPIO_WritePin(lcd->DCPort, lcd->DCPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(lcd->hspi, &LCDSTARTUPSeq[10], 1, 100);

	//write set R area parameter
	//write parameter 0x00,0x00,0x00,127
	HAL_GPIO_WritePin(lcd->DCPort, lcd->DCPin, GPIO_PIN_SET);
	HAL_SPI_Transmit(lcd->hspi, &LCDSTARTUPSeq[11], 4, 100);

	//write to graphic memory
	HAL_GPIO_WritePin(lcd->DCPort, lcd->DCPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(lcd->hspi, &LCDSTARTUPSeq[15], 1, 100);

	//set DC to high to read & send image data
	HAL_GPIO_WritePin(lcd->DCPort, lcd->DCPin, GPIO_PIN_SET);

}

//ส่งข้อมูลของ frame ไปให้จอ
void LCD_flush(LCDHandle *lcd)
{
	//circular dma
	//Framememory มีขนาดเท่ากับหน้าจอ
	HAL_SPI_Transmit_DMA(lcd->hspi, Framememory, LCD_BUFFER_SIZE);
}

uint8_t* LCDBufferAddr()
{
	return Framememory;
}

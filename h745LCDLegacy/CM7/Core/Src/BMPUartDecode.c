/*
 * PNGUartDecode.c
 *
 *  Created on: Aug 21, 2021
 *      Author: AlphaP
 */

#include "BMPUartDecode.h"
typedef enum {
	BMP_idle,
	BMP_Header_2,
	BMP_Size_4,
	BMP_Reserved0_4,
	BMP_Imagestartpoint_4,
	BMP_SizeHeader_4,
	BMP_PicWidth_4,
	BMP_PicHeight_4,
	BMP_ColorPlanes_2,
	BMP_BitPerPixel_2,
	BMP_Notused1_n,
	BMP_Pixeldata_n,
	BMP_Notused2_n

} stateBMP;

//รูปแบบคำสั่ง
//สามารถ convert ข้อมูลที่มี byte แตกต่างกัน
//ต้องมั่นใจว่าอยู่บนระบบ endience เดียวกัน ในที่นี้เป็น little endience
typedef union U8ToU32Convert {
	uint8_t U8[4];
	uint32_t U32;
} Convert8_32;

#define IMG_W 128
#define IMG_H 128
static stateBMP State = 0;
void BMPDecoder(uint8_t dataIn, uint8_t *array) {

	//ข้อมูลที่เก็บขนาดของภาพ, จุดเริ่มต้น, ขนาด header, width per pixel, height per pixel
	static Convert8_32 size, StartPoint, HeaderSize, PW, PH, BPS;
	static uint32_t Substate, offset, imageSize;

	switch (State) {
	case BMP_idle:

		if (dataIn == 0x42) {
			State = BMP_Header_2;

		}
		Substate = 0;
		offset = 0;
		break;

	case BMP_Header_2:
		if (dataIn == 0x4D) {
			State = BMP_Size_4;
			Substate = 0;
		} else {
			State = BMP_idle;
		}
		break;
	case BMP_Size_4:
		//เก็บข้อมูล 4 byte
		size.U8[Substate] = dataIn;
		Substate++;
		if (Substate == 4) {
			State = BMP_Reserved0_4;
			Substate = 0;
		}
		break;

	//reserve เป็นส่วนที่ไม่ใส่ข้อมูล
	case BMP_Reserved0_4:
		Substate++;
		if (Substate == 4) {
			State = BMP_Imagestartpoint_4;
			Substate = 0;
		}
		break;

	case BMP_Imagestartpoint_4:
		StartPoint.U8[Substate] = dataIn;
		Substate++;
		if (Substate == 4) {
			State = BMP_SizeHeader_4;
			Substate = 0;
		}
		break;

	case BMP_SizeHeader_4:
		HeaderSize.U8[Substate] = dataIn;
		Substate++;
		if (Substate == 4) {
			//header ขนาด 40 byte
			if (HeaderSize.U32 == 40) {
				State = BMP_PicWidth_4;
				Substate = 0;
			} else {
				State = BMP_idle;
			}
		}
		break;
	case BMP_PicWidth_4:
		PW.U8[Substate] = dataIn;
		Substate++;
		if (Substate == 4) {
			State = BMP_PicHeight_4;
			Substate = 0;
		}
		break;
	case BMP_PicHeight_4:
		PH.U8[Substate] = dataIn;
		Substate++;
		if (Substate == 4) {
			State = BMP_ColorPlanes_2;
			Substate = 0;
		}
		break;
	case BMP_ColorPlanes_2:

		Substate++;
		if (Substate == 2) {
			State = BMP_BitPerPixel_2;
			Substate = 0;
		}
		break;
	case BMP_BitPerPixel_2:
		BPS.U8[Substate] = dataIn;
		Substate++;
		if (Substate == 4) {
			State = BMP_Notused1_n;
			Substate = 0;
		}
		break;
	case BMP_Notused1_n:
		//offset เป็นตัวนับว่าถึง bit ที่เท่าไหร่
		//จะอ่านจนถึง start point เพื่อเริ่มการอ่านภาพ
		if (offset == StartPoint.U32) {
			State = BMP_Pixeldata_n;
			array[0] = dataIn;
			Substate = 1;
		}
		break;

	//อ่านข้อมูลในส่วนของแต่ละ pixel
	case BMP_Pixeldata_n:
		//ในการเก็บ pixel ของภาพ BMP
		//ดูว่าข้อมูล alignment กับขนาด 32 bit หรือไม่ (หาร 32 ลงตัว)
		//ถ้าหาก 32 ไม่ลงตัว ข้อมูลถัดไปจะใส่ 0 จนกว่าจะครบ 32 bit
		// 1 pixel มี 3 สี สีละ 1 byte
		if ((Substate / (IMG_W * 3)) < PH.U32) {

			if ((Substate % (IMG_W * 3)) < (PW.U32 * 3)) {
				array[Substate++] = dataIn;
			}
			//mod 4 ไม่ลงตัว
			else if(((Substate) % 4))
			{
				//fill 0 ใส่ใน array ให้ครบ 32 bit
				array[Substate++] = dataIn;
			}
			else
			{
				//ถมดำในส่วนของที่เลยข้อมูลใน bmp file
				//ถ้าไม่ถมดำ ภาพเก่าที่มีขนาดใหญ่กว่าจะแสดงค้างขึ้นมา
				while ((Substate % (IMG_W * 3)) != 0) {
					array[Substate++] = 0; 	//fill blankdata with black
				}
				array[Substate++] = dataIn;
			}
		} else {
			while (Substate / (IMG_W * 3) < IMG_H) {
				array[Substate++] = 0; 	//fill blankdata with black
			}

			State = BMP_Notused2_n;

		}

		//U32 เป็นข้อมูลที่ได้จาก U8 เนื่องจากตอนที่เก็บข้อมูล U8 เรียงกัน 4 byte จะได้ข้อมูล U32 ไปในตัว
		//ได้รับ pixel ครบ
		if (offset >= size.U32-1) {
							State = BMP_idle;
							while (Substate / (IMG_W * 3) < IMG_H) {
										array[Substate++] = 0; 	//fill blankdata with black
									}
						}
		break;
	case BMP_Notused2_n:
		if (offset >= size.U32-1) {
			State = BMP_idle;

		}
		break;

	}
	offset++;

}

#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Key.h"
#include "MyCAN.h"

uint32_t RxID;
uint8_t RxLength;
uint8_t RxData[12];

int main(void)
{
	OLED_Init();
	Key_Init();
	MyCAN_Init();
	
	OLED_ShowString(1, 1, "RxID:");
	OLED_ShowString(2, 1, "Data:");
	
	while (1)
	{
		if (MyCAN_ReceiveFlag())
		{
			MyCAN_Receive(&RxID, &RxLength, RxData);
			
			Delay_ms(50);
			
			OLED_ShowString(1, 1, "RxID:");
			OLED_ShowString(2, 1, "Data:");
			OLED_ShowHexNum(1, 14, RxID, 3);
			OLED_ShowHexNum(2, 6, RxData[0], 2);
			OLED_ShowHexNum(2, 9, RxData[1], 2);
			OLED_ShowHexNum(2, 12, RxData[2], 2);
			OLED_ShowHexNum(2, 15, RxData[3], 2);
			OLED_ShowHexNum(3, 6, RxData[4], 2);
			OLED_ShowHexNum(3, 9, RxData[5], 2);
			OLED_ShowHexNum(3, 12, RxData[6], 2);
			OLED_ShowHexNum(3, 15, RxData[7], 2);
			OLED_ShowHexNum(4, 6, RxData[8], 2);
			OLED_ShowHexNum(4, 9, RxData[9], 2);
			OLED_ShowHexNum(4, 12, RxData[10], 2);
			OLED_ShowHexNum(4, 15, RxData[11], 2);
		}
	}
}

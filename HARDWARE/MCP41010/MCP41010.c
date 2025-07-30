#include "MCP41010.h"
#include "main.h"
#include "delay.h"
/*
#define CS_Pin CS_GPIO_Port
#define CS_GPIO_Port GPIOD
#define SCLK_Pin GPIO_PIN_3
#define SCLK_GPIO_Port GPIOD
#define MOSI_Pin GPIO_PIN_4
#define MOSI_GPIO_Port GPIOD
*/


#define   MCP41010_CS_L             HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET)
#define   MCP4101_CS_H              HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET)

#define   MCP4101_SCK_L             HAL_GPIO_WritePin(SCLK_GPIO_Port, SCLK_Pin, GPIO_PIN_RESET)
#define   MCP4101_SCK_H             HAL_GPIO_WritePin(SCLK_GPIO_Port, SCLK_Pin, GPIO_PIN_SET)

#define   MCP4101_SI_L              HAL_GPIO_WritePin(MOSI_GPIO_Port, MOSI_Pin, GPIO_PIN_RESET)
#define   MCP4101_SI_H              HAL_GPIO_WritePin(MOSI_GPIO_Port, MOSI_Pin, GPIO_PIN_SET)

#define SPI_Delay_us 2

// 保存上次写入的电位器值，初始化为一个不可能出现的值
static int16_t last_pot_value = -1;

void MCP41010_Init(void)
{
	MCP4101_CS_H;
}

/*参考 MCP41XXX/42XXX 系列数据手册：
对于 MCP42XXX（双通道）
对于 MCP41010（单通道），只有通道 0 存在：
* P1 is a ‘don’t care’ bit for the MCP41XXX.
P1 因无通道 1，始终为 0。	0xX100
*/
void MCP41010_SHUTDONW(void)
{
	 uint16_t MD = 0x2100;		// 0x2100

	    MCP41010_CS_L;
		delay_us(SPI_Delay_us);

	    for(uint8_t i = 0; i < 16; i++)
		{
		    MCP4101_SCK_L;
			if(MD & 0x8000)
				{MCP4101_SI_H;}
			else 
				{MCP4101_SI_L;}

			delay_us(SPI_Delay_us);

			MCP4101_SCK_H;
			MD <<= 1;

			delay_us(SPI_Delay_us);
		}

	    MCP4101_SCK_L;
	    delay_us(SPI_Delay_us);

	    MCP4101_CS_H ;

}

//SCK第一个边沿移入数据，第二个边沿移出数据
void MCP41010_Write(uint8_t data)
{
	// 如果和上次写入相同，则跳过 SPI 通信，节省时间
    if ((int16_t)data == last_pot_value) 
	{
        return;
    }
    // 数据帧：0x1 (写命令) + 0 + 8-bit 阻值
    uint16_t MD = 0x1100 | data;

    MCP41010_CS_L;
	delay_us(SPI_Delay_us);

	for(uint8_t i = 0; i < 16; i++)
	{// SPI模式1
	    MCP4101_SCK_L;
		if(MD & 0x8000)   // 上升沿 移位, 移出去的位放到引脚上面
			{MCP4101_SI_H;}
		else 
			{MCP4101_SI_L;}
		delay_us(SPI_Delay_us);

		MCP4101_SCK_H;
		MD <<= 1;		// 下降沿, 引脚上的位移入寄存器(采样)

		delay_us(SPI_Delay_us);
	}

    MCP4101_SCK_L;
    delay_us(SPI_Delay_us);

    MCP4101_CS_H ;
}

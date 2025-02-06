/*I3G4250D 3-axis Gyroscope interfacing with stm32f4111e-disco controller
 * I3G4250D.c
 *
 *  Created on: Feb 5, 2025
 *      Author: Lenovo
 */


#include "main.h"
#include "I3G4250D.h"

int16_t x_axis,y_axis,z_axis;
float X,Y,Z;
uint8_t fifo_buffer[6];
int8_t Temperature;

void spi1_cs_low()
{
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_RESET);
}

void spi1_cs_high()
{
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_SET);
}

void i3g4250d_writebyte(uint8_t reg,uint8_t data)
{
	uint8_t write_buffer[2] = {reg,data};

	spi1_cs_low();
	HAL_SPI_Transmit(I3G4250D,write_buffer,sizeof(write_buffer),1000);
	spi1_cs_high();
}

void i3g4250d_write_multibyte(uint8_t reg,uint8_t data,uint8_t length)
{
	spi1_cs_low();
	HAL_SPI_Transmit(I3G4250D,&reg,sizeof(reg),1000);
	HAL_SPI_Transmit(I3G4250D,&data,sizeof(length),1000);
	spi1_cs_high();
}

uint8_t i3g4250d_readbyte(uint8_t reg)
{
	uint8_t read_data = 0;
	uint8_t read_buffer = reg | 0x80;

	spi1_cs_low();
	HAL_SPI_Transmit(I3G4250D,&read_buffer,sizeof(read_buffer),1000);
	HAL_SPI_Receive(I3G4250D,&read_data,sizeof(read_data),1000);
	spi1_cs_high();

	return read_data;
}

void i3g4250d_read_multibyte(uint8_t reg,uint8_t *read_multidata,uint8_t length)
{
    uint8_t read_multibuffer = reg | 0xC0;

	spi1_cs_low();
	HAL_SPI_Transmit(I3G4250D,&read_multibuffer,sizeof(read_multibuffer),1000);
	HAL_SPI_Receive(I3G4250D,read_multidata,length,1000);
	spi1_cs_high();
}

void i3g4250d_powerdown_mode_enable()
{
	i3g4250d_writebyte(CTRL_REG1,0x00);
}

void i3g4250d_normal_mode_enable()
{
	i3g4250d_writebyte(CTRL_REG1,0x08);
}

void i3g4250d_init()
{
	spi1_cs_low();
	i3g4250d_writebyte(CTRL_REG1,0x0F);
	i3g4250d_writebyte(CTRL_REG4,0x30);
	i3g4250d_writebyte(CTRL_REG5,0x40);
	spi1_cs_high();
}

int8_t i3g4250d_read_temp()
{
	Temperature = i3g4250d_readbyte(OUT_TEMP);

	return ((int8_t)Temperature);
}

void i3g4250d_read_fifo()
{
	uint8_t fifo_read = OUT_X_L;

	i3g4250d_read_multibyte(fifo_read,fifo_buffer,6);

	x_axis = ((int16_t)(fifo_buffer[1] << 8) | fifo_buffer[0]);
	y_axis = ((int16_t)(fifo_buffer[3] << 8) | fifo_buffer[2]);
	z_axis = ((int16_t)(fifo_buffer[5] << 8) | fifo_buffer[4]);

	X = ((float)x_axis) / 70.0;
	Y = ((float)y_axis) / 70.0;
	Z = ((float)z_axis) / 70.0;
}

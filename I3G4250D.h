/*I3G4250D 3-axis Gyroscope interfacing with stm32f4111e-disco controller
 * I3G4250D.h
 *
 *  Created on: Feb 5, 2025
 *      Author: Govind Bhandari
 */

#ifndef INC_I3G4250D_H_
#define INC_I3G4250D_H_

#include "main.h"

extern SPI_HandleTypeDef       hspi1;
#define I3G4250D               &hspi1

#define WHO_AM_I               0x0F
#define CTRL_REG1              0x20
#define CTRL_REG2              0x21
#define CTRL_REG3              0x22
#define CTRL_REG4              0x23
#define CTRL_REG5              0x24
#define DATACAPTURE            0x25
#define OUT_TEMP               0x26
#define STATUS_REG             0x27
#define OUT_X_L                0x28
#define OUT_X_H                0x29
#define OUT_Y_L                0x2A
#define OUT_Y_H                0x2B
#define OUT_Z_L                0x2C
#define OUT_Z_H                0x2D
#define FIFO_CTRL_REG          0x2E
#define FIFO_SRC_REG           0x2F
#define INT1_CFG               0x30
#define INT1_SRC               0x31
#define INT1_THS_XH            0x32
#define INT1_THS_XL            0x33
#define INT1_THS_YH            0x34
#define INT1_THS_YL            0x35
#define INT1_THS_ZH            0x36
#define INT1_THS_ZL            0x37
#define INT1_DURATION          0x38


/*I3G4250D functions*/
void spi1_cs_low();
void spi1_cs_high();
void i3g4250d_writebyte(uint8_t reg,uint8_t data);
void i3g4250d_write_multibyte(uint8_t reg,uint8_t data,uint8_t length);
uint8_t i3g4250d_readbyte(uint8_t reg);
void i3g4250d_read_multibyte(uint8_t reg,uint8_t *read_multidata,uint8_t length);
void i3g4250d_powerdown_mode_enable();
void i3g4250d_normal_mode_enable();
void i3g4250d_init();
int8_t i3g4250d_read_temp();
void i3g4250d_read_fifo();


#endif /* INC_I3G4250D_H_ */

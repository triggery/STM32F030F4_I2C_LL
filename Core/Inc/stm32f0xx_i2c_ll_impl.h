/*
 * stm32f0xx_i2c_ll_impl.h
 *
 *  Created on: 16 янв. 2020 г.
 *      Author: kovalchuk
 */

#ifndef INC_STM32F0XX_I2C_LL_IMPL_H_
#define INC_STM32F0XX_I2C_LL_IMPL_H_

#include "stm32f0xx_ll_i2c.h"
#include "stm32f0xx_ll_cortex.h"
#include "stm32f0xx_ll_gpio.h"
#include "stdint.h"
#include "stdbool.h"

void LL_I2C_Master_Transmit(I2C_TypeDef *I2Cx, uint16_t DevAddress, uint8_t *pData, uint16_t Size, bool generateRequest);
void LL_I2C_Mem_Write_Bytes(I2C_TypeDef *I2Cx, uint32_t SlaveAddr, uint8_t MemAddress, uint8_t *pData, uint16_t XferCount);

#endif /* INC_STM32F0XX_I2C_LL_IMPL_H_ */

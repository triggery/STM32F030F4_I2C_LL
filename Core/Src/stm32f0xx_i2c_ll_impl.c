/*
 * stm32f0xx_i2c_ll_impl.c
 *
 *  Created on: 16 янв. 2020 г.
 *      Author: kovalchuk
 */
#include "stm32f0xx_i2c_ll_impl.h"

void LL_I2C_Master_Transmit(I2C_TypeDef *I2Cx, uint16_t DevAddress, uint8_t *pData, uint16_t Size, bool generateRequest) {
	uint8_t *pBuffPtr = pData;
	uint16_t allSize = Size;
	uint16_t packetSendSize;
	/* Set NBYTES to write and reload if hi2c->XferCount > MAX_NBYTE_SIZE */
	if ( allSize > 255U ) {
		packetSendSize = 255U;
		LL_I2C_HandleTransfer(I2Cx, DevAddress, LL_I2C_ADDRSLAVE_7BIT, packetSendSize, LL_I2C_MODE_RELOAD, generateRequest ? LL_I2C_GENERATE_START_WRITE : LL_I2C_GENERATE_NOSTARTSTOP);
	}
	else {
		packetSendSize = allSize;
		LL_I2C_HandleTransfer(I2Cx, DevAddress, LL_I2C_ADDRSLAVE_7BIT, packetSendSize, LL_I2C_MODE_AUTOEND, generateRequest ? LL_I2C_GENERATE_START_WRITE : LL_I2C_GENERATE_NOSTARTSTOP);
	}

	do {
		while(!(LL_I2C_IsActiveFlag_TXE(I2Cx)));
		LL_I2C_TransmitData8(I2Cx, *pBuffPtr);
		pBuffPtr++;
		allSize--;
		packetSendSize--;

	  if ((allSize != 0U) && (packetSendSize == 0U))
	  {
		while(!(LL_I2C_IsActiveFlag_TCR(I2Cx)));
		if ( allSize > 255U ) {
			packetSendSize = 255U;
			LL_I2C_HandleTransfer(I2Cx, DevAddress, LL_I2C_ADDRSLAVE_7BIT, packetSendSize, LL_I2C_MODE_RELOAD, LL_I2C_GENERATE_NOSTARTSTOP);
		}
		else {
			packetSendSize = allSize;
			LL_I2C_HandleTransfer(I2Cx, DevAddress, LL_I2C_ADDRSLAVE_7BIT, packetSendSize, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_NOSTARTSTOP);
		}
	  }
	} while (allSize > 0);
	/* No need to Check TC flag, with AUTOEND mode the stop is automatically generated */
	/* Wait until STOPF flag is reset */
	while(!(LL_I2C_IsActiveFlag_STOP(I2Cx)));
	/* Clear STOP Flag */
	LL_I2C_ClearFlag_STOP(I2Cx);
	/* Clear Configuration Register 2 */
}


void LL_I2C_Mem_Write_Bytes(I2C_TypeDef *I2Cx, uint32_t SlaveAddr, uint8_t MemAddress, uint8_t *pData, uint16_t XferCount) {
	/* Send Slave Address and Memory Address */
	LL_I2C_HandleTransfer(I2Cx, SlaveAddr, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_RELOAD, LL_I2C_GENERATE_START_WRITE);
	while(!(LL_I2C_IsActiveFlag_TXE(I2Cx)));
	LL_I2C_TransmitData8(I2Cx,  MemAddress);
	while(!(LL_I2C_IsActiveFlag_TCR(I2Cx)));

	LL_I2C_Master_Transmit(I2Cx, SlaveAddr, pData, XferCount, false);
}

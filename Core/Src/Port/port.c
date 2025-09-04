/*
 * port.c
 *
 *  Created on: Sep 1, 2025
 *      Author: sahin
 */

#include "helper.h"
#include "port.h"

uint32_t pupdr, afr, moder;

uint8_t TMS_SEQ_ARR[1024];
uint8_t TDI_SEQ_ARR[1024];
uint8_t TDO_SEQ_ARR[1024];
uint8_t TDO_PROCESSED_SEQ_ARR[1024];


void Switch_SPI(void)
{
	/* I won't get into details of this function, I know it looks messy.
	 * But what I do here is pinmuxing of the SPI
	 * and configuring SPI peripheral to conform with JTAG and SWD protocol
	 */
	pupdr = GPIOC->PUPDR;
	afr = GPIOC->AFR[1];
	moder = GPIOC->MODER;

	GPIOC->PUPDR = (0x1 << 16);
	GPIOC->AFR[1] = (0x6 << 8) | (0x6 << 12) | (0x6 << 16);

	GPIOC->MODER = (1 << 16) | (0x1 << 18) | (0x2 << 20) | (0x2 << 22 )| (0x2 << 24);

	TDISPI->CR1 = (0x1 << 8) | (0x1 << 9) |
			(1 << 2) | (0x7 << 3) | (0x1 << 7)
			| (1 << 6) | (0x3);

	  TMSPI->CR1 |= (1 << 15);
	  TMSPI->CR1 |= (1 << 14);


	    TMSPI->CR1 |= (1 << 7);
	    TMSPI->CR1 |= (1 << 9);
	    TMSPI->CR1 |= ( 0x3);
	    TMSPI->CR1 |= ( 1 << 6);

	    TDISPI->CR1 &= ~0x38;

	      TDISPI->CR1 |= (0x7 << 3);

}

void HardResetSPI(void)
{
	/* reset the spi to bring the HW to a known state*/
	  RCC->APB1RSTR |= (1 << 15);
	  RCC->APB2RSTR |= (1 << 13);

	  /* enable the SPI3,
	   * I do it manually because configurator does not permit me to
	   * use SPI3 due to pinmuxing. No need to do it for SPI4 because HAL
	   * does it automatically. */
	  RCC->APB1ENR |= (1 << 15);

	  /* release reset*/
	  RCC->APB1RSTR &= ~(1 << 15);
	  RCC->APB2RSTR &= ~(1 << 13);
}

void Switch_GPIO(void)
{
	/* This function brings the TDISPI (SPI3) pins to GPIO pins
	 * I never use it.*/
	GPIOC->PUPDR = pupdr;
	GPIOC->AFR[1] = afr;

	GPIOC->MODER = moder;

}

static inline void WaitForStart(void)
{
	/* wait until bsy bit is set */
	while( !(TDISPI->SR & (0x1 << 7) ) );
}

static inline void WaitForComplete(void)
{
	/* wait until bsy bit is cleared */
	while( (TDISPI->SR & (0x1 << 7) ) );
}

static inline void xFer(uint32_t *rdData, uint32_t wrData, uint8_t bitSize)
{

	/* at the start of the xFer, we do not expect to SPI to be busy,
	 * but I inserted this line to be defensive...
	 */
	WaitForComplete();


	/* change SPI data size of both TDI and TMS SPIs. */
	TDISPI->CR2 = ( (bitSize -1) << 8);
	TMSPI->CR2 = ( (bitSize -1) << 8);

	/* make sure they are sync
	 * TODO: implement a counter here to prevent
	 * lock-ups */
	while(TDISPI->CR2 != TMSPI->CR2 );


	if(bitSize <= 8)
	{
		*(uint8_t *)&TDISPI->DR = wrData;
	}
	else
	{
		TDISPI->DR = wrData;
	}

	/* wait for the xfer to complete */
	WaitForComplete();

	/* drain the read data registers */
	if(bitSize <= 8)
	{
		*rdData = *(uint8_t *)&TDISPI->DR;
	}
	else
	{
		*rdData = *(uint16_t *)&TDISPI->DR;
	}


}


void SPI_TMS_Transfer(uint64_t data, uint8_t bits)
{

	int b = 0;

	TMSPI->CR2 = ( (bits -1) << 8);



	/* if TMSSPI (SPI4) got stuck somehow,
	 * there is NO way to recover it.
	 * please refer to https://www.st.com/resource/en/errata_sheet/es0290-stm32f74xxx-and-stm32f75xxx-device-limitations-stmicroelectronics.pdf
	 * section 2.11.2
	 * It says that due to a silicon bug, slave SPI may stuck at BSY
	 * So, I reset the CPU instead of implementing a complex logic to reset SPI
	 * and bring it back to a known state.
	 * If I do so, SWDIO and SWCLK line may become unstable and target debug logic may crash.
	 */
	while( (TMSPI->SR & (0x1 << 7) ) )
	{
		b++;
		if(b > 5)
		{
			NVIC_SystemReset();
		}
	}

	uint32_t dummyRead = TMSPI->DR;

	/* clear read data buffer */
	if(bits <= 8)
	{
		*(uint8_t *)&TMSPI->DR = data;
	}
	else
	{
		*(uint16_t *)&TMSPI->DR = data;
	}


}

void SPI_TMSRead(uint64_t *ptr, uint8_t bits)
{
	if(bits <= 8)
	{
		*ptr = *(uint8_t *)&TMSPI->DR;
	}
	else
	{
		*ptr = *(uint16_t *)&TMSPI->DR;
	}

}

void SPI_SwitchPhaseToListen(void)
{
	/* DISABLE SPI and set PHASE to 0 and then REENABLE it
		 * so that we SAMPLE the SWDIO line at
		 * falling edges */

	TMSPI->CR1 &= ~(0x1 << 6);

	TDISPI->CR1 &= ~1;
	TMSPI->CR1 &= ~1;

	TMSPI->CR1 &= ~(1 << 14);
	TMSPI->CR1 |= (0x1 << 6);

}

void SPI_SwitchPhaseToWrite(void)
{
	/* DISABLE SPI and set PHASE to 1 and then REENABLE it
	 * so that we DRIVE the SWDIO line at
	 * falling edges */

	TMSPI->CR1 &= ~(0x1 << 6);

	TDISPI->CR1 |= 1uL;
	TMSPI->CR1 |= 1uL;

	TMSPI->CR1 |= (1uL << 14);
	TMSPI->CR1 |= (0x1 << 6);

}

int SPI_Transfer(uint64_t *rdData, uint64_t wrData, uint8_t bitSize)
{
	uint32_t tempReadVal;

	*rdData = 0;

	xFer(&tempReadVal, wrData, bitSize);

	*rdData = tempReadVal;

}

void ChangeTMS_Size(uint32_t bits)
{
	/* change SPI bits size, */
	SPI4->CR1 &= ~(0x1 << 6);


	SPI4->CR2 = ( (bits -1) << 8);

	SPI4->CR1 |= (0x1 << 6);

}

void JTAG_Reset_Target(void)
{
	HAL_GPIO_WritePin(JTAG_SRST_PORT, JTAG_SRST_PIN, GPIO_PIN_RESET);
	Delayms(DELAY_MS);
	HAL_GPIO_WritePin(JTAG_SRST_PORT, JTAG_SRST_PIN, GPIO_PIN_SET);

}

void JTAG_Reset(void)
{

	HAL_GPIO_WritePin(JTAG_TRST_PORT, JTAG_TRST_PIN, GPIO_PIN_RESET); // Assert TRST
    Delayms(DELAY_MS);
    HAL_GPIO_WritePin(JTAG_TRST_PORT, JTAG_TRST_PIN, GPIO_PIN_SET);   // De-assert TRST
    Delayms(DELAY_MS);

}

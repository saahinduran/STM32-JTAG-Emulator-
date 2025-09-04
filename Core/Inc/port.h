/*
 * port.h
 *
 *  Created on: Aug 31, 2025
 *      Author: sahin
 */

#ifndef INC_PORT_H_
#define INC_PORT_H_

#include "main.h"

#define MAX_SPI_PERIPHERAL_CLOCK 54e6

#define DELAY_MS 50


#define JTAG_TRST_PORT   GPIOC      // Optional
#define JTAG_TRST_PIN    GPIO_PIN_8 // Optional

#define JTAG_SRST_PORT	 GPIOD
#define JTAG_SRST_PIN    GPIO_PIN_2 // Optional

/* PORT: DEFINE YOUR SPI PERIPHERALS FOR DRIVING AND SAMPLING TMS,
 * DEFINE YOUR SPI PERIPHERAL FOR DRIVING TDI AND SAMPLING TDO
 */
#define TMSPI SPI4
#define TDISPI SPI3

void SPI_TMS_Transfer(uint64_t data, uint8_t bits);
void SPI_TMSRead(uint64_t *ptr, uint8_t bits);
void SPI_SwitchPhaseToListen(void);
void SPI_SwitchPhaseToWrite(void);
int SPI_Transfer(uint64_t *rdData, uint64_t wrData, uint8_t bitSize);
void ChangeTMS_Size(uint32_t bits);
void JTAG_Reset(void);
void JTAG_Reset_Target(void);



/* Forward declearion of Delayms from DAP.h */
extern void     Delayms         (uint32_t delay);


/* PORT: PERFORM PINMUXING IN THIS FUNCTION */
void Switch_SPI(void);

/* PORT: PERFORM HARD RESET of THE SPI PERIPHERAL IN THIS FUNCTION */
void HardResetSPI(void);

/* PORT: PERFORM PINMUXING FOR GPIO IN THIS FUNCTION (OPTIONAL, I NEVER USE IT)*/
void Switch_GPIO(void);

#endif /* INC_PORT_H_ */

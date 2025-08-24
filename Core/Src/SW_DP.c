/*
 * Copyright (c) 2013-2017 ARM Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * ----------------------------------------------------------------------
 *
 * $Date:        1. December 2017
 * $Revision:    V2.0.0
 *
 * Project:      CMSIS-DAP Source
 * Title:        SW_DP.c CMSIS-DAP SW DP I/O
 *
 *---------------------------------------------------------------------------*/

#include "DAP_config.h"
#include "DAP.h"

#define IDX_8_BIT 0
#define IDX_RM1_BIT 1
#define IDX_RM2_BIT 2


// SW Macros

#define PIN_SWCLK_SET PIN_SWCLK_TCK_SET
#define PIN_SWCLK_CLR PIN_SWCLK_TCK_CLR

#define SW_CLOCK_CYCLE()                \
  PIN_SWCLK_CLR();                      \
  PIN_DELAY();                          \
  PIN_SWCLK_SET();                      \
  PIN_DELAY()

#define SW_WRITE_BIT(bit)               \
  PIN_SWDIO_OUT(bit);                   \
  PIN_SWCLK_CLR();                      \
  PIN_DELAY();                          \
  PIN_SWCLK_SET();                      \
  PIN_DELAY()

#define SW_READ_BIT(bit)                \
  PIN_SWCLK_CLR();                      \
  PIN_DELAY();                          \
  bit = PIN_SWDIO_IN();                 \
  PIN_SWCLK_SET();                      \
  PIN_DELAY()

#define PIN_DELAY() PIN_DELAY_SLOW(DAP_Data.clock_delay)

void calculate_xfer_sizes(uint16_t input_len, uint8_t *buff);
void SPI_TMS_Transfer(uint64_t data, uint8_t bits);
int SPI_Transfer(uint64_t *rdData, uint64_t wrData, uint8_t bitSize);

// Generate SWJ Sequence
//   count:  sequence bit count
//   data:   pointer to sequence bit data
//   return: none
#if ((DAP_SWD != 0) || (DAP_JTAG != 0))
void SWJ_Sequence (uint32_t count, const uint8_t *data) {
  uint32_t val;
  uint32_t n;

  uint8_t xFerSizes[3];

  	uint8_t dummyVal = 0;
  	calculate_xfer_sizes(count, xFerSizes);

  	uint32_t currentBit = 0;
  	SPI4->CR1 &= ~(1 << 6);

  	SPI3->CR1 &= ~(1 << 6);

  	SPI4->CR1 &= ~(1);
  	SPI3->CR1 &= ~(1);


  	SPI4->CR1 |= (1 << 14);

  	SPI4->CR1 |= (1 << 6);
  	SPI3->CR1 |= (1 << 6);

  	uint32_t delay_cnt = 2500;

  	  		while(delay_cnt--)
  	  	    {
  	  		  __asm("nop");
  	  	    }

  	while(xFerSizes[IDX_8_BIT])
  	{
  		uint8_t tms_val = *data;
  		uint64_t tdo_val;

  		if(tms_val != 0 && currentBit != 0)
  		{
  			dummyVal = 1;
  		}

  		SPI_TMS_Transfer(tms_val, 8);
  		SPI_Transfer(&tdo_val, 0 , 8);

  		data++;
  		xFerSizes[IDX_8_BIT]--;
  		currentBit+= 8;
  	}

  	while(xFerSizes[IDX_RM1_BIT])
  	{
  		uint32_t delay_cnt = 2500;

  		while(delay_cnt--)
  	    {
  		  __asm("nop");
  	    }

  		uint16_t tms_val = extract_nbits_lsb(data, currentBit, xFerSizes[IDX_RM1_BIT]);
  		uint64_t tdo_val;

  		SPI_TMS_Transfer(tms_val, xFerSizes[IDX_RM1_BIT]);
  		SPI_Transfer(&tdo_val, 0 , xFerSizes[IDX_RM1_BIT]);


  		currentBit+= xFerSizes[IDX_RM1_BIT];

  		xFerSizes[IDX_RM1_BIT] = 0;

  	}

  	while(xFerSizes[IDX_RM2_BIT])
  	{
  		//TODO: extract bits!!
  		uint32_t delay_cnt = 2000;

  		while(delay_cnt--)
  		{
  		  __asm("nop");
  		}

  		uint16_t tms_val = extract_nbits_lsb(data, currentBit, xFerSizes[IDX_RM2_BIT]);
  		uint64_t tdo_val;

  		SPI_TMS_Transfer(tms_val, xFerSizes[IDX_RM2_BIT]);
  		SPI_Transfer(&tdo_val, 0 , xFerSizes[IDX_RM2_BIT]);


  		currentBit+= xFerSizes[IDX_RM2_BIT];

  		xFerSizes[IDX_RM2_BIT] = 0;

  	}

}
#endif


// Generate SWD Sequence
//   info:   sequence information
//   swdo:   pointer to SWDIO generated data
//   swdi:   pointer to SWDIO captured data
//   return: none
#if (DAP_SWD != 0)
void SWD_Sequence (uint32_t info, const uint8_t *swdo, uint8_t *swdi) {
  uint32_t val;
  uint32_t bit;
  uint32_t n, k;

  n = info & SWD_SEQUENCE_CLK;
  if (n == 0U) {
    n = 64U;
  }

  if (info & SWD_SEQUENCE_DIN) {
    while (n) {
      val = 0U;
      for (k = 8U; k && n; k--, n--) {
        SW_READ_BIT(bit);
        val >>= 1;
        val  |= bit << 7;
      }
      val >>= k;
      *swdi++ = (uint8_t)val;
    }
  } else {
    while (n) {
      val = *swdo++;
      for (k = 8U; k && n; k--, n--) {
        SW_WRITE_BIT(val);
        val >>= 1;
      }
    }
  }
}
#endif


#if (DAP_SWD != 0)

void write_nbits_lsb(uint8_t *buf, size_t bit_offset, size_t n, uint16_t value);

int SPI_Transfer(uint64_t *rdData, uint64_t wrData, uint8_t bitSize);

void SPI_TMS_Transfer(uint64_t data, uint8_t bits );

// Generate even parity bit for given data
uint8_t generate_even_parity(uint32_t data) {
    uint8_t count = 0;
    uint32_t temp = data;

    // Count number of 1s
    while (temp) {
        count ^= (temp & 1);
        temp >>= 1;
    }

    // For even parity: if count of 1s is odd, parity bit must be 1
    // if count is even, parity bit is 0
    return count; 
}

// Check if data + parity bit satisfies even parity
uint8_t check_even_parity(uint32_t data, uint8_t parity) {
    uint8_t total_ones = parity;
    uint32_t temp = data;

    while (temp) {
        total_ones ^= (temp & 1);
        temp >>= 1;
    }

    // If XOR result is 0, then even parity is satisfied
    return (total_ones == 0);
}

void SPI_TMSRead(uint64_t *ptr, uint8_t bits);

// SWD Transfer I/O
//   request: A[3:2] RnW APnDP
//   data:    DATA[31:0]
//   return:  ACK[2:0]
                                 \
uint8_t SWD_Transfer_LL(uint32_t request, uint32_t *data) {
  uint32_t ack;
  uint8_t writeReq = 1;
  uint32_t val;
  uint8_t parity;
  uint32_t bit;

  uint64_t dummyRead;
                                                                                
  uint32_t n;
  uint32_t read_data = 0;

  uint32_t write_data = 0;
  write_nbits_lsb(&writeReq, 1, 4, request);

  parity = generate_even_parity(request & 0xf);

  write_nbits_lsb(&writeReq, 5, 1, parity);

  write_nbits_lsb(&writeReq, 7, 1, 1);

  SPI_SwitchPhaseToWrite();
  SPI4->CR1 |= (0x1 << 6);

  SPI_TMS_Transfer(writeReq, 8);
  SPI_Transfer(&dummyRead, 0, 8);
  SPI_TMSRead(&dummyRead, 8);

  if (request & DAP_TRANSFER_RnW)
  {
	  //TODO: correct the shiftin according to n variable...
	  SPI_SwitchPhaseToListen();
	  /* Turnaround */

	  n = DAP_Data.swd_conf.turnaround;

	  SPI4->CR2 = ( (16 -1) << 8);
	  SPI_Transfer(&dummyRead, 0, 16);
	  SPI_TMSRead(&dummyRead, 16);

	  ack = dummyRead & 0x7;


	  read_data |= dummyRead >> 3;

	  SPI4->CR2 = ( (16 -1) << 8);
	  SPI_Transfer(&dummyRead, 0, 16);
	  SPI_TMSRead(&dummyRead, 16);

	  read_data |= (dummyRead) << 13;

	  SPI4->CR2 = ( (5 + n -1) << 8);
	  SPI_Transfer(&dummyRead, 0, (5 + n) );
	  SPI_TMSRead(&dummyRead, (5 + n) );

	  read_data |= ( dummyRead & 0x7) << 29;

	  parity = (dummyRead & 0x8) >> 3;

	  if( !check_even_parity(read_data, parity) && DAP_TRANSFER_OK == ack)
	  {
		  ack = DAP_TRANSFER_ERROR;
	  }

	  *data = read_data;
	  SPI4->CR1 &= ~(0x1 << 6);




  }
  else
  {
	  SPI_SwitchPhaseToListen();
	  /* Turnaround */

	  n = DAP_Data.swd_conf.turnaround;


	  SPI4->CR2 = ( ( 2*n +3 -1) << 8);
	  SPI_Transfer(&dummyRead, 0, 2*n+3);
	  SPI_TMSRead(&dummyRead, 2*n+3);

	  SPI_SwitchPhaseToWrite();
	  //SPI4->CR1 &= ~(0x1 << 6);


	  ack = (dummyRead & 0x7);

	  if(DAP_TRANSFER_OK == ack)
	  {
		  parity = generate_even_parity(*data);
		  write_data = *data;
		  SPI4->CR1 |= (0x1 << 6);

		  SPI_TMS_Transfer(write_data, 16);
		  SPI_Transfer(&dummyRead, 0, 16);
		  SPI_TMSRead(&dummyRead, 16);

		  write_data >>= 16;

		  SPI_TMS_Transfer(write_data, 8);
		  SPI_Transfer(&dummyRead, 0, 8);
		  SPI_TMSRead(&dummyRead, 8);

		  uint32_t delay_cnt = 2000;

		  while(delay_cnt--)
		  {
			  __asm("nop");
		  }

		  write_data >>= 8;

		  write_data |= parity << 8;

		  SPI_TMS_Transfer(write_data, 16);
		  SPI_Transfer(&dummyRead, 0, 16);
		  SPI_TMSRead(&dummyRead, 16);



	  }
	  else
	  {
		  SPI4->CR1 &= ~(0x1 << 6);
		  SPI_SwitchPhaseToListen();
		  SPI4->CR1 |= (0x1 << 6);

		  n = DAP_Data.swd_conf.turnaround;

		  SPI_Transfer(&dummyRead, 0, 16);
		  SPI_TMSRead(&dummyRead, 16);

		  SPI_Transfer(&dummyRead, 0, 8);
		  SPI_TMSRead(&dummyRead, 8);

		  SPI_Transfer(&dummyRead, 0, 33-24 +n);
		  SPI_TMSRead(&dummyRead, 33-24 +n);

	  }

	  SPI4->CR1 &= ~(0x1 << 6);



  }

  /* Capture Timestamp */
      if (request & DAP_TRANSFER_TIMESTAMP) {
        DAP_Data.timestamp = TIMESTAMP_GET();
      }

  return ((uint8_t)ack);

  while(1)
  {
	  n = 16;
	  SPI_TMS_Transfer(writeReq, n);
	  SPI_Transfer(&dummyRead, 0, n);
	  SPI_TMSRead(&dummyRead, n);
  }
  /* Acknowledge response */                                                    
  SW_READ_BIT(bit);                                                             
  ack  = bit << 0;                                                              
  SW_READ_BIT(bit);                                                             
  ack |= bit << 1;                                                              
  SW_READ_BIT(bit);                                                             
  ack |= bit << 2;                                                              
                                                                                
  if (ack == DAP_TRANSFER_OK) {         /* OK response */                       
    /* Data transfer */                                                         
    if (request & DAP_TRANSFER_RnW)
    {
      /* Read data */                                                           
      val = 0U;                                                                 
      parity = 0U;                                                              
      for (n = 32U; n; n--) {                                                   
        SW_READ_BIT(bit);               /* Read RDATA[0:31] */                  
        parity += bit;                                                          
        val >>= 1;                                                              
        val  |= bit << 31;                                                      
      }                                                                         
      SW_READ_BIT(bit);                 /* Read Parity */                       
      if ((parity ^ bit) & 1U) {                                                
        ack = DAP_TRANSFER_ERROR;                                               
      }                                                                         
      if (data) { *data = val; }                                                
      /* Turnaround */                                                          
      for (n = DAP_Data.swd_conf.turnaround; n; n--) {                          
        SW_CLOCK_CYCLE();                                                       
      }                                                                         
      PIN_SWDIO_OUT_ENABLE();                                                   
    }
    else
    {
      /* Turnaround */                                                          
      for (n = DAP_Data.swd_conf.turnaround; n; n--) {                          
        SW_CLOCK_CYCLE();                                                       
      }                                                                         
      PIN_SWDIO_OUT_ENABLE();                                                   
      /* Write data */                                                          
      val = *data;                                                              
      parity = 0U;                                                              
      for (n = 32U; n; n--) {                                                   
        SW_WRITE_BIT(val);              /* Write WDATA[0:31] */                 
        parity += val;                                                          
        val >>= 1;                                                              
      }                                                                         
      SW_WRITE_BIT(parity);             /* Write Parity Bit */                  
    }                                                                           
    /* Capture Timestamp */
    if (request & DAP_TRANSFER_TIMESTAMP) {
      DAP_Data.timestamp = TIMESTAMP_GET();
    }
    /* Idle cycles */                                                           
    n = DAP_Data.transfer.idle_cycles;                                          
    if (n) {                                                                    
      PIN_SWDIO_OUT(0U);                                                        
      for (; n; n--) {                                                          
        SW_CLOCK_CYCLE();                                                       
      }                                                                         
    }                                                                           
    PIN_SWDIO_OUT(1U);                                                          
    return ((uint8_t)ack);                                                      
  }                                                                             
                                                                                
  if ((ack == DAP_TRANSFER_WAIT) || (ack == DAP_TRANSFER_FAULT)) {              
    /* WAIT or FAULT response */                                                
    if (DAP_Data.swd_conf.data_phase && ((request & DAP_TRANSFER_RnW) != 0U)) { 
      for (n = 32U+1U; n; n--) {                                                
        SW_CLOCK_CYCLE();               /* Dummy Read RDATA[0:31] + Parity */   
      }                                                                         
    }                                                                           
    /* Turnaround */                                                            
    for (n = DAP_Data.swd_conf.turnaround; n; n--) {                            
      SW_CLOCK_CYCLE();                                                         
    }                                                                           
    PIN_SWDIO_OUT_ENABLE();                                                     
    if (DAP_Data.swd_conf.data_phase && ((request & DAP_TRANSFER_RnW) == 0U)) { 
      PIN_SWDIO_OUT(0U);                                                        
      for (n = 32U+1U; n; n--) {                                                
        SW_CLOCK_CYCLE();               /* Dummy Write WDATA[0:31] + Parity */  
      }                                                                         
    }                                                                           
    PIN_SWDIO_OUT(1U);                                                          
    return ((uint8_t)ack);                                                      
  }                                                                             
                                                                                
  /* Protocol error */                                                          
  for (n = DAP_Data.swd_conf.turnaround + 32U + 1U; n; n--) {                   
    SW_CLOCK_CYCLE();                   /* Back off data phase */               
  }                                                                             
  PIN_SWDIO_OUT_ENABLE();                                                       
  PIN_SWDIO_OUT(1U);                                                            
  return ((uint8_t)ack);                                                        
}




// SWD Transfer I/O
//   request: A[3:2] RnW APnDP
//   data:    DATA[31:0]
//   return:  ACK[2:0]
uint8_t  SWD_Transfer(uint32_t request, uint32_t *data) {
  if (DAP_Data.fast_clock) {
    return SWD_Transfer_LL(request, data);
  } else {
    return SWD_Transfer_LL(request, data);
  }
}


#endif  /* (DAP_SWD != 0) */

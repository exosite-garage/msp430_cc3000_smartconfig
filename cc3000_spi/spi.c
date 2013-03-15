
/*****************************************************************************
*
*  spi.c - CC3000 Host Driver Implementation.
*  Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the   
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*****************************************************************************/

//*****************************************************************************
//
//! \addtogroup link_buff_api
//! @{
//
//*****************************************************************************
#include "hci.h"
#include "spi.h"
#include "os.h"
#include "evnt_handler.h"
#include "utils.h"
#include <msp430.h>


#define READ                    3
#define WRITE                   1

#define HI(value)               (((value) & 0xFF00) >> 8)
#define LO(value)               ((value) & 0x00FF)

#define ASSERT_CS()          (P1OUT &= ~BIT3)

#define DEASSERT_CS()        (P1OUT |= BIT3)

#define HEADERS_SIZE_EVNT       (SPI_HEADER_SIZE + 5)

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//packed is used for preventing padding before sending the structure over the SPI                       ///
//for every IDE, exist different syntax:          1.   __MSP430FR5739__ for CCS v5                      ///
//                                                2.  __IAR_SYSTEMS_ICC__ for IAR Embedded Workbench    ///
// THIS COMMENT IS VALID FOR EVERY STRUCT DEFENITION!                                                   ///
///////////////////////////////////////////////////////////////////////////////////////////////////////////


#ifdef __MSP430FR5739__
typedef struct __attribute__ ((__packed__)) _btspi_hdr
#elif __IAR_SYSTEMS_ICC__
#pragma pack(1)
typedef struct _btspi_hdr
#endif
{
    unsigned char   cmd;
    unsigned short  length;
    unsigned char   pad[2];
}btspi_hdr;



#define 	eSPI_STATE_POWERUP 				 (0)
#define 	eSPI_STATE_INITIALIZED  		 (1)
#define 	eSPI_STATE_IDLE					 (2)
#define 	eSPI_STATE_WRITE_IRQ	   		 (3)
#define 	eSPI_STATE_WRITE_FIRST_PORTION   (4)
#define 	eSPI_STATE_WRITE_EOT			 (5)
#define 	eSPI_STATE_READ_IRQ				 (6)
#define 	eSPI_STATE_READ_FIRST_PORTION	 (7)
#define 	eSPI_STATE_READ_EOT				 (8)


#define SPI_BUFFER_SIZE						 (200) //HAR TODO - original size options were 100 and 1700... if WLAN acts unstable, may need to increase this

typedef struct
{
	unsigned long ulPioPortAddress;
	unsigned long ulPioSpiPort;
	unsigned long ulPioSpiCs;
	unsigned long ulPortInt;
	unsigned long ulPioSlEnable;
	
	unsigned long uluDmaPort;
	unsigned long uluDmaRxChannel;
	unsigned long uluDmaTxChannel;
	
	unsigned long ulSsiPort;
	unsigned long ulSsiPortAddress;
	unsigned long ulSsiTx;
	unsigned long ulSsiRx;
	unsigned long ulSsiClck;
	unsigned long ulSsiPortInt;
}tSpiHwConfiguration;

typedef struct
{
	gcSpiHandleRx  SPIRxHandler;

	unsigned short usTxPacketLength;
	unsigned short usRxPacketLength;
	unsigned long  ulSpiState;
	unsigned char *pTxPacket;
	unsigned char *pRxPacket;
	tSpiHwConfiguration	sHwSettings;
}tSpiInformation;


tSpiInformation sSpiInformation;

//
// Static buffer for 5 bytes of SPI HEADER
//
btspi_hdr tSpiReadHeader = {READ, 0, 0, 0};


void SpiWriteDataSynchronous(unsigned char *data, unsigned short size);
void SpiWriteAsync(const unsigned char *data, unsigned short size);
void SpiPauseSpi(void);
void SpiResumeSpi(void);
void SSIContReadOperation(void);


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//#pragma is used for determine the memory location for a specific variable.                            ///        ///
//__no_init is used to prevent the buffer initialization in order to prevent hardware WDT expiration    ///
// before entering to 'main()'.                                                                         ///
//for every IDE, different syntax exists :          1.   __MSP430FR5739__ for CCS v5                    ///
//                                                  2.  __IAR_SYSTEMS_ICC__ for IAR Embedded Workbench  ///
// *CCS does not initialize variables - therefore, __no_init is not needed.                             ///
///////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef __MSP430FR5739__
#pragma DATA_SECTION(spi_buffer, ".FRAM_DATA")
char spi_buffer[SPI_BUFFER_SIZE];

#elif __IAR_SYSTEMS_ICC__
#pragma location = "FRAM_DATA"
__no_init char spi_buffer[SPI_BUFFER_SIZE];
#endif



#ifdef __MSP430FR5739__
#pragma DATA_SECTION(wlan_tx_buffer, ".FRAM_DATA")
unsigned char wlan_tx_buffer[SPI_BUFFER_SIZE];

#elif __IAR_SYSTEMS_ICC__
#pragma location = "FRAM_DATA"
__no_init unsigned char wlan_tx_buffer[SPI_BUFFER_SIZE];
#endif
//*****************************************************************************
// 
//!  This function get the reason for the GPIO interrupt and clear cooresponding
//!  interrupt flag
//! 
//!  \param  none
//! 
//!  \return none
//! 
//!  \brief  This function This function get the reason for the GPIO interrupt
//!          and clear cooresponding interrupt flag
// 
//*****************************************************************************
void
SpiCleanGPIOISR(void)
{
	P2IFG &= ~BIT3;
}
 

//*****************************************************************************
//
//! This function: init_spi
//!
//!  \param  buffer
//!
//!  \return none
//!
//!  \brief  initializes an SPI interface
//
//*****************************************************************************

int init_spi(void)
{
    // Select the SPI lines: MISO/MOSI on P1.6,7 CLK on P2.2
    P1SEL1 |= (BIT6 + BIT7);
    P1SEL0 &= (~(BIT6 + BIT7));

    P2SEL1 |= (BIT2);
    P2SEL0 &= ~BIT2;

    UCB0CTLW0 |= UCSWRST;                     // **Put state machine in reset**
    UCB0CTLW0 |= (UCMST+UCSYNC+UCMSB);          // 3-pin, 8-bit SPI master
    // Clock polarity high, MSB
    UCB0CTLW0 |= UCSSEL_2;                    // ACLK
    UCB0BR0 = 0x02;                           // /2 change to /1
    UCB0BR1 = 0;                              //

    UCB0CTLW0 &= ~UCSWRST;                    // **Initialize USCI state machine**

    return(ESUCCESS);
}



//*****************************************************************************
//
//!  SpiClose
//!
//!  \param  none
//!
//!  \return none
//!
//!  \brief  Cofigure the SSI
//
//*****************************************************************************
void
SpiClose(void)
{
	if (sSpiInformation.pRxPacket)
	{
		sSpiInformation.pRxPacket = 0;
	}

	//
	//	Disable Interrupt in GPIOA module...
	//
    tSLInformation.WlanInterruptDisable();
}


//*****************************************************************************
//
//!  SpiClose
//!
//!  \param  none
//!
//!  \return none
//!
//!  \brief  Cofigure the SSI
//
//*****************************************************************************
void 
SpiOpen(gcSpiHandleRx pfRxHandler)
{
	sSpiInformation.ulSpiState = eSPI_STATE_POWERUP;

	sSpiInformation.SPIRxHandler = pfRxHandler;
	sSpiInformation.usTxPacketLength = 0;
	sSpiInformation.pTxPacket = NULL;
	sSpiInformation.pRxPacket = (unsigned char *)spi_buffer;
	sSpiInformation.usRxPacketLength = 0;



	//
	// Enable interrupt on the GPIOA pin of WLAN IRQ
	//
	tSLInformation.WlanInterruptEnable();
}

 

//*****************************************************************************
//
//! This function enter point for write flow
//!
//!  \param  buffer
//!
//!  \return none
//!
//!  \brief  ...
//
//*****************************************************************************
long
SpiFirstWrite(unsigned char *ucBuf, unsigned short usLength)
{
    //
    // workaround for first transaction
    //
    ASSERT_CS();
	
    // Assuming we are running on 24 MHz ~50 micro delay is 1200 cycles;
    __delay_cycles(1200);
	
    // SPI writes first 4 bytes of data
    SpiWriteDataSynchronous(ucBuf, 4);

    __delay_cycles(1200);
	
    SpiWriteDataSynchronous(ucBuf + 4, usLength - 4);

    // From this point on - operate in a regular way
    sSpiInformation.ulSpiState = eSPI_STATE_IDLE;
    
    DEASSERT_CS();

    return(0);
}



//*****************************************************************************
//
//! This function enter point for write flow
//!
//!  \param  buffer
//!
//!  \return none
//!
//!  \brief  ...
//
//*****************************************************************************
long
SpiWrite(unsigned char *pUserBuffer, unsigned short usLength)
{
    unsigned char ucPad = 0;

	//
	// Figure out the total length of the packet in order to figure out if there is padding or not
	//
    if(!(usLength & 0x0001))
    {
        ucPad++;
    }


    pUserBuffer[0] = WRITE;
    pUserBuffer[1] = HI(usLength + ucPad);
    pUserBuffer[2] = LO(usLength + ucPad);
    pUserBuffer[3] = 0;
    pUserBuffer[4] = 0;

    usLength += (sizeof(btspi_hdr) + ucPad);

	if (sSpiInformation.ulSpiState == eSPI_STATE_POWERUP)
	{
		while (sSpiInformation.ulSpiState != eSPI_STATE_INITIALIZED)
			;
	}
	
	if (sSpiInformation.ulSpiState == eSPI_STATE_INITIALIZED)
	{
		//
		// This is time for first TX/RX transactions over SPI: the IRQ is down - so need to send read buffer size command
		//
		SpiFirstWrite(pUserBuffer, usLength);
	}
	else 
	{
		//
		// We need to prevent here race that can occur in case 2 back to back packets are sent to the 
		// device, so the state will move to IDLE and once again to not IDLE due to IRQ
		//
		tSLInformation.WlanInterruptDisable();

		while (sSpiInformation.ulSpiState != eSPI_STATE_IDLE)
		{
			;
		}

		
		sSpiInformation.ulSpiState = eSPI_STATE_WRITE_IRQ;
		sSpiInformation.pTxPacket = pUserBuffer;
		sSpiInformation.usTxPacketLength = usLength;
		
		//
		// Assert the CS line and wait till SSI IRQ line is active and then initialize write operation
		//
		ASSERT_CS();

		//
		// Re-enable IRQ - if it was not disabled - this is not a problem...
		//
		tSLInformation.WlanInterruptEnable();
	}


	//
	// Due to the fact that we are currently implementing a blocking situation
	// here we will wait till end of transaction
	//

	{
          volatile unsigned long vDelay = 3 * 0xAAAAA;
          while (eSPI_STATE_IDLE != sSpiInformation.ulSpiState) {           
            if (!vDelay--) {
            	errorHandler(ERROR_SPI_TIMEOUT);  //TODO - Exosite workaround - can't have infinit loops with real HW folks!
            }
          }
        }
	
    return(0);
}

 


//*****************************************************************************
//
//! This function enter point for write flow
//!
//!  \param  buffer
//!
//!  \return none
//!
//!  \brief  ...
//
//*****************************************************************************
void
SpiWriteDataSynchronous(unsigned char *data, unsigned short size)
{
	while (size)
    {
    	while (!(UCB0IFG&UCTXIFG));
		UCB0TXBUF = *data;
		while (!(UCB0IFG&UCRXIFG));
		UCB0RXBUF;
		size --;
        data++;
    }
}

//*****************************************************************************
//
//! This function enter point for write flow
//!
//!  \param  buffer
//!
//!  \return none
//!
//!  \brief  ...
//
//*****************************************************************************
void
SpiReadDataSynchronous(unsigned char *data, unsigned short size)
{
	unsigned long i = 0;
    char *data_to_send = ((char *)(&tSpiReadHeader.cmd));
	
	for (i = 0; i < size; i ++)
    {
    	while (!(UCB0IFG&UCTXIFG));
		UCB0TXBUF = data_to_send[i];
		while (!(UCB0IFG&UCRXIFG));
		data[i] = UCB0RXBUF;
    }
}



//*****************************************************************************
//
//! This function enter point for read flow: first we read minimal 5 SPI header bytes and 5 Event
//!	Data bytes
//!
//!  \param  buffer
//!
//!  \return none
//!
//!  \brief  ...
//
//*****************************************************************************
void
SpiReadHeader(void)
{
	SpiReadDataSynchronous(sSpiInformation.pRxPacket, 10);
}


//*****************************************************************************
//
//! This function processes received SPI Header and in accordance with it - continues reading 
//!	the packet
//!
//!  \param  None
//!
//!  \return None
//!
//!  \brief  ...
//
//*****************************************************************************
long
SpiReadDataCont(void)
{
    hci_hdr_t *hci_hdr;
    long data_to_recv;
	hci_evnt_hdr_t *hci_evnt_hdr;
	unsigned char *evnt_buff;
	hci_data_hdr_t *pDataHdr;
    
	
    //
    //determine what type of packet we have
    //
    evnt_buff =  sSpiInformation.pRxPacket;
    data_to_recv = 0;
	hci_hdr = (hci_hdr_t *)(evnt_buff + sizeof(btspi_hdr));
	
    switch(hci_hdr->ucType)
    {
        case HCI_TYPE_DATA:
        {
			pDataHdr = (hci_data_hdr_t *)(evnt_buff + sizeof(btspi_hdr));

			//
			// We need to read the rest of data..
			//
			data_to_recv = pDataHdr->usLength;

			if (!((HEADERS_SIZE_EVNT + data_to_recv) & 1))
			{	
    	        data_to_recv++;
			}

			if (data_to_recv)
			{
            	SpiReadDataSynchronous(evnt_buff + 10, data_to_recv);
			}
            break;
        }
        case HCI_TYPE_EVNT:
        {
        	//
            //configure buffer to read rest of the data
            //
            hci_evnt_hdr = (hci_evnt_hdr_t *)hci_hdr;

			// 
			// Calculate the rest length of the data
			//
            data_to_recv = hci_evnt_hdr->ucLength - 1;
			
			// 
			// Add padding byte if needed
			//
			if ((HEADERS_SIZE_EVNT + data_to_recv) & 1)
			{
				
	            data_to_recv++;
			}
			
			if (data_to_recv)
			{
            	SpiReadDataSynchronous(evnt_buff + 10, data_to_recv);
			}

			sSpiInformation.ulSpiState = eSPI_STATE_READ_EOT;
            break;
        }
    }
	
    return (0);
}


//*****************************************************************************
//
//! This function enter point for write flow
//!
//!  \param  SpiPauseSpi
//!
//!  \return none
//!
//!  \brief  The function triggers a user provided callback for 
//
//*****************************************************************************

void 
SpiPauseSpi(void)
{
	P2IE &= ~BIT3;
}


//*****************************************************************************
//
//! This function enter point for write flow
//!
//!  \param  SpiResumeSpi
//!
//!  \return none
//!
//!  \brief  The function triggers a user provided callback for 
//
//*****************************************************************************

void 
SpiResumeSpi(void)
{
	P2IE |= BIT3;
}



//*****************************************************************************
//
//! This function enter point for write flow
//!
//!  \param  SpiTriggerRxProcessing
//!
//!  \return none
//!
//!  \brief  The function triggers a user provided callback for 
//
//*****************************************************************************
void 
SpiTriggerRxProcessing(void)
{
	//
	// Trigger Rx processing
	//
	SpiPauseSpi();
	DEASSERT_CS();
	
	sSpiInformation.ulSpiState = eSPI_STATE_IDLE;
	sSpiInformation.SPIRxHandler(sSpiInformation.pRxPacket + sizeof(btspi_hdr));
}

//*****************************************************************************
// 
//!  The IntSpiGPIOHandler interrupt handler
//! 
//!  \param  none
//! 
//!  \return none
//! 
//!  \brief  GPIO A interrupt handler. When the external SSI WLAN device is
//!          ready to interact with Host CPU it generates an interrupt signal.
//!          After that Host CPU has registrated this interrupt request
//!          it set the corresponding /CS in active state.
// 
//*****************************************************************************
#pragma vector=PORT2_VECTOR
__interrupt void IntSpiGPIOHandler(void)
{
	//__bic_SR_register(GIE);
	switch(__even_in_range(P2IV,P2IV_P2IFG3))
    {
	  case P2IV_P2IFG3:
		if (sSpiInformation.ulSpiState == eSPI_STATE_POWERUP)
		{
			/* This means IRQ line was low call a callback of HCI Layer to inform on event */
	 		sSpiInformation.ulSpiState = eSPI_STATE_INITIALIZED;
		}
		else if (sSpiInformation.ulSpiState == eSPI_STATE_IDLE)
		{
			sSpiInformation.ulSpiState = eSPI_STATE_READ_IRQ;
			
			/* IRQ line goes down - we are start reception */
			ASSERT_CS();

			//
			// Wait for TX/RX Compete which will come as DMA interrupt
			// 
	        SpiReadHeader();

			sSpiInformation.ulSpiState = eSPI_STATE_READ_EOT;
			
			//
			//
			//
			SSIContReadOperation();
		}
		else if (sSpiInformation.ulSpiState == eSPI_STATE_WRITE_IRQ)
		{
			SpiWriteDataSynchronous(sSpiInformation.pTxPacket, sSpiInformation.usTxPacketLength);

			sSpiInformation.ulSpiState = eSPI_STATE_IDLE;

			DEASSERT_CS();
		}
		break;
	default:
		break;
	}

	//__bis_SR_register(GIE);
}

//*****************************************************************************
//
//! This function enter point for write flow
//!
//!  \param  SSIContReadOperation
//!
//!  \return none
//!
//!  \brief  The function triggers a user provided callback for 
//
//*****************************************************************************

void
SSIContReadOperation(void)
{
	//
	// The header was read - continue with  the payload read
	//
	if (!SpiReadDataCont())
	{
		
		
		//
		// All the data was read - finalize handling by switching to teh task
		//	and calling from task Event Handler
		//
		SpiTriggerRxProcessing();
	}
}




//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

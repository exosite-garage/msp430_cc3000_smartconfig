/*****************************************************************************
*
*  utils.c - Generic helper functions
*  Copyright (C) 2012 Exosite LLC
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

#include <msp430.h>
#include "board.h"
#include "utils.h"

// externs

// globals

// external functions

// local functions

//local defines

//*****************************************************************************
//
//!  busyWait
//!
//!  \param  delay in approximate mS - adjust delay to suit
//!
//!  \return none
//!
//!  \brief  Busy loop by spinning a short volatile, delays up to 65k mS (65 s)
//
//*****************************************************************************
void
busyWait(unsigned short delay)
{
    unsigned short loops;

    // __delay_cycles is about 40nS per cycle or 25,000 per millisecond or 25 million per second
    for (loops = 0; loops < delay; loops++)
      __delay_cycles(25000);

}


//*****************************************************************************
//
//!  errorHandler
//!
//!  \param  Error code from the enum
//!
//!  \return none
//!
//!  \brief  Generic Error Handler that blinks LEDs
//
//*****************************************************************************
void
errorHandler(unsigned char code)
{
	unsigned char count, blink;

	for (count=0; count < 20; count++)
    {
        turnLedOn(CC3000_UNUSED1_IND);
        busyWait(100);
        turnLedOff(CC3000_UNUSED1_IND);
        busyWait(100);
    }

	busyWait(2000);

	switch (code) {
		case ERROR_NOT_ASSOCIATED:
			blink = 1;
			break;
		case ERROR_WRITE_FAILURE:
			blink = 2;
			break;
		case ERROR_CONNECT_FAILURE:
			blink = 3;
			break;
		case ERROR_SPI_TIMEOUT:
			blink = 4;
			break;
		case ERROR_ACCEPT_BLOCK:
			blink = 5;
			break;
		default:
			blink = 10;
			break;
	}


	for (count=0; count < blink; count++)
    {
        turnLedOn(CC3000_UNUSED1_IND);
        busyWait(500);
        turnLedOff(CC3000_UNUSED1_IND);
        busyWait(500);
    }

	restartMSP430();
}


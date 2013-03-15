/*****************************************************************************
*
*  sensors.c - FRAM board sensor functions
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
#include "msp430fr5739.h"
#include "sensors.h"
#include "utils.h"

const char sensorNames[10][11] = {
                                "adc0",
                                "adc1",
                                "adc2",
                                "thermistor",
                                "adc5",
                                "adc7",
                                "acc_x",
                                "acc_y",
                                "acc_z",
                                "adc15"
                                };

int getSensorResult(unsigned char sensorNum)
{
  ADC10IFG = 0;
  ADC10CTL0 &= ~ADC10ENC;
  ADC10CTL0 &= ~ADC10SC;
  
  switch (sensorNum) {
    case A0:
      ADC10MCTL0 = ADC10INCH_0;     // A0 ADC input select; Vref=AVCC   
      break;
    case A1:
      ADC10MCTL0 = ADC10INCH_1;     // A1 ADC input select; Vref=AVCC         
      break;
    case A2:
      ADC10MCTL0 = ADC10INCH_2;     // A2 ADC input select; Vref=AVCC         
      break;
    case THERMISTOR:
      P2OUT |= (BIT7);				// power P2.7 for our accelerometer and thermistor readings
      busyWait(100);				// allow for power-up settling
      ADC10MCTL0 = ADC10INCH_4;     // A4 ADC input select; Vref=AVCC
      break;
    case A5:
      ADC10MCTL0 = ADC10INCH_5;     // A5 ADC input select; Vref=AVCC
      break;
    case A7:
      ADC10MCTL0 = ADC10INCH_7;     // A7 ADC input select; Vref=AVCC
      break;
   case ACC_X:
      P2OUT |= (BIT7);				// power P2.7 for our accelerometer and thermistor readings
      busyWait(100);				// allow for power-up settling
      ADC10MCTL0 = ADC10INCH_12;    // A12 ADC input select; Vref=AVCC
      break;
    case ACC_Y:
      P2OUT |= (BIT7);				// power P2.7 for our accelerometer and thermistor readings
      busyWait(100);				// allow for power-up settling
      ADC10MCTL0 = ADC10INCH_13;    // A13 ADC input select; Vref=AVCC
      break;
    case ACC_Z:
      P2OUT |= (BIT7);				// power P2.7 for our accelerometer and thermistor readings
      busyWait(100);				// allow for power-up settling
      ADC10MCTL0 = ADC10INCH_14;    // A14 ADC input select; Vref=AVCC
      break;
    case A15:
      ADC10MCTL0 = ADC10INCH_15;    // A15 ADC input select; Vref=AVCC
      break;
    default:
      break;      
  }

  ADC10CTL0 |= ADC10ENC + ADC10SC ; // Start conversion

  while (ADC10CTL1 & BUSY);
  
  P2OUT &= ~(BIT7);                 //bring P2.7 power pin back low (regardless of whether it was high or not)

  return ADC10MEM0;
}


void setupSensors(void)
{
	// ADC10 on MSP430FR5739 has 12 external inputs: A0 -> A7 and A12 -> A15
	// These are mapped to I/O lines by the Port Select Registers (P1SEL -> P3SEL)
	// ADC is the ternary module function for the I/O, so both corresponding
	// bits in each SEL1/0 register must be set to enable the ADC

	// ADC Channel	-> Port (pin) 	-> FRAM Board I/O
	//---------------------------------------------------------------------------------------
	// A0 			-> P1.0 (1)		-> FREE (SV1 pin1, eZ-RF pin4, RF1 pin5, RF2 pin15)
	// A1 			-> P1.1 (2)		-> FREE (SV1 pin2, eZ-RF pin6, RF1 pin6, RF1 pin7)
	// A2 			-> P1.2 (3)		-> FREE (SV1 pin3, eZ-RF pin8, RF1 pin8, RF1 pin9)
	// A3 			-> P1.3 (8)		-> RF (SV1 pin8, eZ-RF pin17, RF1 pin 14)
	// A4 			-> P1.4 (9)		-> THERMISTOR (SV1 pin9, NTC)
	// A5 			-> P1.5 (10)	-> FREE (SV1 pin10)
	// A6 			-> P2.3 (34)	-> RF (eZ-RF pin10, RF1 pin12)
	// A7 			-> P2.4 (35)	-> FREE (eZ-RF pin14, RF1 pin3)
	// A12 			-> P3.0 (4)		-> ACCELEROMETER X (SV1 pin4, ACC XOUT)
	// A13 			-> P3.1 (5)		-> ACCELEROMETER Y (SV1 pin5, ACC YOUT)
	// A14 			-> P3.2 (6)		-> ACCELEROMETER Z (SV1 pin6, ACC ZOUT)
	// A15 			-> P3.3 (7)		-> FREE (SV1 pin7, LDR)

    // Configure P2.7 as digital out (this is our power pin for the accelerometer and thermistor)
    P2DIR  |= (BIT7);
    P2SEL1 &= ~(BIT7);
    P2SEL0 &= ~(BIT7);

    // Set P2.7 low (power off)
    P2OUT &= ~(BIT7);

	// Configure ADC A0 (FREE), A1 (FREE), A2 (FREE), A4 (THERMISTOR), A5 (FREE)
    P1SEL1 |= (BIT0 | BIT1 | BIT2 | BIT4 | BIT5);  
    P1SEL0 |= (BIT0 | BIT1 | BIT2 | BIT4 | BIT5); 

    // Configure ADC A7 (FREE)
    P2SEL1 |= (BIT4);  
    P2SEL0 |= (BIT4); 

    // Configure ADC A12 (ACC X), A13 (ACC Y), A14 (ACC Z), A15 (FREE)
    P3SEL1 |= (BIT0 | BIT1 | BIT2 | BIT3);
    P3SEL0 |= (BIT0 | BIT1 | BIT2 | BIT3);
    
    // Allow for settling delay 
    busyWait(2);
    
    // Configure ADC
    ADC10CTL0 &= ~ADC10ENC; 
    ADC10CTL0 = ADC10SHT_7 + ADC10ON;        // ADC10ON, S&H=192 ADC clks
    // ADCCLK = MODOSC = 5MHz
    ADC10CTL1 = ADC10SHS_0 + ADC10SHP + ADC10SSEL_0; 
    ADC10CTL2 = ADC10RES;                    // 10-bit conversion results
    //ADC10IE = ADC10IE0;                    // Enable ADC conv complete interrupt
}

/*****************************************************************************
*
*  cc3000.h - CC3000 Function Definitions
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

#ifndef CC3000_H
#define CC3000_H

#include "netapp.h"

#define SOCKET_INACTIVE_ERR -57


#define NUM_STATES 6
#define FIRST_STATE_LED_NUM 1
#define MAX_SSID_LEN        32

// CC3000 State Machine Definitions
enum cc3000StateEnum
{
    CC3000_UNINIT           = 0x01, // CC3000 Driver Uninitialized
    CC3000_INIT             = 0x02, // CC3000 Driver Initialized
    CC3000_ASSOC            = 0x04, // CC3000 Associated to AP
    CC3000_IP_ALLOC         = 0x08, // CC3000 has IP Address
    CC3000_SERVER_INIT      = 0x10, // CC3000 Server Initialized
    CC3000_CLIENT_CONNECTED = 0x20  // CC3000 Client Connected to Server
};



int ConnectUsingSSID(char * ssidName);
void setupLocalSocket(void);
void ConnectToServer(void);
void ConnectToServer(void);


char *sendDriverPatch(unsigned long *Length);
char *sendBootLoaderPatch(unsigned long *Length);
char *sendWLFWPatch(unsigned long *Length);

void CC3000_UsynchCallback(long lEventType, char * data, unsigned char length);

int initDriver(void);
void StartSmartConfig(void);
void closeLocalSocket(void);
void disconnectAll();

char isFTCSet();
void setFTCFlag();


// Machine State
char currentCC3000State();
void setCC3000MachineState(char stat);
void unsetCC3000MachineState(char stat);
void resetCC3000StateMachine();
char highestCC3000State();

tNetappIpconfigRetArgs * getCC3000Info();
#endif

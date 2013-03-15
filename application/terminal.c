/*****************************************************************************
*
*  terminal.c - Terminal functions implementation
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

#include <msp430.h>
#include "common.h"
#include "string.h"
#include "uart.h"
#include "cc3000.h"
#include "wlan.h"
#include "netapp.h"
#include "board.h"
#include "strlib.h"
#include "terminal.h"


unsigned char * ptrSSIDInd = (unsigned char *)0x1810;   
unsigned char * ptrSSID = (unsigned char *)0x1820;

// CC3000 Status Info
extern tNetappIpconfigRetArgs ipinfo;

extern char aucCC3000_prefix[3];

//*****************************************************************************
//
//!  \brief Initializes the UART Terminal interface
//!
//!  \param none
//!
//!  \return none
//
//*****************************************************************************
void initTerminal()
{
    initUart();
}

//*****************************************************************************
//
//!  \brief returns a pointer to the RX buffer at the specific position
//!
//!  \param  pos is the location in the RX buffer to which pointer will point
//!
//!  \return a pointer to the RX UART buffer
//
//*****************************************************************************
void runUARTTerminal()
{
    char validPos = 0;
    char * ftcPrefixptr;
    
    if(uartRXByte(bytesInUart()-1) != '\b' && uartRXByte(bytesInUart()-1) != 0x7F)
    {
            sendByte(uartRXByte(bytesInUart()-1));
    }
    else
    {
        // Do backspace only if it doesn't erase '>'
        // Note that the backspace is itself in the buffer
        if(bytesInUart() > 1)
        {  
            // Echo Backspace and remove latest byte
            sendByte(uartRXByte(bytesInUart()-1));
            
            // Erase Backspace from buffer
            removeLastByteinBuf();
            
            // Erase last character
            removeLastByteinBuf();           
        }
        else
        {
            // Erase Backspace from buffer
            removeLastByteinBuf();
        }
    }
    
    
    switch(uartRXByte(bytesInUart()-1)) 
    {
    case '\r':
    case '\n':
        
        // Erase \r or \n from buffer
        removeLastByteinBuf();
        
        // Skip all non characters
        validPos = 0;
        while(uartRXByte(validPos) < 'A' && validPos <= bytesInUart() )
            validPos++;
        
        // Process Command
        if(validPos <= bytesInUart())
        {
            sendString("\n");
            
            // help command
            if(checkCommand(uartRXBytePointer(validPos),"help") == 1)
            {
                printCommandList();
            }
            else if(checkCommand(uartRXBytePointer(validPos),"assoc") == 1)
            {                
                // Disable RX interrupt so it does not interfere
                // with GUI's command
                UCA0IE &= ~UCRXIE;
                
                validPos += strlen("assoc");
                while(uartRXByte(validPos) < 'A' && validPos <= bytesInUart())
                    validPos++;
                
                *ptrSSIDInd = FRAM_FLAG_SET;
                memset((char *)ptrSSID, 0, MAX_SSID_LEN);
                memcpy(ptrSSID,uartRXBytePointer(validPos),MAX_SSID_LEN);
                
                sendString("OK\r\n");
                
                // If server is running, accept is blocking us. We can
                // therefore just restart the MSP430. Since the SSID indicator
                // has been set, at startup it will attempt to associate to the
                // AP requested by the user.
                if(currentCC3000State() & CC3000_SERVER_INIT)
                {  
                     terminalPrint("Restarting MSP430...\r\n");                  
                    restartMSP430();
                }
                else
                {
                    // Associate command
                    ConnectUsingSSID((char *)ptrSSID);
                }
                UCA0IE |= UCRXIE;                         // Enable RX interrupt
            }
            else if(checkCommand(uartRXBytePointer(validPos),"stat") == 1)
            {
                #ifdef    SENSOR_APP_VERSION
                    terminalPrint("Sensor App Version: ");
                    terminalPrint(SENSOR_APP_VERSION);
                    terminalPrint("\r\n");
                #endif
//#ifndef CC3000_TINY_DRIVER
//                printConnectionInfo(getCC3000Info());
//#endif
            }
            else if(checkCommand(uartRXBytePointer(validPos),"prfx") == 1)
            {
                // parameter sent with prfx should be 3 letters
                // that are the prefix. If they're not, we issue an error
                
                validPos += strlen("prfx");
                while(uartRXByte(validPos) < 'A' && validPos <= bytesInUart() )
                    validPos++;
                
                if(validPos <= bytesInUart())
                {
                    // Verify letters
                    if(isUppercaseChar(uartRXByte(validPos)) && 
                       isUppercaseChar(uartRXByte(validPos+1)) && 
                       isUppercaseChar(uartRXByte(validPos+2)))
                    {
                        // Wait for 3 characters from UART
                        ftcPrefixptr = (char *)(&aucCC3000_prefix[0]);
                        *ftcPrefixptr = uartRXByte(validPos);
                        
                        ftcPrefixptr = (char *)(&aucCC3000_prefix[1]);
                        *ftcPrefixptr = uartRXByte(validPos+1);
                        
                        
                        ftcPrefixptr = (char *)(&aucCC3000_prefix[2]);
                        *ftcPrefixptr = uartRXByte(validPos+2);
                        
                        // Send new prefix to CC3000
                        wlan_smart_config_set_prefix((char *)aucCC3000_prefix);
                        
                        char prfStr[4];
                        prfStr[0] = aucCC3000_prefix[0];
                        prfStr[1] = aucCC3000_prefix[1];
                        prfStr[2] = aucCC3000_prefix[2];
                        prfStr[3] = '\0';
                        
                        turnLedOff(CC3000_UNUSED1_IND);
                        sendString("\r\nSmart Config Prefix changed to: ");
                        sendString (prfStr);
                        sendString("\r\n");
                    }
                    else
                    {
                        sendString("Prefix Error");
                    }
                    
                }
                else
                {
                    sendString("Prefix Error");   
                }                        
            }
            else
            {
                sendString("Invalid or incomplete command. Type help for command list");
            }
        }
        
        // Send '>'
        sendString("\r\n> ");
        resetUARTBuffer();
        break;
        
    }
}

//*****************************************************************************
//
//!
//!  \param  None
//!
//!  \return none
//!
//!  \brief   Checks whether a character is uppercase letter
//
//*****************************************************************************
char isUppercaseChar(char c)
{
    if(c >= 'A' && c <= 'Z')
        return 1;
    else
        return 0;    
}

//*****************************************************************************
//
//!
//!  \param  None
//!
//!  \return none
//!
//!  \brief   Prints Help menu with info on current commands 
//
//*****************************************************************************
void printCommandList()
{

    sendString("===================================");
    sendString("\r\nhelp - shows this menu ");
    sendString("\r\nassoc [XXXXX] - associates CC3000 to AP with SSID XXXXX");
    sendString("\r\n                Smart config info will be erased");
    sendString("\r\nprfx [XXX] - changes Smart Config prefix to XXX");
    sendString("\r\nstat - provides status information");
    sendString("\r\n");
}


//*****************************************************************************
//
//!  \brief   Prints an IP Address to UART, Little Endian Format
//!
//!  \param  ip is a pointer to a 4 byte array with IP octets
//!
//!  \return none
//!
//
//*****************************************************************************
void printIpAddr(char * ip)
{
    char str[20];
    memset(str,0,sizeof(str));
    itoa(ip[3],str,10);
    // Send First octet
    sendString(str);   
    
    sendString(".");
    memset(str,0,sizeof(str));
    itoa(ip[2],str,10);
    // Send Second octet
    sendString(str);
    sendString(".");
    memset(str,0,sizeof(str));
    itoa(ip[1],str,10);
    // Send Third octet
    sendString(str);
    sendString(".");
    memset(str,0,sizeof(str));
    itoa(ip[0],str,10);
    // Send Fourth octet
    sendString(str);    
}


//*****************************************************************************
//
//!  \brief   Prints the MAC Address to UART, Little Endian Format
//!  
//!  \param  mac is a pointer to a 6 byte array with the MAC address
//!
//!  \return none
//!  
//
//*****************************************************************************
void printMACAddr(char * mac)
{
    char str[25];
    memset(str,0,sizeof(str));
    
    itoa(mac[5],str,16);
    // Send First octet
    sendString(str);   
    
    sendString(":");
    memset(str,0,sizeof(str));
    itoa(mac[4],str,16);
    // Send Second octet
    sendString(str);
    sendString(":");
    memset(str,0,sizeof(str));
    itoa(mac[3],str,16);
    // Send Third octet
    sendString(str);
    sendString(":");
    memset(str,0,sizeof(str));
    itoa(mac[2],str,16);
    // Send Fourth octet
    sendString(str);
    sendString(":");
    memset(str,0,sizeof(str));
    itoa(mac[1],str,16);
    // Send Fourth octet
    sendString(str);
    sendString(":");
    memset(str,0,sizeof(str));
    itoa(mac[0],str,16);
    // Send Fourth octet
    sendString(str);    
}

//*****************************************************************************
//
//!  \brief   Prints the CC3000 Connection information
//!  
//!  \param  inf is a pointer to the Configuration Information to print
//!
//!  \return none
//!  
//
//*****************************************************************************
/*
void printConnectionInfo(tNetappIpconfigRetArgs * inf)
{    
    if(highestCC3000State() == CC3000_INIT)
    {
        sendString("CC3000 Initialized\r\n");
    }
    if(!(currentCC3000State() & CC3000_INIT))
    {
        sendString("Status: CC3000 Uninitialized\r\n");
    }
    if(currentCC3000State() & CC3000_ASSOC)
    {
        sendString("Connected to: ");
        sendString((char *)inf->uaSSID);
        sendString("\r\n");
    }
    if(currentCC3000State() & CC3000_IP_ALLOC)
    {
        sendString("CC3000 MAC: ");
        printMACAddr((char *)inf->uaMacAddr);
        sendString("\r\n");
        sendString("CC3000 IP: ");        
        printIpAddr((char *)inf->aucIP);
        sendString("\r\n");        
    }
    if(currentCC3000State() & CC3000_SERVER_INIT)
    {
        sendString("Server Initialized\r\n");
    }
    if( currentCC3000State() & CC3000_CLIENT_CONNECTED)
    {
        sendString("Client Connected\r\n");
    }
}
*/
//*****************************************************************************
//
//!  \brief   Obtains the SSID of the assoc terminal command
//!  
//!  \param none
//!
//!  \return none
//!  
//
//*****************************************************************************
unsigned char * getassocSSID()
{
    return ptrSSID;
}

//*****************************************************************************
//
//!  \brief   Indicate whether the assoc command has a valid SSID stored
//!  
//!  \param none
//!
//!  \return none
//!  
//
//*****************************************************************************
char isAssocSSIDValid()
{
    if(*ptrSSIDInd == FRAM_FLAG_SET)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

//*****************************************************************************
//
//!  \brief   Print a string using the terminal
//!  
//!  \param msg is a pointer to the null terminated string to be sent
//!
//!  \return none
//!  
//
//*****************************************************************************
void terminalPrint(char * msg)
{
    sendString(msg);
}

//*****************************************************************************
//
//!  \brief  Checks whether command in buffer is the command in str
//!  
//!  \param buf is a pointer to the buffer with the command to checked
//!  \param cmdStr is the command string used in the check
//!
//!  \return none
//!  
//
//*****************************************************************************
char checkCommand(char * buf, char * cmdStr)
{
    // Check that the command is correct and that it is not followed
    // by other characters except space
    if(memcmp(buf,cmdStr,strlen(cmdStr)-1) == 0 &&
       (strlen(buf) ==  strlen(cmdStr)|| buf[strlen(cmdStr)] == ' ' ))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

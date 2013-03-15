/*****************************************************************************
*
*  exosite_hal.c - Exosite hardware & environmenat adapation layer.
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
#include "exosite_hal.h"
#include "exosite_meta.h"

#include <socket.h>
#include <nvmem.h>
#include <string.h>
#include <evnt_handler.h>    // for socketaddr extern
#include <board.h>
#include <common.h>

// local defines
#ifdef __MSP430FR5739__
#pragma DATA_SECTION(exo_meta, ".exo_meta") //create a meta section in FRAM
char exo_meta[META_SIZE];
#elif __IAR_SYSTEMS_ICC__
#pragma location = "EXO_META"
__no_init char exo_meta[META_SIZE];
#endif

// local functions

// externs
extern sockaddr tSocketAddr;

// global variables


//*****************************************************************************
//
//! exoHAL_ReadHWMAC
//!
//!  \param  Interface Number (1 - WiFi), buffer to return hexadecimal MAC
//!
//!  \return None
//!
//!  \brief  Reads the MAC address from the hardware
//
//*****************************************************************************
void
exoHAL_ReadHWMAC(unsigned char if_nbr, unsigned char * macBuf)
{
  switch (if_nbr) {
    case IF_WIFI:
      nvmem_read(NVMEM_MAC_FILEID, 6, 0, (unsigned char *)macBuf);
      break;
    default:
      break;
  }
}


//*****************************************************************************
//
//! exoHAL_EnableNVMeta
//!
//!  \param  None
//!
//!  \return None
//!
//!  \brief  Enables meta non-volatile memory, if any
//
//*****************************************************************************
void
exoHAL_EnableMeta(void)
{
  return; //nothing to do on msp430 (FRAM is awesome)
}


//*****************************************************************************
//
//! exoHAL_EraseNVMeta
//!
//!  \param  None
//!
//!  \return None
//!
//!  \brief  Wipes out meta information - replaces with 0's.
//
//*****************************************************************************
void
exoHAL_EraseMeta(void)
{
  memset(exo_meta, 0, META_SIZE); //on msp430, simply set the region to 0
  return;
}


//*****************************************************************************
//
//! exoHAL_WriteMetaItem
//!
//!  \param  buffer - string buffer containing info to write to meta; len -
//!          size of string in bytes; offset - offset from base of meta
//!          location to store the item.
//!
//!  \return None
//!
//!  \brief  Stores information to the NV meta structure.
//
//*****************************************************************************
void
exoHAL_WriteMetaItem(unsigned char * buffer, unsigned char len, int offset)
{
  memcpy((char *)(exo_meta + offset), buffer, len); //on msp430, simply put the info into mem
  return;
}


//*****************************************************************************
//
//! exoHAL_ReadMetaItem
//!
//!  \param  buffer - string buffer containing info to write to meta; len -
//!          size of string in bytes; offset - offset from base of meta
//!          location to store the item.
//!
//!  \return None
//!
//!  \brief  Stores information to the NV meta structure.
//
//*****************************************************************************
void
exoHAL_ReadMetaItem(unsigned char * buffer, unsigned char len, int offset)
{
  memcpy(buffer, (char *)(exo_meta + offset), len); //on msp430, simply read the info from mem
  return;
}


//*****************************************************************************
//
//! exoHAL_SocketClose
//!
//!  \param  socket - socket handle
//!
//!  \return None
//!
//!  \brief  The function closes a socket
//
//*****************************************************************************
void
exoHAL_SocketClose(long socket)
{
  closesocket(socket);
}


//*****************************************************************************
//
//! exoHAL_SocketOpenTCP
//!
//!  \param  None
//!
//!  \return socket - socket handle
//!
//!  \brief  The function opens a TCP socket
//
//*****************************************************************************
long
exoHAL_SocketOpenTCP()
{
  return((long)socket(AF_INET, SOCK_STREAM, IPPROTO_TCP));
}


//*****************************************************************************
//
//! exoHAL_ServerConnect
//!
//!  \param  None
//!
//!  \return socket - socket handle
//!
//!  \brief  The function opens a TCP socket
//
//*****************************************************************************
long
exoHAL_ServerConnect(long sock)
{
  unsigned char server[META_SERVER_SIZE];

  tSocketAddr.sa_family = 2;


  exosite_meta_read(server, META_SERVER_SIZE, META_SERVER);

  //TODO - use DNS or check m2.exosite.com/ip to check for updates
  tSocketAddr.sa_data[0] = 0;   //server[4];//(port & 0xFF00) >> 8;
  tSocketAddr.sa_data[1] = 80;  //server[5];//(port & 0x00FF);
  tSocketAddr.sa_data[2] = 173; //server[0];//173;  // First octet of destination IP
  tSocketAddr.sa_data[3] = 255; //server[1];//255;  // Second Octet of destination IP
  tSocketAddr.sa_data[4] = 209; //server[2];//209;  // Third Octet of destination IP
  tSocketAddr.sa_data[5] =  28; //server[3];//28;   // Fourth Octet of destination IP

  return connect(sock, &tSocketAddr, sizeof(tSocketAddr));
}


//*****************************************************************************
//
//! exoHAL_SocketSend
//!
//!  \param  socket - socket handle; buffer - string buffer containing info to
//!          send; len - size of string in bytes;
//!
//!  \return Number of bytes sent
//!
//!  \brief  Sends data out the network interface
//
//*****************************************************************************
unsigned char
exoHAL_SocketSend(long socket, char * buffer, unsigned char len)
{
  return (unsigned char)send(socket, buffer, (long)len, 0); //always set flags to 0 for CC3000
}


//*****************************************************************************
//
//! exoHAL_SocketRecv
//!
//!  \param  socket - socket handle; buffer - string buffer to put info we
//!          receive; len - size of buffer in bytes;
//!
//!  \return Number of bytes received
//!
//!  \brief  Sends data out the network interface
//
//*****************************************************************************
unsigned char
exoHAL_SocketRecv(long socket, char * buffer, unsigned char len)
{
  return (unsigned char)recv(socket, buffer, (long)len, 0); //always set flags to 0 for CC3000
}


//*****************************************************************************
//
//! exoHAL_HandleError
//!
//!  \param  code - error code;
//!
//!  \return None
//!
//!  \brief  Handles errors in platform-specific way
//
//*****************************************************************************
void
exoHAL_HandleError(unsigned char code)
{
  switch (code) {
    case EXO_ERROR_WRITE:
      errorHandler(ERROR_WRITE_FAILURE);
      break;
    case EXO_ERROR_CONNECT:
      errorHandler(ERROR_CONNECT_FAILURE);
      break;
    default:
      errorHandler(ERROR_UNKNOWN);
      break;
  }

  return;
}


//*****************************************************************************
//
//! exoHAL_ShowUIMessage
//!
//!  \param  code - UI code for message to display;
//!
//!  \return None
//!
//!  \brief  Displays message in a platform specific way
//
//*****************************************************************************
void
exoHAL_ShowUIMessage(unsigned char code)
{
  switch (code) {
    case EXO_SERVER_CONNECTED:
      turnLedOn(CC3000_SERVER_INIT_IND);
      break;
    case EXO_CLIENT_RW:
      turnLedOn(CC3000_CLIENT_CONNECTED_IND);
      break;
    default:
      break;
  }

  return;
}


//*****************************************************************************
//
//! exoHAL_MSDelay
//!
//!  \param  delay - milliseconds to delay
//!
//!  \return None
//!
//!  \brief  Delays for specified milliseconds
//
//*****************************************************************************
void
exoHAL_MSDelay(unsigned short delay)
{
  busyWait(delay);
  return;
}


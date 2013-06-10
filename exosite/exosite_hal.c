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
#include "exosite.h"
#include "exosite_hal.h"
//#include "exosite_meta.h"

#include <socket.h>
#include <nvmem.h>
#include <string.h>
#include <evnt_handler.h>    // for socketaddr extern
#include <board.h>
#include <common.h>

// local defines

// local functions

// externs
extern sockaddr tSocketAddr;

// global variables


/*****************************************************************************
*
*   exoHAL_ReadUUID
*
*   \param  Interface Number (1 - WiFi), buffer to return hexadecimal MAC
*
*   \return 0 if failure; length of UUID if success;
*
*   \brief  Reads the MAC address from the hardware
*
*****************************************************************************/
int
exoHAL_ReadUUID(unsigned char if_nbr, unsigned char * UUID_buf)
{
  int retval = 0;
  const char hex[] = "0123456789abcdef";

  unsigned char macBuf[10];

  switch (if_nbr) {
    case IF_GPRS:
      break;
    case IF_ENET:
      break;
    case IF_WIFI:
      nvmem_read(NVMEM_MAC_FILEID, 6, 0, (unsigned char *)macBuf);

      UUID_buf[0]  = hex[macBuf[0] >> 4];
      UUID_buf[1]  = hex[macBuf[0] & 15];
      UUID_buf[2]  = hex[macBuf[1] >> 4];
      UUID_buf[3]  = hex[macBuf[1] & 15];
      UUID_buf[4]  = hex[macBuf[2] >> 4];
      UUID_buf[5]  = hex[macBuf[2] & 15];
      UUID_buf[6]  = hex[macBuf[3] >> 4];
      UUID_buf[7]  = hex[macBuf[3] & 15];
      UUID_buf[8]  = hex[macBuf[4] >> 4];
      UUID_buf[9]  = hex[macBuf[4] & 15];
      UUID_buf[10] = hex[macBuf[5] >> 4];
      UUID_buf[11] = hex[macBuf[5] & 15];

      UUID_buf[12] = 0;
      retval = strlen((char *)UUID_buf);
      break;
    default:
      break;
  }

  return retval;
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
exoHAL_SocketOpenTCP(void)
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
  long retval;

  tSocketAddr.sa_family = 2;

  //TODO - use DNS or check m2.exosite.com/ip to check for updates
  tSocketAddr.sa_data[0] = 0;   //server[4];//(port & 0xFF00) >> 8;
  tSocketAddr.sa_data[1] = 80;  //server[5];//(port & 0x00FF);
  tSocketAddr.sa_data[2] = 173; //server[0];//173;  // First octet of destination IP
  tSocketAddr.sa_data[3] = 255; //server[1];//255;  // Second Octet of destination IP
  tSocketAddr.sa_data[4] = 209; //server[2];//209;  // Third Octet of destination IP
  tSocketAddr.sa_data[5] =  28; //server[3];//28;   // Fourth Octet of destination IP

  retval = connect(sock, &tSocketAddr, sizeof(tSocketAddr));

  if (retval >= 0)
    turnLedOn(CC3000_SERVER_INIT_IND);

  return retval;
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
  int result;
  result = send(socket, buffer, (long)len, 0); //always set flags to 0 for CC3000
  hci_unsolicited_event_handler();
  return  (unsigned char)result;
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
/*
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
*/



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


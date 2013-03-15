/*****************************************************************************
*
*  exosite.c - Exosite cloud communications.
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

#include <string.h>

//local defines
#define RX_SIZE 50
#define CIK_LENGTH 40
#define MAC_LEN 6

typedef enum
{
  CIK_LINE,
  HOST_LINE,
  CONTENT_LINE,
  ACCEPT_LINE,
  LENGTH_LINE,
  GETDATA_LINE,
  POSTDATA_LINE,
  VENDOR_LINE,
  EMPTY_LINE
} lineTypes;

#define STR_CIK_HEADER "X-Exosite-CIK: "
#define STR_CONTENT_LENGTH "Content-Length: "
#define STR_GET_URL "GET /api:v1/stack/alias?"
#define STR_HTTP "  HTTP/1.1\r\n"
#define STR_HOST "Host: m2.exosite.com\r\n"
#define STR_ACCEPT "Accept: application/x-www-form-urlencoded; charset=utf-8\r\n"
#define STR_CONTENT "Content-Type: application/x-www-form-urlencoded; charset=utf-8\r\n"
#define STR_VENDOR "vendor=exosite&model=cc3000wifismartconfig&sn="
#define STR_CRLF "\r\n"

// local functions
void activate_device(void);
void init_mac_address(unsigned char if_nbr);
void update_m2ip(void);
int readResponse(long socket, char * expectedCode);
long connect_to_exosite();
void sendLine(long socket, unsigned char LINE, char * payload);

// global functions
int Exosite_Write(char * pbuf, unsigned char bufsize);
int Exosite_Read(char * palias, char * pbuf, unsigned char bufsize);
int Exosite_Init(void);
int Exosite_ReInit(void);
void Exosite_SetCIK(char * pCIK);

// externs
extern char *itoa(int n, char *s, int b);

// global variables
static unsigned char exositeWriteFailures = 0;


//*****************************************************************************
//
//! Exosite_Init
//!
//!  \param  None
//!
//!  \return 0 success; -1 failure
//!
//!  \brief  The function initializes the cloud connection to Exosite
//
//*****************************************************************************
int
Exosite_Init(void)
{
  exosite_meta_init();          //always initialize our meta structure
  init_mac_address(IF_WIFI);    //always check to see if the MAC is up to date

  //setup some of our globals for operation
  exositeWriteFailures = 0;

  return Exosite_ReInit();
}


//*****************************************************************************
//
//! Exosite_ReInit
//!
//!  \param  None
//!
//!  \return 0 success; -1 failure
//!
//!  \brief  Called after Init has been ran in the past, but maybe comms were
//!          down and we have to keep trying...
//
//*****************************************************************************
int
Exosite_ReInit(void)
{
  update_m2ip();        //check our IP api to see if the old IP is advertising a new one
  activate_device();    //the moment of truth - can this device provision with the Exosite cloud?...

  {
    char tempCIK[CIK_LENGTH];
    unsigned char i;
    //sanity check on the CIK
    exosite_meta_read((unsigned char *)tempCIK, CIK_LENGTH, META_CIK);
    for (i = 0; i < CIK_LENGTH; i++)
    {
      if (!(tempCIK[i] >= 'a' && tempCIK[i] <= 'f' || tempCIK[i] >= '0' && tempCIK[i] <= '9'))
      {
        return -1;
      }
    }
  }

  return 0;
}


//*****************************************************************************
//
//! Exosite_SetCIK
//!
//!  \param  pointer to CIK
//!
//!  \return None
//!
//!  \brief  Programs a new CIK to flash / non volatile
//
//*****************************************************************************
void
Exosite_SetCIK(char * pCIK)
{
  exosite_meta_write((unsigned char *)pCIK, CIK_LENGTH, META_CIK);
}


//*****************************************************************************
//
//! Exosite_Write
//!
//!  \param  pbuf - string buffer containing data to be sent
//!          bufsize - number of bytes to send
//!
//!  \return 0 success; -1 failure
//!
//!  \brief  The function writes data to Exosite
//
//*****************************************************************************
int
Exosite_Write(char * pbuf, unsigned char bufsize)
{
  char strBuf[10];
  long sock = -1;

  while (sock < 0)
    sock = connect_to_exosite();

// This is an example write POST...
//  s.send('POST /api:v1/stack/alias HTTP/1.1\r\n')
//  s.send('Host: m2.exosite.com\r\n')
//  s.send('X-Exosite-CIK: 5046454a9a1666c3acfae63bc854ec1367167815\r\n')
//  s.send('Content-Type: application/x-www-form-urlencoded; charset=utf-8\r\n')
//  s.send('Content-Length: 6\r\n\r\n')
//  s.send('temp=2')

  itoa((int)bufsize, strBuf, 10); //make a string for length

  sendLine(sock, POSTDATA_LINE, "/api:v1/stack/alias");
  sendLine(sock, HOST_LINE, NULL);
  sendLine(sock, CIK_LINE, NULL);
  sendLine(sock, CONTENT_LINE, NULL);
  sendLine(sock, LENGTH_LINE, strBuf);
  exoHAL_SocketSend(sock, pbuf, bufsize); //alias=value

  if (0 == readResponse(sock, "204")) {
    exositeWriteFailures = 0;
  } else exositeWriteFailures++;

  exoHAL_SocketClose(sock);

  if (exositeWriteFailures > 5) {
    // sometimes transport connect works even if no connection...
    exoHAL_HandleError(EXO_ERROR_WRITE);
  }

  if (!exositeWriteFailures) {
    exoHAL_ShowUIMessage(EXO_CLIENT_RW);
    return 0; // success
  }

  return -1;
}    


//*****************************************************************************
//
//! Exosite_Read
//!
//!  \param  palias - string, name of the datasource alias to read from
//!          pbuf - read buffer to put the read response into
//!          buflen - size of the input buffer
//!
//!  \return number of bytes read
//!
//!  \brief  The function reads data from Exosite
//
//*****************************************************************************
int
Exosite_Read(char * palias, char * pbuf, unsigned char buflen)
{
  unsigned char strLen, len, vlen;
  char *p, *pcheck;
  long sock = -1;

  while (sock < 0)
    sock = connect_to_exosite();

// This is an example read GET
//  s.send('GET /api:v1/stack/alias?temp HTTP/1.1\r\n')
//  s.send('Host: m2.exosite.com\r\n')
//  s.send('X-Exosite-CIK: 5046454a9a1666c3acfae63bc854ec1367167815\r\n')
//  s.send('Accept: application/x-www-form-urlencoded; charset=utf-8\r\n\r\n')

  sendLine(sock, GETDATA_LINE, palias);
  sendLine(sock, HOST_LINE, NULL);
  sendLine(sock, CIK_LINE, NULL);
  sendLine(sock, ACCEPT_LINE, "\r\n");

  pcheck = palias;
  vlen = 0;

  if (0 == readResponse(sock, "200"))
  {
    char strBuf[RX_SIZE];
    unsigned char crlf = 0;
    exoHAL_ShowUIMessage(EXO_CLIENT_RW);
    do
    {
      strLen = exoHAL_SocketRecv(sock, strBuf, RX_SIZE);
      len = strLen;
      p = strBuf;

      // Find 4 consecutive \r or \n - should be: \r\n\r\n
      while (0 < len && 4 > crlf)
      {
        if ('\r' == *p || '\n' == *p)
        {
          ++crlf;
        }
        else
        {
          crlf = 0;
        }
        ++p;
        --len;
      }

      // The body is "<key>=<value>"
      if (0 < len && 4 == crlf && buflen > vlen)
      {
        // Move past "<key>"
        while (0 < len && 0 != *pcheck)
        {
          if (*pcheck == *p)
          {
            ++pcheck;
          }
          else
          {
            pcheck = palias;
          }
          ++p;
          --len;
        }

        // Match '=',  we should now have '<key>='
        if (0 < len && 0 == *pcheck && '=' == *p)
        {
          ++p;
          --len;
        }

        // read in the rest of the body as the value
        while (0 < len && buflen > vlen)
        {
          pbuf[vlen++] = *p++;
          --len;
        }
      }
    } while (RX_SIZE == strLen);
  }

  exoHAL_SocketClose(sock);

  return vlen;
}


//*****************************************************************************
//
//! activate_device
//!
//!  \param  none
//!
//!  \return none
//!
//!  \brief  Calls activation API - if successful, it saves the returned
//!          CIK to non-volatile
//
//*****************************************************************************
void
activate_device(void)
{
  long sock = -1;
  volatile int length;
  char strLen[5];

  while (sock < 0)
    sock = connect_to_exosite();

  length = strlen(STR_VENDOR) + META_UUID_SIZE;
  itoa(length, strLen, 10); //make a string for length

  sendLine(sock, POSTDATA_LINE, "/provision/activate");
  sendLine(sock, HOST_LINE, NULL);
  sendLine(sock, CONTENT_LINE, NULL);
  sendLine(sock, LENGTH_LINE, strLen);
  sendLine(sock, VENDOR_LINE, NULL);

  if (0 == readResponse(sock, "200"))
  {
    char strBuf[RX_SIZE];
    unsigned char strLen, len;
    char *p;
    unsigned char crlf = 0;
    unsigned char ciklen = 0;
    char NCIK[CIK_LENGTH];

    do
    {
      strLen = exoHAL_SocketRecv(sock, strBuf, RX_SIZE);
      len = strLen;
      p = strBuf;

      // Find 4 consecutive \r or \n - should be: \r\n\r\n
      while (0 < len && 4 > crlf)
      {
        if ('\r' == *p || '\n' == *p)
        {
          ++crlf;
        }
        else
        {
          crlf = 0;
        }
        ++p;
        --len;
      }

      // The body is the CIK
      if (0 < len && 4 == crlf && CIK_LENGTH > ciklen)
      {
        // TODO, be more robust - match Content-Length header value to CIK_LENGTH
        unsigned char need, part;
        need = CIK_LENGTH - ciklen;
        part = need < len ? need : len;
        strncpy(NCIK + ciklen, p, part);
        ciklen += part;
      }
    } while (RX_SIZE == strLen);

    if (CIK_LENGTH == ciklen)
    {
      Exosite_SetCIK(NCIK);
    }
  }

  exoHAL_SocketClose(sock);
}


//*****************************************************************************
//
//! update_m2ip
//!
//!  \param  none
//!
//!  \return none
//!
//!  \brief  Checks /ip API to see if a new server IP address should be used
//
//*****************************************************************************
void
update_m2ip(void)
{
  //TODO - stubbed out
  return;
}


//*****************************************************************************
//
//! init_mac_address
//!
//!  \param  Interface Number (1 - WiFi)
//!
//!  \return None
//!
//!  \brief  Reads the MAC address from the hardware
//
//*****************************************************************************
void init_mac_address(unsigned char if_nbr)
{
  const char hex[] = "0123456789abcdef";
  char strmac[12];
  unsigned char addr_hw[MAC_LEN];

  exoHAL_ReadHWMAC(if_nbr, addr_hw);

  strmac[0]  = hex[addr_hw[0] >> 4];
  strmac[1]  = hex[addr_hw[0] & 15];
  strmac[2]  = hex[addr_hw[1] >> 4];
  strmac[3]  = hex[addr_hw[1] & 15];
  strmac[4]  = hex[addr_hw[2] >> 4];
  strmac[5]  = hex[addr_hw[2] & 15];
  strmac[6]  = hex[addr_hw[3] >> 4];
  strmac[7]  = hex[addr_hw[3] & 15];
  strmac[8]  = hex[addr_hw[4] >> 4];
  strmac[9]  = hex[addr_hw[4] & 15];
  strmac[10] = hex[addr_hw[5] >> 4];
  strmac[11] = hex[addr_hw[5] & 15];

  exosite_meta_write((unsigned char *)strmac, 12, META_UUID);
}


//*****************************************************************************
//
//! connect_to_exosite
//!
//!  \param  None
//!
//!  \return socket handle
//!
//!  \brief  Establishes a connection with the Exosite API server
//
//*****************************************************************************
long
connect_to_exosite(void)
{    
  static unsigned char connectRetries = 0;
  long sock;

  if (connectRetries++ > 5) {
    connectRetries = 0;
    exoHAL_HandleError(EXO_ERROR_CONNECT);
  }

  sock = exoHAL_SocketOpenTCP();

  if (sock == -1)
  {
    //wlan_stop();  //TODO - if we stop the wlan, we have to recover somehow...
    exoHAL_MSDelay(100);
    return -1;
  }

  if (exoHAL_ServerConnect(sock) < 0)  // Try to connect
  {
    // TODO - the typical reason the connect doesn't work is because
    // something was wrong in the way the CC3000 was initialized (timing, bit
    // error, etc...). There may be a graceful way to kick th CC3000 module
    // back into gear at the right state, but for now, we just
    // return and let the caller retry us if they want
    exoHAL_MSDelay(100);
    return -1;
  } else {
    connectRetries = 0;
    exoHAL_ShowUIMessage(EXO_SERVER_CONNECTED);
  }

  // Success
  return sock;
}


//*****************************************************************************
//
//! readResponse
//!
//!  \param  socket handle, pointer to expected HTTP response code
//!
//!  \return 0 if match, -1 if no match
//!
//!  \brief  Reads first 12 bytes of HTTP response and extracts the 3 byte code
//
//*****************************************************************************
int
readResponse(long socket, char * code)
{
  char rxBuf[12];
  unsigned char rxLen;

  rxLen = exoHAL_SocketRecv(socket, rxBuf, 12);

  if (12 == rxLen && code[0] == rxBuf[9] && code[1] == rxBuf[10] && code[2] == rxBuf[11])
  {
    return 0;
  }

  return -1;
}


//*****************************************************************************
//
//! sendLine
//!
//!  \param  Which line type
//!
//!  \return socket handle
//!
//!  \brief  Sends data out the socket
//
//*****************************************************************************
void
sendLine(long socket, unsigned char LINE, char * payload)
{
  char strBuf[80];
  unsigned char strLen;

  switch(LINE) {
    case CIK_LINE:
      strLen = 15;
      memcpy(strBuf,STR_CIK_HEADER,strLen);
      exosite_meta_read((unsigned char *)&strBuf[strLen], CIK_LENGTH, META_CIK);
      strLen += CIK_LENGTH;
      memcpy(&strBuf[strLen],STR_CRLF, 2);
      strLen += 2;
      break;
    case HOST_LINE:
      strLen = 22;
      memcpy(strBuf,STR_HOST,strLen);
      break;
    case CONTENT_LINE:
      strLen = 64;
      memcpy(strBuf,STR_CONTENT,strLen);
      break;
    case ACCEPT_LINE:
      strLen = 58;
      memcpy(strBuf,STR_ACCEPT,strLen);
      memcpy(&strBuf[strLen],payload, strlen(payload));
      strLen += strlen(payload);
      break;
    case LENGTH_LINE: // Content-Length: NN
      strLen = 16;
      memcpy(strBuf,STR_CONTENT_LENGTH,strLen);
      memcpy(&strBuf[strLen],payload, strlen(payload));
      strLen += strlen(payload);
      memcpy(&strBuf[strLen],STR_CRLF, 2);
      strLen += 2;
      memcpy(&strBuf[strLen],STR_CRLF, 2);
      strLen += 2;
      break;
    case GETDATA_LINE:
      strLen = 24;
      memcpy(strBuf,STR_GET_URL,strLen);
      memcpy(&strBuf[strLen],payload, strlen(payload));
      strLen += strlen(payload);
      memcpy(&strBuf[strLen],STR_HTTP, 12);
      strLen += 12;
      break;
    case VENDOR_LINE:
      // OS Name = "MSP430_CC3000_CLOUD" <- MAX Length = 24 (unused)
      // OS Ver  = "0.0.1" <- MAX Length = 8 (unused)
      strLen = strlen(STR_VENDOR);
      memcpy(strBuf, STR_VENDOR, strLen);
      exosite_meta_read((unsigned char *)&strBuf[strLen], META_UUID_SIZE, META_UUID);
      strLen += META_UUID_SIZE;
      break;
    case POSTDATA_LINE:
      strLen = 5;
      memcpy(strBuf,"POST ", strLen);
      memcpy(&strBuf[strLen],payload, strlen(payload));
      strLen += strlen(payload);
      memcpy(&strBuf[strLen],STR_HTTP, 12);
      strLen += 12;
      break;
    case EMPTY_LINE:
      strLen = 2;
      memcpy(strBuf,STR_CRLF,strLen);
      break;
    default:
      break;
  }
  exoHAL_SocketSend(socket, strBuf, strLen);
  return;

}


/*****************************************************************************
*
*  demo_config.h - Sensor Demo Configuration File
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

#ifndef DEMO_CONFIG_H
#define DEMO_CONFIG_H

//
// Modify the following settings as necessary to run the demo
//
#define USE_DHCP      1
#define USE_STATIC_IP 2

#define NONE 0
#define WEP  1
#define WPA  2
#define WPA2 3

#define IP_ALLOC_METHOD USE_DHCP

// Default SSID Settings
#define DEFAULT_OUT_OF_BOX_SSID       "exosite_demo_wpa"
#define AP_SECURITY                   WPA
#define AP_KEY                        "ex0s1t3wpamix"
#define CIK                           "d971e58d6d2e2d6394cbc2b53a7f59412e2d9e9e"

#if IP_ALLOC_METHOD == USE_STATIC_IP
#define STATIC_IP_OCT1 192
#define STATIC_IP_OCT2 168
#define STATIC_IP_OCT3 1
#define STATIC_IP_OCT4 104
#endif

#define SERVER_PORT 1204
#endif

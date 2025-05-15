/*
 * Copyright (c) 2001-2003 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Simon Goldschmidt
 *
 */
#ifndef LWIP_HDR_LWIPOPTS_H__
#define LWIP_HDR_LWIPOPTS_H__

#define WITH_RTOS                     1
#define CHECKSUM_BY_HARDWARE          0

#define TCPIP_MBOX_SIZE               64
#define TCPIP_THREAD_STACKSIZE        4096
#define TCPIP_THREAD_PRIO             5
#define DEFAULT_THREAD_PRIO           5
#define DEFAULT_THREAD_STACKSIZE      4096
#define DEFAULT_RAW_RECVMBOX_SIZE     32
#define DEFAULT_UDP_RECVMBOX_SIZE     64
#define DEFAULT_TCP_RECVMBOX_SIZE     64
#define DEFAULT_ACCEPTMBOX_SIZE       32

#define LWIP_NETIF_LOOPBACK           1
#define LWIP_HAVE_LOOPIF              1
#define LWIP_LOOPBACK_MAX_PBUFS       0

#define LWIP_CHKSUM_ALGORITHM         3
#define LWIP_TCPIP_CORE_LOCKING_INPUT 1

//#define PBUF_LINK_ENCAPSULATION_HLEN  388

#define MEM_MIN_TCP                   1024
#define MEM_MIN                       MEM_MIN_TCP
#define MEM_ALIGNMENT                 4

#ifdef LWIP_HEAP_SIZE
#define MEM_SIZE LWIP_HEAP_SIZE
#else
#if MEM_MIN > 8192
#define MEM_SIZE MEM_MIN
#else
#define MEM_SIZE 8192
#endif
#endif

#define LWIP_RAW                      1
#define LWIP_ICMP                     1
#define LWIP_ARP                      1
#define LWIP_MULTICAST_TX_OPTIONS     1

#define LWIP_DNS_SERVER               0
#define LWIP_DNS                      1
#define LWIP_SOCKET                   1
#define LWIP_NETDB                    1
#define LWIP_IPV4                     1
#define LWIP_IPV6                     0   // or 1 if you're using IPv6 too

#define LWIP_DHCP                     1
#define LWIP_SO_RCVTIMEO              1
#define LWIP_SO_SNDTIMEO              1
#define SO_REUSE                      1
#define LWIP_TCP_KEEPALIVE            1

#define LWIP_TIMEVAL_PRIVATE          0 // use sys/time.h for struct timeval
// #define LWIP_PROVIDE_ERRNO         1
#define LWIP_ERRNO_STDINCLUDE         1
#define LWIP_SOCKET_SET_ERRNO         1

#define LWIP_NETIF_STATUS_CALLBACK    0
#define LWIP_NETIF_API                1
#define ETHARP_SUPPORT_STATIC_ENTRIES 1

#define LWIP_DEBUG                    1

#define SOCKETS_DEBUG  LWIP_DBG_OFF
#ifdef BL616_DHCP_DEBUG
#define DHCP_DEBUG LWIP_DBG_ON
#else
#define DHCP_DEBUG LWIP_DBG_OFF
#endif

extern int *__errno(void);
#define errno                     (*__errno())
#define LWIP_RAND()               ((u32_t)random())

#endif /* LWIP_HDR_LWIPOPTS_H__ */

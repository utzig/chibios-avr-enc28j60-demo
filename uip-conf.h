#ifndef __UIP_CONF_H__
#define __UIP_CONF_H__

#include <stdint.h>

#define PACK_STRUCT_FIELD(x) x __attribute__((packed))
#define PACK_STRUCT_STRUCT __attribute__((packed))
#define PACK_STRUCT_BEGIN
#define PACK_STRUCT_END

typedef uint8_t u8_t;
typedef uint16_t u16_t;
typedef unsigned short uip_stats_t;

#define UIP_FIXEDETHADDR          1
#define UIP_ETHADDR0              0xC2
#define UIP_ETHADDR1              0xAF
#define UIP_ETHADDR2              0x51
#define UIP_ETHADDR3              0x03
#define UIP_ETHADDR4              0xCF
#define UIP_ETHADDR5              0x46

#define UIP_FIXEDADDR             1
#define UIP_IPADDR0               10
#define UIP_IPADDR1               1
#define UIP_IPADDR2               1
#define UIP_IPADDR3               10
#define UIP_DRIPADDR0             10
#define UIP_DRIPADDR1             1
#define UIP_DRIPADDR2             1
#define UIP_DRIPADDR3             1
#define UIP_NETMASK0              255
#define UIP_NETMASK1              255
#define UIP_NETMASK2              255
#define UIP_NETMASK3              0

#define UIP_CONF_IPV6             0
#define UIP_CONF_MAX_CONNECTIONS  0
#define UIP_CONF_MAX_LISTENPORTS  0
#define UIP_CONF_BUFFER_SIZE      700
#define UIP_CONF_BYTE_ORDER       UIP_LITTLE_ENDIAN
#define UIP_CONF_LOGGING          0
#define UIP_CONF_UDP              0
#define UIP_CONF_UDP_CHECKSUMS    0
#define UIP_CONF_STATISTICS       0

/*#include "smtp.h"*/
#include "hello-world.h"
/*#include "telnetd.h"*/
/*#include "webserver.h"*/
/*#include "dhcpc.h"*/
/*#include "resolv.h"*/
/*#include "webclient.h"*/

#endif

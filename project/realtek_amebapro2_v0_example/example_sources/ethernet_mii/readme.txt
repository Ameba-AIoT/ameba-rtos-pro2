Example Description

This example show how to use ethernet driver.

To initialize ethernet driver, we need to update following configurations in platform_opts.h

#define CONFIG_ETHERNET 1
#if CONFIG_ETHERNET
//Choice the different interface for Ethernet
#define MII_INTERFACE 1
#define USB_INTERFACE 0
#define ETHERNET_INTERFACE MII_INTERFACE

If setup correctly, it should shows "Ethernet_mii Init done" with IP address provided by connected AP.

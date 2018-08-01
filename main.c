#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>

#include <OMM_machine_common.h>
#include <spi_common.h>

#include "main.h"
#include "lwip/opt.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/timeouts.h"
#include "netif/etharp.h"
#include "tcp_echoserver.h"
#include "app_ethernet.h"
#include "app_ping.h"
#include "testapp.h"
#include "mchdrv.h"
#include "enc28j60.h"

struct ip4_addr mch_myip_addr;
struct ip4_addr gw_addr;
struct ip4_addr netmask;

static struct netif mchdrv_netif;

static enc_device_t mchdrv_hw;

void mch_net_init(void)
{

    IP4_ADDR(&mch_myip_addr, IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
    IP4_ADDR(&netmask, NETMASK_ADDR0, NETMASK_ADDR1 , NETMASK_ADDR2, NETMASK_ADDR3);
    IP4_ADDR(&gw_addr, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);

    // Initialize LWIP
    lwip_init();

    mchdrv_netif.hwaddr_len = 6;
    /* demo mac address */
    mchdrv_netif.hwaddr[0] = 0;
    mchdrv_netif.hwaddr[1] = 1;
    mchdrv_netif.hwaddr[2] = 2;
    mchdrv_netif.hwaddr[3] = 3;
    mchdrv_netif.hwaddr[4] = 4;
    mchdrv_netif.hwaddr[5] = 5;

    // Add our netif to LWIP (netif_add calls our driver initialization function)
    if (netif_add(&mchdrv_netif, &mch_myip_addr, &netmask, &gw_addr, &mchdrv_hw,
                mchdrv_init, ethernet_input) == NULL) {
        LWIP_ASSERT("mch_net_init: netif_add (mchdrv_init) failed\n", 0);
    }

    netif_set_default(&mchdrv_netif);
    netif_set_up(&mchdrv_netif);
}

void mch_net_poll(void)
{
    mchdrv_poll(&mchdrv_netif);
}

uint32_t sys_now(void)
{
  return OMM_get_clktick()/100;
}

spi_device_t *enc28j60_spi = NULL;

int main(void)
{
    OMM_machine_t *machine = machine_setup();
    enc28j60_spi = OMM_get_pdev_by_name(machine, "enc28j60_spi");

#if 1
    mch_net_init();

    testapp_init();
    //tcp_echoserver_init();
    //ping_init();

    printf("Setup completed\n");

    while (1) {
        mch_net_poll();
        sys_check_timeouts();
        //ping_send_now();
        //printf("%s: get tick: %lu\n", machine->name, OMM_get_clktick()/100);
        //OMM_busy_delay(1000);
    }
#endif

#if 0
    uint8_t i = 0;
    char msg[] = {0xff, 0xff, 0xff, 0xff};
    //char msg[] = {0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9};
    char rec_msg[sizeof(msg)] = {0};

    //calibrate();

    while(1)
    {
        printf("%s:send spi %d get tick: %lu\n", machine->name, i++, OMM_get_clktick()/100);
        //SPI_transfer_byte(enc28j60_spi, 0xA);
        //SPI_transfer_msg(enc28j60_spi, (uint8_t *)msg, (uint8_t *)rec_msg, sizeof(msg)-1);
        SPI_set_cs(enc28j60_spi, SPI_COMMON_LOW);
        SPI_transfer_msg(enc28j60_spi, NULL, (uint8_t *)rec_msg, sizeof(msg));
        //SPI_transfer_byte_cs_off(enc28j60_spi, 0x10);
        //SPI_transfer_byte_cs_off(enc28j60_spi, 0x00);
        //SPI_transfer_byte_cs_off(enc28j60_spi, 0x00);
        //SPI_transfer_byte_cs_off(enc28j60_spi, 0x00);
        //SPI_transfer_byte_cs_off(enc28j60_spi, 0x00);
        SPI_set_cs(enc28j60_spi, SPI_COMMON_HIGH);
        printf("%s:receive spi %d %d %d %d\n", machine->name, rec_msg[0], rec_msg[1], rec_msg[2], rec_msg[3]);
        //up_udelay(1000000);
        OMM_busy_delay(1000);
    }
#endif

}


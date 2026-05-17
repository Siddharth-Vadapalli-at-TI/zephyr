.. zephyr:code-sample:: ti_am62l_evm_cpsw_ethernet
   :name: AM62L EVM CPSW Ethernet

   Enable Ethernet functionality on the AM62L EVM with CPSW3G.

Overview
********

This sample demonstrates basic Ethernet functionality with the 1 Gbps
capable MAC Ports of CPSW3G on the AM62L EVM.

Requirements
************

* TI AM62L EVM board
* Ethernet cable connected to any or both of the RJ45 connectors on the EVM

Building and Running
********************

.. zephyr-app-commands::
   :zephyr-app: samples/boards/ti/am62l_evm/cpsw_ethernet
   :board: am62l_evm_am62l3_a53
   :goals: build

After building the sample copy it to the boot partition of the bootable SD Card
and perform the following:

1. Connect to the UART terminal for the EVM and power on the board
2. Halt at U-Boot prompt and load and start Zephyr
3. Various logs containing 'eth_ti_am62l_cpsw' will be displayed on the console
4. At the prompt, run `net iface` command to view the active interfaces
5. Identify the CPSW interfaces (8000000.ethernet) and assign an IPv4 address
   to them using:
   net ipv4 add <interface_index> <ipv4_address> <netmask>
6. Ping any device on the network to verify Ethernet functionality using:
   net ping <ipv4_address_of_remote_device_on_network>
7. Network performance can be measured using zperf command

Sample Output
=============

.. code-block:: console

   *** Booting Zephyr OS build v4.4.0-rc1-3318-g8e98d854e377 ***
   Secondary CPU core 1 (MPID:0x1) is up
   [00:00:03.017,000] <inf> eth_ti_am62l_cpsw: MAC port 1: link up 1000 Mbps full-duplex
   [00:00:06.524,000] <inf> eth_ti_am62l_cpsw: MAC port 2: link up 1000 Mbps full-duplex
   uart:~$
   uart:~$ net iface
   Default interface: 1


   Interface eth0 (0x82056d50) (Ethernet) [1]
   ===================================
   Link addr : 44:88:BE:8B:E4:9D
   MTU       : 1500
   Flags     : AUTO_START,IPv4,IPv6
   Device    : ethernet@8000000 port 1 (0x82043d80)
   Status    : oper=UP, admin=UP, carrier=ON
   Ethernet capabilities supported:
           10 Mbits
           100 Mbits
           1 Gbits
   Ethernet PHY device: ethernet-phy@0 (0x82043ce0)
   Ethernet link speed: 1 Gbits full-duplex
   IPv6 unicast addresses (max 2):
           fe80::4688:beff:fe8b:e49d autoconf preferred infinite
   IPv6 multicast addresses (max 3):
           ff02::1
           ff02::1:ff8b:e49d
   IPv6 prefixes (max 2):
           <none>
   IPv6 hop limit           : 64
   IPv6 base reachable time : 30000
   IPv6 reachable time      : 39182
   IPv6 retransmit timer    : 0
   IPv4 unicast addresses (max 1):
           <none>
   IPv4 multicast addresses (max 2):
           224.0.0.1
   IPv4 gateway : 0.0.0.0
   DHCPv4 state      : disabled

   Interface eth1 (0x82056e00) (Ethernet) [2]
   ===================================
   Link addr : BA:67:F9:0A:AD:54
   MTU       : 1500
   Flags     : AUTO_START,IPv4,IPv6
   Device    : ethernet@8000000 port 2 (0x82043d30)
   Status    : oper=UP, admin=UP, carrier=ON
   Ethernet capabilities supported:
           10 Mbits
           100 Mbits
           1 Gbits
   Ethernet PHY device: ethernet-phy@1 (0x82043c90)
   Ethernet link speed: 1 Gbits full-duplex
   IPv6 unicast addresses (max 2):
           fe80::b867:f9ff:fe0a:ad54 autoconf preferred infinite
   IPv6 multicast addresses (max 3):
           ff02::1
           ff02::1:ff0a:ad54
   IPv6 prefixes (max 2):
           <none>
   IPv6 hop limit           : 64
   IPv6 base reachable time : 30000
   IPv6 reachable time      : 37007
   IPv6 retransmit timer    : 0
   IPv4 unicast addresses (max 1):
           <none>
   IPv4 multicast addresses (max 2):
           224.0.0.1
   IPv4 gateway : 0.0.0.0
   DHCPv4 state      : disabled
   uart:~$
   uart:~$ net ipv4 add 1 192.168.1.2 255.255.255.0
   uart:~$ net ipv4 add 2 192.168.2.2 255.255.255.0
   uart:~$ zperf udp upload 192.168.1.1 5001 10 1500 1000M
   Remote port is 5001
   Connecting to 192.168.1.1
   Duration:       10.00 s
   Packet size:    1500 bytes
   Rate:           1000.00 Mbps
   Starting...
   Rate:           1000.00 Mbps
   Packet duration 11 us
   -
   Upload completed!
   Statistics:             server  (client)
   Duration:               9.99 s  (10.00 s)
   Num packets:            560255  (560255)
   Num packets out order:  0
   Num packets lost:       1
   Jitter:                 30 us
   Rate:                   458.96 Mbps     (476.88 Mbps)
   uart:~$ zperf udp upload 192.168.2.1 5001 10 1500 1000M
   Remote port is 5001
   Connecting to 192.168.2.1
   Duration:       10.00 s
   Packet size:    1500 bytes
   Rate:           1000.00 Mbps
   Starting...
   Rate:           1000.00 Mbps
   Packet duration 11 us
   -
   Upload completed!
   Statistics:             server  (client)
   Duration:               9.99 s  (10.00 s)
   Num packets:            559568  (559568)
   Num packets out order:  0
   Num packets lost:       1
   Jitter:                 30 us
   Rate:                   458.39 Mbps     (476.30 Mbps)
   uart:~$ zperf tcp upload 192.168.1.1 5002 10 1500
   Remote port is 5002
   Connecting to 192.168.1.1
   Duration:       10.00 s
   Packet size:    1500 bytes
   Rate:           10 Kbps
   Starting...
   -
   Upload completed!
   Duration:               10.00 s
   Num packets:            271968
   Num errors:             0 (retry or fail)
   Rate:                   231.49 Mbps
   uart:~$ zperf tcp upload 192.168.2.1 5002 10 1500
   Remote port is 5002
   Connecting to 192.168.2.1
   Duration:       10.00 s
   Packet size:    1500 bytes
   Rate:           10 Kbps
   Starting...
   -
   Upload completed!
   Duration:               10.00 s
   Num packets:            272100
   Num errors:             0 (retry or fail)
   Rate:                   231.61 Mbps
   uart:~$ net iface set_link 1 100 f
   [00:02:05.126,000] <inf> eth_ti_am62l_cpsw: MAC port 1: link down
   [00:02:07.127,000] <inf> eth_ti_am62l_cpsw: MAC port 1: link up 100 Mbps full-duplex
   uart:~$ net ping 192.168.1.1
   PING 192.168.1.1
   28 bytes from 192.168.1.1 to 192.168.1.2: icmp_seq=1 ttl=64 time=0.58 ms
   28 bytes from 192.168.1.1 to 192.168.1.2: icmp_seq=2 ttl=64 time=0.67 ms
   28 bytes from 192.168.1.1 to 192.168.1.2: icmp_seq=3 ttl=64 time=0.32 ms
   uart:~$ net iface set_link 2 100 f
   [00:02:27.302,000] <inf> eth_ti_am62l_cpsw: MAC port 2: link down
   [00:02:29.303,000] <inf> eth_ti_am62l_cpsw: MAC port 2: link up 100 Mbps full-duplex
   uart:~$ net ping 192.168.2.1
   PING 192.168.2.1
   28 bytes from 192.168.2.1 to 192.168.2.2: icmp_seq=1 ttl=64 time=0.59 ms
   28 bytes from 192.168.2.1 to 192.168.2.2: icmp_seq=2 ttl=64 time=0.37 ms
   28 bytes from 192.168.2.1 to 192.168.2.2: icmp_seq=3 ttl=64 time=0.30 ms
   uart:~$

Features
********

* PKTDMA-based packet exchange between Software and CPSW3G
* 10/100/1000 Mbps auto-negotiation via DP83867 Gigabit Ethernet PHY
* MDIO bus management of both MAC Port 1 and MAC Port 2 PHYs for link state
  detection
* The MAC Address for MAC Port 1 is fetched from the eFuse block in the MAIN
  CTRL MMR register region in the SoC

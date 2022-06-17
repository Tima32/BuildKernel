Grif network device driver
=========================

Based on network part of ETN network driver for soc-based devices (etn_net).

There are 2 sides of network device implementation - network device driver and
FPGA.

In general we are on the way to use regmap for any fpga registers access, but
still using number of functions getting info about registers offsets from
`grif_fpga_mgr`.

This driver uses direct/read write to FPGA registers without any DMA, but we
want to add DMA (maybe SDMA) in future releases.

### Transmission of packets (tx path)

1. Write data from skbuf to FPGA memory (`TX_WR_DATA_W0`).
   XXX:
     Driver must use transactions in multiples of 32 or 4 bytes.
     Otherwise transaction with only 1 valid byte will be generated on EIM bus.
     But our EIM bus does not support byte enable signal.

2. Write packet size (in bytes, without CRC) to (`TX_PKT_SIZE_CR`).

### Reciving of packet (rx path)

1. FPGA recives packet and store it to internal memory.
2. FPGA generates interrupt
3. In ISR we disable interrupts and schedule napi.
4. In RX poll called by napi:
    - read metainformation (packet availability and it size) from
      FPGA registers (`RX_PKT_STATUS_SR` or `RX_PKT_STATUS_WITH_POP_SR`, see regs.h)

    - read whole packet from FPGA memory (`RX_PKT_DATA_SR`)

    - report to FPGA that we have already processed the current packet
      and associated resources can be released (`RX_PKT_STATUS_UPDATE_CR` or
      `RX_PKT_STATUS_WITH_POP_AND_UPDATE_SR`).

Network driver implements usual network devices for each of the available ports
with statistics and ethtool commands.

### Link handling

Network driver handles link status changes by issuing commands on MDIO interface
to the PHY.

### ethtool support

grif_net supported ethtool commands:

* Driver info - grif_ethtool_get_drvinfo
* Get/Set message level - grif_ethtool_get_msglevel/grif_ethtool_set_msglevel
* Get/Set settings -
	phy_ethtool_get_link_ksettings/phy_ethtool_set_link_ksettings

Driver info is:

- Firmware version
- Driver name - "grif_net"

Message level is pointless - doesn't affect anything.

Get/Set settings is ksettings default stubs.

### Timestamping

Timestamping not supported in the driver, but some code from `etn_net` is leaved
for support in future.

### Promiscuity

The driver supports enabling of promiscuous mode on network interfaces
provided by it. Normally, the network interface is not in promiscuous mode
thus the Network Interface Card (NIC) drops all received frames whose
destination MAC address doesn't match the address assigned to the
corresponding network interface. Enabling a promiscuous mode prevents NIC from
filtering the received frames by destination MAC address.

Even when the promiscuous mode is enabled there are still some frames that are
dropped: ones that were identified as "test frames" or ones having corrupted
CRC. For more details on this behaviour refer to the documentation on NIC_CTRL
FPGA feature.

There are many ways to manage network interface promiscuity. For more info on
some of them please refer to manpages: netdevice(7), ifconfig(8).

## Device Tree bindings

The grif_net driver handles platfrom_devices with

  compatible = "stcmtk,grif-net"

The following DT properties are supported:

  * target-fpga = < /* phandle to grif_fpga_mgr device */ >

    Specifies device supplied regmap and FPGA features info.

  * mac-address = [ /* 6 octets */ ];

    Specifies default MAC address of created ethernet device. This mac address
    can be changed by user with commands like `ifconfig`.

  * port-num = < /* signle integer */ >;

    Specifies Port Number. This property doesn't affect the network device
    operation and used only by userspace. Developed devices may have
    two Gigabit Ethernet network ports numbered with 0 and 1. This property
    defines the port number.

  * interrupts = < /* standard DT interrupt specifier */ >

    Must descrbe two interrupts. First one for TX, second one
    for RX.

  * link-led-gpios = < /* GPIO connected to Link LED */ >;

    GPIO that handles Link LED indication. Unnecessary binding.

  * wait-for-sfp = < /* SFP phandle */ >;

    The driver will wait until this SFP device have been fully probed. It helps
    to avoid races between SFP and other network components (like PHY device).
    Unnecessary binding.
  
  * phc-mux = < /* phandle to grif-phc-mux  */>;  
    
    The driver will use hardware multiplexer pointed by phc-mux entry. 
    Using phc-mux entry with no hardware multiplexer will cause malfunctioning.  
    If no phc-mux entry presented the driver will require phc entry.   
  
  * phc = < /* phandle to grif-phc */>  
  
  If no hardware multiplexer presented the driver will use phc entry.  
  phc entry values should be set according to hardware configuration  
  (interconnection between grif-net interfaces and PTP clocks).   

  * use-gro;

    Pass received packets to NAPI GRO (Generic Recieve Offload) routine rather
    than pass each packets up network stack individually. This can improve
    throughput on RX.

  * use-tx-sg;

    Support paged sk_buffs (aka Scatter-Gather) on TX. Use of paged data
    significantly reduce memory manipulation on upper levels.

  * dmas = <&sdma 0 16 0>,
	   <&sdma 0 16 0>;
    dma-names = "rx", "tx";

    Use SDMA instead of MMIO to copy data to/from FPGA. May be used separately
    for TX and RX path.

Also default MAC properties (`phy-handle` and `phy-mode`) and nodes
(`mdio` or `fixed-link`) are supported and must be valid MAC<->PHY link
configuration.
See:
    Documentation/devicetree/bindings/net/ethernet.txt
    Documentation/devicetree/bindings/net/phy.txt
    Documentation/devicetree/bindings/net/fixed-link.txt



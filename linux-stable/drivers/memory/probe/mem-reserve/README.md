mem-reserve
===========

mem-reserve -- driver for using reserved-memory region.

DESCRIPTION
-----------

This driver is char device, which provides methods for accsess
to reserved-memory region for user-space application:

* open
* write
* read
* lseek
* close

USAGE
-----

Device tree example:

```dts
reserved-memory {
	#address-cells = <1>;
	#size-cells = <1>;
	ranges;

	reserved0: buffer@22328000 {
		compatible = "shared-dma-pool";
		no-map;
		reg = <0x22328000 0x7736000>; /* 125 MB */
	};
};

payload-dump@1{
	compatible = "mem_reserve";
	dev-name = "payload_dump1";   /* Device name                 */
	mem-size = <0x7735940>;       /* Memory size using by driver */
	memory-region = <&reserved0>; /* Phandle memory-reserved region */
};
```
It is also necessary to indicate in Uboot that a part of the memory
is reserved with the help of bootargs "mem".

If at probe time driver message "vmap allocation for size failed: use vmalloc=<size> to increase size",
add to Uboot bootargs "vmalloc" size.

At this moment using kernel version 3.18.9, which have not finished API 
for reserved-memory. When upgrading to a higher version,
need to change the module.

obj-$(CONFIG_SOC_NET) += soc_net.o
soc_net-objs := main.o net.o phy.o regs.o dma_regs.o timestamp_regs.o ethtool.o

obj-$(CONFIG_SOC_MDIO) += soc_mdio.o
subdir-ccflags-y += -I$(srctree)/drivers/fpga/probe/ -I$(srctree)/drivers/net/ethernet/probe/

soc_mdio-objs := mdio.o regs.o

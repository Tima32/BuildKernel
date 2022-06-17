

# Device Tree Bindings

 * etn-net: ethernet port
   - compatible: Should be
     - "stcmtk,etln-10g-net": for ETLN-10G
   - port-num: port number
   - phy-handle, phy-mode see
     Documentation/devicetree/bindings/net/ethernet.txt
     Documentation/devicetree/bindings/net/phy.txt

 * etn-mdio: MDIO bus with phys
   - compatible: Should be
     - "stcmtk,etn-mdio"
   - target-fpga: FPGA device that implements MDIO
   - child nodes with phys

Example:

	etn-port0 {
		compatible = "stcmtk,etln-10g";
		port-num = <0>;
		phy-handle = <&mv_ch0>;
		phy-mode = "xgmii";
	};

	mdio-0 {
		compatible = "stcmtk,etn-mdio";
		target-fpga = "&fpga";
		#address-cells = <1>;
		#size-cells = <0>;
		status = "okay";

		mv_ch0:88X2222@0 {
			compatible = "marvell,88x2222",
				     "ethernet-phy-ieee802.3-c45";
			reg = <0x0>;
			mv,line-mode = <0>;
			mv,host-mode = <2>;
			status = "okay";
		};
	};

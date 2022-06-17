NT25L90 driver
--------------
Hardware monitor for NT25L90.

Description
-----------

This driver implements standart linux hwmon subsystem inside ```/sys/class/hwmon```  

Attributes:  
1. Temperature ```temp1_input``` in milliDegrees, C
2. Current in milliAmperes:  
2.1. Bias current: ```curr1_input```  
2.2. Modulation current: ```curr2_input```  
3. Voltage in milliVoltages:  
3.1. Tx Supply Voltage: ```in0_input```  
3.2. Rx Supply Voltage: ```in1_input```  
3.3. Digital Supply Voltage: ```in2_input```  
4. Power in nanoAmperes:  
4.1. Tx Power: ```power1_input```  
4.2. Rx Power: ```power2_input```  

This driver creates new sysfs class "optical_driver" (it can be found in /sys/class/)  

Device Tree
-----------
Example:
```
nt25l90_device@54 {
	compatible = "linux,nt25l90";
	status = "okay";
	reg = <0x54>;
};
```

Usage
-----

In ```/sys/class/optical_driver``` in device folder you can:  
1. Read/write registers using sysfs attributes:  
    1.1. To read a register value:   
 	```cat /sys/class/optical_driver/od0/%reg_num%```  
    1.2. To write data to register:  
  ```echo mask val > /sys/class/optical_driver/od0/%reg_num%```  
  For example, to set the first and third bits to 1, the mask should be 0b00000101 and the value should be 0bxxxx1x1.  
  To clear the seventh bit, the mask must be 0b01000000, the value should be 0bx0xxxxxx.

2. Enable/disable regulator using ```regulator_controller``` file  
```echo "option" > /sys/class/optical_driver/od0/register_controller```  
where ```option``` is a enable/disable
/*
 * Copyright (c) 2013 Espressif System.
 *
 *  sdio stub code for allwinner
 */

#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/of_gpio.h>

#include "esp_sif.h"

//#define SPI_FREQ 0 // default speed
#define SPI_FREQ (20000000)
//#define SPI_FREQ (30000000)

//Below are for spi HZ 22.5M

#if (SPI_FREQ == 0)

#define CMD_RESP_SIZE 1
#define DATA_RESP_SIZE_R 1
#define DATA_RESP_SIZE_W 1

#define BLOCK_R_DATA_RESP_SIZE_1ST   1
#define BLOCK_R_DATA_RESP_SIZE_EACH  1
#define BLOCK_W_DATA_RESP_SIZE_EACH  1
#define BLOCK_W_DATA_RESP_SIZE_FINAL 1

#elif (SPI_FREQ == 30000000)

#define CMD_RESP_SIZE   (10) //(50)    //Common respon wait time
#define DATA_RESP_SIZE_W   (142+45) // (1024*13)//   (1024*16)  //(398+400) // (1024*10)    //Only for Write bytes function, data write response.  max:(361+109) 
#define DATA_RESP_SIZE_R   (231+75) //  (340+102)  //(231+75)//(340+102)   //Only for Read bytes function, data write response    max:(340+102) 

#define BLOCK_W_DATA_RESP_SIZE_EACH          (10)           //For each data write resp size, in block write 
#define BLOCK_W_DATA_RESP_SIZE_FINAL (152) // (142+52)   //For final data write resp size, in block write ,max: 119

#define BLOCK_R_DATA_RESP_SIZE_1ST   (265)  // (231+75)    //For each data read resp size, in block read  ,max: 134
#define BLOCK_R_DATA_RESP_SIZE_EACH    (10)  // (20)   //For each data read resp size, in block read 

#elif(SPI_FREQ == 20000000)

#define CMD_RESP_SIZE (10)    //Common respon wait time
#define DATA_RESP_SIZE_W (103+40)    //Only for Write bytes function, data write response.  max: 103
#define DATA_RESP_SIZE_R (118+40)    //Only for Read bytes function, data write response  max: 118
//w: oxFF : 218 clock.     oxFE : 214 clock.  

#define BLOCK_W_DATA_RESP_SIZE_EACH          (20)           //For each data write resp size, in block write 
#define BLOCK_W_DATA_RESP_SIZE_FINAL     (112+40)    //For final data write resp size, in block write ,max :112

#define BLOCK_R_DATA_RESP_SIZE_1ST          (123+40)   //For each data read resp size, in block read ,max: 123
#define BLOCK_R_DATA_RESP_SIZE_EACH       (20)   //For each data read resp size, in block read 

#endif

//0xE5 ~0xFF  30us totoal 
//

#define ESPRESSIF_VENDOR_NAME "espressif"
#define ESP8089_NAME "esp8089"
#define ESP8266_NAME "esp8266"

const struct spi_device_id esp_spi_id[] = {
        {ESP8089_NAME, 0},
        {ESP8266_NAME, 0},
        {},
};

const struct of_device_id esp_spi_of_match[] = {
	{ .compatible = ESPRESSIF_VENDOR_NAME","ESP8089_NAME, },
	{ .compatible = ESPRESSIF_VENDOR_NAME","ESP8266_NAME, },
	{}
};

int sif_platform_get_irq_no(struct esp_pub* epub)
{
	return gpio_to_irq(epub->int_gpio);
}

int sif_platform_is_irq_occur(struct esp_pub* epub)
{      
	return 1;
}

void sif_platform_irq_clear(struct esp_pub* epub)
{
}

void sif_platform_irq_mask(struct esp_pub* epub, int mask)
{
	if (mask)
		disable_irq_nosync(sif_platform_get_irq_no(epub));
	else
		enable_irq(sif_platform_get_irq_no(epub));
}

int sif_platform_irq_init(struct esp_pub* epub)
{
	int ret;
	int esp_int_gpio = of_get_named_gpio(epub->dev->of_node, "int-gpio", 0);
	if (esp_int_gpio < 0)
		return esp_int_gpio;
	if ( (ret = gpio_request(esp_int_gpio, "esp_spi_int")) != 0) {
		dev_err(epub->dev, "request gpio error\n");
		return ret;
	}
	
	epub->int_gpio = esp_int_gpio;

	gpio_direction_input(esp_int_gpio);

	sif_platform_irq_clear(epub);
	sif_platform_irq_mask(epub, 1);

	udelay(1);

	return 0;
}


void sif_platform_irq_deinit(struct esp_pub* epub)
{
	gpio_free(epub->int_gpio);
}


int sif_platform_reset_target(struct device_node *node)
{
	int esp_en_gpio = of_get_named_gpio(node, "enable-gpio", 0);
	if (esp_en_gpio < 0){
		esp_dbg(ESP_DBG_ERROR, "can't get enable gpio (%d)", esp_en_gpio);
		return esp_en_gpio;
	}
	gpio_request(esp_en_gpio, "esp_enable");
	gpio_direction_output(esp_en_gpio,0);
	msleep(200);
	gpio_direction_input(esp_en_gpio);
	gpio_free(esp_en_gpio);

	return 0;
}

int sif_platform_target_poweroff(struct device_node *node)
{
	/* reset ESP before unload so that the esp can be probed on
	 * warm reboot */
	return sif_platform_reset_target(node);
}

int sif_platform_target_poweron(struct device_node *node)
{
	return sif_platform_reset_target(node);
}

void sif_platform_target_speed(int high_speed)
{
}

#ifdef ESP_ACK_INTERRUPT
void sif_platform_ack_interrupt(struct esp_pub *epub)
{
	sif_platform_irq_clear();
}
#endif //ESP_ACK_INTERRUPT


module_init(esp_spi_init);
module_exit(esp_spi_exit);


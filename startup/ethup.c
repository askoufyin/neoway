#include "nwy_ethernet.h"
#include "nwy_common.h"
#include "nwy_gpio.h"


#define ETH_POWER_DEV "/dev/wifi_power_en"
#define PMIC_GPIO 3


static int
ethernet_power_en_gpio(int gpio_n, int fn)
{
	nwy_gpio_set_dir(gpio_n, NWY_OUTPUT);
	if (fn)
		nwy_gpio_set_val(gpio_n, NWY_HIGH);
	else
		nwy_gpio_set_val(gpio_n, NWY_LOW);

	return 0;
}


static int
ethernet_power_en_node(int fn)
{
	int fd;
	char wbuf;

	fd = open(ETH_POWER_DEV, O_RDWR);
	if (fd < 0) {
		printf("can't open eth power dev!\n");
		return -1;
	}

	if (fn)
		wbuf = '1';
	else
		wbuf = '0';
	write(fd, &wbuf, sizeof(wbuf));
	close(fd);

	return 0;
}


int
main(int argc, char *argv[])
{
	int ret;
	int gpio_n = 3;

	printf("ethernet loading...\n");
	if (gpio_n == PMIC_GPIO)
		ethernet_power_en_node(NWY_LOAD);
	else
		ethernet_power_en_gpio(gpio_n, NWY_LOAD);
	ret = nwy_ethernet_load();
	printf("ethernet load done\n");

	return ret;
}

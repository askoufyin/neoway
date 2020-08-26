#include "watchdog.h"

#include "nwy_common.h"
#include "nwy_gpio.h"

#include <unistd.h>

void *
watchdog_thread_main(void *arg)
{
    options_t *opts = (options_t *)arg;
    int pin = NWY_GPIO_77;
    int state = 0;

    printf("Init: WATCHDOG\n");

    nwy_gpio_set_dir(pin, NWY_OUTPUT);

    for(;;) {
        nwy_gpio_set_val(pin, state);
        state = 1 - state;
        usleep(250);
    }   
}

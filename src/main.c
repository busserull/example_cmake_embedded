#include <stdint.h>

#include "cmu.h"
#include "gpio.h"

int main() {
    cmu_clock_enable_gpio();

    volatile Gpio * const gpio = (Gpio *)(GPIO_ADDRESS);

    gpio->px[GPIO_PORT_E].model = (4 << 8) | (4 << 12);

    gpio->px[GPIO_PORT_E].douttgl = (1 << 2);

    while (1) {
        volatile int _delay = 1000000;
        while (--_delay)
            ;

        gpio->px[GPIO_PORT_E].douttgl = (1 << 2);
        gpio->px[GPIO_PORT_E].douttgl = (1 << 3);
    }
}

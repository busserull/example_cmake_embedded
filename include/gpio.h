#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>

#define GPIO_ADDRESS 0x40006000
#define GPIO_PORT_A 0
#define GPIO_PORT_B 1
#define GPIO_PORT_C 2
#define GPIO_PORT_D 3
#define GPIO_PORT_E 4
#define GPIO_PORT_F 5

typedef struct {
    uint32_t ctrl;
    uint32_t model;
    uint32_t modeh;
    uint32_t dout;
    uint32_t doutset;
    uint32_t doutclr;
    uint32_t douttgl;
    uint32_t din;
    uint32_t pinlockn;
} GpioCore;

typedef struct {
    GpioCore px[6];
    uint32_t reserved0[10];
    uint32_t extipsell;
    uint32_t extipselh;
    uint32_t extirise;
    uint32_t extifall;
    uint32_t ien;
    uint32_t ifr;
    uint32_t ifs;
    uint32_t ifc;
    uint32_t route;
    uint32_t insense;
    uint32_t lock;
    uint32_t ctrl;
    uint32_t cmd;
    uint32_t em4wuen;
    uint32_t em4wupol;
    uint32_t em4wucause;
} Gpio;

#endif

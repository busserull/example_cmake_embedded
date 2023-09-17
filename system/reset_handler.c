#include <stdint.h>
#include <stdnoreturn.h>

int main();

extern uint8_t _etext;
extern uint8_t _sdata;
extern uint8_t _edata;
extern uint8_t _sbss;
extern uint8_t _ebss;

/* Memory System Controller */
#define MSC_CMD_ADDRESS 0x400C0040

noreturn void Reset_Handler() {
    /* Copy data from Flash to RAM */
    uint8_t * p_source = &_etext;
    for (uint8_t * p_dest = &_sdata; p_dest < &_edata; p_dest++, p_source++) {
        *p_dest = *p_source;
    }

    /* Zero BSS */
    for (uint8_t * p_bss = &_sbss; p_bss < &_ebss; p_bss++) {
        *p_bss = 0;
    }

    /* Invalidate instruction cache */
    *((volatile uint32_t *)MSC_CMD_ADDRESS) = 1;

    main();

    while (1) {}
}

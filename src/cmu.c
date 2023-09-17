#include <stdint.h>

#include "cmu.h"

#define CMU_ADDRESS 0x400C8000

#define CMU_HFPERCLKEN0_GPIO (1 << 13)

typedef struct {
	uint32_t ctrl;
	uint32_t hfcoreclkdiv;
	uint32_t hfperclkdiv;
	uint32_t hfrcoctrl;
	uint32_t lfrcoctrl;
	uint32_t auxhfrcoctrl;
	uint32_t calctrl;
	uint32_t calcnt;
	uint32_t oscencmd;
	uint32_t cmd;
	uint32_t lfclksel;
	uint32_t status;
	uint32_t ifr;
	uint32_t ifs;
	uint32_t ifc;
	uint32_t ien;
	uint32_t hfcoreclken0;
	uint32_t hfperclken0;
	uint32_t reserved0[2];
	uint32_t syncbusy;
	uint32_t freeze;
	uint32_t lfaclken0;
	uint32_t reserved1;
	uint32_t lfbclken0;
	uint32_t reserved2;
	uint32_t lfapresc0;
	uint32_t reserved3;
	uint32_t lfbpresc0;
	uint32_t reserved4;
	uint32_t pcntctrl;
	uint32_t lcdctrl;
	uint32_t route;
	uint32_t lock;
} Cmu;

void cmu_clock_enable_gpio()
{
	volatile Cmu *const cmu = (Cmu *)(CMU_ADDRESS);

	cmu->hfperclken0 = CMU_HFPERCLKEN0_GPIO;
}

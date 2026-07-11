#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "libc.h"
#include "main.h"

#include "taskdefs.h"

/* This is the helper task that reads AS5047 magnetic rotary encoder.
 * */

#define AS5047_PARD		0x8000U
#define AS5047_READ		0x4000U
#define AS5047_EF		0x4000U
#define AS5047_DATA		0x3FFFU

#define AS5047_REG_NOP		0x0000U
#define AS5047_REG_ERRFL	0x0001U
#define AS5047_REG_PROG		0x0003U
#define AS5047_REG_DIAAGC	0x3FFCU
#define AS5047_REG_MAG		0x3FFDU
#define AS5047_REG_ANGLEUNC	0x3FFEU
#define AS5047_REG_ANGLECOM	0x3FFFU

typedef struct {

	int		EP;

	uint16_t	txbuf[16] LD_DMA;
	uint16_t	rxbuf[16] LD_DMA;

	int		EF_errcnt;
	int		PA_errcnt;
}
priv_AS5047_t;

static priv_AS5047_t		priv_AS5047;

static int
AS5047_parity(uint16_t x)
{
	x ^= x >> 8;
	x ^= x >> 4;

	x = (0x6996U >> (x & 0xFU)) & 1U;

	return (int) x;
}

int AS5047_get_EP()
{
	uint16_t	ANGLE;

	if (SPI_dma_busy(ap.SPI_busnum) == HAL_ENABLED)
		return (int) -1;

	ANGLE = priv_AS5047.rxbuf[1];

	if ((ANGLE & AS5047_EF) == 0U) {

		if (AS5047_parity(ANGLE) == 0) {

			priv_AS5047.EP = (int) (ANGLE & AS5047_DATA);
		}
		else {
			priv_AS5047.EP = (int) -1;
			priv_AS5047.PA_errcnt++;
		}
	}
	else {
		priv_AS5047.EP = (int) -1;
		priv_AS5047.EF_errcnt++;
	}

	priv_AS5047.txbuf[0] = AS5047_PARD | AS5047_READ | AS5047_REG_ANGLECOM;
	priv_AS5047.txbuf[1] = AS5047_PARD | AS5047_READ | AS5047_REG_NOP;

	SPI_dma_transfer(ap.SPI_busnum, priv_AS5047.txbuf, priv_AS5047.rxbuf, 2);

	return priv_AS5047.EP;
}

AP_TASK_DEF(AS5047)
{
	AP_KNOB(knob);

	if (SPI_halted(ap.SPI_busnum) != HAL_OK) {

		printf("Unable to start application when SPI is busy" EOL);

		AP_TERMINATE(knob);
	}

	SPI_startup(ap.SPI_busnum, ap.SPI_clock, SPI_LOW_FALLING | SPI_DMA | SPI_NSS_ON_WORD);

	vTaskDelay((TickType_t) 1);

	ap.proc_get_EP = &AS5047_get_EP;

	do {
		vTaskDelay((TickType_t) 1000);

		ap.SPI_errate =   priv_AS5047.EF_errcnt
				+ priv_AS5047.PA_errcnt;

		if (		   priv_AS5047.EF_errcnt != 0
				|| priv_AS5047.PA_errcnt != 0) {

			if (		hal.DPS_mode == DPS_DRIVE_ON_SPI
					&& pm.eabi_RECENT == PM_ENABLED) {

				if (		   priv_AS5047.EF_errcnt >= 1000
						|| priv_AS5047.PA_errcnt >= 1000) {

					pm.fsm_errno = PM_ERROR_SPI_DATA_FAULT;
					pm.fsm_req = PM_STATE_HALT;
				}
			}

			log_TRACE("AS5047 errate EF %i PA %i" EOL,
					priv_AS5047.EF_errcnt,
					priv_AS5047.PA_errcnt);

			priv_AS5047.EF_errcnt = 0;
			priv_AS5047.PA_errcnt = 0;
		}
	}
	while (AP_CONDITION(knob));

	ap.proc_get_EP = NULL;

	vTaskDelay((TickType_t) 5);

	SPI_halt(ap.SPI_busnum);

	AP_TERMINATE(knob);
}


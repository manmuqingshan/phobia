#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "libc.h"
#include "main.h"

#include "taskdefs.h"

/* This is the helper task that reads TLE5012 angle sensor.
 * */

#define TLE5012_FREQUENCY	4000000U	/* (Hz) */

#define TLE5012_READ		0x8000U
#define TLE5012_UPDATE		0x0400U
#define TLE5012_ND1		0x0001U
#define TLE5012_DATA		0x7FFFU

#define TLE5012_SAFETY_STAT	0xF000U
#define TLE5012_SAFETY_CRC	0x00FFU

#define TLE5012_REG_STAT	0x0000U
#define TLE5012_REG_AVAL	0x0020U

typedef struct {

	int		EP;

	uint16_t	txbuf[16] LD_DMA;
	uint16_t	rxbuf[16] LD_DMA;

	int		ST_errcnt;
	int		CR_errcnt;
}
priv_TLE5012_t;

static priv_TLE5012_t		priv_TLE5012;

static uint8_t
TLE5012_crc8_j1850_be(const uint16_t *text, int len)
{
	const uint16_t		*end = text + len;
	uint32_t		crcsum = 0xFF00U;

	int			i;

	while (text < end) {

		crcsum = crcsum ^ * (text++);

		for (i = 0; i < 16; ++i) {

			crcsum = (crcsum & 0x8000U) ? (crcsum << 1) ^ 0x1D00U : crcsum << 1;
		}
	}

	return (uint8_t) (crcsum >> 8) ^ 0xFFU;
}

int TLE5012_get_EP()
{
	uint16_t		STAT, CRC8;

	if (SPI_dma_busy(HW_SPI_EXT_ID) == HAL_ENABLED)
		return priv_TLE5012.EP;

	STAT = priv_TLE5012.rxbuf[2] & TLE5012_SAFETY_STAT;
	CRC8 = priv_TLE5012.rxbuf[2] & TLE5012_SAFETY_CRC;

	if (STAT == TLE5012_SAFETY_STAT) {

		if (TLE5012_crc8_j1850_be(priv_TLE5012.rxbuf, 2) == CRC8) {

			priv_TLE5012.EP = (int) (priv_TLE5012.rxbuf[1] & TLE5012_DATA);
		}
		else {
			priv_TLE5012.CR_errcnt++;
		}
	}
	else {
		priv_TLE5012.ST_errcnt++;
	}

	priv_TLE5012.txbuf[0] = TLE5012_READ | TLE5012_REG_AVAL | TLE5012_ND1;
	priv_TLE5012.txbuf[1] = 0xFFFFU;
	priv_TLE5012.txbuf[2] = 0xFFFFU;

	SPI_dma_transfer(HW_SPI_EXT_ID, priv_TLE5012.txbuf, priv_TLE5012.rxbuf, 3);

	return priv_TLE5012.EP;
}

static void
TLE5012_startup(int bus)
{
	uint16_t		txbuf, rxbuf, safet;
	int			tle5012_NSS;

	tle5012_NSS = SPI_gpio_NSS(bus);

	GPIO_set_LOW(tle5012_NSS);
	TIM_wait_ns(1500);

	GPIO_set_HIGH(tle5012_NSS);
	TIM_wait_ns(900);

	GPIO_set_LOW(tle5012_NSS);
	TIM_wait_ns(500);

	txbuf = TLE5012_READ | TLE5012_REG_STAT | TLE5012_ND1;

	SPI_transfer(bus, txbuf);
	rxbuf = SPI_transfer(bus, 0xFFFFU);
	safet = SPI_transfer(bus, 0xFFFFU);

	TIM_wait_ns(500);

	GPIO_set_HIGH(tle5012_NSS);
	TIM_wait_ns(900);

	if ((rxbuf & 0xFFFEU) != 0x8000U) {

		log_TRACE("TLE5012 STAT %4x %4x" EOL, rxbuf, safet);
	}
}

AP_TASK_DEF(TLE5012)
{
	AP_KNOB(knob);

	if (SPI_halted(HW_SPI_EXT_ID) != HAL_OK) {

		printf("Unable to start application when SPI is busy" EOL);

		AP_TERMINATE(knob);
	}

	SPI_startup(HW_SPI_EXT_ID, TLE5012_FREQUENCY, SPI_LOW_FALLING
			| SPI_DMA | SPI_NSS_ON_TRANSFER | SPI_MOSI_OPEN_DRAIN);

	TLE5012_startup(HW_SPI_EXT_ID);

	vTaskDelay((TickType_t) 1);

	ap.proc_get_EP = &TLE5012_get_EP;

	do {
		vTaskDelay((TickType_t) 1000);

		if (		   priv_TLE5012.ST_errcnt != 0
				|| priv_TLE5012.CR_errcnt != 0) {

			if (		hal.DPS_mode == DPS_DRIVE_ON_SPI
					&& pm.lu_MODE != PM_LU_DISABLED) {

				if (		   priv_TLE5012.ST_errcnt >= 100
						|| priv_TLE5012.CR_errcnt >= 100) {

					pm.fsm_errno = PM_ERROR_SPI_DATA_FAULT;
					pm.fsm_req = PM_STATE_HALT;
				}
			}

			log_TRACE("TLE5012 errate ST %i CR %i" EOL,
					priv_TLE5012.ST_errcnt,
					priv_TLE5012.CR_errcnt);

			priv_TLE5012.ST_errcnt = 0;
			priv_TLE5012.CR_errcnt = 0;
		}
	}
	while (AP_CONDITION(knob));

	ap.proc_get_EP = NULL;

	vTaskDelay((TickType_t) 5);

	SPI_halt(HW_SPI_EXT_ID);

	AP_TERMINATE(knob);
}


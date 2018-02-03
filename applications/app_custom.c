/*
	Copyright 2017 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

/*
 * App for UDP communication with W5500 ethernet shield via SPI.
 *
 *  Created on: Feb 03, 2018
 *      Author: Guoping Zhao
 */

#include "conf_general.h"

#ifdef APP_CUSTOM_TO_USE
#include APP_CUSTOM_TO_USE
#endif


#include "ch.h" // ChibiOS
#include "hal.h" // ChibiOS HAL
#include "mc_interface.h" // Motor control functions
#include "mcpwm_foc.h"
#include "encoder.h" // encoder
#include "hw.h" // Pin mapping on this hardware
#include "timeout.h" // To reset the timeout
#include <math.h>

// define the wizchip as 5500, _WIZCHIP_
#include "./ioLibrary/Ethernet/wizchip_conf.h"
#include "./ioLibrary/Ethernet/socket.h"


// udp thread
static THD_WORKING_AREA(myudp_thread_wa, 2048); // 2kb stack for this thread
static THD_FUNCTION(myudp_thread, arg);



/*
 * Maximum speed SPI configuration (10.5MHz, CPHA=0, CPOL=0, MSb first).
 * Peripherial Clock 42MHz SPI2 SPI3
	Peripherial Clock 84MHz SPI1                                SPI1        SPI2/3
	#define SPI_BaudRatePrescaler_2         ((uint16_t)0x0000) //  42 MHz      21 MHZ
	#define SPI_BaudRatePrescaler_4         ((uint16_t)0x0008) //  21 MHz      10.5 MHz
	#define SPI_BaudRatePrescaler_8         ((uint16_t)0x0010) //  10.5 MHz    5.25 MHz
	#define SPI_BaudRatePrescaler_16        ((uint16_t)0x0018) //  5.25 MHz    2.626 MHz
	#define SPI_BaudRatePrescaler_32        ((uint16_t)0x0020) //  2.626 MHz   1.3125 MHz
	#define SPI_BaudRatePrescaler_64        ((uint16_t)0x0028) //  1.3125 MHz  656.25 KHz
	#define SPI_BaudRatePrescaler_128       ((uint16_t)0x0030) //  656.25 KHz  328.125 KHz
	#define SPI_BaudRatePrescaler_256       ((uint16_t)0x0038) //  328.125 KHz 164.06 KHz

	static const SPIConfig spicfg = { NULL, TP_CS_PORT, TP_CS_PIN, SPI_BaudRatePrescaler_32 };
 */
static const SPIConfig spiudpcfg = {
  NULL,
  HW_SPI_PORT_NSS,
  HW_SPI_PIN_NSS,
  SPI_CR1_BR_1
};


/*
 * SPI TX and RX buffers.
 */
static uint8_t spiBufSiz = 32;
static uint8_t spiTxBuf[32];
static uint8_t spiRxBuf[32];




#define SOCK_TCPS       0
#define SOCK_UDPS       1
#define PORT_TCPS		5000
#define PORT_UDPS		3000



void 	  wiz_cris_enter(void)
{
	spiAcquireBus(&HW_SPI_DEV);              /* Acquire ownership of the bus.    */
	spiStart(&HW_SPI_DEV, &spiudpcfg);       /* Setup transfer parameters.       */
}
void 	  wiz_cris_exit(void)
{
	spiReleaseBus(&HW_SPI_DEV);              /* Ownership release.               */
}

void 	wiz_cs_select(void)
{
	spiSelect(&HW_SPI_DEV);                  /* Slave Select assertion.          */
}

void 	wiz_cs_deselect(void)
{
	spiUnselect(&HW_SPI_DEV);                /* Slave Select de-assertion.       */
}

uint8_t wiz_spi_readbyte(void)
{
	uint8_t rxbuf = 0;
	spiReceive(&HW_SPI_DEV, 1, &rxbuf);
	return rxbuf;
}
void 	wiz_spi_writebyte(uint8_t wb)
{
	spiSend(&HW_SPI_DEV, 1, &wb);          /* Atomic transfer operations.      */
}
void 	wiz_spi_readburst(uint8_t* pBuf, uint16_t len)
{
	spiReceive(&HW_SPI_DEV, len, pBuf);
}
void 	wiz_spi_writeburst(uint8_t* pBuf, uint16_t len)
{
	spiSend(&HW_SPI_DEV, len, pBuf);          /* Atomic transfer operations.      */
}


void app_custom_start(void) {
	/*
	* SPI1 I/O pins setup
	* // SPI pins
	#define HW_SPI_DEV				SPID1
	#define HW_SPI_GPIO_AF			GPIO_AF_SPI1
	#define HW_SPI_PORT_NSS			GPIOA
	#define HW_SPI_PIN_NSS			4
	#define HW_SPI_PORT_SCK			GPIOA
	#define HW_SPI_PIN_SCK			5
	#define HW_SPI_PORT_MOSI		GPIOA
	#define HW_SPI_PIN_MOSI			7
	#define HW_SPI_PORT_MISO		GPIOA
	#define HW_SPI_PIN_MISO			6
	*/
	palSetPadMode(HW_SPI_PORT_SCK, HW_SPI_PIN_SCK, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);       /* New SCK.     */
	palSetPadMode(HW_SPI_PORT_MISO, HW_SPI_PIN_MISO, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);       /* New MISO.    */
	palSetPadMode(HW_SPI_PORT_MOSI, HW_SPI_PIN_MOSI, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);       /* New MOSI.    */
	palSetPadMode(HW_SPI_PORT_NSS, HW_SPI_PIN_NSS, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);       /* New CS.      */
	palSetPad(HW_SPI_PORT_NSS, HW_SPI_PIN_NSS);


//	// W5500 setup
	reg_wizchip_cris_cbfunc(wiz_cris_enter, wiz_cris_exit);
	reg_wizchip_cs_cbfunc(wiz_cs_select, wiz_cs_deselect);
	reg_wizchip_spi_cbfunc(wiz_spi_readbyte, wiz_spi_writebyte);
	reg_wizchip_spiburst_cbfunc(wiz_spi_readburst, wiz_spi_writeburst);

	//	reg_wizchip_cris_cbfunc(NULL, NULL);
	//	reg_wizchip_cs_cbfunc(NULL, NULL);
	//	reg_wizchip_bus_cbfunc(NULL, NULL);
	//	reg_wizchip_spi_cbfunc(NULL, NULL);
	//	reg_wizchip_spiburst_cbfunc(NULL, NULL);

	uint8_t bufSize[] = {32, 32, 32, 32};
//	wizchip_init(bufSize, bufSize); // tx and rx buffer size 2KB
//	wizchip_init(spiTxBuf, spiRxBuf); // tx and rx buffer size 2KB

	wiz_NetInfo netInfo = { .mac 	= {0x00, 0x08, 0xdc, 0xab, 0xcd, 0xef},	// Mac address
			.ip 	= {192, 168, 1, 20},					// IP address
			.sn 	= {255, 255, 255, 0},					// Subnet mask
			.gw 	= {192, 168, 1, 100}, 					// Gateway address
			.dns = {0, 0, 0, 0},
			.dhcp = NETINFO_STATIC
	};
	wizchip_setnetinfo(&netInfo);


	// open udp socket
	switch(getSn_SR(SOCK_UDPS))
	{
	case SOCK_UDP:
		// port already opened as UDP
		break;
	case SOCK_CLOSED:
		socket(SOCK_UDPS, Sn_MR_UDP, PORT_UDPS, 0x00); // open the port
		break;
	default:
		break;
	}




	// Start the example thread
	chThdCreateStatic(myudp_thread_wa, sizeof(myudp_thread_wa),
		NORMALPRIO, myudp_thread, NULL);
}


void app_custom_stop(void) {
	spiReleaseBus(&HW_SPI_DEV);              /* Ownership release.               */
	spiStop(&HW_SPI_DEV);

	palSetPadMode(HW_SPI_PORT_SCK, HW_SPI_PIN_SCK, PAL_MODE_INPUT);       /* New SCK.     */
	palSetPadMode(HW_SPI_PORT_MISO, HW_SPI_PIN_MISO, PAL_MODE_INPUT);       /* New MISO.    */
	palSetPadMode(HW_SPI_PORT_MOSI, HW_SPI_PIN_MOSI, PAL_MODE_INPUT);       /* New MOSI.    */
	palSetPadMode(HW_SPI_PORT_NSS, HW_SPI_PIN_NSS, PAL_MODE_INPUT);       /* New CS.      */

}


static THD_FUNCTION(myudp_thread, arg) {
	(void)arg;

	chRegSetThreadName("UDP COMM");

	float batVol = 0.0;
	float batCur = 0.0;
	float motCur = 0.0;
	float motAng = 0.0;
	float motRpm = 0.0;
	uint8_t motSta = 0;
	uint8_t motFal = 0;
	uint8_t encIndexFound = 0;
	unsigned char *chptr;

	float motCurDes, motRpmDes, motPosDes;
	motCurDes = 0.0;
	motRpmDes = 0.0;
	motPosDes = 0.0;
	float motDes, motKP, motKI, motKD;
	motDes = 0.0;
	uint8_t motCtr = 0;
	uint8_t tmpData[4];


	int32_t  ret;
	uint16_t size, sentsize;
	uint8_t  destip[4];
	uint16_t destport;
	uint8_t* bufRev;

	uint8_t  remoteip[4] = {192, 168, 1, 4}; // xpc target ip
	uint16_t remoteport = 8001;


	for(;;) {
//		motCur = mc_interface_get_tot_current_directional(); // sign denotes the direction in which the motor generates torque
		motCur = mc_interface_get_tot_current_directional_filtered();
		motAng = encoder_read_deg();
		motRpm = mc_interface_get_rpm();
		motSta = mc_interface_get_state();
		motFal = mc_interface_get_fault();
		batVol = GET_INPUT_VOLTAGE();
		batCur = mc_interface_get_tot_current_filtered(); // sign denotes motor draw(+) or generate(-) current

		encIndexFound = encoder_index_found();

		chptr = (unsigned char *) &motCur;
		for(int i=0; i<4; i++) { spiTxBuf[i] = *chptr++; }
		chptr = (unsigned char *) &motAng;
		for(int i=0; i<4; i++) { spiTxBuf[i+4] = *chptr++; }
		chptr = (unsigned char *) &motRpm;
		for(int i=0; i<4; i++) { spiTxBuf[i+8] = *chptr++; }
		spiTxBuf[12] = motSta*10 + motFal;

		chptr = (unsigned char *) &batVol;
		for(int i=0; i<4; i++) { spiTxBuf[i+13] = *chptr++; }
		chptr = (unsigned char *) &batCur;
		for(int i=0; i<4; i++) { spiTxBuf[i+17] = *chptr++; }

		spiTxBuf[21] = encIndexFound;



		if((size = getSn_RX_RSR(SOCK_UDPS)) > 0)
		{
			//if(size > DATA_BUF_SIZE) size = DATA_BUF_SIZE;
			ret = recvfrom(SOCK_UDPS, spiRxBuf, spiBufSiz, destip, (uint16_t*)&destport);
			if(ret <= 0)
			{ // error in receiving
				// NEEDS TO BE tested, not sure
				spiTxBuf[12] = 111;  // error code
				ret = sendto(SOCK_UDPS, spiTxBuf, spiBufSiz, remoteip, remoteport);  // send  data
			}
			else
			{
				// motor control
				motCtr = spiRxBuf[0];

//				for (int i=0; i<4; i++) { tmpData[i] = spiRxBuf[i+1]; }
//				motCurDes = *((float*)(&tmpData));
//				for (int i=0; i<4; i++) { tmpData[i] = spiRxBuf[i+5]; }
//				motRpmDes = *((float*)(&tmpData));
//				for (int i=0; i<4; i++) { tmpData[i] = spiRxBuf[i+9]; }
//				motPosDes = *((float*)(&tmpData));

				for (int i=0; i<4; i++) { tmpData[i] = spiRxBuf[i+1]; }
				motDes = *((float*)(&tmpData));
				// get the PID value
				for (int i=0; i<4; i++) { tmpData[i] = spiRxBuf[i+5]; }
				motKP = *((float*)(&tmpData));
				for (int i=0; i<4; i++) { tmpData[i] = spiRxBuf[i+9]; }
				motKI = *((float*)(&tmpData));
				for (int i=0; i<4; i++) { tmpData[i] = spiRxBuf[i+13]; }
				motKD = *((float*)(&tmpData));

				chptr = (unsigned char *) &motDes;
				for(int i=0; i<4; i++) { spiTxBuf[i+22] = *chptr++; }

				if (encIndexFound) {
					if(motCtr==0) { // motor free
						mc_interface_release_motor();
						spiTxBuf[12] = 10;
					}
					else if(motCtr==1) { // current control mode
						motCurDes = motDes;
						mc_interface_set_current(motCurDes);
						spiTxBuf[12] = 11;
					}
					else if(motCtr==2) { // speed control mode
						mc_interface_release_motor();  // disable
						//					motRpmDes = motDes;
						//					mc_interface_set_pid_speed(motRpmDes);
						spiTxBuf[12] = 12;
					}
					else if(motCtr==3) { // position control mode
						motPosDes = motDes;
						// set PID value
						mc_interface_set_pid_para_pos(motKP, motKI, motKD); // set the PID value
						mc_interface_set_pid_pos(motPosDes);
						spiTxBuf[12] = 13;
					}
					else {
						mc_interface_release_motor();
						spiTxBuf[12] = 14;
					}
				}
				else { // encoder index hasn't been found
					mc_interface_release_motor();
					spiTxBuf[12] = 99;
				}

//				ret = sendto(SOCK_UDPS, spiRxBuf, spiBufSiz, remoteip, remoteport);  // send back received data
				ret = sendto(SOCK_UDPS, spiTxBuf, spiBufSiz, remoteip, remoteport);  // send  data
			}



//			size = (uint16_t) ret;
//			sentsize = 0;
//			while(sentsize != size)
//			{
//				ret = sendto(SOCK_UDPS, buf+sentsize, size-sentsize, destip, destport);
//				if(ret < 0)
//				{ // error
//
//				}
//				sentsize += ret; // Don't care SOCKERR_BUSY, because it is zero.
//			}
		}


//		// open udp socket
//		switch(getSn_SR(SOCK_UDPS))
//		{
//		case SOCK_UDP:
//			// port already opened as UDP
//			ret = sendto(SOCK_UDPS, spiTxBuf, spiBufSiz, remoteip, remoteport);
//			break;
//		case SOCK_CLOSED:
//			socket(SOCK_UDPS, Sn_MR_UDP, PORT_UDPS, 0x00); // open the port
//			break;
//		default:
//			break;
//		}


//		ret = sendto(SOCK_UDPS, spiTxBuf, spiBufSiz, remoteip, remoteport);


		// Run this loop at 1000Hz
//		chThdSleepMilliseconds(1);

//		systime_t sleep_time = CH_CFG_ST_FREQUENCY / app_get_configuration()->send_can_status_rate_hz;
//		if (sleep_time == 0) {
//			sleep_time = 1;
//		}

		systime_t sleep_time = 3; // has to be greater than 1 to be sure it is not blocking other threads,
		// the system runs at CH_CFG_ST_FREQUENCY (10kHz)
		chThdSleep(sleep_time);

		// Reset the timeout
		timeout_reset();
	}
}





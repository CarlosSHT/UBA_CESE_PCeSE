/*
 * WM8960.c
 *
 *  Created on: Jun 16, 2022
 *      Author: carlo
 */

/*
 * Register Map WM8960
 */
#include "WM8960.h"

#define REGADDRS_LEN	0x37
#define WM8960_ADDRESS	0x34
// RESET
#define	RESET_IC_CODEC	0x0F	//	Reset

// Power
#define	PWR_MGMT1		0x19	//	Pwr Mgmt (1)
#define	PWR_MGMT2		0x1A	//	Pwr Mgmt (2)
#define	PWR_MGMT3		0x2F	//	Pwr Mgmt (3)

// PLLs
#define	PLL_K1			0x35	//	PLL K 1
#define	PLL_K2			0x36	//	PLL K 2
#define	PLL_K3			0x37	//	PLL K 3
#define	PLL_N			0x34	//	PLL N
#define	CLK1			0x04	//	Clocking (1)
#define	CLK2			0x08	//	Clocking (2)

#define	CTRL_3D		0x10	//	3D control
#define	ADCL_SPATH		0x20	//	ADCL signal path
#define	ADCR_SPATH		0x21	//	ADCR signal path
#define	ADD_CTRL1		0x17	//	Additional control(1)
#define	ADD_CTRL2		0x18	//	Additional control(2)
#define	ADD_CTRL3		0x1B	//	Additional Control (3)
#define	ADD_CTRL4		0x30	//	Additional Control (4)
#define	ALC1			0x11	//	ALC1
#define	ALC2			0x12	//	ALC2
#define	ALC3			0x13	//	ALC3
#define	ANTI_POP1		0x1C	//	Anti-pop 1
#define	ANTI_POP2		0x1D	//	Anti-pop 2
#define	AUD_INTERF1		0x07	//	Audio Interface
#define	AUD_INTERF2		0x09	//	Audio Interface
#define	BYPASS1			0x2D	//	Bypass (1)
#define	BYPASS2			0x2E	//	Bypass (2)
#define	CLASSD_CTRL1	0x31	//	Class D Control (1)
#define	CLASSD_CTRL3	0x33	//	Class D Control (3)
#define	CTR1			0x05	//	ADC & DAC Control (CTR1)
#define	CTR2			0x06	//	ADC & DAC Control (CTR2)
#define	IBOOST_MIX1		0x2B	//	Input boost mixer (1)
#define	IBOOST_MIX2		0x2C	//	Input boost mixer (2)
#define	LOUT_MIX1		0x22	//	Left out Mix (1)
#define	MOU_VOL			0x2A	//	MONOOUT volume
#define	MOUT_MIX1		0x26	//	Mono out Mix (1)
#define	MOUT_MIX2		0x27	//	Mono out Mix (2)

#define	NOISE_GATE		0x14	//	Noise Gate
#define	ROUT_MIX2		0x25	//	Right out Mix (2)

#define	LOUT1_VOL		0x02	//	LOUT1 volume
#define	ROUT1_VOL		0x03	//	ROUT1 volume
#define	LOUT2_VOL		0x28	//	LOUT2 volume
#define	ROUT2_VOL		0x29	//	ROUT2 volume

#define	LIN_VOL			0x00	//	Left Input volume
#define	RIN_VOL			0x01	//	Right Input volume
#define	LADC_VOL		0x15	//	Left ADC volume
#define	RADC_VOL		0x16	//	Right ADC volume
#define	LDAC_VOL		0x0A	//	Left DAC volume
#define	RDAC_VOL		0x0B	//	Right DAC volume

#define OFFSETLOUTVOL	0b0110000
#define OFFSETSPKVOL	0b0110000
#define OFFSETDACVOL	0b0000000

uint16_t regValues[REGADDRS_LEN + 1] = { 0b10010111, 0b10010111, 0b00000000,
		0b00000000, 0b00000000, 0b00001000, 0b00000000, 0b00001010, 0b111000000,
		0b00000000, 0b11111111, 0b11111111, 0b00000000, 0b00000000, 0b00000000,
		0b00000000, 0b00000000, 0b01111011, 0b100000000, 0b00110010, 0b00000000,
		0b11000011, 0b11000011, 0b111000000, 0b00000000, 0b00000000, 0b00000000,
		0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b100000000,
		0b100000000, 0b01010000, 0b01010000, 0b01010000, 0b01010000, 0b00000000,
		0b00000000, 0b00000000, 0b00000000, 0b01000000, 0b00000000, 0b00000000,
		0b01010000, 0b01010000, 0b00000000, 0b00000010, 0b00110111, 0b01001101,
		0b10000000, 0b00001000, 0b00110001, 0b00100110, 0b11101001 };

static wm8960_t wm8960_control;

static bool	continuePlaying;


static uint32_t WM8960_maximDMAsize(uint32_t audSiz);

static void WM8960_Reset(void);
static void WM8960_pwrMngmt(void);
static void WM8960_setRateDAC(void);
static void WM8960_controlDAC(void);
static void WM8960_setInterfaceAudio(void);
static void WM8960_setGainL1out(uint8_t vol);
static void WM8960_setGainSPKout(uint8_t vol);
static void WM8960_setGainDAC(float vol);
static void WM8960_enableOuts(ena_ClassD stateEn);
static void WM8960_controlMixer(void);
static void WM8960_detectHeadPjack(bool detectFlag);
static void WM8960_setADDcontrols(void);

uint32_t remAudiosize;
uint16_t *actualPosition;
uint32_t sizeAudiocplt;

void WM8960_InitDriver(wm8960_t configWM) {
	wm8960_control.i2c_select_port = configWM.i2c_select_port;
	wm8960_control.i2c_sendDat_port = configWM.i2c_sendDat_port;
	wm8960_control.i2c_recvDat_port = configWM.i2c_recvDat_port;
	wm8960_control.i2s_select_port = configWM.i2s_select_port;
	wm8960_control.i2s_sendDat_port=configWM.i2s_sendDat_port;

	continuePlaying=false;

	if (wm8960_control.i2c_select_port(1, WM8960_ADDRESS) < 0) {
		printf("Error, Instancia i2c no disponible\n\r");
	} else {
		printf("Instancia i2c seleccionada\n\r");
	}

	wm8960_control.i2s_select_port(2);

	WM8960_Reset();
	WM8960_pwrMngmt();
	WM8960_setRateDAC();
	WM8960_controlDAC();
	WM8960_setInterfaceAudio();
	WM8960_setGainL1out(47);
	WM8960_setGainSPKout(79);
	WM8960_setGainDAC(127.0);
	WM8960_enableOuts(CLASS_D_BOTH);
	WM8960_controlMixer();
	WM8960_detectHeadPjack(true);
	WM8960_setADDcontrols();
}

static void WM8960_Reset(void) {
	uint16_t val = 0;
	if (wm8960_control.i2c_sendDat_port(RESET_IC_CODEC, val, regValues) < -1) {
		printf("Reset fallido\n\r");
	} else {
		printf("Reset Exitoso\n\r");
	}
}


static void WM8960_pwrMngmt(void) {
	wm8960_control.i2c_sendDat_port(PWR_MGMT1, 0b111000000, regValues);	// Fast start + Vref up
	wm8960_control.i2c_sendDat_port(PWR_MGMT2, 0b111111000, regValues);	// DACL On + DACR On + LOUT1 On+ ROUT1 On+ SPKL On+ SPKR On
	wm8960_control.i2c_sendDat_port(PWR_MGMT3, 0b000001100, regValues);	// LEFT OutMix On + RIGHT OutMix On
}

static void WM8960_setRateDAC(void) {
	// MCLK 25MHZ crystal included

	wm8960_control.i2c_sendDat_port(CLK1, 0b000000000, regValues);//	DACDIV (/1.0*256) + SYSCLKDIV (MCLK/1)

}

static void WM8960_controlDAC(void) {
	wm8960_control.i2c_sendDat_port(CTR1, 0b000000000, regValues);//	DAC Soft mute OFF (No mute)
}

static void WM8960_setInterfaceAudio(void) {
	wm8960_control.i2c_sendDat_port(AUD_INTERF1, 0b000000010, regValues);//	Audio Data Word Length 24

}

static void WM8960_setGainL1out(uint8_t vol) {
	uint8_t offset = OFFSETLOUTVOL; //-73dB

	uint16_t valReg = 0, current_valReg = 0;

	current_valReg = wm8960_control.i2c_recvDat_port(LOUT1_VOL, regValues);
	valReg = current_valReg & 0b110000000;
	valReg = valReg | (uint16_t) (vol + offset);	// set volume level in dB
	valReg = valReg | 0b100000000;			// Update volume value
	wm8960_control.i2c_sendDat_port(LOUT1_VOL, valReg, regValues);//	Audio Data Word Length 24
	wm8960_control.i2c_sendDat_port(ROUT1_VOL, valReg, regValues);//	Audio Data Word Length 24

}

static void WM8960_setGainSPKout(uint8_t vol) {
	uint8_t offset = OFFSETSPKVOL; //-73dB

	uint16_t valReg = 0, current_valReg = 0;

	current_valReg = wm8960_control.i2c_recvDat_port(LOUT2_VOL, regValues);
	valReg = current_valReg & 0b110000000;
	valReg = valReg | (uint16_t) (vol + offset);	// set volume level in dB
	valReg = valReg | 0b100000000;			// Update volume value
	wm8960_control.i2c_sendDat_port(LOUT2_VOL, valReg, regValues);//	Audio Data Word Length 24
	wm8960_control.i2c_sendDat_port(ROUT2_VOL, valReg, regValues);//	Audio Data Word Length 24

}

static void WM8960_enableOuts(ena_ClassD stateEn) {
	uint16_t valReg = 0;

	valReg = (uint16_t) (stateEn << 6);
	valReg = valReg | 0b110111;

	wm8960_control.i2c_sendDat_port(CLASSD_CTRL1, valReg, regValues);//	Enable ClassD Boths

}

static void WM8960_setGainDAC(float vol) {
	uint8_t offset = OFFSETDACVOL; // 0dB increase in 0.5dB

	uint16_t steps = (uint16_t) (vol / 0.5);

	uint16_t valReg = steps + offset;

	valReg = valReg | 0b100000000;			// Update volume value DAC
	wm8960_control.i2c_sendDat_port(LDAC_VOL, valReg, regValues);//	Audio Data Word Length 24
	wm8960_control.i2c_sendDat_port(RDAC_VOL, valReg, regValues);//	Audio Data Word Length 24

}

static void WM8960_controlMixer(void) {
	// Only DAC to Left and Righ Output, no LINPUT
	wm8960_control.i2c_sendDat_port(LOUT_MIX1, 0b100000000, regValues);
	wm8960_control.i2c_sendDat_port(ROUT_MIX2, 0b100000000, regValues);

}

static void WM8960_detectHeadPjack(bool detectFlag) {
	uint16_t valReg = 0;

	valReg = wm8960_control.i2c_recvDat_port(ADD_CTRL2, regValues);
	valReg = valReg & 0b110011111;
	valReg = valReg | 0b001000000;

	wm8960_control.i2c_sendDat_port(ADD_CTRL2, valReg, regValues);
}

static void WM8960_setADDcontrols(void) {
	wm8960_control.i2c_sendDat_port(ADD_CTRL1, 0b111000011, regValues);
	wm8960_control.i2c_sendDat_port(ADD_CTRL4, 0b000001001, regValues);
}

uint32_t WM8960_audioPlay(uint16_t *pBuffer, uint32_t audSize) {
	uint8_t res;

	if (continuePlaying) {
		return 1;
	}

	sizeAudiocplt = audSize;

	wm8960_control.i2s_sendDat_port(pBuffer, (WM8960_maximDMAsize(audSize / 2)));

	remAudiosize = (audSize / 2) - WM8960_maximDMAsize(sizeAudiocplt);

	actualPosition = pBuffer + WM8960_maximDMAsize(sizeAudiocplt);

	return res;
}

static uint32_t WM8960_maximDMAsize(uint32_t audSiz)
{
	uint32_t retSize=0;

	retSize=audSiz;
	if (audSiz>0xFFFF) {
		retSize=audSiz;
	}

	return retSize;
}

void WM8960_setFlagplay(void)
{
	continuePlaying=true;
}

void WM8960_stopFlagplay(void)
{
	continuePlaying=false;
}

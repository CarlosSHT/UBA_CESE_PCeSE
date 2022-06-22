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

#define WM8960_ADDRESS	0x34

#define INIT_BUFFER_POS	0
#define REGMAP_LEN		0x37
#define MAX_SIZE_VECTOR	0xFFFF

/**
 * Definiciones del mapa de registros del codec WM8960 (total 56)
 */
#define 	LIN_VOL			0x00	//	Left Input volume
#define 	RIN_VOL			0x01	//	Right Input volume
#define 	LOUT1_VOL		0x02	//	LOUT1 volume
#define 	ROUT1_VOL		0x03	//	ROUT1 volume
#define 	CLK1			0x04	//	Clocking (1)
#define 	CTR1			0x05	//	ADC & DAC Control (CTR1)
#define 	CTR2			0x06	//	ADC & DAC Control (CTR2)
#define 	AUD_IF1			0x07	//	Audio Interface
#define 	CLK2			0x08	//	Clocking (2)
#define 	AUD_IF2			0x09	//	Audio Interface
#define 	LDAC_VOL		0x0A	//	Left DAC volume
#define 	RDAC_VOL		0x0B	//	Right DAC volume
#define 	Reserved_1		0x0C	//	Reserved
#define 	Reserved_2		0x0D	//	Reserved
#define 	Reserved_3		0x0E	//	Reserved
#define 	RESET_IC		0x0F	//	Reset
#define 	CTRL_3D			0x10	//	3D control
#define 	ALC1			0x11	//	ALC1
#define 	ALC2			0x12	//	ALC2
#define 	ALC3			0x13	//	ALC3
#define 	NOISE_GATE		0x14	//	Noise Gate
#define 	LADC_VOL		0x15	//	Left ADC volume
#define 	RADC_VOL		0x16	//	Right ADC volume
#define 	ADD_CTRL1		0x17	//	Additional control(1)
#define 	ADD_CTRL2		0x18	//	Additional control(2)
#define 	PWR_MGMT1		0x19	//	Pwr Mgmt (1)
#define 	PWR_MGMT2		0x1A	//	Pwr Mgmt (2)
#define 	ADD_CTRL3		0x1B	//	Additional Control (3)
#define 	ANTI_POP1		0x1C	//	Anti-pop 1
#define 	ANTI_POP2		0x1D	//	Anti-pop 2
#define 	Reserved_4		0x1E	//	Reserved
#define 	Reserved_5		0x1F	//	Reserved
#define 	ADCL_SPATH		0x20	//	ADCL signal path
#define 	ADCR_SPATH		0x21	//	ADCR signal path
#define 	LOUT_MIX1		0x22	//	Left out Mix (1)
#define 	Reserved_6		0x23	//	Reserved
#define 	Reserved_7		0x24	//	Reserved
#define 	ROUT_MIX2		0x25	//	Right out Mix (2)
#define 	MOUT_MIX1		0x26	//	Mono out Mix (1)
#define 	MOUT_MIX2		0x27	//	Mono out Mix (2)
#define 	LOUT2_VOL		0x28	//	LOUT2 volume
#define 	ROUT2_VOL		0x29	//	ROUT2 volume
#define 	MOU_VOL			0x2A	//	MONOOUT volume
#define 	INB_MIX1		0x2B	//	Input boost mixer (1)
#define 	INB_MIX2		0x2C	//	Input boost mixer (2)
#define 	BYPASS1			0x2D	//	Bypass (1)
#define 	BYPASS2			0x2E	//	Bypass (2)
#define 	PWR_MGMT3		0x2F	//	Pwr Mgmt (3)
#define 	ADD_CTRL4		0x30	//	Additional Control (4)
#define 	CLASSD_C1		0x31	//	Class D Control (1)
#define 	Reserved_8		0x32	//	Reserved
#define 	CLASSD_C3		0x33	//	Class D Control (3)
#define 	PLL_N			0x34	//	PLL N
#define 	PLL_K1			0x35	//	PLL K 1
#define 	PLL_K2			0x36	//	PLL K 2
#define 	PLL_K3			0x37	//	PLL K 3

/**
 * Valores por defecto de los registros del codec, debido a que no se puede realizar una lectura I2C
 */
uint16_t defval_regs[REGMAP_LEN + 1] = { 0x097, 0x097, 0x000, 0x000, 0x000,
		0x008, 0x000, 0x00A, 0x1C0, 0x000, 0x0FF, 0x0FF, 0x000, 0x000, 0x000,
		0x000, 0x000, 0x07B, 0x100, 0x032, 0x000, 0x0C3, 0x0C3, 0x1C0, 0x000,
		0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x100, 0x100, 0x050,
		0x050, 0x050, 0x050, 0x000, 0x000, 0x000, 0x000, 0x040, 0x000, 0x000,
		0x050, 0x050, 0x000, 0x002, 0x037, 0x04D, 0x080, 0x008, 0x031, 0x026,
		0x0E9 };

/**
 * Definición de LABELs utilizando mascaras de 1 bit
 * Se encuentran los registros utilizados para el driver
 *
 * Permite el reconocimiento de la función puntual que se desea modificar
 */
//R25 (19h) Power Management (1)
#define		VMIDSEL_b0	1<<7
#define		VMIDSEL_b1	1<<8
#define 	VREF		1<<6
// R26 (1Ah) Power Management (2)
#define		DACL	1<<8	//	Left Channel DAC Enable
#define		DACR	1<<7	//	Right Channel DAC Enable
#define		LOUT1	1<<6	//	LOUT1 Output Buffer Enable
#define		ROUT1	1<<5	//	ROUT1 Output Buffer Enable
#define		SPKL	1<<4	//	Desactivar
#define		SPKR	1<<3	//	Desactivar
// R47 (2Fh) Power Management (3)
#define 	LOMIX	1<<3	//	Left Output Mixer Enable Control
#define 	ROMIX	1<<2	//	Right Output Mixer Enable Control

#define 	DACMU	1<<3

#define		OUT1VU	1<<8
#define 	HP_MinVOL	0x30
#define 	HP_MaxVOL	0x7F
#define 	HP_MutVOL	0x2F

#define		SPKVU	1<<8
#define 	SPK_MinVOL	0x30
#define 	SPK_MaxVOL	0x7F
#define 	SPK_MutVOL	0x2F

#define 	DACVU	1<<8
#define 	DAC_MinVOL	0x01
#define 	DAC_MaxVOL	0xFF
#define 	DAC_MutVOL	0x00

#define		LD2LO	1<<8
#define 	LI2LO	1<<7
#define		RD2RO	1<<8
#define 	RI2RO	1<<7

#define 	HPSWEN	1<<6
#define		HPSWPOL	1<<5

#define		TSENSEN	1<<1
#define 	MBSEL	1<<0
#define		HPSEL_b0	1<<2
#define		HPSEL_b1	1<<3

/**
 * Tipo de formato para la comunicación I2S en el STM32: Justificado izquierda, justificado derecha, I2S, formato DSP
 */
typedef enum _WM8960_i2sFormat {
	FORMAT_RLEFT = 0, FORMAT_JLEFT, FORMAT_I2S, FORMAT_DSP,
} wmAFormat_t;

/**
 * Tipo de interfaz utilizada para el envío de datos, es decir el largo de la palabra (se utilizará 24 bits)
 */
typedef enum _WM8960_i2sWL {
	IF_WL_16bits = 0, IF_WL_20bits, IF_WL_24bits, IF_WL_32bits,
} wmWordLen_t;

/**
 * Estado de activación del amplificador Clase D que posee el codec: apagado, canal izquierd, canal derecho, ambos canales
 */
typedef enum _ena_ClassD {
	CLASS_D_OFF = 0, CLASS_D_LSPK, CLASS_D_RSPK, CLASS_D_BOTH,
} ena_ClassD;

/**
 * Estados para la activación desactivación de los bits de cada LABEL
 */
typedef enum _labelState {
	LBL_CLEAR = 0, LBL_SET
} _wm_lblState;

/**
 * Estados de MUTE y NOMUTE para el silenciamiento por sofware del DAC del codec
 */
typedef enum _dacMute {
	NOMUTE = 0, MUTE
} _wm_dacMute;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Funciones Restringidas
 */

/**
 * @function _WM8960_updtRegbit
 * Permite la actualización de un bit del registro deseado, obtiene el valor
 * almacenado y se actualiza utilizando una mascara otorgada por el LABEL
 *
 * @param reg		Registro a configurar (tipo byte)
 * @param lbl_val	Mascara LABEL de 2 bytes
 * @param lbl_flag	Flag de actualización, LBL_CLEAR establece en 0 la posición dada por la mascara LABEL
 * 					LBL_SET establece en 1 la posición dadad por la mascara LABEL
 */
static void _WM8960_updtRegbit(uint8_t reg, uint16_t lbl_val,
		_wm_lblState lbl_flag);

/**
 * @function _WM8960_updtRegbytes
 * La función actualiza el valor de un registro no reservado del codec.
 * Se actualiza completamente con el nuevo valor, no se aplica máscara alguna
 *
 * @param reg		: Identificador de registro en byte
 * @param lbl_val	: Valor en 2 bytes del registro
 */
static void _WM8960_updtRegbytes(uint8_t reg, uint16_t lbl_val);

// Realiza un restablecimiento de valores por defecto del codec: RESET
static void _WM8960_Reset(void);

// Configuración de registros Power Managment
static void _WM8960_pwrMgtDAC(void);

// Configuración del codec para el funcionamiento con frecuencia de muestreo 44.1Khz (cercano)
static void _WM8960_setClkSR(void);

// Habilita o deshabilita la función MUTE por software del codec
static void _WM8960_softMuteDAC(_wm_dacMute mute_flag);

// Formato de la interfaz de audio del codec tipo de formato y tamaño de la palabra
static void _WM8960_setIF1Audio(void);

// Ganancia del canal izquierdo del DAC con parametro de entrada en dB
static void _WM8960_dacGAINleft(uint16_t ldacvol);

// Ganancia del canal izquierdo del DAC con parametro de entrada en dB
static void _WM8960_dacGAINright(uint16_t rdacvol);

// Ganancia del canal izquierdo de la salida HEADPHONE con parametro de entrada en dB
static void _WM8960_hdGAINleft(uint16_t ldacvol);

// Ganancia del canal derecho de la salida HEADPHONE con parametro de entrada en dB
static void _WM8960_hdGAINright(uint16_t rdacvol);

// Ganancia del canal izquierdo de la salida SPEAKER con parametro de entrada en dB
static void _WM8960_spkGAINleft(uint16_t ldacvol);

// Ganancia del canal derecho de la salida SPEAKER con parametro de entrada en dB
static void _WM8960_spkGAINright(uint16_t rdacvol);

/**
 * Habilita o deshabilita el amplificador clase D de las salidas Speaker
 * @param stateEn
 */
static void _WM8960_enCDSPKouts(ena_ClassD stateEn);

static void _WM8960_enMIXleft();
static void _WM8960_enMIXright();
/**
 * Habilita la configuración de MIXER solo de canales DAC: derecha e izquierda
 */
static void _WM8960_setMIX(void);

/**
 * Configuración de registros de controles adicionales
 * para la reprodución de audio por conector jack
 */
static void _WM8960_setEXTRActrls(void);

/**
 * Función que realiza la transmisión I2S de bloques de datos del vector de audio
 * cuya cantidad de elementos máximo es 0xFFFF y actualiza la posición inicial del
 * siguiente conjunto de datos a enviar.
 *
 * @param pBuffer	Vector de audio del tipo uint16_t
 * @param audSize	Cantidad de elementos del vector de audio
 */
static void _WM8960_playBuffer(uint16_t *pBuffer, uint32_t audSize);

/**
 * Maquina de estados finito para la reproducción de una variable de audio (vector)
 * realizado por bloques de tamaño máximo 0xFFFF
 *
 * @param pBuffer	Vector de audio del tipo uint16_t conteniendo los canales utilizados (izquierda y/o derecha)
 * @param audSize	Tamaño del vector de audio
 */
static void _WM8960_playFSM(uint16_t *pBuffer, uint32_t audSize);

//static void _WM8960_initRegs2Record(void);
//static void _WM8960_recordBuffer(uint16_t *pBuffer, uint32_t audSize);
//static void _WM8960_recordFSM(uint16_t *pBuffer, uint32_t audSize) ;


/**
 * Variables globales restringidas a uso interno del driver
 *
 */
static const char *msg_01 = "Reset Fallido\n\r";
static const char *msg_02 = "Reset Exitoso\n\r";
static uint32_t init_Tx_pos = INIT_BUFFER_POS;
//static uint32_t init_Rx_pos = INIT_BUFFER_POS;

/**
 * Variables Globales Leidas: Se actualizan o leen tanto por el driver como
 * externamente a traves de funciones
 */
static wm8960_t wm8960_control;
static bool_t g_wm8960_play_IRQ = true;
static bool_t g_wm8960_pause_IRQ = false;
static bool_t g_wm8960_stop_IRQ = false;
//static bool_t g_wm8960_rec_IRQ = true;
static WM8960_prStates g_playState = pr_stopped;
static WM8960_prStates g_plastState = pr_stopped;
//static WM8960_prStates g_recState = pr_stopped;

/**
 * Funciones para manipulación de registros del CODEC WM8960
 */
static void _WM8960_updtRegbit(uint8_t reg, uint16_t lbl_val,
		_wm_lblState lbl_flag) {
	//	Lee valor registro
	uint16_t curr_val = 0;
	wm8960_control.i2c_recvDat_port(reg, &curr_val, defval_regs);

	switch (lbl_flag) {
	case LBL_CLEAR:
		lbl_val = ~lbl_val;
		curr_val = curr_val & lbl_val;
		break;
	case LBL_SET:
		curr_val = curr_val | lbl_val;
		break;
	default:
		lbl_val = ~lbl_val;
		curr_val = curr_val & lbl_val;
		break;
	}
	//	Escribe el(los) bit(s) actualizado(s)
	wm8960_control.i2c_sendDat_port(reg, curr_val, defval_regs);
}

static void _WM8960_updtRegbytes(uint8_t reg, uint16_t lbl_val) {
	wm8960_control.i2c_sendDat_port(reg, lbl_val, defval_regs);
}

/**
 * Funciones de Configuración, Restringidas para uso interno del driver.
 */

static void _WM8960_Reset(void) {
	if (wm8960_control.i2c_sendDat_port(RESET_IC, 0, defval_regs) < -1) {
		printf(msg_01);
	} else {
		printf(msg_02);
	}
}

static void _WM8960_pwrMgtDAC(void) {

	// Fast start + Vref up
	_WM8960_updtRegbit(PWR_MGMT1, VMIDSEL_b0, LBL_SET);
	_WM8960_updtRegbit(PWR_MGMT1, VMIDSEL_b1, LBL_SET);
	_WM8960_updtRegbit(PWR_MGMT1, VREF, LBL_SET);

	_WM8960_updtRegbit(PWR_MGMT2, DACL, LBL_SET);
	_WM8960_updtRegbit(PWR_MGMT2, DACR, LBL_SET);
	_WM8960_updtRegbit(PWR_MGMT2, LOUT1, LBL_SET);
	_WM8960_updtRegbit(PWR_MGMT2, ROUT1, LBL_SET);
	_WM8960_updtRegbit(PWR_MGMT2, SPKL, LBL_CLEAR);
	_WM8960_updtRegbit(PWR_MGMT2, SPKR, LBL_CLEAR);

	_WM8960_updtRegbit(PWR_MGMT3, LOMIX, LBL_SET);
	_WM8960_updtRegbit(PWR_MGMT3, ROMIX, LBL_SET);
}

static void _WM8960_setClkSR(void) {
	// MCLK 25MHZ crystal included

	wm8960_control.i2c_sendDat_port(CLK1, 0x000, defval_regs); //	DACDIV (/1.0*256) + SYSCLKDIV (MCLK/1)
//	wm8960_control.i2c_sendDat_port(CLK1, 1 << 4, defval_regs); //	DACDIV (/1.0*256) + SYSCLKDIV (MCLK/1)

}

static void _WM8960_softMuteDAC(_wm_dacMute mute_flag) {
	_WM8960_updtRegbit(CTR1, DACMU, mute_flag);
}

static void _WM8960_setIF1Audio(void) {
	_WM8960_updtRegbytes(AUD_IF1, IF_WL_16bits << 2 | FORMAT_I2S << 0);	//	Audio Data Word Length 24

}

static void _WM8960_dacGAINleft(uint16_t ldacvol) {

	_WM8960_updtRegbit(LDAC_VOL, ldacvol | DACVU, LBL_SET);
}

static void _WM8960_dacGAINright(uint16_t rdacvol) {

	_WM8960_updtRegbit(RDAC_VOL, rdacvol | DACVU, LBL_SET);
}

static void _WM8960_hdGAINleft(uint16_t ldacvol) {

	_WM8960_updtRegbit(LOUT1_VOL, ldacvol | OUT1VU, LBL_SET);
}

static void _WM8960_hdGAINright(uint16_t rdacvol) {

	_WM8960_updtRegbit(ROUT1_VOL, rdacvol | OUT1VU, LBL_SET);
}

static void _WM8960_spkGAINleft(uint16_t ldacvol) {

	_WM8960_updtRegbit(LOUT2_VOL, ldacvol | OUT1VU, LBL_SET);
}

static void _WM8960_spkGAINright(uint16_t rdacvol) {

	_WM8960_updtRegbit(ROUT2_VOL, rdacvol | OUT1VU, LBL_SET);
}

static void _WM8960_enCDSPKouts(ena_ClassD stateEn) {
	_WM8960_updtRegbit(CLASSD_C1, stateEn << 6, LBL_SET); // Start OFF
}

static void _WM8960_enMIXleft() {

	_WM8960_updtRegbit(LOUT_MIX1, LD2LO, LBL_SET);
	_WM8960_updtRegbit(LOUT_MIX1, LI2LO, LBL_CLEAR);
}

static void _WM8960_enMIXright() {

	_WM8960_updtRegbit(ROUT_MIX2, RD2RO, LBL_SET);
	_WM8960_updtRegbit(LOUT_MIX1, RI2RO, LBL_CLEAR);
}

static void _WM8960_setMIX(void) {
	// Only DAC to Left and Righ Output, no LINPUT
	_WM8960_enMIXleft();
	_WM8960_enMIXright();
}

static void _WM8960_setEXTRActrls(void) {
	// Register ADD_CTRL1 default values no jackdetect, no speaker
	// Register ADDCTRL4 disable TSENSEN, MBSEL=1, HPSEL=2
	_WM8960_updtRegbit(ADD_CTRL4, TSENSEN, LBL_CLEAR);
	_WM8960_updtRegbit(ADD_CTRL4, MBSEL, LBL_SET);
	_WM8960_updtRegbit(ADD_CTRL4, HPSEL_b0, LBL_CLEAR);
	_WM8960_updtRegbit(ADD_CTRL4, HPSEL_b1, LBL_SET);
}



/*static void _WM8960_initRegs2Record(void) {
	_WM8960_updtRegbytes(0x19, 0x00E8);
	_WM8960_updtRegbytes(0x2F, 0x003C);
	_WM8960_updtRegbytes(0x00, 0x003F | 0x0100);
	_WM8960_updtRegbytes(0x01, 0x003F | 0x0100);
	_WM8960_updtRegbytes(0x20, 0x0008 | 0x0100);
	_WM8960_updtRegbytes(0x21, 0x0000);
	_WM8960_updtRegbytes(0x15, 0x00C3 | 0x0100);
	_WM8960_updtRegbytes(0x16, 0x00C3 | 0x0100);
	_WM8960_updtRegbytes(0x14, 0x00F9);

}*/


static void _WM8960_playBuffer(uint16_t *pBuffer, uint32_t audSize) {

	uint16_t txI2Ssize = 0;
	uint32_t new_initpos = 0;

	if ((audSize - init_Tx_pos) > MAX_SIZE_VECTOR) {
		new_initpos = init_Tx_pos + MAX_SIZE_VECTOR;
		txI2Ssize = MAX_SIZE_VECTOR;
	} else {
		txI2Ssize = (uint16_t) (audSize - init_Tx_pos + 1);
	}

	wm8960_control.i2s_sendDat_port(pBuffer + init_Tx_pos, txI2Ssize);

	init_Tx_pos = new_initpos;

}

static void _WM8960_playFSM(uint16_t *pBuffer, uint32_t audSize) {

	switch (g_playState) {
	case pr_start:
		init_Tx_pos = 0;
		_WM8960_playBuffer(pBuffer, audSize);
		g_playState = pr_pending;
		break;
	case pr_pending:
		if (audSize - init_Tx_pos <= 0xFFFF) {
			g_playState = pr_stopped;
		}
		_WM8960_playBuffer(pBuffer, audSize);
		break;
	case pr_paused:

		break;
	case pr_stopped:
		init_Tx_pos = 0;
		break;
	default:
		break;
	}
}


/*static void _WM8960_recordBuffer(uint16_t *pBuffer, uint32_t audSize) {

	uint16_t rxI2Ssize = 0;
	uint32_t new_initRXpos = 0;

	if ((audSize - init_Rx_pos) > MAX_SIZE_VECTOR) {
		new_initRXpos = init_Rx_pos + MAX_SIZE_VECTOR;
		rxI2Ssize = MAX_SIZE_VECTOR;
	} else {
		rxI2Ssize = (uint16_t) (audSize - init_Rx_pos + 1);
	}

	wm8960_control.i2s_recvDat_port(pBuffer + init_Rx_pos, rxI2Ssize);

	init_Rx_pos = new_initRXpos;

}

static void _WM8960_recordFSM(uint16_t *pBuffer, uint32_t audSize) {
	switch (g_recState) {
	case pr_start:
		init_Rx_pos = 0;
		_WM8960_recordBuffer(pBuffer, audSize);
		g_recState = pr_pending;
		break;
	case pr_pending:
		if (audSize - init_Rx_pos <= 0xFFFF) {
			g_recState = pr_stopped;
		}
		_WM8960_recordBuffer(pBuffer, audSize);
		break;
	case pr_stopped:
		init_Rx_pos = 0;
		break;
	default:
		break;
	}
}*/


/**
 * Funciones provistas por el Driver para el acceso externo
 */

void WM8960_InitDriver(wm8960_t configWM) {
	wm8960_control.i2c_sendDat_port = configWM.i2c_sendDat_port;
	wm8960_control.i2c_recvDat_port = configWM.i2c_recvDat_port;
	wm8960_control.i2s_sendDat_port = configWM.i2s_sendDat_port;
	wm8960_control.i2s_pausDat_port = configWM.i2s_pausDat_port;
	wm8960_control.i2s_resuDat_port = configWM.i2s_resuDat_port;
	wm8960_control.i2s_stopDat_port = configWM.i2s_stopDat_port;
//	wm8960_control.i2s_recvDat_port = configWM.i2s_recvDat_port;

	_WM8960_Reset();
	_WM8960_pwrMgtDAC();
	_WM8960_setClkSR();
	_WM8960_softMuteDAC(NOMUTE);
	_WM8960_setIF1Audio();

	WM8960_setGainHPs(80);
	WM8960_setGainSPKs(0);
	WM8960_setGainDAC(80);
	_WM8960_enCDSPKouts(CLASS_D_OFF);
	_WM8960_setMIX();
	WM8960_dtcHPjack(false);
	_WM8960_setEXTRActrls();

//	_WM8960_initRegs2Record();
}

void WM8960_setGainDAC(float vol) {
	uint16_t msteps = 0;

	msteps = (uint16_t) (vol / 0.5) + DAC_MinVOL;
	if (msteps > DAC_MaxVOL)
		msteps = DAC_MaxVOL;
	if (msteps < DAC_MinVOL)
		msteps = DAC_MutVOL;

	_WM8960_dacGAINleft(msteps);
	_WM8960_dacGAINright(msteps);

}

void WM8960_setGainHPs(int8_t hp_vol) {
	uint16_t chs_vol = 0;

	chs_vol = HP_MinVOL + (uint16_t) hp_vol;
	if (chs_vol > HP_MaxVOL)
		chs_vol = HP_MaxVOL;
	if (chs_vol < HP_MinVOL)
		chs_vol = HP_MutVOL;

	_WM8960_hdGAINleft(chs_vol);
	_WM8960_hdGAINright(chs_vol);
}

void WM8960_dtcHPjack(bool_t detectFlag) {
	if (detectFlag==false) {
		_WM8960_updtRegbit(ADD_CTRL2, HPSWEN, LBL_CLEAR); // << Disable HP jack detection
		_WM8960_updtRegbit(ADD_CTRL2, HPSWPOL, LBL_CLEAR); // << HPDETECT high = headphone
	}
	else {
		_WM8960_updtRegbit(ADD_CTRL2, HPSWEN, LBL_SET);
		_WM8960_updtRegbit(ADD_CTRL2, HPSWPOL, LBL_SET);
	}
}

void WM8960_setGainSPKs(uint8_t spk_vol) {
	uint16_t chs_vol = 0;

	chs_vol = SPK_MinVOL + (uint16_t) spk_vol;
	if (chs_vol > SPK_MaxVOL)
		chs_vol = SPK_MaxVOL;
	if (chs_vol < SPK_MinVOL)
		chs_vol = SPK_MutVOL;

	_WM8960_spkGAINleft(chs_vol);
	_WM8960_spkGAINright(chs_vol);
}

/********** PLAY *************/
void WM8960_setIRQplay(void) {
	g_wm8960_play_IRQ = true;
}
void WM8960_setIRQpause(void) {
	if (g_playState==pr_start || g_playState==pr_pending) {
		g_plastState=g_playState;
		g_playState=pr_paused;
		g_wm8960_pause_IRQ=true;
	}
	else if (g_playState==pr_paused) {
		g_playState=g_plastState;
		g_plastState=pr_paused;
		g_wm8960_pause_IRQ=true;
	}
}
void WM8960_setIRQstop(void) {
	if (g_playState==pr_start || g_playState==pr_pending) {
		g_plastState=pr_stopped;
		g_wm8960_stop_IRQ=true;
	}
}

void WM8960_audioPlayVar(uint16_t *pBuffer, uint32_t audSize) {
	if (g_playState != pr_stopped && g_wm8960_play_IRQ) {
		g_wm8960_play_IRQ = false;
		_WM8960_playFSM(pBuffer, audSize);
	}
	if (g_playState == pr_paused && g_wm8960_pause_IRQ) {
		g_wm8960_pause_IRQ=false;
		wm8960_control.i2s_pausDat_port();
	}
	else if ((g_playState == pr_start || g_playState == pr_pending) && g_plastState==pr_paused && g_wm8960_pause_IRQ) {
		g_wm8960_pause_IRQ=false;
		wm8960_control.i2s_resuDat_port();
	}
	else if (g_playState== pr_stopped && g_wm8960_stop_IRQ) {
		g_wm8960_stop_IRQ=false;
		g_plastState= pr_stopped;
		init_Tx_pos = 0;
		wm8960_control.i2s_stopDat_port();
	}
}

WM8960_prStates WM8960_getStatePlayer(void) {
	WM8960_prStates auxiliar;
	auxiliar = g_playState;
	return auxiliar;
}

void WM8960_setStatePlayer(WM8960_prStates newPlayState) {
	g_playState = newPlayState;
}

/********** RECORD (funcionamiento con error "Ruido") *************/
/*
void WM8960_setIRQrec(void) {
	g_wm8960_rec_IRQ = true;
}

void WM8960_audioRecVar(uint16_t *pBuffer, uint32_t audSize) {
	if (g_recState != pr_stopped && g_wm8960_rec_IRQ) {
		g_wm8960_rec_IRQ = false;
		_WM8960_recordFSM(pBuffer, audSize);
	}
}

WM8960_prStates WM8960_getStateRecorder(void) {
	WM8960_prStates auxiliar;
	auxiliar = g_recState;
	return auxiliar;
}

void WM8960_setStateRecorder(WM8960_prStates newRecState) {
	g_recState = newRecState;
}
*/

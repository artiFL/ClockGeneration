/*
 * adf435x.h
 *
 *  Created on: 20 сент. 2022 г.
 *      Author: flegler.a
 */

#ifndef INC_ADF435X_H_
#define INC_ADF435X_H_

#include <stdio.h>

#define SHL(x,y) ((uint32_t)1<<y)*x

typedef enum
{
	ADF4350_TYPE = 0, ADF4351_TYPE = 1
} TypeIcTypeDef;

typedef struct
{
	TypeIcTypeDef Type_ic;
	uint32_t ref_clk;

	// Register 0:
	uint16_t INT;
	uint16_t FRAC;

	// Register 1:
	uint8_t phase_adj;
	uint8_t prescaler;
	uint16_t phase;
	uint16_t MOD;

	// Register 2:
	uint8_t low_noise_spur;
	uint8_t muxout;

	uint8_t ref_doubler;

	uint8_t rdiv2;
	uint16_t r_counter;
	uint8_t dbl_buf;
	uint8_t charge_pump_current;
	uint8_t ldf;
	uint8_t ldp;
	uint8_t pd_polarity;
	uint8_t powerdown;
	uint8_t cp_three_state;
	uint8_t counter_reset;

	// Register 3:
	uint8_t band_mode_clksel;
	uint8_t abp;
	uint8_t chg_cancel;
	uint8_t csr;
	uint8_t clkdiv_mode;
	uint16_t clock_divider;

	// Register 4:
	uint8_t feedback_sel;
	uint8_t rf_div_sel;   // 0 = /1, 1=/2, 2=/4 ...
	uint8_t band_select_clkdiv;
	uint8_t vco_pwrdown;
	uint8_t mtld;
	uint8_t aux_outsel;
	uint8_t aux_outena;
	uint8_t aux_pwr;
	uint8_t rf_ena; // 0 - output disabled
	uint8_t out_pwr; // 0 - min, 3 - max

	// Register 5:
	uint8_t ld_pinmode;

	uint32_t reg[6];

	uint32_t pfd_freq;
	uint32_t freqence;

	void (*low_CS)(void);
	void (*high_CS)(void);
	void (*send_message)(uint8_t *buff, uint8_t size);
	void (*delay)(uint32_t value);

} adf435xSettings;

uint8_t adf435x_init(adf435xSettings *obj, TypeIcTypeDef type,
		uint32_t reference_clock, void (*cs_low_t)(void),
		void (*cs_high_t)(void), void (*SPI_send)(uint8_t*, uint8_t),
		void (*delay_t)(uint32_t));

uint8_t prepare_registers(adf435xSettings *obj, uint32_t freq);
void sendRegisterToAdf(adf435xSettings *obj, uint16_t reg_id);
void updateAllRegisters(adf435xSettings *obj);
uint8_t disable_out(adf435xSettings *obj);
uint8_t enable_out(adf435xSettings *obj);
void set_power_out(adf435xSettings *obj, uint8_t pwr);
uint8_t set_clock(adf435xSettings *obj, uint32_t freq);


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

typedef struct
{
	uint32_t CONTROL_BITS				:3;
	uint32_t FRAC						:12;	//0 - 4095
	uint32_t INT						:16;	//0 - 65535
	uint32_t RESERVED					:1;
}Register0;

typedef struct
{
	uint32_t CONTROL_BITS				:3;
	uint32_t MOD						:12;	//0 - 4095
	uint32_t PHASE						:12;	//0 - 4095
	uint32_t PRESCALLER					:1;		//0 - 1 (4/5, 8/9)
	uint32_t PHASE_ADJUST				:1;		//0 - 1 (OFF - ON)
	uint32_t RESERVED					:3;
}Register1;

typedef struct
{
	uint32_t CONTROL_BITS				:3;
	uint32_t COUNTER_RESET				:1;		//0- DISABLED 1- ENABLED
	uint32_t CP_THREE_STATE				:1;		//0- DISABLED 1- ENABLED
	uint32_t POWER_DOWN					:1;		//0- DISABLED 1- ENABLED
	uint32_t PD_POLARITY				:1;		//0- NEGATIVE 1- POSITIVE
	uint32_t LDP						:1;		//0- 10ns 1- 6ns
	uint32_t LDF						:1;		//0- FRAC-N 1- INT-N
	uint32_t CHARGE_PUMP_CURRENT		:4;		//0 - 0.31mA 15 - 5mA
	uint32_t DOUBLE_BUFFER				:1;		//0- DISABLED 1- ENABLED
	uint32_t R_COUNTER					:10;	//0-1023
	uint32_t REFERENCE_DIVIDE			:1;		//0- DISABLED 1- ENABLED
	uint32_t REFERENCE_DOUBLER			:1;		//0- DISABLED 1- ENABLED
	uint32_t MUXOUT						:3;		//
	uint32_t LOW_NOISE					:2;		//0 - LOW NOISE MODE 1- LOW SPUR MODE
	uint32_t RESERVED					:1;
}Register2;

typedef struct
{
	uint32_t CONTROL_BITS				:3;
	uint32_t CLOCK_DIVIDER				:12;	//0-4095
	uint32_t CLOCK_DIV_MODE				:2;		//0- Clock divider off
												//1- fast lock enable
												//2- resync enable
												//3- reserved
	uint32_t RESERVED					:1;
	uint32_t CSR						:1;		//0- DISABLED 1- ENABLED
	uint32_t RESERVED2					:2;
	uint32_t CHARGE_CANCELATION			:1;		//0- DISABLED 1- ENABLED
	uint32_t ABP						:1;		//0- 6ns 1- 3ns
	uint32_t BAND_SELECT_CLOCK			:1;		//0- LOW 1- HIGH
	uint32_t RESERVED3					:8;
}Register3;


typedef struct
{
	uint32_t CONTROL_BITS				:3;
	uint32_t OUTPUT_POWER				:2;		//{-4, -1, +2, +5}dBm
	uint32_t RF_OUTPUT_DIS				:1;		//0- DISABLED 1- ENABLED

	uint32_t AUX_OUTPUT_POWER			:2;		//{-4, -1, +2, +5}dBm
	uint32_t AUX_OUTPUT_DIS				:1;		//0- DISABLED 1- ENABLED
	uint32_t AUX_OUTPUT_SELECT			:1;		//0- DIVIDED OUTPUT 1- FUNDAMENTAL

	uint32_t MUTE_TILL_LOCK_DETECT		:1;		///0- MUTE DISABLED 1- MUTE ENABLED
	uint32_t VCO_POWER_DOWN				:1;		///0- VCO POWER UP 1- VCO POWER DOWN
	uint32_t CLOCK_DEVIDER				:8;		///0-255
	uint32_t RF_DIVIDER					:3;		///{1, 2, 4, 8, 16, 32, 64}
	uint32_t FEEDBACK_SECELT			:3;		///0- DIVIDED 1- FUNDAMENTAL
	uint32_t RESERVED					:8;		//
}Register4;

typedef struct
{
	uint32_t CONTROL_BITS				:3;
	uint32_t RESERVED					:19;	//
	uint32_t LOCK_DETECT_PIN			:2;		//{LOW, DIGITAL LOCK DETECT, LOW, HIGH}
	uint32_t RESERVED2					:8;		//
}Register5;


typedef struct
{
	Register0 reg0;
	Register1 reg1;
	Register2 reg2;
	Register3 reg3;
	Register4 reg4;
	Register5 reg5;
}InternalRegister;

typedef struct
{
	InternalRegister reg;

	uint32_t frequence;

	void (*low_CS)(void);
	void (*high_CS)(void);
	void (*send_message)(uint8_t *buff, uint8_t size);
	void (*delay)(uint32_t value);

}adfTypeDef;

#endif /* INC_ADF435X_H_ */

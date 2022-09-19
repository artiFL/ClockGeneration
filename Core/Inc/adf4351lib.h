/*
 * adf4351lib.h
 *
 *  Created on: 17 сент. 2022 г.
 *      Author: artif
 */

#ifndef INC_ADF4351LIB_H_
#define INC_ADF4351LIB_H_
#include <stdint.h>
#include <string.h>

#define SHL(x,y) ((uint32_t)1<<y)*x

typedef struct
{
	uint32_t REF_CLK;

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

	void (*low_CS)(void);
	void (*high_CS)(void);
	void (*send_message)(uint8_t *buff, uint8_t size);
	void (*delay)(uint32_t value);

} adf435xSettings;

adf435xSettings* adf435x_init(adf435xSettings* obj, uint32_t ref_clk,
		void (*low)(void), void (*high)(void), void (*send)(uint8_t*, uint8_t), void (*delay_t)(uint32_t));
adf435xSettings* prepare_registers(adf435xSettings* obj, uint32_t frequency);
adf435xSettings* send_register(adf435xSettings* obj, uint16_t reg_id);
adf435xSettings* update_all_register(adf435xSettings* obj);

#endif /* INC_ADF4351LIB_H_ */

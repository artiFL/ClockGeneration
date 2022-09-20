/*
 * adf435x.c
 *
 *  Created on: 20 сент. 2022 г.
 *      Author: flegler.a
 */
#include "adf435x.h"

uint8_t adf435x_init(adf435xSettings *obj, TypeIcTypeDef type,
		uint32_t reference_clock, void (*cs_low_t)(void),
		void (*cs_high_t)(void), void (*SPI_send)(uint8_t*, uint8_t),
		void (*delay_t)(uint32_t))
{
	if (obj == NULL) 				return 1;
	if (reference_clock > 50000000) return 2;
	if (reference_clock < 10000000) return 2;
	if (cs_low_t == NULL) 			return 3;
	if (cs_high_t == NULL) 			return 4;
	if (SPI_send == NULL) 			return 5;
	if (delay_t == NULL) 			return 6;

	obj->Type_ic = type;

	// Register 0:
	obj->INT = 0;
	obj->FRAC = 0;

	// Register 1:
	obj->phase_adj = 0;
	obj->prescaler = 0;
	obj->phase = 1;
	obj->MOD = 4095;
	// Register 2:
	obj->low_noise_spur = 0;
	obj->muxout = 0;

	if (reference_clock < 30000000)
		obj->ref_doubler = 1;
	else
		obj->ref_doubler = 0;

	obj->rdiv2 = 1;
	obj->r_counter = 10;
	obj->dbl_buf = 0;
	obj->charge_pump_current = 0b1111;
	obj->ldf = 1;
	obj->ldp = 0;
	obj->pd_polarity = 1;
	obj->powerdown = 0;
	obj->cp_three_state = 0;
	obj->counter_reset = 0;
	// Register 3:
	obj->band_mode_clksel = 0;
	obj->abp = 0;
	obj->chg_cancel = 0;
	obj->csr = 0;
	obj->clkdiv_mode = 0;
	obj->clock_divider = 150;
	// Register 4:
	obj->feedback_sel = 1;
	obj->rf_div_sel = 2;   // 0 = /1, 1=/2, 2=/4 ...
	obj->band_select_clkdiv = 4;
	obj->vco_pwrdown = 0;
	obj->mtld = 1;
	obj->aux_outsel = 0;
	obj->aux_outena = 0;
	obj->aux_pwr = 0;
	obj->rf_ena = 1; // 0 - output disabled
	obj->out_pwr = 3; // 0 - min, 3 - max
	obj->ld_pinmode = 1;
	obj->pfd_freq = 0;

	obj->pfd_freq = (reference_clock * (1.0 + obj->ref_doubler))
			/ (obj->r_counter * ((1.0 + obj->rdiv2)));

	obj->ref_clk = reference_clock;

	obj->low_CS = cs_low_t;
	obj->high_CS = cs_high_t;
	obj->send_message = SPI_send;
	obj->delay = delay_t;

	return 0;
}

uint8_t prepare_registers(adf435xSettings *obj, uint32_t freq)
{
	if(obj->Type_ic == ADF4351_TYPE)
	{
		if (freq >= 4400000000) return 1;
		if (freq >= 2200000000) obj->rf_div_sel = 0;
		if (freq < 2200000000) 	obj->rf_div_sel = 1;
		if (freq < 1100000000) 	obj->rf_div_sel = 2;
		if (freq < 550000000) 	obj->rf_div_sel = 3;
		if (freq < 275000000) 	obj->rf_div_sel = 4;
		if (freq < 137500000) 	obj->rf_div_sel = 5;
		if (freq < 68750000) 	obj->rf_div_sel = 6;
		if (freq <= 35000000) 	return 2;
	}
	else
	{
		if (freq >= 4400000000) return 1;
		if (freq >= 2200000000)	obj->rf_div_sel = 0;
		if (freq < 2200000000)	obj->rf_div_sel = 1;
		if (freq < 1100000000)	obj->rf_div_sel = 2;
		if (freq < 550000000)	obj->rf_div_sel = 3;
		if (freq < 275000000)	obj->rf_div_sel = 4;
		if (freq <= 137500000)	return 2;
	}

	obj->INT = (freq * (1 << obj->rf_div_sel)) / obj->pfd_freq;
	obj->FRAC = (((freq * (1 << obj->rf_div_sel)) % obj->pfd_freq) * 4095) / obj->pfd_freq;

	obj->reg[0] = SHL(obj->INT, 15) | SHL(obj->FRAC, 3);
	obj->reg[1] = SHL(obj->phase_adj, 28) | SHL(obj->prescaler, 27)
			| SHL(obj->phase, 15) | SHL(obj->MOD, 3) | 0b001;
	obj->reg[2] = SHL(obj->low_noise_spur, 29) | SHL(obj->muxout, 26)
			| SHL(obj->ref_doubler, 25) | SHL(obj->rdiv2, 24)
			| SHL(obj->r_counter, 14) | SHL(obj->dbl_buf, 13)
			| SHL(obj->charge_pump_current, 9) | SHL(obj->ldf, 8)
			| SHL(obj->ldp, 7) | SHL(obj->pd_polarity, 6)
			| SHL(obj->powerdown, 5) | SHL(obj->cp_three_state, 4)
			| SHL(obj->counter_reset, 3) | 0b010;
	obj->reg[3] = SHL(obj->band_mode_clksel, 23) | SHL(obj->abp, 22)
			| SHL(obj->chg_cancel, 21) | SHL(obj->csr, 18)
			| SHL(obj->clkdiv_mode, 15) | SHL(obj->clock_divider, 3) | 0b011;
	obj->reg[4] = SHL(obj->feedback_sel, 23) | SHL(obj->rf_div_sel, 20)
			| SHL(obj->band_select_clkdiv, 12) | SHL(obj->vco_pwrdown, 9)
			| SHL(obj->mtld, 10) | SHL(obj->aux_outsel, 9)
			| SHL(obj->aux_outena, 8) | SHL(obj->aux_pwr, 6)
			| SHL(obj->rf_ena, 5) | SHL(obj->out_pwr, 3) | 0b100;
	obj->reg[5] = SHL(obj->ld_pinmode, 22) | SHL(0b11, 19) | 0b101;

	obj->freqence = freq;
	return 0;
}

void sendRegisterToAdf(adf435xSettings *obj, uint16_t reg_id)
{
	obj->low_CS();
	obj->delay(20);

	uint8_t data = (uint8_t) (obj->reg[reg_id] >> 24);
	obj->send_message(&data, 1);
//#warning uncomment "&"
	data = (uint8_t) (obj->reg[reg_id] >> 16);
	obj->send_message(&data, 1);

	data = (uint8_t) (obj->reg[reg_id] >> 8);
	obj->send_message(&data, 1);

	data = (uint8_t) (obj->reg[reg_id]);
	obj->send_message(&data, 1);

	obj->high_CS();
	obj->delay(20);
}

void updateAllRegisters(adf435xSettings *obj)
{
	for (int i = 5; i >= 0; i--)
	{
		sendRegisterToAdf(obj, i);
	}
}

uint8_t disable_out(adf435xSettings *obj)
{
	if (obj->rf_ena == 1)
	{
		obj->rf_ena = 0;
		prepare_registers(obj, obj->freqence);
		updateAllRegisters(obj);
		return 0;
	}
	return 1;
}

uint8_t enable_out(adf435xSettings *obj)
{
	if (obj->rf_ena == 0)
	{
		obj->rf_ena = 1;
		prepare_registers(obj, obj->freqence);
		updateAllRegisters(obj);
		return 0;
	}
	return 1;
}

void set_power_out(adf435xSettings *obj, uint8_t pwr)
{
	if (pwr >= 0 && pwr <= 3)
	{
		obj->out_pwr = pwr;
		prepare_registers(obj, obj->freqence);
		updateAllRegisters(obj);
	}
}

uint8_t set_clock(adf435xSettings *obj, uint32_t freq)
{
	uint8_t status = 1;

	status = prepare_registers(obj, freq);
	if (status == 0)
	{
		updateAllRegisters(obj);
	}
	return status;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void adf_init(adfTypeDef* obj)
{
	obj->reg.reg0.CONTROL_BITS = 0;
	obj->reg.reg0.FRAC = 0;
	obj->reg.reg0.INT = 0;

	obj->reg.reg1.CONTROL_BITS = 1;
	obj->reg.reg1.PHASE = 1;
	obj->reg.reg1.MOD = 4095;




}








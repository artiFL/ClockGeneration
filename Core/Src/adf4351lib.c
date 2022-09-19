#include "adf4351lib.h"

adf435xSettings* adf435x_init(adf435xSettings* obj, uint32_t ref_clk,
		void (*low)(void), void (*high)(void), void (*send)(uint8_t*, uint8_t), void (*delay_t)(uint32_t))
{
	uint8_t refDoubler = 0;
	ref_clk < 30000000 ? (refDoubler = 1) : (refDoubler = 0);

	//if(low == NULL){}
	//if(high == NULL){return;}
	//if(send == NULL){return;}
	//if(delay_t == NULL){return;}

			obj->low_CS = low,
			obj->high_CS = high,
			obj->send_message = send,
			obj->delay = delay_t,

			obj->INT = 0,
			obj->FRAC = 0,

			obj->phase_adj = 0,
			obj->prescaler = 0,
			obj->phase = 1,
			obj->MOD = 4095,

			obj->low_noise_spur = 0,
			obj->muxout = 0,
			obj->REF_CLK = ref_clk,
			obj->ref_doubler = ref_clk < 30000000 ? (1) : (0),

					obj->rdiv2 = 1,
					obj->r_counter = 10,
					obj->charge_pump_current = 0b111,
					obj->ldf = 1,
					obj->pd_polarity = 1,

	// Register 3:
					obj->clock_divider = 150,

	// Register 4:
					obj->feedback_sel = 1,
					obj->rf_div_sel = 2, // 0 = /1, 1=/2, 2=/4 ...
	obj->band_select_clkdiv = 4,
	obj->vco_pwrdown = 0,
	obj->mtld = 1,
	obj->aux_outsel = 0,
	obj->rf_ena = 1, // 0 - output disabled
	obj->out_pwr = 3, // 0 - min, 3 - max

	// Register 5:
	obj->ld_pinmode = 1;
	obj->pfd_freq = (obj->REF_CLK * (1.0 + obj->ref_doubler)) / (obj->r_counter * ((1.0 + obj->rdiv2)));

	memset(obj->reg, 0, 6);
	return obj;
}

adf435xSettings* prepare_registers(adf435xSettings* obj, uint32_t frequency)
{
	if (frequency >= 2200000000)
		obj->rf_div_sel = 0;
	if (frequency < 2200000000)
		obj->rf_div_sel = 1;
	if (frequency < 1100000000)
		obj->rf_div_sel = 2;
	if (frequency < 550000000)
		obj->rf_div_sel = 3;
	if (frequency < 275000000)
		obj->rf_div_sel = 4;
	if (frequency < 137500000)
		obj->rf_div_sel = 5;
	if (frequency < 68750000)
		obj->rf_div_sel = 6;

	obj->INT = (frequency * (1 << obj->rf_div_sel)) / obj->pfd_freq;
	obj->FRAC = (((frequency * (1 << obj->rf_div_sel)) % obj->pfd_freq) * 4095)
			/ obj->pfd_freq;

	obj->reg[0] = SHL(obj->INT, 15) | SHL(obj->FRAC, 3);
	obj->reg[1] = SHL(obj->phase_adj, 28) | SHL(obj->prescaler, 27)
			| SHL(obj->phase, 15) | SHL(obj->MOD, 3) | 0b001;
	obj->reg[2] = SHL(obj->low_noise_spur, 29) | SHL(obj->muxout, 26)
			| SHL(obj->ref_doubler, 25) | SHL(obj->rdiv2, 24)
			| SHL(obj->r_counter, 14) | SHL(obj->dbl_buf, 13)
			| SHL(obj->charge_pump_current, 9) | SHL(obj->ldf, 8)
			| SHL(obj->ldp, 7) | SHL(obj->pd_polarity, 6) | SHL(obj->powerdown, 5)
			| SHL(obj->cp_three_state, 4) | SHL(obj->counter_reset, 3) | 0b010;
	obj->reg[3] = SHL(obj->band_mode_clksel, 23) | SHL(obj->abp, 22)
			| SHL(obj->chg_cancel, 21) | SHL(obj->csr, 18)
			| SHL(obj->clkdiv_mode, 15) | SHL(obj->clock_divider, 3) | 0b011;
	obj->reg[4] = SHL(obj->feedback_sel, 23) | SHL(obj->rf_div_sel, 20)
			| SHL(obj->band_select_clkdiv, 12) | SHL(obj->vco_pwrdown, 9)
			| SHL(obj->mtld, 10) | SHL(obj->aux_outsel, 9)
			| SHL(obj->aux_outena, 8) | SHL(obj->aux_pwr, 6) | SHL(obj->rf_ena, 5)
			| SHL(obj->out_pwr, 3) | 0b100;
	obj->reg[5] = SHL(obj->ld_pinmode, 22) | SHL(0b11, 19) | 0b101;
	return obj;
}

adf435xSettings* send_register(adf435xSettings* obj, uint16_t reg_id)
{
	uint8_t data[4] = {0,};

	data[0] = (uint8_t) (obj->reg[reg_id] >> 24);
	data[1] = (uint8_t) (obj->reg[reg_id] >> 16);
	data[2] = (uint8_t) (obj->reg[reg_id] >> 8);
	data[3] = (uint8_t) (obj->reg[reg_id]);

	obj->low_CS();

	obj->delay(20);

	obj->send_message(data, 4);

	obj->high_CS();

	obj->delay(20);

	obj->low_CS();

	return obj;
}

adf435xSettings* update_all_register(adf435xSettings* obj)
{
	for (int i = 5; i >= 0; i--)
	{
		obj = send_register(obj, i);
	}
	return obj;
}


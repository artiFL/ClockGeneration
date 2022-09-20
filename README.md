# ClockGeneration
## STM32F103C8T6 Blue pill
16MHz clock quarz oscilator

- SPI
    - MOSI - PA7
    - SCK  - PA5
    - CS   - PA4
- USB-CDC Boudrate 115200 
___
## ADF435x
ADF4351
- Clock 35-4400MHz
- Jitter 0.3ps
- Power -4dbm - +5dBm
- Interface SPI Half-duplex

ADF4350
- Clock 137.5-4400MHz
- Jitter 0.4ps
- Power -4dbm - +5dBm
- Interface SPI Half-duplex
___
## Function
``` C
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
```
### Initialization Ic, where
- "reference_clock" - clock oscilator
- "cs_low_t" - function ptr on CS LOW state
- "cs_high_t" - function ptr on CS HIGH state
- "SPI_send" - function ptr on send SPI packet
- "delay_t" - function ptr on delay
``` C
adf435xSettings adf4351; // Create prototype IC

uint8_t adf435x_init(adf435xSettings *obj, TypeIcTypeDef type,
		uint32_t reference_clock, void (*cs_low_t)(void),
		void (*cs_high_t)(void), void (*SPI_send)(uint8_t*, uint8_t),
		void (*delay_t)(uint32_t));
```
### Calculating register IC value, where
- "freq" - clock value set
``` C
uint8_t prepare_registers(adf435xSettings *obj, uint32_t freq);
```
### Send value register IC
``` C
void updateAllRegisters(adf435xSettings *obj);
```
### Set frequence output, where
- "freq" - clock value set
``` C
uint8_t set_clock(adf435xSettings *obj, uint32_t freq);
```





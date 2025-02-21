/*
 * si5351.h
 *
 * Interface for controlling the SI5351 clock generator.
 *
 * The SI5351 is used to generate the RF carrier in the SDR.
 * FM mode support in the SDR does not require changes to this interface,
 * but it is noted here for completeness.
 */

void si5351_set_calibration(int32_t cal);
void si5351bx_init(); 
void si5351bx_setfreq(uint8_t clknum, uint32_t fout);
void si5351_reset();
void si5351a_clkoff(uint8_t clk);


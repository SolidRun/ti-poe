/*
 * Implement Linux compatible routines that tps238x code needs
 */

#include <time.h>

/* The tps_UpdateSRAMCode() routine needs 100000 cycles to make a 12ms delay.
 * This means that ~8333 cycles are needed for 1ms. Use a smaller value per ms
 * (i.e. wait longer) to be on the safe side.
 */
#define CYCLES_PER_MS	5000
#define NSEC_PER_MS	1000000

void __delay_cycles(unsigned int cycles)
{
	unsigned int wait_ms = cycles/CYCLES_PER_MS + 1;
	struct timespec wait_ts = { wait_ms/1000, (wait_ms % 1000) * NSEC_PER_MS };

	nanosleep(&wait_ts, NULL);
}


/* Empty stub */
void uart_puts(char *str)
{
}

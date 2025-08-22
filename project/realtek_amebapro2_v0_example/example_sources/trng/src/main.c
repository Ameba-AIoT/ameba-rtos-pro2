#include "device.h"
#include "trng_api.h"
#include "main.h"




int main(void)
{
	u32 random_number;

	trng_init();
	random_number = trng_get_rand();
	dbg_printf("random_number = %d \n\r", random_number);

	while (1) {;}
}






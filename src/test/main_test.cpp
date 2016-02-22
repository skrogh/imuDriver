#include "imuDriver.h"

void
main(void)
{
	ImuDriver imuDriver("/dev/spidev1.0","/sys/class/gpio/gpio199/value");
	while(1);
}
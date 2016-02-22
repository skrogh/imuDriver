#include "ImuDriver.h"

int
main( int argc, const char* argv[] )
{
	ImuDriver imuDriver("/dev/spidev1.0","/sys/class/gpio/gpio199/value",100);
	while(1);

	return 0;
}
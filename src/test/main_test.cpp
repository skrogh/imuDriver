#include "ImuDriver.h"

int
main( int argc, const char* argv[] )
{
	ImuDriver imuDriver("/dev/spidev1.0","/sys/class/gpio/gpio199/value",100);
	while(1)
	{
		ImuMeas_t imuData;
		imuDriver.imuBuffer.popBlocking(imuData);
		std::cout << "Data: " << imuData.acc[0] << "\t Time: "<<
		std::chrono::duration_cast<std::chrono::nanoseconds>(imuData.timeStamp.time_since_epoch()).count()
		<< std::endl;
	}

	return 0;
}
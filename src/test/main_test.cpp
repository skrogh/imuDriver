#include "ImuDriver.h"

int
main( int argc, const char* argv[] )
{
	ImuDriver imuDriver("/dev/spidev1.0","/sys/class/gpio/gpio199/value",400);
	std::cout << "ax,\tay,\taz,\tox,\toy,\toz,\tdz,\tdv,\tt" << std::endl;
	std::cout.precision(4);
	std::cout << std::scientific;
	unsigned long long int startTime;
	unsigned long int i = 0;
	while(1)
	{
		ImuMeas_t imuData;
		imuDriver.imuBuffer.popBlocking(imuData);
		if (i++==0)
			startTime = std::chrono::duration_cast<std::chrono::nanoseconds>(imuData.timeStamp.time_since_epoch()).count();
		std::cout
			<< imuData.acc[0] << ",\t"
			<< imuData.acc[1] << ",\t"
			<< imuData.acc[2] << ",\t"
			<< imuData.gyro[0] << ",\t"
			<< imuData.gyro[1] << ",\t"
			<< imuData.gyro[2] << ",\t"
			<< imuData.dist << ",\t"
			<< imuData.distValid << ",\t"
			<< std::chrono::duration_cast<std::chrono::nanoseconds>(imuData.timeStamp.time_since_epoch()).count() << ",\t"
			<< std::chrono::duration_cast<std::chrono::nanoseconds>(imuData.timeStamp.time_since_epoch()).count() - startTime
			<< std::endl;
	}

	return 0;
}

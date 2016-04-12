#include <stdint.h>
#include <boost/thread.hpp>
#include <chrono>
#include <string>
#include "lib/concurrentBuffers/src/ConcurrentBuffers.h"

struct ImuMeas_t{
	double acc[3];
	double gyro[3];
	double alpha[3];
	double dist;
	bool distValid;
	double steerX;
	double steerY;
	double steerYaw;
	double steerThrust;
	double voltage;
	double current;
	double u[4];
	std::chrono::high_resolution_clock::time_point timeStamp;
};

struct FlightControllerOut_t{
	float x;
	float y;
	float yaw;
	float z;
};

class ImuDriver {
public:
	ImuDriver(const std::string& spiDevice, const std::string& gpioDevice, int fifoSize);
	~ImuDriver(void);
	void SetOutput(double x, double y, double yaw, double z);
	//
	// Input buffer (this is fine to expose, as it is thread safe, just don't be an idiot and add samples to it yourself)
	//
	ConcurrentFifo<ImuMeas_t> imuBuffer;
private:
	// filedescriptors
	int gpioFd, spiFd;
	
	//
	// Spi settings
	//
	uint32_t mode;
	uint32_t bits;
	uint32_t speed;
	uint16_t delay;
	uint32_t timeout;

	//
	// Thread handing
	//
	//pthread_t thread;
	mutable boost::mutex threadsShouldExitMutex;
	bool threadsShouldExit = false;

	//
	// Output to flight controller
	//
	FlightControllerOut_t flightControllerOut;
	mutable boost::mutex flightControllerOutMutex;

	//
	// Functions
	//

	// Clear SPI interrupt
	char inline ClearSpiInt(void);
	// Interrupt handler
	void GpioIntHandler(const std::chrono::high_resolution_clock::time_point& timeStamp);
	// Interrupt thread
	void InterruptThread(void);
	boost::thread* interruptThread;
	
};
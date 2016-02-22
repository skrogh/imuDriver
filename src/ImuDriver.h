#include <stdint.h>
#include <boost/thread.hpp>
#include <string>


struct ImuMeas_t{
	double acc[3];
	double gyro[3];
	double alpha[3];
	double dist;
	bool distValid;
	long timeStamp;
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
	void GpioIntHandler(long timestamp);
	// Interrupt thread
	void InterruptThread(void);
	boost::thread* interruptThread;
	
};
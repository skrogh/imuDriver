#include <stdint.h>
#include <boost/thread.hpp>

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
	ImuDriver(const string& spiDevice, const string& gpioDevice, int fifoSize);
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
	mutable boost::mutex endThreadMutex;
	bool threadsShouldExit;

	//
	// Output to flight controller
	//
	FlightControllerOut_t flightControllerOut;
	mutable boost::mutex flightControllerOutMux;

	//
	// Functions
	//

	// Clear SPI interrupt
	char inline ClearSpiInt( void );
	// Interrupt handler
	void GpioIntHandler( double timestamp );
	// Interrupt thread
	void InterruptThread(void);
	boost::thread* interruptThread;
	
};
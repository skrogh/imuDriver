#include "ImuDriver.h"
#include <linux/spi/spidev.h>
#include <fcntl.h>
#include <iostream>
#include <poll.h>
#include <cstring> // memcpy
#include <boost/chrono.hpp>
#include <pthread.h>
#include <sys/ioctl.h> // for SPI control

#define MESSAGE_LENGTH  (20*2) // flight controller sends 16bit bytes

static inline uint32_t
unpackUint32(uint8_t* from)
{
	return (from[3] << 0) | (from[2] << 8) | (from[1] << 16) | (from[0] << 24);
}

static inline void
repackUint8( uint8_t* area, int n )
{
	for( int i = 0; i < n*4; i+=4 ) {
		uint8_t tmp = area[0+i];
		area[0+i] = area[3+i];
		area[3+i] = tmp;
		tmp = area[1+i];
		area[1+i] = area[2+i];
		area[2+i] = tmp;
	}
}

static inline float
unpackFloat( uint8_t* from )
{
	uint32_t tmp = (from[ 3] << 0) | (from[ 2] << 8) | (from[1] << 16) | (from[0] << 24);
	return *(float*)&tmp;
}

static inline void
unpackFloats( uint8_t* from, float to[], uint32_t n ) {
	uint32_t i;
	for( i = 0; i < n; i++ ) {
		to[i] = unpackFloat( from );
		from += 4;
	}
}

ImuDriver::ImuDriver(const std::string& spiDevice, const std::string& gpioDevice, int fifoSize)
:
imuBuffer(fifoSize)
{
	//
	// Init variables
	//
	mode = 0;
	bits = 8;
	speed = 6000000;
	delay = 0;
	timeout = 500;

	//
	// open I/O files
	//
	// Open gpio file and check for error
	gpioFd = open( gpioDevice.c_str(), O_RDONLY | O_NONBLOCK );
	if ( gpioFd < 0 ) {
		std::cerr << "gpio file open" << std::endl;
	}

	// Open spi file and check for error
	spiFd = open( spiDevice.c_str(), O_RDWR );
	if ( spiFd < 0 ) {
		std::cerr << "spi file open" << std::endl;
	}

	//
	// Setup spi
	//
	int ret;
	// spi mode
	ret = ioctl(spiFd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1) {
		std::cerr <<"ERROR: can't set spi mode" << std::endl;
	}

	ret = ioctl(spiFd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1) {
		std::cerr <<"ERROR: can't get spi mode" << std::endl;
	}

	//bits per word
	ret = ioctl(spiFd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1) {
		std::cerr <<"ERROR: can't set bits per word" << std::endl;
	}

	ret = ioctl(spiFd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1) {
		std::cerr <<"ERROR: can't get bits per word" << std::endl;
	}

	 // max speed hz
	ret = ioctl(spiFd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1) {
		std::cerr <<"ERROR: can't set max speed hz" << std::endl;
	}

	ret = ioctl(spiFd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1) {
		std::cerr <<"ERROR: can't get max speed hz" << std::endl;
	}

	std::clog << "spi mode: 0x" << std::hex << mode << std::endl;
	std::clog << "bits per word: " << std::dec << bits << std::endl;
	std::clog << "max speed: " << speed << "Hz (" << speed/1000 << " KHz)" << std::endl;

	//
	// start thread
	//
	struct sched_param param;
	int policy = SCHED_FIFO;
	param.sched_priority = sched_get_priority_max( policy );

	boost::thread::attributes attrs;
	pthread_attr_setinheritsched( attrs.native_handle(), PTHREAD_EXPLICIT_SCHED );
	pthread_attr_setschedpolicy( attrs.native_handle(), policy );
	pthread_attr_setschedparam( attrs.native_handle(), &param );

	interruptThread = new boost::thread(attrs, boost::bind(&ImuDriver::InterruptThread, this)); // Spawn thread running the interrupt waiter and handler

}

ImuDriver::~ImuDriver( ) {
	std::clog << "Signaling to end imu thread and join" << std::endl;
	boost::mutex::scoped_lock lock(threadsShouldExitMutex);
	threadsShouldExit = true;
	lock.unlock();

	interruptThread->join();
	delete interruptThread;
	std::clog << "Thread joined" << std::endl;
	close(gpioFd);
	close(spiFd);
}

char inline
ImuDriver::ClearSpiInt(void)
{
	// clear interrupt
	char c;
	lseek(gpioFd, 0, SEEK_SET);
	(void) read(gpioFd, &c, 1);
	return c;
}

void
ImuDriver::InterruptThread(void) {
	std::clog << "Imu server: Started" << std::endl;
	int rc; // return code

	// Poll struct
	struct pollfd fdset = {};
	fdset.fd = gpioFd;
	fdset.events = POLLPRI;

	ClearSpiInt();

	while( 1 ) {
		boost::mutex::scoped_lock lock(threadsShouldExitMutex);
		if (threadsShouldExit)
			break;
		lock.unlock();

		fdset.revents = 0;

		// start waiting for next interrupt
		rc = poll( &fdset, 1, timeout );
		// clear interrupt, read status
		char value = ClearSpiInt();
		// Get time
		std::chrono::high_resolution_clock::time_point timeStamp = std::chrono::high_resolution_clock::now();

		// Check if interrupt request failed
		if (rc < 0) {
			std::cerr << "ERROR: poll() failed!" << std::endl;
		}

		// Check if timed out
		if (rc == 0){
			// check if interrupt is high, if it is we previously missed a read/write so take it now
			if ( value == '1' ) {
				std::clog << "int timeout" << std::endl;
				GpioIntHandler( timeStamp );
				continue;
			}
		}

		// Check if correct interrupt
		if (fdset.revents & POLLPRI) {
			GpioIntHandler( timeStamp );
		}
	}
	std::clog << "Imu server: Ended" << std::endl;
}

void ImuDriver::GpioIntHandler( const std::chrono::high_resolution_clock::time_point& timeStamp ) {
	int ret;
	// Create buffer and struct for SPI IO
	uint8_t tx[MESSAGE_LENGTH] = { 0 };
	uint8_t rx[MESSAGE_LENGTH] = { 0 };
	struct spi_ioc_transfer tr = {};
		tr.tx_buf = (unsigned long)tx;
		tr.rx_buf = (unsigned long)rx;
		tr.len = MESSAGE_LENGTH;
		tr.delay_usecs = delay;
		tr.speed_hz = speed;
		tr.bits_per_word = bits;

	// Copy output to flightcontroller
	boost::mutex::scoped_lock lock(flightControllerOutMutex);
	std::memcpy( tx, &flightControllerOut, sizeof(flightControllerOut) );
	lock.unlock();
	repackUint8( tx, sizeof(flightControllerOut)/sizeof(float) );


	ret = ioctl( spiFd, SPI_IOC_MESSAGE(1), &tr );
	if ( ret < 1 )
		std::cerr << "ERROR: can't send spi message" << std::endl;


	float acc[3];
	unpackFloats( &rx[0], acc, 3 );
	float gyro[3];
	unpackFloats( &rx[sizeof(acc)], gyro, 3 );
	float alpha[3];
	unpackFloats( &rx[sizeof(acc)+sizeof(gyro)], alpha, 3 );
	uint32_t ping = unpackUint32( &rx[sizeof(acc)+sizeof(gyro)+sizeof(alpha)] );
	double dist = ping * ( 340.29 / 80000000.0 ) / 2.0;

	ImuMeas_t element;
	element.timeStamp = timeStamp;
	element.dist = dist;
	element.distValid = ( ping != 0 );
	for ( int i = 0; i < 3; i++ ) {
		element.acc[i] = acc[i];
		element.gyro[i] = gyro[i];
		element.alpha[i] = alpha[i];
	}
	imuBuffer.push(element);
}


void
ImuDriver::SetOutput( double x, double y, double yaw, double z )
{
	boost::mutex::scoped_lock lock(flightControllerOutMutex);
	flightControllerOut.x = x;
	flightControllerOut.y = y;
	flightControllerOut.yaw = yaw;
	flightControllerOut.z = z;
}
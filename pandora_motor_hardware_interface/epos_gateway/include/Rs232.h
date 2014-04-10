#ifndef RS232_H
#define RS232_H

#include <iostream>
#include <string>

#include <cstring>

#include <stdint.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>

#include "ros/ros.h"

/**
 * \brief Simple C++ wrapper of unix serial port interface
 * 
 * Open serial ports for read and write in binary mode. Provides the
 * necessary locking mechanisms to prevent concurrent access to the
 * serial port.
 * 
 * \author Charalampos Serenis
 * \author Electical and Computer Engineer
 * \author Department of Electrical and Computer Engineering
 * \author Aristotle University of Thessaloniki, Greece
 * 
 */

class Rs232{
	// serial port file descriptor. fd is -1 when port is closed
	int fd;
public:
	///Initializes internal variables but doesn't open serial port
	Rs232(void);
	/** Initializes internal variables and opens serial port for binary
	 * read / write.
	 * \param port the serial device to open
	 * \param baudRate port baud rate in bits per second
	 * \param timeout time in ms to wait for rear/write operations to
	 * complete
	 * \see open
	 */
	Rs232(std::string port,unsigned int baudRate,unsigned int timeout);
	/** Open serial port for binary rear / write
	 * 
	 * Opens the serial device defined by parameter port for binary read /
	 * write. In the event that the port is already opened an error is 
	 * generated, and no new port is open.
	 * 
	 * \param port the serial port to open
	 * \param baudRate port baud rate in bits per second
	 * \param timeout time in ms to wait fore read / write operations to
	 * complete
	 */
	int open(std::string port,unsigned int speed=38400,int iTimeOut=500);
	/** Close serial port
	 * 
	 * Close serial port currently accessed by the object. In the event
	 * that no port is open by the object a warning is generated and no
	 * action is taken.
	 */
	int close(void);
	/** Write data to serial port
	 * 
	 * Write size number of bytes to the serial port, from memory pointed
	 * by data. This is a blocking operation. During write the object is
	 * locked and any concurrent access to the serial port (i.e. read,
	 * write, close) will wait until the operation finishes or fails.
	 * Access to the same serial port by another program or by another
	 * object holding a valid file descriptor to the serial port will
	 * not be blocked.
	 * 
	 * \param data a pointer to the data that need to be written to the
	 * serial port
	 * \param size the number of bytes that need to be written to the
	 * serial port
	 * \return on success the number of bytes written to the serial port,
	 * which is the same as size
	 * \return on error the error number returned by the OS. An error
	 * message is output to stderr
	 */
	int write(char *data,unsigned int size);
//	int write(char data);
	int write(unsigned char data);
	int write(unsigned char *data, unsigned int size);
	int read(char *data,unsigned int size);
	int read(unsigned char *data, unsigned int size);
	int read(char& data);
	void forceUnlock(void);
};

#endif


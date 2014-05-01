/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, P.A.N.D.O.R.A. Team.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Orestis Zachariadis
*********************************************************************/

using namespace std;

int main(void) {
	int fd;

//	fd = open("/dev/head", O_RDWR | O_NOCTTY | O_NDELAY);	//to make read non-blocking
	fd = open("/dev/head", O_RDWR | O_NOCTTY );
	if (fd == -1) {
		puts("[Head]: cannot open port");
		printf("\n open() failed with error [%s]\n", strerror(errno));
		return -1;
	} else {
		puts("[Head]: usb port successfully opened\n");
	}
	usleep(30000);	//needs some time to initialize, even though it opens succesfully. tcflush() didn't work
					//without waiting at least 8 ms

	int CO2nbytes = 4;
	int TPA81nbytes = 8*8;
	int nbytesOUT = 1;

	union {
		unsigned char CO2bufIN[4];
		float CO2bufIN_float;
	};

	unsigned char TPAbufIN[8*8];

	int bufOUT;

	int nr;

//	fcntl(fd, F_SETFL, FNDELAY);	//make read() non-blocking
//	fcntl(fd, F_SETFL, 0);	//make read() blocking

	for (;;) {
		tcflush(fd, TCIFLUSH);	//empties incoming buffer

		bufOUT = 1;
		nr = write(fd, (const void *)&bufOUT, nbytesOUT);
		if (nr!=1) {
			reconnectUSB(fd);
			continue;
		}
		nr=read(fd, TPAbufIN, TPA81nbytes);	//blocking
		if (nr<0) cout << "Read Error" << endl;
		cout << "TPA1 = ";
		for (int i = 0; i < TPA81nbytes; ++i) {
			cout << (int)TPAbufIN[i] << " ";
		}
		cout << endl;


		bufOUT = 2;
		nr = write(fd, (const void *)&bufOUT, nbytesOUT);
		if (nr!=1) {
			reconnectUSB(fd);
			continue;
		}
		nr=read(fd, TPAbufIN, TPA81nbytes);	//blocking
		if (nr<0) cout << "Read Error" << endl;
		cout << "TPA2 = ";
		for (int i = 0; i < TPA81nbytes; ++i) {
			cout << (int)TPAbufIN[i] << " ";
		}
		cout << endl;


		bufOUT = 3;
		nr = write(fd, (const void *)&bufOUT, nbytesOUT);
		if (nr!=1) {
			reconnectUSB(fd);
			continue;
		}
		nr=read(fd, TPAbufIN, TPA81nbytes);	//blocking
		if (nr<0) cout << "Read Error" << endl;
		cout << "TPA3 = ";
		for (int i = 0; i < TPA81nbytes; ++i) {
			cout << (int)TPAbufIN[i] << " ";
		}
		cout << endl;


		bufOUT = 4;
		nr = write(fd, (const void *)&bufOUT, nbytesOUT);
		if (nr!=1) {
			reconnectUSB(fd);
			continue;
		}
		nr=read(fd, CO2bufIN, CO2nbytes);	//blocking
		if (nr<0) cout << "Read Error" << endl;
		cout << "CO2 = " << CO2bufIN_float << endl;

		usleep(200*1000);
	}

	close(fd);
	cout << "Closed" << endl;
	return EXIT_SUCCESS;
}

void reconnectUSB(int fd) {
	cout << "Write Error" << endl;
	close(fd);
	cout << "Closed" << endl;
	//If usb disconnects and reconnects again 1.5s should be fine, if uC resets 4.5s required.
	//reconnectUSB() is called until communication is restored.
	usleep(1500*1000);
	fd = open("/dev/head", O_RDWR | O_NOCTTY );
	if (fd == -1) {
		puts("[Head]: cannot open port");
		printf("\n open() failed with error [%s]\n", strerror(errno));
	} else {
		puts("[Head]: usb port successfully opened\n");
	}
	usleep(30000);
}


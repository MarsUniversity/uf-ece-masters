#include "serial.h"
#include <sys/ioctl.h>

/*!
	Returns the number of bytes specified by the
	portDescriptor.
*/
int kbytesInInputBuffer( int portDescriptor ){
  int bytes = 0;

  ioctl(portDescriptor, FIONREAD, &bytes) ;

  return bytes ;
}


/*!
	Returns the number of bytes specified by the
	portDescriptor.
	\todo this may not work properly!
*/
int kbytesInOutputBuffer( int portDescriptor ){
	int bytes = 0;

	ioctl(portDescriptor, FIONREAD, &bytes);

	return bytes;
}


int print_speed(int fd){
	struct termios p;

	tcgetattr(fd,&p); // get port settings

	printf("speed out/in: ");
	switch(cfgetospeed(&p)){ // output speed
		case B9600:
			printf("9600");
			break;
		case B38400:
			printf("38400");
			break;
		default:
			printf("unknown");
	}

	switch(cfgetispeed(&p)){ // input speed
		case B9600:
			printf("/9600");
			break;
		case B38400:
			printf("/38400");
			break;
		default:
			printf("/unknown");
	}

	printf("\n");

	return 1;
}

/*!
	Opens a serial port with the flags:
	O_RDWR - read/write mode
	O_NOCTTY - not the controlling terminal
	O_NDELAY - do not pay attentention to Data Carrier Detect (DCD)
	\param port a character string of the device
	\param baud speed to open port
	\return file descriptor
*/
int open_serial_port(char *port, int baud){
  int fd;
	int speed;
	char msg[70];
  struct termios opts;

  fd = open( port, O_RDWR | O_NOCTTY | O_NDELAY ); //may not need the nodelay
  if( fd <= 0 ){
		sprintf(msg,"ERROR: Unable to open port %s:%d - %s",port,baud,strerror(errno));
		perror(msg);
		return(-1);
	}

  // Set the baud rate
  switch(baud){
		case 1200:
			speed = B1200; break;
		case 2400:
			speed = B2400; break;
		case 4800:
			speed = B4800; break;
		case 9600:
			speed = B9600; break;
		case 19200:
			speed = B19200; break;
		case 38400:
			speed = B38400; break;
		case 57600:
			speed = B57600; break;
		default:
			printf("ERROR: open_serial_port() invalid speed\n");
	}

  bzero( &opts, sizeof( opts ) );
  opts.c_cflag = speed | CS8 | CLOCAL | CREAD /*| CRTSCTS*/;
  opts.c_iflag = IGNPAR;
  opts.c_oflag = 0;
  opts.c_lflag = 0;
  opts.c_cc[VTIME] = 0;
  opts.c_cc[VMIN] = 0;

  tcflush( fd, TCIOFLUSH ); // clean out old crap

	tcsetattr( fd, TCSANOW, &opts );

  return fd;
}

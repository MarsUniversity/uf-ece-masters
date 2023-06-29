
#include "portOpsLib.h"

/*<><><><><><><><><><><><><><><><><><><><><><><><>*\

         Hardware Dependent Section
   Various functions will need to be change to
   work on various platforms

\*<><><><><><><><><><><><><><><><><><><><><><><><>*/


/*------------------------------------------------*\
   bytesInInputBuffer( port_t portDescriptor )

   Returns the number of bytes specified by the
   portDescriptor.
\*------------------------------------------------*/
int
bytesInInputBuffer( port_t portDescriptor )
{
  int numBytesBuffer ;

  ioctl(portDescriptor, FIONREAD, &numBytesBuffer) ;

  return numBytesBuffer ;

}  /* end of bytesInInputBuffer()... */


/*------------------------------------------------*\
   bytesInOutputBuffer( port_t portDescriptor )

   Returns the number of bytes specified by the
   portDescriptor.
\*------------------------------------------------*/
int
bytesInOutputBuffer( port_t portDescriptor )
{
  int numBytesBuffer=0 ;

  /*  ioctl(portDescriptor, FIONREAD, &numBytesBuffer) ; */

  return numBytesBuffer ;

}  /* end of bytesInInputBuffer()... */


/*---------------------------------------------------*\
   clearToSend( port_t portDescriptor )

   Waits for the outgoing buffer to specifed by the
   portDescriptor to clear before returning.

   Currently does nothing.
\*---------------------------------------------------*/
void
clearToSend( port_t portDescriptor )
{

  while( bytesInOutputBuffer(portDescriptor) != 0 )
    ;

}  /* end of clearToSend()... */


/*-------------------------------------------------------*\
   closePort( port_t portDescriptor )

   Close the port specifid by "portDescriptor".

   Upon successful completion, a value of 0 is returned.
   Otherwise, a value of -1 is returned and errno is set
   to indicate the error.
\*-------------------------------------------------------*/
int
closePort( port_t portDescriptor )
/*
   Close the port described by "portDescriptor".

   Upon successful completion, a value of 0 is returned.  Otherwise,
   a value of -1 is returned and errno is set to indicate the error.
*/
{

  /* Wait for the serial output to complete */
  clearToSend( portDescriptor ) ;

  return ( close(portDescriptor) ) ;

} /* end of closePort()... */

/*--------------------------------------------------*\
   findStartMessage( port_t portDescriptor,
                     u_char startOfMsg )

   Searches the port designated by "portDescriptor"
   for the "startOfMsg" character.  Returns OK, if
   the character was found, otherwise returns ERROR.
\*-------------------------------------------------------*/
status_t
findStartMsg( port_t portDescriptor, u_char startOfMsg )
{
  u_char c ;
  int nChars ;

  nChars = bytesInInputBuffer( portDescriptor ) ;

  if ( nChars == 0 ) /* Nothing to read */
    return Func_Error ;

  do {
    readBuf( portDescriptor, &c, 1 ) ;
    --nChars ;
  } while ( (c!=startOfMsg) && nChars ) ;

  if ( c != startOfMsg )
    return( Func_Error ) ;
  else
    return( Func_Ok ) ;

}  /* end of findStartMsg()... */

/*--------------------------------------------------*\
   flushBuf( port_t portDescriptor, int buf )

   Flush the buffer specified by portDescriptor.
   "buf" can be:
      IN_BUF   - flush the input buffer
      OUT_BUF  - flush the output buffer
      BOTH_BUF - flush both buffers

   Upon successful completion, the value returned
   depends upon the device control function, but must
   be a non-negative integer. Otherwise, a value of
   -1 is returned and errno is set to indicate the
   error.
\*--------------------------------------------------*/
int
flushBuf( port_t portDescriptor, int buf )
{
    /* Flush input and output */
    return (ioctl(portDescriptor, TCFLSH, buf) ) ;

}  /* end of flushBuf()... */


/*------------------------------------------------------*\
   initPort( int whichPort, int baud, char *control)

   Initialize the port specified by "whichPort".  "baud"
   specifies a valid baud rate, while "*control" sets
   up the port parameters.  "*control" is a three
   character string looking like this: 8N1.  Where
   the first character specifies the number of bits
   ( 7 or 8 ), the second character specifies the
   parity ( E - even, O - odd, N - none ), and the last
   character specifies the stop bit (1 - one, or
   2 - two ).
\*------------------------------------------------------*/
port_t
initPort( int whichPort, int baud, char *control)
{
  port_t portDescriptor;
  int bit, stop ;
  char par ;
  tcflag_t bitFlag[]={CS5, CS6, CS7, CS8}, PAR_CTRL=IGNPAR,
           BITS, PARITY=0x00, STOP_BIT=0x00, SPEED ;
  struct termios t ;
  char serial[][20]={"/dev/ttyS0","/dev/ttyS1","/dev/ttyS2","/dev/ttyS3",
"/dev/ttyS4"} ;

  if ( (portDescriptor = open(serial[whichPort-1], O_RDWR )) == -1 ) {
          perror("initPort") ;

	  exit( 9 ) ;
  }

  /* Strip the control parameters from the control string */
  sscanf( control, "%1d%1c%1d", &bit, &par, &stop ) ;

  //#ifdef DEBUG_PRINT
    printf ("opened port: %d(%s) descriptor:%d speed:%d c:%1d%c%1d\n",
             whichPort,  serial[whichPort-1], portDescriptor, baud,
             bit, par, stop );
    //#endif

  /* Read in the current port attributes */
      if ( tcgetattr(portDescriptor, &t) == -1 ) {
     perror("tcgetattr") ;
    printf ("opened port: %d(%s) descriptor:%d speed:%d c:%1d%c%1d\n",
             whichPort,  serial[whichPort-1], portDescriptor, baud,
             bit, par, stop );

     exit( 2 ) ;
     }

  /* Set the # bits, parity and stop bits */
  if ( (bit < 5) || (bit > 8) ) {
    fprintf(stderr, "Invalid character bit size:%d [5-8]\n",bit ) ;
    exit( 2 ) ;
  }

  BITS = bitFlag[bit-5] ;

  if ( par == 'E' || par == 'e' ) {
    PARITY   = PARENB ; /* Even parity */
    PAR_CTRL = INPCK ;  /* Enable parity check */
  }
  else if ( par == 'O' || par == 'o' ) {
    PARITY   = PARENB | PARODD ;   /* Odd parity */
    PAR_CTRL = INPCK ;             /* Enable parity check */
  }

  if ( stop == 2 )
    STOP_BIT = CSTOPB ; /* Two stop bits */

  /* Set the baud rate */
  if      ( baud == 1200 )
    SPEED = B1200 ;
  else if ( baud == 2400 )
    SPEED = B2400 ;
  else if ( baud == 4800 )
    SPEED = B4800 ;
  else if ( baud == 9600 )
    SPEED = B9600 ;
  else if ( baud == 19200 )
    SPEED = B19200 ;
  else if ( baud == 38400 )
    SPEED = B38400 ;
  else if ( baud == 57600 )
    SPEED = B57600 ;


  /*-------------*\
     Input Modes
  \*-------------*/
  t.c_iflag &= ~( BRKINT |     /* Ignore break */
                  ISTRIP |     /* Don't mask */
                  INLCR |      /* Leave NL */
                  ICRNL |      /* Leave CR */
                  IGNCR |      /* "      " */
                  IXON |       /* No flow control */
                  IXOFF  );    /* No flow control */

  t.c_iflag |=(   IGNBRK |     /* Ignore break */
                  PAR_CTRL ) ; /* Parity control */

  /*--------------*\
     Output Modes
  \*--------------*/
  t.c_oflag &= ~( OPOST ) ;     /* No output flags */

  /*---------------*\
     Control Modes
  \*---------------*/
  t.c_cflag &= ~( CSIZE |       /* bits-per-byte */
                  CSTOPB |      /* Two stop bits */
                  HUPCL |       /* Hang up */
                  PARENB ) ;    /* Parity */

  t.c_cflag |= (  CLOCAL |      /* Ignore modem status lines */
                  CREAD |       /* Enable receiver */
                  STOP_BIT |    /* # stop bits */
                  BITS |        /* bits-per-byte */
                  PARITY ) ;    /* Parity check */

  /*-------------*\
     Local Modes
  \*-------------*/
  t.c_lflag &= ~( ECHO |        /* No echo of any kind */
                  ECHOE |
                  ECHOK |
                  ECHONL |
                  ICANON |      /* No canonical input */
                  ISIG |        /* No signal generation */
                  NOFLSH |      /* Enable flushing */
                  TOSTOP ) ;    /* No job control */

  /*--------------------*\
     Control Characters
  \*--------------------*/
  /*
     When VMIN and VTIME are zero the read() function will
     return as much data as possible without waiting
  */
  t.c_cc[VMIN]  = 0 ;
  t.c_cc[VTIME] = 0 ;

  /* Set the baud rate */
  if ( cfsetispeed( &t, SPEED ) == -1 ) {
    perror("cfsetispeed") ;
    exit( 2 ) ;
  }
  if ( cfsetospeed( &t, SPEED ) == -1 ) {
    perror("cfsetospeed") ;
    exit( 2 ) ;
  }

  if ( tcsetattr(portDescriptor, TCSANOW, &t) == -1 ) {
    perror("tcsetattr") ;
    exit( 2 ) ;
  }

  return portDescriptor ;

} /* end of initPort()... */


/*----------------------------------------------------------*\
   readBuf( port_t portDescriptor, void *buf, int nBytes)

   Reads in the number of bytes specified by "nBytes", from
   the port "portDescriptor" into "*buf".

   On success, returns the number of bytes actually read
   and placed in the buffer.  This number may be less
   than "nBytes" if the file is associated with a communication
   line, or if the number of bytes left in the file is less

\*----------------------------------------------------------*/
int
readBuf( port_t portDescriptor, void *buf, int readBytes )
{
  int nBytes, actualBytes ;

  if ( waitForData(portDescriptor, SEC, &nBytes) == Func_Error )
    return( -1 ) ;
  else {
    actualBytes = nBytes >= readBytes ? readBytes : nBytes ;
    return( read(portDescriptor, (char *)buf, actualBytes) ) ;
  }

}  /* end of readBuf()... */


/*----------------------------------------------------------*\
   waitForData( port_t portDescriptor, int waitTime,
                int *nChars ) ;

   Checks the port "portDescriptor" to determine if
   any characters are there to read in.  Waits
   "waitTime" for any characters to arrive.  Returns OK if
   the characters arrive on time, or ERROR otherwise.
\*----------------------------------------------------------*/
status_t
waitForData( port_t portDescriptor, int waitTime, int *nChars )
{
  void waiting( int mSec ) ;

  int mSec=100 ;
  int count=0, tiredOfWaiting ;

  tiredOfWaiting = waitTime/mSec ;

  while( (*nChars=bytesInInputBuffer(portDescriptor)) == 0 ) {
    waiting( mSec ) ;

    /* Time out if there's nothing there */
    if ( ++count > tiredOfWaiting )
      return Func_Error ;
  }

  return Func_Ok ;

}  /* end of waitForData()... */


/*----------------------------------------------------------*\
   waitForNdata( port_t portDescriptor, int waitTime,
                int nChars ) ;

   Checks the port "portDescriptor" to determine if
   "nChars" characters are there to read in.  Waits
   "waitTime" for the characters to arrive.  Returns OK if
   the characters arrive on time, or ERROR otherwise.
\*----------------------------------------------------------*/
status_t
waitForNdata( port_t portDescriptor, int waitTime, int nChars )
{
  void waiting( int uSec ) ;

  int uSec=100 ;
  int count=0, tiredOfWaiting ;
  int bytes ;

  tiredOfWaiting = waitTime/uSec ;

  while( (bytes=bytesInInputBuffer(portDescriptor)) < nChars ) {
    waiting( uSec ) ;

    /* Time out if there's nothing there */
    if ( ++count > tiredOfWaiting )
      return Func_Error ;
  }

  return Func_Ok ;

}  /* end of waitForNData()... */


/*----------------------------------------------------------*\
   waitForResponse( port_t portDescriptor, u_char waitChar,
                    int time )

   Checks the port "portDescriptor" for "waitChar".  If
   the character arrives before "time", OK is returned,
   otherwise ERROR is returned.
\*----------------------------------------------------------*/
status_t
waitForResponse( port_t portDescriptor, u_char waitChar,
                 int time )
{
  void waiting( int mSec ) ;

  int mSec=100 ;
  int count=0, tiredOfWaiting ;
  u_char incomming ;

  tiredOfWaiting = time/mSec ;

  do {
    while( bytesInInputBuffer(portDescriptor) < 1 ) {
      waiting( mSec ) ;

      if( ++count > tiredOfWaiting )
        return( Func_Error ) ;
    }

    readBuf( portDescriptor, &incomming, 1 ) ;

  } while ( incomming != waitChar ) ;

  return( Func_Ok ) ;

}  /* end of waitForResponse()... */

/*
   This is the only function that will have to change
   from platform to platform, depending on the implementation
   of the various sleep function calls.
*/
void
waiting( int mSec )
{
  struct timespec nSec ;

  nSec.tv_sec = 0 ;
  nSec.tv_nsec = mSec*1.e6 ;

  nanosleep( &nSec, NULL ) ;

}  /* end of waiting()... */


/*-----------------------------------------------------------*\
   writeBuf( port_t portDescriptor, void *buf, int nBytes )

   Write the number of bytes ("*buf) specified by "nBytes" to
   the port specified by "portDescriptor".

   On success, write returns the number of bytes actually
   written.  The number of bytes written may be less than
   "nBytes".  This can occur if the write gets interrupted for
   some reason, but some bytes have already been written.
   Otherwise, it returns -1 and sets errno to  identify the
   error.
\*-----------------------------------------------------------*/
int
writeBuf( port_t portDescriptor, void *buf, int nBytes )
{

  return ( write(portDescriptor, (char*)buf, nBytes) ) ;

}  /* end of writeBuf()... */




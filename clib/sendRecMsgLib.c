
#include "sendRecMsgLib.h"
#include <time.h>

/*<><><><><><><><><><><><><><><><><><><><><><><><><>*\

        Hardware Independent Section
   Nothing here should have to be changed when
   going from one platform to another.

\*<><><><><><><><><><><><><><><><><><><><><><><><><>*/


/*-----------------------------------------------------*\
   calculateByteChecksum( void *data, int dataSize)

   Calculates the checksum of "*data"

   Returns the byte checksum.
\*-----------------------------------------------------*/
u_char
calculateByteChecksum( void *data, int dataSize )
{

  u_char checksum=0x00,
        *tmpPtr=(u_char*)data ;
  int i ;

  for ( i=0 ; i<dataSize ; ++i )
    checksum += tmpPtr[i] ;

  checksum &= 0xFF ;

  return checksum ;

} /* end of calculateByteChecksum()... */


/*------------------------------------------------*\
   checkMAXasciiMsg( messageString_t *messageStr,
                     message_t *message )

   Check the message to be sure the data obtained
   is correct.  Extract the header and data, and
   checks the checksum againts the checksum sent.

   Returns OK if the message is OK, or ERROR,
   if there was a problem with the message.
\*-----------------------------------------------*/
status_t
checkMAXasciiMsg( messageString_t *messageStr,
                  message_t *message )
{
  u_char c, sum=0, checksum ;
  u_int chksum, dataSize ;
  static unsigned int chksumCount=0 ;
  int i, index=0, commaCount=0 ;

  if ( messageStr->numMessage < HEADER_MINIMUM )
    return( Func_Error ) ;

  /*-------------------------------------*\
     Extract the header from the message
  \*-------------------------------------*/
  i = sscanf(messageStr->message, "%c%04X,%3s,%3s,%3s,%d,%d",
        &c,
        &message->MsgID,
         message->Dest,
         message->Source,
         message->Vehicle,
        &message->dataStatus,
        &message->dataSize ) ;

  if ( i != HEADER_COMMA_COUNT+1 )
    return( Func_Error ) ;

  /*---------------------*\
     Check the checksum
  \*---------------------*/
  sum = calculateByteChecksum( messageStr->message,
        messageStr->numMessage-3 ) ;
  sum -= START_MSG ;

  /* Extract the checksum */
  sscanf( &messageStr->message[messageStr->numMessage-3], "%02X",
          &chksum ) ;
  checksum = (u_char)chksum ;

  if ( sum != checksum ) {
   fprintf(stderr, "Checksum error (#%d): %02X(calc) != %02X(trans)\n",
           ++chksumCount, sum, checksum ) ;
   return Func_Error ;
  }

  /* Find the end of the header */
  do
    if ( messageStr->message[index++] == ',' )
      ++commaCount ;
  while( (commaCount < HEADER_COMMA_COUNT) &&
         (messageStr->message[index]!='\0')) ;

  if ( commaCount != HEADER_COMMA_COUNT )
    return Func_Error ;

  /*-----------------------*\
     Clean up data portion
  \*-----------------------*/
  free( message->data ) ;
  dataSize = (message->dataStatus == START_DATA) ?
              DATA_OVERFLOW : message->dataSize ;
  if ( (message->data = (char *)malloc(dataSize+1)) ==
        NULL ) {
    fprintf(stderr, "checkMAXasciiMsg: unable to allocate memory\n") ;
    return Func_Error ;
  }
  memcpy( message->data, &messageStr->message[index], dataSize ) ;
  message->data[dataSize] = '\0' ;

  return Func_Ok ;

}  /* end of checkMAXasciiMsg()... */


/*--------------------------------------------------------*\
   clearMessage( message_t *message )

   Frees the memory pointed to by the structure of
   message_t, sets the pointers pointing to NULL,
   and set the number of characters in the message string
   to zero.
\*--------------------------------------------------------*/
void
clearMessage( message_t *message )
{
  free( message->data ) ;
  message->MsgID = message->dataSize = 0 ;

}  /* end of clearMessage()... */


/*--------------------------------------------------------*\
   clearMsgString( messageString_t *messageStr )

   Frees the memory pointed to by the structure of
   messageString_t, sets the pointers pointing to NULL,
   and set the number of characters in the message string
   to zero.
\*--------------------------------------------------------*/
void
clearMsgString ( messageString_t *messageStr )
{

  free( messageStr->message ) ;
  free( messageStr->data ) ;

  messageStr->message = messageStr->data = NULL ;

  messageStr->numMessage = 0 ;

}  /* end of clearMessage()... */


/*-----------------------------------------------------------*\
   copyData( messageString_t *messageStr, u_char *from,
             int size )

   Copies data from "*from" to the message string "*message".
   Reallocates memory for the addition of the data.

   Returns OK, if everything was OK, or ERROR otherwise.
\*-----------------------------------------------------------*/
status_t
copyData( messageString_t *messageStr, u_char *from, int size )
{
  int i, start=messageStr->numMessage  ;

  if ( (messageStr->message = (u_char *)realloc(messageStr->message,
         (sizeof(u_char))*(messageStr->numMessage+size+1))) == NULL )
    return Func_Error ;

  /*
     Can't use strncat, because the data field could contain a
     binary type (not just an ASCII string), or a 0x00 = \0
     (string terminator)
  */
  for ( i=0 ; i < size ; ++i )
    messageStr->message[start+i] = from[i] ;

  messageStr->numMessage += size ;

  return Func_Ok ;

}  /* end of copyData()... */


/*-------------------------------------------------------------*\
   parseMAXHeader( messageString_t *messageStr,
                   message_t *message )

   Parses the header string into it's associated message
   components.

   Returns OK if everything went alrigh, otherwise returns
   ERROR.
\*-------------------------------------------------------------*/
status_t
parseMAXHeader( messageString_t *messageStr, message_t *message )
{
#ifdef ASCII_MSG
  u_char c ;
#endif

#ifdef BINARY_MSG
  u_char *tmp ;
  int i, size, index=0 ;
#endif

  strcpy( message->data, messageStr->data ) ;

#ifdef ASCII_MSG
  sscanf(messageStr->head, "%c%04X,%3s,%3s,%3s,%d,%d",
        &c,
        &message->MsgID,
         message->Dest,
         message->Source,
         message->Vehicle,
        &message->dataStatus,
        &message->dataSize ) ;
#endif

#ifdef BINARY_MSG
  tmp = (u_char *)&message->MsgID ;
  size = sizeof( message->MsgID ) ;
  for (i=0 ; i<size ; ++i )
    tmp[i] = messageStr->head[index++] ;
  convertDataType( BIG_END, &message->MsgID, sizeof(message->MsgID),
                   &message->MsgID ) ;

  message->Dest    = messageStr->head[index++] ;
  message->Source  = messageStr->head[index++] ;
  message->Vehicle = messageStr->head[index++] ;

  tmp = (u_char *)&message->Reserved ;
  size = sizeof( message->Reserved ) ;
  for ( i=0 ; i<size ; ++i )
    tmp[i] = messageStr->head[index++] ;
  convertDataType( BIG_END, &message->Reserved, sizeof(message->Reserved),
                   &message->Reserved ) ;


  message->dataStatus = messageStr->head[index++] ;
/* not correct */
  message->dataSize   = messageStr->head[index++] ;
#endif

#ifdef DEBUG_PRINT
  printf("\nReceived Message!\n") ;
#ifdef ASCII_MSG
  printMAXmsg( message, ASCII ) ;
#endif
#ifdef BINARY_MSG
  printMAXmsg( message, HEX ) ;
#endif
#endif

  return Func_Ok ;

}  /* end of parseMAXHeader()... */


void
initMessage( message_t *message )
{
  message->data = NULL ;
  message->MsgID = message->dataSize = 0 ;

}  /* end of initMessage()... */


void
initMsgString( messageString_t *messageStr )
{
  messageStr->message = messageStr->data = NULL ;
  messageStr->numMessage = 0 ;

}  /* end of initMsgString()... */


/*-------------------------------------------------------*\
   printMAXmsg( message_t *message, dispType format )

   Prints the message specified by "*messagePrt", in a
   uniform format on the screen.  dispType is either
   ASCII (displaying the message as an ascii string),
   or HEX (displaying the message as hexidecimal
   values).

\*-------------------------------------------------------*/
void
printMAXmsg( message_t *message, dispType format )
{
  int i ;

#ifdef ASCII_MSG
  printf("\n") ;
  printf("   MsgID:0x%04X\n",     message->MsgID ) ;
  printf("    Dest:%s\n",         message->Dest ) ;
  printf("  Source:%s\n",         message->Source ) ;
  printf("     Veh:%s\n",         message->Vehicle ) ;
  printf("  Status:%d\n",         message->dataStatus ) ;
  printf(" Data Sz:%4d bytes\n",  message->dataSize ) ;
#endif

#ifdef BINARY_MSG
  printf("\n") ;
  printf("   MsgID:0x%04X\n",             message->MsgID ) ;
  printf("    Dest:0x%02X\n",             message->Dest ) ;
  printf("  Source:0x%02X\n",             message->Source ) ;
  printf("     Veh:0x%02X\n",             message->Vehicle ) ;
  printf("    Rsvd:0x%04X\n",             message->Reserved ) ;
  printf("  Status:0x%02X\n",             message->dataStatus ) ;
  printf(" Data Sz:0x%02X (%4d bytes)\n", message->dataSize,
                                          message->dataSize ) ;
#endif

  if ( message->dataSize ) {
    u_char *Data = (u_char *)message->data ;
    printf("    Data:") ;
    for ( i=0 ; i<message->dataSize ; ++i ) {
      if ( format == ASCII ) {
        printf("%c", Data[i] ) ;

        if ( !((i+1)%60) )
          printf("\n         ") ;
      }
      else {
        printf("0x%02X  ",  Data[i]) ;

        if ( !((i+1)%10) )
          printf("\n         ") ;
      }
    }  /* end of for(i=0... */
    printf("\n") ;
  }
  if ( !message->dataSize )
    printf("\n") ;

} /* end of printMAXmsg()... */


/*------------------------------------------------------------*\
   recASCIImsg( port portDescriptor,
                messageString_t *messageStr,
                char start_of_message, char end_of_message )

   Reads the serial port, maps the message to the message
   structure designate by "message".  The start_of_message
   and end_of_message characters are included.

   Upon successful completion, OK is returned.  Otherwise,
   ERROR is returned.
\*-------------------------------------------------------------*/
status_t
recASCIImsg( port_t portDescriptor, messageString_t *messageStr,
             char start_of_message, char end_of_message )
{
  u_char c, buffer[DATA_OVERFLOW] ;
  int i=0, nChars, count=0 ;

  clearMsgString( messageStr ) ;

  nChars = bytesInInputBuffer( portDescriptor ) ;

  if ( nChars == 0 )  /* Nothing in the buffer to read */
    return Func_Error ;

  /* Find the beginning of the message */
  if ( findStartMsg(portDescriptor, start_of_message) == Func_Error )
    return Func_Error ;

  /* Include the start_of_message */
  buffer[i++] = start_of_message ;

  /* Read in new number of characters */
  nChars = bytesInInputBuffer( portDescriptor ) ;

  /* add the rest of the message, looking for END_OF_MESSAGE */
  do {
    readBuf( portDescriptor, &c, 1 ) ;
    buffer[i++] = c ;
    --nChars ;

    /* Reset the waiting counter, if necessary */
    if ( count != 0 )
      count = 0 ;

    /* get a new START_OF_MESSAGE?  Begin again */
    if ( c == start_of_message ) {
      i = 0 ;
      buffer[i++] = c ;
      clearMsgString( messageStr ) ;
    }

    /* about to overflow?  Transfer data, and start over */
    if ( i == DATA_OVERFLOW ) {
      if ( copyData(messageStr, buffer, i) == Func_Error ) {
        fprintf( stderr, "recASCIImsg: Error allocating memory!\n") ;
        return ( Func_Error ) ;
      }
      i=0 ;
    }


    /* No more characters?  Wait for some more. */
    if ( (nChars == 0) && (c != end_of_message) )
      if ( waitForData( portDescriptor, SEC, &nChars) == Func_Error )
        return Func_Error ;

  } while ( c != end_of_message ) ;

  /* Tranfer the buffer to the message queue */
  if ( copyData(messageStr, buffer, i) == Func_Error ) {
    fprintf( stderr, "Error allocating memory!\n") ;
    return ( Func_Error ) ;
  }

  messageStr->message[messageStr->numMessage]='\0' ;

  return ( Func_Ok ) ;

} /* end of recASCIImsg()... */


/*------------------------------------------------------------*\
   recMAXmsg( port portDescriptor, message_t *message )

   Reads the serial port, maps the message to the message
   structure designate by "message".

   Upon successful completion, OK is returned.  Otherwise,
   ERROR is returned.
\*-------------------------------------------------------------*/
status_t
recMAXmsg( port_t portDescriptor, message_t *message )
{
  status_t recMAXasciiMsg(  port_t portDescriptor, message_t *message ) ;
  status_t recMAXbinaryMsg( port_t portDescriptor, message_t *message ) ;

#ifdef ASCII_MSG
  if ( recMAXasciiMsg(portDescriptor, message) == Func_Error )
    return Func_Error ;
  else
    return Func_Ok ;
#endif

#ifdef BINARY_MSG
  if ( recMAXbinaryMsg(portDescriptor, message) == Func_Error )
    return Func_Error ;
  else
    return Func_Ok ;
#endif

} /* end of recMAXmsg()... */


#ifdef ASCII_MSG
/*------------------------------------------------------------*\
   recMAXasciiMsg( port portDescriptor, message_t *message )

   Subfunction of recMAXmsg().  Reads the serial port, maps
   the ASCII message to the message  structure designate by
   "message".

   Upon successful completion, OK is returned.  Otherwise,
   ERROR is returned.
\*-------------------------------------------------------------*/
status_t
recMAXasciiMsg( port_t portDescriptor, message_t *message )
{
  status_t checkMAXasciiMsg( messageString_t *messageStr,
                           message_t *message ) ;

  messageString_t messageStr ;
  char startMsg = START_MSG,
       endMsg   = END_MSG ;

  initMsgString( &messageStr ) ;

  /*------------------------*\
     Read in entire message
  \*------------------------*/
  if( recASCIImsg(portDescriptor, &messageStr,
                  startMsg, endMsg) == Func_Error )
    return Func_Error ;

  /*----------------------------------------------*\
     Check message checksum, and separate header
     from data
  \*----------------------------------------------*/
  if ( checkMAXasciiMsg(&messageStr, message) == Func_Error )
    return Func_Error ;

  clearMsgString( &messageStr ) ;

  return( Func_Ok ) ;

}  /* end of recMAXasciiMsg()... */
#endif


#ifdef BINARY_MSG
/*------------------------------------------------------------*\
   recMAXbinaryMsg( port portDescriptor, message_t *message )

   Subfunction of recMAXmsg().  Reads the serial port, maps
   the BINARY message to the message  structure designate by
   "message".

   Upon successful completion, OK is returned.  Otherwise,
   ERROR is returned.
\*-------------------------------------------------------------*/
status_t
recMAXbinaryMsg( port_t portDescriptor, message_t *message )
{
  status_t parseMAXHeader( messageString_t *messageStr,
                         message_t *message ) ;

  u_char c, chkSum, calChkSum, *tmp, nDataBytes ;
  u_char startMsg1 = START_MSG1,
         startMsg2 = START_MSG2 ;
  int i=0, dCount, nChars, header=HEADER_SIZE ;
  static unsigned int chksumCount=0 ;
  messageString_t messageStr ;

  initMsgString( &messageStr ) ;

  /* Find the beginning of the message */
  if ( findStartMsg(portDescriptor, startMsg1) == Func_Error )
    return Func_Error ;

  readBuf( portDescriptor, &c, 1 ) ;
  if ( c != startMsg2 )
    return Func_Error ;

  /* Read in new number of characters */
  if ( waitForData(portDescriptor, SEC, &nChars) == Func_Error )
    return Func_Error ;

  /*--------------------*\
     Read in the header
  \*--------------------*/
  do {
    readBuf( portDescriptor, &c, 1 ) ;
    messageStr.head[i++] = c ;
    --nChars ;
    --header ;

    /* No more characters?  Wait for some more. */
    if ( nChars == 0 )
      if ( waitForData(portDescriptor, SEC, &nChars) == Func_Error )
        return Func_Error ;

  } while ( header ) ;
  messageStr.head[i++] = '\0' ;

  /* Obtain the number of bytes for the message */
  tmp = &(messageStr.head[HEADER_SIZE-1]) ;
  sscanf( tmp, "%c", &nDataBytes ) ;

  /*---------------------------*\
     Blindly read in the data
  \*---------------------------*/
  if ( (messageStr.data=(u_char *)malloc(sizeof(u_char)*nDataBytes)) ==
        NULL )
    return ( Func_Error ) ;

  if ( waitForData( portDescriptor, SEC, &nChars) == Func_Error )
    return Func_Error ;

  i = 0 ;
  for ( dCount=0 ; dCount<nDataBytes ; ++dCount ) {
    readBuf( portDescriptor, &c, 1 ) ;
    messageStr.data[i++] = c ;
    --nChars ;

    /* No more characters?  Wait for some more. */
    if ( nChars == 0 )
      if ( waitForData(portDescriptor, SEC, &nChars) == Func_Error )
        return Func_Error ;
  }

  /*-----------------------------------------*\
     Read in the checksum and end of message
  \*-----------------------------------------*/
  if ( waitForData(portDescriptor, SEC, &nChars) == Func_Error )
    return Func_Error ;

  readBuf( portDescriptor, &chkSum, 1 ) ;  /* The checksum */

  calChkSum  = calculateByteChecksum( messageStr.head, HEADER_SIZE ) ;
  calChkSum += calculateByteChecksum( messageStr.data, nDataBytes ) ;
  if ( chkSum == calChkSum )
    return ( Func_Ok ) ;
  else {
    fprintf( stderr,
             "Checksum error (#%4d): 0x%02X(sent) != 0x%02X(calc)\n",
             ++chksumCount, chkSum, calChkSum ) ;

    return ( Func_Error ) ;
  }

  /*------------------------------------------------*\
     Extract the header information from the string
  \*------------------------------------------------*/
  if ( parseMAXHeader(&messageStr, message) == Func_Error )
    return Func_Error ;
  else
    return Func_Ok ;


}  /* end of recMAXbinaryMsg()... */
#endif


/*----------------------------------------------------------*\
   sendMAXmsg( int port_d, message_t *message )

   Created the message string from "*message" and
   sends it out port "portDescriptor".

   Upon successful completion, OK is returned.  Otherwise,
   ERROR is returned.
\*-----------------------------------------------------------*/
status_t
sendMAXmsg( port_t portDescriptor, message_t *message )
{
  status_t sendMAXasciiMsg(  port_t portDescriptor, message_t *message ) ;
  status_t sendMAXbinaryMsg( port_t portDescriptor, message_t *message ) ;

#ifdef ASCII_MSG
  if ( sendMAXasciiMsg(portDescriptor, message) == Func_Error )
    return Func_Error ;
  else
    return Func_Ok ;
#endif

#ifdef BINARY_MSG
  if ( sendMAXbinaryMsg(portDescriptor, message) == Func_Error )
    return Func_Error ;
  else
    return Func_Ok ;
#endif

}  /* end of sendMAXmsg()... */


#ifdef ASCII_MSG
/*------------------------------------------------------------*\
   sendMAXasciiMsg( port portDescriptor, message_t *message )

   Subfunction of sendMAXmsg().  Creates an ASCII message
   from the structure designated by "message", and sends
   it out the serial port.

   Upon successful completion, OK is returned.  Otherwise,
   ERROR is returned.
\*-------------------------------------------------------------*/
status_t
sendMAXasciiMsg( port_t portDescriptor, message_t *message )
{
  u_char chkSum, dataStatus,
         messageSend[HEADER_SIZE + DATA_OVERFLOW + 2] ;
  u_int msgDataSize, dataSize, loop, loopCount ;
  int nBytes, index ;
  char startMsg = START_MSG,
       endMsg   = END_MSG,
       header[HEADER_SIZE],
       body[HEADER_SIZE + DATA_OVERFLOW] ;

  /* Check the data size */
  msgDataSize = message->dataSize ;

  /* Do we have a large data structure? */
  loop = msgDataSize/DATA_OVERFLOW ;
  if ( ((msgDataSize%DATA_OVERFLOW) != 0) || (msgDataSize == 0) )
    ++loop ;

  if ( msgDataSize <= DATA_OVERFLOW )
    dataStatus = FULL_DATA ;
  else
    dataStatus = START_DATA ;

  dataSize = msgDataSize ;

  /*--------------------*\
     Format the message
  \*--------------------*/

  /* Unchanging Header */
  sprintf(header, "%c%04x,%3s,%3s,%3s",
          startMsg,
          message->MsgID,
          message->Dest,
          message->Source,
          message->Vehicle ) ;

  index = 0 ;
  for ( loopCount=0 ; loopCount<loop ; ++loopCount ) {
    /* The rest of the Header */
    if ( message->dataSize ) {
      sprintf( body, "%s,%d,%d,",
               header,
               dataStatus,
               dataSize ) ;
      strncat( body, &message->data[index], DATA_OVERFLOW ) ;
      strcat( body, "," ) ;
    }
    else
      sprintf( body, "%s,%d,%d,,",
               header,
               dataStatus,
               dataSize ) ;

    /* Checksum and the end of message for ASCII_MSG */
    chkSum  = calculateByteChecksum( body, strlen(body) ) ;
    chkSum -= startMsg ;

    sprintf(messageSend, "%s%02X%c",
            body,
            chkSum,
            endMsg ) ;

    /*--------------------------------*\
       Write the message out the port
    \*--------------------------------*/
    nBytes = writeBuf( portDescriptor, messageSend, strlen(messageSend) ) ;

#ifdef DEBUG_PRINT
  printf("\nTransmitting Message!\n") ;
  printfMAXmsg( message, ASCII ) ;
#endif

    index       += DATA_OVERFLOW ;
    msgDataSize -= DATA_OVERFLOW ;
    if ( msgDataSize > DATA_OVERFLOW ) {
      dataStatus = CONT_DATA ;
      dataSize   = DATA_OVERFLOW ;
    }
    else if ( msgDataSize <= DATA_OVERFLOW ) {
      dataStatus = END_DATA ;
      dataSize   = (u_char)msgDataSize ;
    }

    /* Give someone else a change to send */
    if ( loopCount < (loop-1) )
      waiting( 100 ) ;

  }  /* end of for (i=0... */

  return ( Func_Ok ) ;

}  /* end of sendMAXasciiMsg()... */
#endif


#ifdef BINARY_MSG
/*------------------------------------------------------------*\
   sendMAXbinaryMsg( port portDescriptor, message_t *message )

   Subfunction of sendMAXmsg().  Creates a BINARY message
   from the structure designated by "message", and sends
   it out the serial port.

   Upon successful completion, OK is returned.  Otherwise,
   ERROR is returned.
\*-------------------------------------------------------------*/
status_t
sendMAXbinaryMsg( port_t portDescriptor, message_t *message )
{
  u_char chkSum, dataSize, dataStatus, *messageSend,
         *data ;
  u_int msgDataSize, loop, loopCount ;
  int nBytes, index=0 ;
  u_int MsgID ;
  u_short Reserved ;
  int i, dataIndex=0 ;
  int headCount ;
  char startMsg1 = START_MSG1,
       startMsg2 = START_MSG2 ;

  data = (u_char *)message->data ;

  if ( (messageSend=malloc(sizeof(message) + DATA_OVERFLOW + 2)) == NULL ) {
    fprintf(stderr,"SendMAXmsg: unable to allocate memory\n") ;
    exit( 1 ) ;
  }

  /* Check the data size */
  msgDataSize = message->dataSize ;

  /* Do we have a large data structure? */
  loop = msgDataSize/DATA_OVERFLOW ;
  if ( ((msgDataSize%DATA_OVERFLOW) != 0) || (msgDataSize == 0) )
    ++loop ;

  if ( msgDataSize <= DATA_OVERFLOW ) {
    dataSize   = (u_char)msgDataSize ;
    dataStatus = FULL_DATA ;
  }
  else {
    dataSize   = (u_char)DATA_OVERFLOW ;
    dataStatus = START_DATA ;
  }

  /*--------------------*\
     Format the message
  \*--------------------*/

  /* Unchanging Header */

  /* Force feed "head" since sprintf replaces 0x00's with spaces */
  messageSend[index++] = startMsg1 ;
  messageSend[index++] = startMsg2 ;

  convertDataType( BIG_END, &message->MsgID,
                   sizeof(message->MsgID), &MsgID ) ;
  memcpy( &messageSend[index], &MsgID,
          sizeof(message->MsgID) ) ;
  index += sizeof( message->MsgID ) ;

  messageSend[index++] = message->Dest ;
  messageSend[index++] = message->Source ;
  messageSend[index++] = message->Vehicle ;

  convertDataType( BIG_END, &message->Reserved,
                   sizeof(message->Reserved), &Reserved ) ;
  memcpy( &messageSend[index], &Reserved,
          sizeof(message->Reserved) ) ;
  index += sizeof( message->Reserved ) ;

  headCount = index ;

  for ( loopCount=0 ; loopCount<loop ; ++loopCount ) {
    /*
       Header size is fixed, so at this
       point, we know the size.

       Set/Reset index.
    */
    index            = headCount ;
    messageSend[index++] = message->dataStatus ;
    messageSend[index++] = dataSize ;
/* include count */

    /* Copy the data to the message string */
    for( i=0 ; i<dataSize ; ++i, ++dataIndex )
      messageSend[index++] = data[dataIndex] ;

    /* Checksum and the end of message for ASCII_MSG */

    chkSum  = calculateByteChecksum( messageStr, index ) ;
    chkSum -= (startMsg1 + startMsg2) ;

    messageSend[index++] = chkSum ;

    /*--------------------------------*\
       Write the message out the port
    \*--------------------------------*/
    nBytes = writeBuf( portDescriptor, messageSend, index ) ;

#ifdef DEBUG_PRINT
  printf("\nTransmitting Message!\n") ;
  printMAXmsg( message, HEX ) ;
  printf("  chkSum:0x%02X\n", chkSum ) ;
#endif

    msgDataSize -= DATA_OVERFLOW ;
    if ( msgDataSize > DATA_OVERFLOW ) {
      message->dataStatus = CONT_DATA ;
      dataSize            = DATA_OVERFLOW ;
    }
    else if ( msgDataSize <= DATA_OVERFLOW ) {
      message->dataStatus = END_DATA ;
      dataSize            = (u_char)msgDataSize ;
    }
  }  /* end of for (i=0... */

  free( messageSend ) ;

  return ( Func_Ok ) ;

}  /* end of sendMAXbinaryMsg()... */
#endif




#include "convertDataTypeLib.h"
#include <stdio.h>

void printEndian(void){
	switch(determineMemoryType()){
		case BIG_END:
			printf("This machine is big endian\n");
			break;
		case LITTLE_END:
			printf("This machine is little endian\n");
			break;
	}
}

/*
 network is big endian
 */
float htonf(float value){
	float converted=0;
	convertDataTo((void*)&converted,sizeof(float),(void*)&value,sizeof(float),BIG_END);

	return converted;
}

double htond(double value){
	double converted=0;
	convertDataTo((void*)&converted,sizeof(double),(void*)&value,sizeof(double),BIG_END);

	return converted;
}


float ntohf(float value){
	float converted=0;
	convertDataTo((void*)&converted,sizeof(float),(void*)&value,sizeof(float),determineMemoryType());

	return converted;
}

double ntohd(double value){
	double converted=0;
	convertDataTo((void*)&converted,sizeof(double),(void*)&value,sizeof(double),determineMemoryType());

	return converted;
}

/*--------------------------------------------------------*\
   convertDataFrom( void *convert, int cnvSize,
                    void *value, int size,
                    memory_t memory )

   Convert date between big ended and little ended.  The
   data is passed in as "value", along with it's "size".
   The converted data is passed out at "convert" of
   size "cnvSize".

   Writes a "1" to an int value, and checks to see
   what the first byte is.  Sets a static variable
   "BIG_ENDED" to correspond with the memory storage
   type.

   In this way, the function can be called either way,
   and if the storage is BIG_ENDED, then the data is just
   returned.  However, if the storage it LITTLE_ENDED, then
   it is converted to BIG_ENDED.

   Assumes that the value to convert to, "cnvSize", is
   equal to or larger than "size".
\*--------------------------------------------------------*/
void
convertDataFrom( void *convert, int cnvSize,
                 void *value, int size,
                 memory_t memory )
{
  void     convertDataType( void *convert,
                            void *value, int size ) ;
  void     padZeros( memory_t, void *padded, int padSize,
                     void *original, int originalSize ) ;

  static int BIG_ENDED=-1 ;
  int memAlloc=FALSE ;
  u_char *padValue ;

  if ( cnvSize < size ) {
    fprintf(stderr, "Convert size must be >= size\n" ) ;
    return ;
  }

  /* Determine memory type on system */
  if ( BIG_ENDED < 0 ) {
    memory_t memTst ;
    memTst = determineMemoryType() ;
    if ( memTst == BIG_END )
      BIG_ENDED = TRUE ;
    else
      BIG_ENDED = FALSE ;
  }

  /* Make the two variables the same size */
  if ( size != cnvSize ) {
    if ( (padValue=(u_char *)malloc(cnvSize)) == NULL ) {
      fprintf(stderr, "convertDataFrom: unable to allocate memory\n") ;
      return ;
    }
    memAlloc = TRUE ;
    padZeros( memory, padValue, cnvSize, value, size ) ;
  }
  else
    padValue = (u_char *)value ;

  switch( memory ) {
    case BIG_END:
      if ( !BIG_ENDED )
        convertDataType( convert, padValue, cnvSize ) ;
      else
        memcpy( convert, padValue, cnvSize ) ;
    break ;

    case LITTLE_END:
      if ( BIG_ENDED )
        convertDataType( convert, padValue, cnvSize ) ;
      else
        memcpy( convert, padValue, cnvSize ) ;
    break ;
  }

  if ( memAlloc )
    free( padValue ) ;

}  /* end of convertDataFrom()... */


/*--------------------------------------------------------*\
   convertDataTo( void *convert, int cnvSize,
                  void *value, int size,
                  memory_t memory )

   Convert date between big ended and little ended.  The
   data is passed in as "value", along with it's "size".
   The converted data is passed out at "convert" of size
   "cnvSize".

   Writes a "1" to an int value, and checks to see
   what the first byte is.  Sets a static variable
   "BIG_ENDED" to correspond with the memory storage
   type.

   In this way, the function can be called either way,
   and if the storage is BIG_ENDED, then the data is just
   returned.  However, if the storage it LITTLE_ENDED, then
   it is converted to BIG_ENDED.

   Assumes that the value to convert to, "cnvSize", is
   equal to or less than "size".
\*--------------------------------------------------------*/
void
convertDataTo( void *convert, int cnvSize,
               void *value, int size,
               memory_t memory )
{
  void convertDataType( void *convert,
                        void *value, int size ) ;
  void reduceZeros( memory_t memory, void *reduced, int reducedSize,
                    void *original, int originalSize ) ;

  static int BIG_ENDED=-1 ;
  int memAlloc=FALSE ;
  u_char *padConvert ;

  if ( cnvSize > size ) {
    fprintf(stderr, "Convert size must be <= to size\n") ;
    return ;
  }

  if ( BIG_ENDED < 0 ) {
    memory_t memTst ;
    memTst = determineMemoryType() ;
    if ( memTst == BIG_END )
      BIG_ENDED = TRUE ;
    else
      BIG_ENDED = FALSE ;
  }

  /* Make the two variables the same size */
  if ( size != cnvSize ) {
    if ( (padConvert=(u_char *)malloc(size)) == NULL ) {
      fprintf(stderr, "convertDataTo: unable to allocate memory\n") ;
      return ;
    }
    memAlloc = TRUE ;
    memset( padConvert, 0, cnvSize ) ;
  }
  else
    padConvert = (u_char *)convert ;

  switch( memory ) {
    case BIG_END:
      if ( !BIG_ENDED )
        convertDataType( padConvert, value, size ) ;
      else
        memcpy( padConvert, value, size ) ;
    break ;

    case LITTLE_END:
      if ( BIG_ENDED )
        convertDataType( padConvert, value, size ) ;
      else
        memcpy( padConvert, value, size ) ;
    break ;
  }

  if ( memAlloc ) {
    reduceZeros( memory, convert, cnvSize, padConvert, size ) ;
    free( padConvert ) ;
  }
  else
    memcpy( convert, padConvert, size ) ;

}  /* end of convertDataTo()... */



/*--------------------------------------------------------*\
   convertDataType( void *convert, void *value, int size )

   Convert date between big ended and little ended.  The
   data is passed in as "value", along with it's "size".
   The converted data is passed out at "convert".
\*--------------------------------------------------------*/
void
convertDataType( void *convert, void *value, int size )
{
  u_char *chValue, *chConvert ;
  int i ;

  chValue   = (u_char *)value ;
  chConvert = (u_char *)convert ;
  memset( chConvert, 0, size ) ;  /* Fill the memory with 0's */

  for ( i=0 ; i<size ; ++i )
    chConvert[size-i-1] = chValue[i] ;

}  /* end of convertDataType()... */

/*-------------------------------------------------------*\
   determineMemoryType()

   Check to see which type of memory is used (big ended
   or little ended).

   Returns the memory type, memory_t, BIG_END or
   LITTLE_END.
\*-------------------------------------------------------*/
memory_t
determineMemoryType( void )
{
  u_char *data ;
  int i=1 ;

  data = (unsigned char *)&i ;

  if ( data[0] == 0x00 )
    return( BIG_END ) ;
  else
    return( LITTLE_END ) ;

}  /* end of determineMemoryType()... */

void
packData( char *dataString, int *index, void *data, int size )
{
  u_char *cData ;

  if ( (cData=(u_char *)malloc(size)) == NULL )
    fprintf(stderr, "packData: malloc error\n") ;
  else {
    convertDataTo( cData, size, data, size, BIG_END ) ;
    memcpy( &dataString[*index], cData, size ) ;

    *index += size ;

    free( cData ) ;
  }

}  /* end of packData()... */

/*-------------------------------------------------------*\
   void padZeros( memory_t memory,
                  void *padded, int padSize,
                  void *original, int originalSize )

  For converting between variables of different sizes,
  pads the "padded" variable of size "padSize" with
  zero.  For little ended memory types, the zeros
  are added to the end, while for big ended memory
  types, the zeros are added to the beginning.
\*-------------------------------------------------------*/
void
padZeros( memory_t memory, void *padded, int padSize,
          void *original, int originalSize )
{
  int i, diff ;
  u_char *chPadded, *chOriginal ;

  memset( padded, 0, padSize ) ;

  chPadded   = (u_char *)padded ;
  chOriginal = (u_char *)original ;

  diff = padSize - originalSize ;

  switch (memory) {
    case (BIG_END):
      for ( i=0 ; i<originalSize ; ++i )
        chPadded[i+diff] = chOriginal[i] ;
    break ;

    case (LITTLE_END):
      for ( i=0 ; i<originalSize ; ++i )
        chPadded[i] = chOriginal[i] ;
    break ;
  }

}  /* end of padZeros()... */

/*-------------------------------------------------------*\
   void parseStatus( char *statusStr,
                     unsigned char status[],
                     int statusSize )

   Parses the status string "statusStr" of size
   "statusSize", and places it into the array "status"
\*-------------------------------------------------------*/
void
parseStatus( char *statusStr, unsigned char status[], int statusSize )
{
  int i, index=0, statInt ;

  for ( i=0 ; i<statusSize ; ++i ) {
    sscanf( &statusStr[index], "%02X", &statInt ) ;

    status[i] = (unsigned char)statInt ;
    index += 2  ;
  }

}  /* end of parseStatus()... */

/*-------------------------------------------------------*\
   void reduceZeros( memory_t memory,
                     void *reduced, int reducedSize,
                     void *original, int originalSize )

  For converting between variables of different sizes,
  removes exceess zeros from "original" of size
  "originalSize" and places it in "reduced" of size
  "reducedSize". For little ended memory types, the zeros
  are removed from the end, while for big ended memory
  types, the zeros are removed from the beginning.
\*-------------------------------------------------------*/
void
reduceZeros( memory_t memory, void *reduced, int reducedSize,
             void *original, int originalSize )
{
  int i, diff ;
  u_char *chReduced, *chOriginal ;

  chReduced  = (u_char *)reduced ;
  chOriginal = (u_char *)original ;

  diff = originalSize - reducedSize ;

  switch (memory) {
    case (BIG_END):
      for ( i=0 ; i<reducedSize ; ++i )
        chReduced[i] = chOriginal[i+diff] ;
    break ;

    case (LITTLE_END):
      for ( i=0 ; i<reducedSize ; ++i )
        chReduced[i] = chOriginal[i] ;
    break ;
  }


}  /* end of reduceZeros()... */

void
unpackData( char *dataString, int *index, void *data, int size )
{
  u_char *cData ;

  if ( (cData=(u_char *)malloc(size)) == NULL )
    fprintf(stderr, "unpackData: malloc error\n") ;
  else {
    memcpy( cData, &dataString[*index], size ) ;
    convertDataFrom( data, size, cData, size, BIG_END ) ;

    *index += size ;

    free( cData ) ;
  }

}  /* end of unpackData()... */

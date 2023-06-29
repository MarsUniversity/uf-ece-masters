#include "textFileLib.h"

/*-----------------------------------------------------------*\
   delimiterCount( char *s, char delimiter, int count,
                   int *index )

   Used to count to the "count" number of delimiters
   ("delimiter") in character string "s".  "Index" is
   returned which points to the "count"th delimiter.

   Returns the number of delimiters counted.
\*-----------------------------------------------------------*/
int
delimiterCount( char *s, char delimiter, int count,
                   int *index )
{
  int currentCount=0 ;

  do {
    if( s[(*index)++] == delimiter )
      ++currentCount ;
  }
  while( (currentCount<count) && (s[*index]!='\0') ) ;

  return( currentCount ) ;

}  /* end of delimiterCount()... */

/*-------------------------------------------------------*\
   char *pathFileRead( char *filename )

   Reads in the path file specified by "filename",
   and returns the path as a string.  User is expected
   to free the string when done.
\*-------------------------------------------------------*/
char *
pathFileRead( char *filename )
{
  FILE *fp ;
  int i, count ;
  char *path, line[80] ;

  if ( (fp=fopen(filename, "r")) == NULL ) {
    fprintf( stderr, "Unable to open \"%s\".\n", filename ) ;
    return( NULL ) ;
  }

  /* Read in th number of data points */
  do
    readLine( fp, line ) ;
  while( line[0] == '#' ) ;

  sscanf( line, "%d,", &count ) ;
  path = (char *)malloc( (count+1)*70 ) ;

  /* strip off the newline */
  line[strlen(line)-1] = '\0' ;
  strcpy( path, line ) ;

  for( i=0 ; i<count ; ) {
    readLine( fp, line ) ;
    if ( line[0] != '#' ) {
      ++i ;

      /* strip off the newline */
      line[strlen(line)-1] = '\0' ;
      strcat( path, line ) ;
    }
  }

  fclose( fp ) ;

  return( path ) ;

}  /* end of pathFileRead()... */

/*-------------------------------------------------------*\
   status pathFileWrite( char *filename, char *path )

   Writes the path stored in "path" to the file
   "filename".  If you don't wan't the first bit
   of the path written,
\*-------------------------------------------------------*/
status_t
pathFileWrite( char *filename, char *path )
{
  FILE *fp ;
  int i, c, count, index, indexOld ;
  char string[80] ;

  if ( (fp=fopen(filename, "w")) == NULL ) {
    fprintf( stderr, "Unable to open \"%s\".\n", filename ) ;
    return( Func_Error ) ;
  }

  fprintf( fp, "# Number of data points\n" ) ;

  indexOld = index = 0 ;

  /* strip off the length and number of points */
  c = delimiterCount( path, ',', 1, &index ) ;
  strncpy( string, &path[indexOld], index-indexOld ) ;
  string[index-indexOld] = '\0' ;
  indexOld = index ;
  fprintf( fp, "%s\n", string ) ;

  sscanf(string, "%d", &count ) ;

  /* Write the path points to the file */
  fprintf( fp, "# Data points\n") ;
  for ( i=0 ; i<count ; ++i ) {
    c = delimiterCount( path, ',', 6, &index ) ;
    strncpy( string, &path[indexOld], index-indexOld ) ;
    string[index-indexOld] = '\0' ;
    indexOld = index ;
    fprintf( fp, "%s\n", string ) ;
  }

  fclose( fp ) ;

  return( Func_Ok ) ;


}  /* end of pathFileWrite()... */

void
readComments( FILE *fp, char *line, char comment, int *lineCount )
{
  do {
    readLine( fp, line ) ;
    ++(*lineCount) ;
  }
  while( ((line[0] == comment) || (line[0] == '\n')) ) ;

}  /* end of readComments()... */

void
readLine( FILE *fp, char *line )
{
  char c ;
  int read, index=0 ;

  do {
    read = fscanf( fp, "%c", &c ) ;
    line[index++] = c ;
  }
  while( (c != '\n') && (read != EOF) ) ;

  line[index++] = '\0' ;

}  /* end of readLine()... */


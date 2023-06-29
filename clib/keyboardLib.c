#include "keyboardLib.h"

escKey_t
decodeESC( int choice )
{
  escKey_t decodeKeypad(int choice) ;

  escKey_t key=KEY_UNKN ;
#if 0
  escKey_t = f[12]={F1, F2, F3, F4, F5, F6, F7, F8, F9, F10, F11, F12} ;
#endif

  /* Handle 'keypad' calls */
  if ( choice > 255 ) {
    return( decodeKeypad(choice) ) ;
  }

  /* Handle 'ESC' sequences */
  else if ( choice == ESC ) {
    choice = readKeyboard() ;

    if ( choice == ERR ) 
      key = KEY_ESC ;
  }

  while( readKeyboard() != ERR )
    ;

  return( key ) ;

}  /* end of decodeESC()... */

escKey_t
decodeKeypad( int choice )
{
  escKey_t f[12]={F1, F2, F3, F4, F5, F6, F7, F8, F9, F10, F11, F12} ;
  /*extern sm_t *sm ;*/

  switch( choice ) {
    /* Arrow Keys */
    case  KEY_UP:
      return( Up_Arrow ) ;
    break ;

    case  KEY_DOWN:
      return( Down_Arrow ) ;
    break ;

    case  KEY_RIGHT:
      return( Right_Arrow ) ;
    break ;

    case  KEY_LEFT:
      return( Left_Arrow ) ;
    break ;

    /* Other keypad keys */
    case KEY_A1:
    case KEY_HOME:
      return( KEY_7 ) ;
    break ;

    case KEY_A3:
    case KEY_PPAGE:
      return( KEY_9 ) ;
    break ;

     case KEY_B2:
      return( KEY_5 ) ;
    break ;

    case KEY_C1:
    case KEY_END:
      return( KEY_1 ) ;
    break ;

    case KEY_C3:
    case KEY_NPAGE:
      return( KEY_3 ) ;
    break ;

  }  /* end of switch( choice )... */

  /* Function Keys */
  if ( (choice >= 0410)  && (choice<0510) )
    return( f[choice-0410] ) ;
  else
    return( KEY_UNKN ) ;

}  /* end of decodeKeypad()... */

void
getInput( char *string, int size ) 
{
  echo() ;
  nocbreak() ;
  getnstr( string, size ) ;
  noecho() ;
  cbreak() ;
}  /* end of getInput()... */

int
readKeyboard( void )
{
  int done, nbytes ;

  struct timespec sleep ;

  sleep.tv_sec  = 0 ;
  sleep.tv_nsec = 200*1e3 ;

  /* Wait a while for characters */
  do
    done = nanosleep( &sleep, &sleep ) ;
  while( done == -1 ) ;

  ioctl( 0, FIONREAD, &nbytes ) ;

  if ( nbytes == 0 )
    return( ERR ) ;
  else
    return( getch() ) ;

}  /* end of readKeyboard()... */

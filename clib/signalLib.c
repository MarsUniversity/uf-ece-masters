#include "signalLib.h"

int
trapTerminate( void (*signalHandler)() )
{
  struct sigaction act ;

  /* Handle the INT (Ctrl-c) and TERM (kill) signals */
  act.sa_handler = signalHandler ;
  if (sigemptyset( &act.sa_mask )) {
    perror("sigemptyset") ;
    return(-1) ;
  }
  act.sa_flags = 0 ;

  if ( sigaction(SIGINT, &act, NULL) ) {
    perror("sigaction INT") ;
    return(-1) ;
  }

  if ( sigaction(SIGTERM, &act, NULL) ) {
    perror("sigaction TERM") ;
    return(-1) ;
  }

  return(0) ;
}  /* end of trapTerminate()... */

int
sendSignal( pid_t pid, int signo )
{
  if ( kill(pid, signo) < 0 ) {
    perror("signal") ;
    return( -1 ) ;
  }
  return( 0 ) ;

}  /* end of sendSignal()... */

int
setupAlarmSignal( float hertz )
{
  struct itimerval interval ;
  long sec, usec ;

  sec = (long)(1.0/hertz) ;
  usec = (long)(((1.0/hertz) - (float)sec)*1000000) ;

  interval.it_value.tv_sec     = sec ;
  interval.it_value.tv_usec    = usec ;
  interval.it_interval.tv_sec  = sec ;
  interval.it_interval.tv_usec = usec ;

  if (setitimer( ITIMER_REAL, &interval, NULL) < 0) {
    perror("setitimer") ;
    return(-1) ;
  }
  return(0) ;
}


int
setupSignalHandler( int signo, sigset_t mask, void (*signalHandler)() )
{
  struct sigaction act ;

  act.sa_handler = signalHandler ;
  act.sa_mask    = mask ;
  act.sa_flags   = 0 ;

  if (sigaction(signo, &act, NULL)) {
    perror("sigaction") ;
    return(-1) ;
  }
  return(0) ;
}  /* end of setupSignalHandler()... */


int
clearMask( sigset_t *mask )
{
  if (sigemptyset(mask)) {
    perror("sigemptyset") ;
    return(-1) ;
  }
  return(0) ;
}  /* end of clearMask()... */


int
setMask( sigset_t *mask )
{
  if (sigfillset(mask)) {
    perror("sigfillset") ;
    return(-1) ;
  }
  return(0) ;
}  /* end of setMask()... */


int
unmaskSignal( sigset_t *mask, int signo )
{
  int member ;

  if ((member = sigismember(mask, signo)) == -1) {
    perror("sigismember") ;
    return(-1) ;
  }

  if (member) {
    if (sigdelset(mask, signo)) {
      perror("sigdelset") ;
      return(-1) ;
    }
  }
  return(0) ;
}  /* end of unmaskSignal()... */


int
maskSignal( sigset_t *mask, int signo )
{
  int member ;

  if ((member = sigismember(mask, signo)) == -1) {
    perror("sigismember") ;
    return(-1) ;
  }

  if (!member) {
    if (sigaddset(mask, signo)) {
      perror("sigaddset");
      return(-1) ;
    }
  }

  return(0) ;
}  /* end of maskSignal()... */


int
waitForSignal( int signo )
{
  sigset_t mask ;
  void *value ;
  int sig_number ;

  if (sigemptyset(&mask)) {
    perror("sigemptyset") ;
    return(-1) ;
  }

  if (sigaddset(&mask, signo)) {
    perror("sigaddset") ;
    return(-1) ;
  }

  if ((sig_number = sigwait(&mask, value)) == -1) {
    perror("sigwait") ;
    return(-1) ;
  }

  return(sig_number) ;
}  /* end of waitForSignal()... */


int
ignoreSignal( int signo )
{
  struct sigaction act ;

  act.sa_handler = SIG_IGN ;
  if (sigemptyset(&act.sa_mask)) {
    perror("sigemptyset") ;
    return(-1) ;
  }
  act.sa_flags = 0 ;

  if (sigaction(signo,&act,NULL)) {
    perror("sigaction") ;
    return(-1) ;
  }

  return(0) ;
} /* end of ignoreSignal()... */

int
killProcess( pid_t pid )
{

  if ( sendSignal(pid, SIGTERM) < 0 ) {
    perror("kill") ;
    return( -1 ) ;
  }
  return( 0 ) ;
}  /* end of killProcess()... */


int
delay( time_t usecs )
{
  struct timespec sleepyTime ;
  int done ;

  sleepyTime.tv_sec = (int)(usecs / 1e6) ;
  sleepyTime.tv_nsec = (int)(usecs % (int)1e6)*1e3 ;

  if (sleepyTime.tv_nsec > 1e9)
    return(-1) ;

  do {
    done = nanosleep(&sleepyTime,&sleepyTime) ;
  }
  while(done == -1) ;

  return(0) ;
}




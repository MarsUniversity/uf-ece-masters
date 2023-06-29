#include "smLib.h"
#if 0
/*
   Attaches the shared memory segment associated with the shared
   memory identifer specified by "shmid"

   Upon successful completion, the data segment start address of
   the attached shared memory segment.  Otherwise, a value of -1
   is returned and "errno" is set to indicate the error.

*/
void *shMemAttach( int shmid )
{
  int shmflg=SHM_RND ;

#ifdef DEBUG_PRINT
  printf("Attaching to shmem shmid:%d\n", shmid ) ;
#endif

  return( shmat(shmid, NULL, shmflg) ) ;

}  /* end of shMemAttach()... */

#if 0
/*
   Attaches the shared memory segment associated with the shared
   memory identifer specified by "size" bytes to the data segment
   of the calling process.

   Upon successful completion, the data segment start address of
   the attached shared memory segment.  Otherwise, a value of -1
   is returned and "errno" is set to indicate the error.

   The "numAttach" parameter is used to specify the number of
   attachments to the shared memory.  The first program attaching
   to shared memory should use 1, the second should use 2, etc.
*/
void *shMemAttach( int size, int numAttach, int *shmid )
{
  int shmflg=SHM_RND ;

  *shmid = shMemCheck( size, numAttach ) ;

#ifdef DEBUG_PRINT
  printf("Attaching to shmem of size %d bytes, shmid:%d\n",
          size, *shmid ) ;
#endif

  return( shmat(*shmid, (void *)0, shmflg) ) ;

}  /* end of shMemAttach()... */
#endif

/*
   Check to see if an existing segment of shared memory already
   exists of the specifed "size" (in bytes), and that there are less
   then "numAttach" attached to it.  If it does, it returns 
   the shared memory id of the segment.  If it does not, it first 
   creates one, and then returns the newly created id.
*/
int shMemCheck( int size, int numAttach )
{
  struct shmid_ds buf ;
  int i=0, found=FALSE, shmid, memCount ;
  char shMemCount[40]="ipcs -m | grep -c 0x\0",
       shMemList[40] ="ipcs -m | grep 0x | awk '{print $2}'\0" ;
  FILE *memCheck ;

  /* Count the number of shared memory segments created */
  memCheck = popen( shMemCount, "r" ) ;
    fscanf( memCheck, "%d", &memCount ) ;
  pclose( memCheck ) ;

  if ( !memCount ) /* No segmets exists, so let's create one */
    return ( shMemCreate( size ) ) ;
  else { /* Check each segment to see if it's the correct size */
    memCheck = popen( shMemList, "r" ) ;
      do {
        ++i ;
	    fscanf( memCheck, "%d", &shmid ) ;
        shmctl( shmid, IPC_STAT, &buf ) ;
        if ( (buf.shm_segsz == size) && 
             (shMemNumAttach(shmid) < numAttach) )
          found = TRUE ;
      } while ( !found && i<=memCount ) ;
    pclose( memCheck ) ;

    if ( found ) /* If a segment was found, return the id */
      return ( shmid ) ;
    else /* else create a new one and return the id */
      return ( shMemCreate( size ) ) ;
  }
}  /* end of shMemCheck()... */


/*
   A shared memory identifier and associated data structure 
   and shared memory segment of a least "size" bytes is created.

   Upon successful completion, a non-negative integer, namely 
   a shared memory identifier is returned.  Otherwise, a value of 
   -1 is returned and errno is set to indicate the error.
*/
int shMemCreate( int size )
{
  int shmflg=0666 ;  /* Everyone has read/write access, Octal base */

  shmflg |= IPC_EXCL ;

  return ( shmget(0, size, shmflg) ) ;

}  /* end of shMemCreate()... */


/* 
   Removes the shared memory identifier specified by "shmid"
   from the system and destroys the shared memory segment
   and data structure associated with it.

   if "remorse" is REMORSE, then the shared memory is not
   destroyed if there are still attachments.  However, if
   "remorse" is NO_REMORSE, then the shared memory is
   destroyed.

   Upon successful completion, a value of 0 is returned.
   Otherwise, a value of -1 is returned and "errno" is set
   to indicate the error.
*/
int shMemDestroy( int shmid, level remorse )
{
  struct shmid_ds buf ;

  shmctl( shmid, IPC_STAT, &buf ) ;
  if ( (buf.shm_nattch) && (remorse == REMORSE) ) {
#ifdef DEBUG_PRINT
    printf("There's still someone (%d) attached to shmid: %d\n", 
            buf.shm_nattch, shmid) ;
#endif

    return ( -1 ) ;
  }
  else {
#ifdef DEBUG_PRINT
    printf("Destroying shmid: %d\n",shmid ) ;
#endif

    return ( shmctl(shmid, IPC_RMID, &buf) ) ;
  }

}  /* end of shMemDestroy()... */


/*
   Detaches from the calling process's data segment the
   shared memory segment located at the addres specified by
   "shmaddr".

   Upon successful completion, a value of 0 is returned.
   Otherwise, a value of -1 is returned and "errno" is set
   to indicate the error.
 */
int shMemDetach( void *shmaddr )
{

  return ( shmdt(shmaddr) ) ;

}  /* end of shMemDetach()... */


/*
   Returns the number of attachments to the shared memory 
   segment specified by "shmid".

   Upon successful completion, the number of attachements 
   is returned.  Otherwise, a value of -1 is returned and 
   "errno" is set to indicate the error.
*/
int shMemNumAttach( int shmid )
{

  struct shmid_ds buf ;

  if ( shmctl(shmid, IPC_STAT, &buf) )
    return ( -1 ) ;
  else {
#ifdef DEBUG_PRINT
    printf( "%d attached to ID:%d\n", buf.shm_nattch, shmid ) ;
#endif
    return ( buf.shm_nattch ) ;
  }

}  /* end of shMemNumAttach()... */

#endif


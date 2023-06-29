#include "socket.h"


/*!
	This function returns a socket descriptor which has
	already connected to the internet address given.
	\param addr string containing the address to connect to.
	\param port port on remote host.
	\return returns the socket descriptor or -1 on error.
*/
int initSocket(char *addr, int port){
	int sd = 0;
  struct sockaddr_in tcpaddr;
	
  if((sd=socket(PF_INET,SOCK_STREAM,IPPROTO_TCP))<0){
  	perror("socket() failed");
		return -1;
  }
	
  memset(&tcpaddr,0,sizeof(tcpaddr));
  tcpaddr.sin_family = AF_INET;
  tcpaddr.sin_addr.s_addr = inet_addr(addr);
  tcpaddr.sin_port = htons(port);

  if(connect(sd,(struct sockaddr*)&tcpaddr,sizeof(tcpaddr))<0){
  	perror("connect() failed");
		return -1;
  }

	return sd;
}

/*!
	This function is a wrapper around recv(). For small transmitions
	of data, just use recv, but for larger ones this function will
	ensure that all data is properly recieved.
	\param sd socket descriptor.
	\param buf buffer to store data to.
	\param len lenght of data to be read in.
	\param flags flags to  be sent to recv().
	\return returns number of bytes read, or -1 on error.
*/
int recvSocket(int sd, void *buf, int len, unsigned int flags){
	int rBytes = 0;
	int rv = 0;
	
	for(rBytes = 0; rBytes < len; rBytes += rv){
		if ((rv = recv(sd, buf+rBytes, len-rBytes, flags)) <= 0){
			perror("recv() failed or connection closed prematurely");
			printf("error size: %d\n",rBytes);
			return -1;
		}
	}

	return rBytes;
}


/*!
	This function sets up the server socket which will listen to
	any incomming interface.
	\param port for server to listen on.
	\return socket for server to use or -1 on error.
*/
int initServerSocket(unsigned short port) {
  int sock;
  struct sockaddr_in servaddr;

  //Construct socket for incoming connections
  if ((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0){
    perror("socket() failed");
		return -1;
	}

  // Construct local address structure
  memset(&servaddr, 0, sizeof(servaddr));   // Zero out structure
  servaddr.sin_family = AF_INET;                // Internet address family
  servaddr.sin_addr.s_addr = htonl(INADDR_ANY); // Any incoming interface
  servaddr.sin_port = htons(port);              // Local port

  // Bind to the local address
  if (bind(sock, (struct sockaddr *) &servaddr, sizeof(servaddr)) < 0){
    perror("bind() failed");
    return -1;
  }

  // Mark the socket so it will listen for incoming connections
  if (listen(sock, 10) < 0){
    perror("listen() failed");
		return -1;
	}

  printf("Ready for connections on %s : %u\n",
				 inet_ntoa(servaddr.sin_addr),
				 port);
	
  fflush(stdout);

  return sock;
}



/*!
	Functions for low level TCP Connections between a server and a
	client.
	\param servSock is the server socket that is already set up.
	\return clientSocket or -1 on error.
*/
int waitForClient(int servSock) {
  int clntSock;                    // Socket descriptor for client
  struct sockaddr_in clntAddr;     // Client address
  unsigned int clntLen;            // Length of client address data structure

  // Set the size of the in-out parameter
  clntLen = sizeof(clntAddr);

  // Wait for a client to connect
  if ((clntSock = accept(servSock, (struct sockaddr *) &clntAddr, &clntLen)) < 0){
    perror("accept() failed");
		return -1;
	}

  // clntSock is connected to a client!
  else {
    printf("Handling client %s ", inet_ntoa(clntAddr.sin_addr));
    fflush(stdout);
  }
  return clntSock;
}

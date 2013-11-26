/*
 * tcp_socket.c
 *
 *  Created on: Nov 26, 2013
 *      Author: mavlab
 */

#include <sys/socket.h>       /*  socket definitions        */
#include <sys/types.h>        /*  socket types              */
#include <arpa/inet.h>        /*  inet (3) funtions         */
#include <unistd.h>           /*  misc. UNIX functions      */
#include <errno.h>

#include <netinet/in.h>
#include <arpa/inet.h>

#include <stdio.h>            /* stderr                     */
#include <string.h>           /* memset                     */
#include <stdlib.h>


#include "tcp_socket.h"

/*  Global constants  */
#define MAX_LINE           (1000)



/*  Global variables  */
int       list_s;                /*  listening socket          */
struct    sockaddr_in servaddr;  /*  socket address structure  */
char      buffer[MAX_LINE];      /*  character buffer          */
char     *endptr;                /*  for strtol()              */


int closeSocket(void) {
  return close(list_s);
}

int initSocket() {

  /*  Create the listening socket  */
  if ( (list_s = socket(AF_INET, SOCK_STREAM, 0)) < 0 ) {
    fprintf(stderr, "tcp server: Error creating listening socket.\n");
    return -1;
  }


  /*  Set all bytes in socket address structure to
        zero, and fill in the relevant data members   */

  char ipa[10];
  sprintf(ipa, "127.0.0.1");

  memset(&servaddr, 0, sizeof(servaddr));
  servaddr.sin_family      = AF_INET;
  servaddr.sin_port        = htons(PORT);
  if(inet_pton(AF_INET, ipa, &servaddr.sin_addr)<=0)
  {
    printf("\n inet_pton error occured\n");
    return 1;
  }



  if( connect(list_s, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
  {
    printf("\n Error : Video Connect Failed. Is gst-launch running? \n");
    return 1;
  }

  printf("\n Video framework connected! \n");

  return 1;
}

/*  Read a line from a socket  */

int Read_msg_socket(char * data, unsigned int size) {

  int n;
  n = read(list_s, data, size);
  return n;

}


/*  Write a line to a socket  */

ssize_t Write_msg_socket(char * data, unsigned int size) {
  size_t      nleft;
  ssize_t     nwritten;
  nleft  = size;
  nwritten =0;

  while ( nleft > 0 ) {
    if ( (nwritten = write(list_s, data, nleft)) <= 0 ) {
      if ( errno == EINTR )
        nwritten = 0;
      else
        return -1;
    }
    nleft  -= nwritten;
    data += nwritten;
  }

  return nwritten;

}

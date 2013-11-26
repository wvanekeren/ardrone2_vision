/*
 * tcp_socket.c
 *
 *  Created on: Nov 26, 2013
 *      Author: mavlab
 */

#include <arpa/inet.h>        // inet (3) funtions
#include <stdio.h>            // stderr
#include <string.h>           // memset

#include "tcp_socket.h"


/*  Global variables  */
int       list_s;                /*  listening socket          */
struct    sockaddr_in servaddr;  /*  socket address structure  */
char     *endptr;                /*  for strtol()              */


int closeSocket(void) {
  return close(list_s);
}

int initSocket() {
  //  Create the listening socket
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
    return -1;
  }

  if( connect(list_s, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
  {
    printf("\n Error : Video Connect Failed. Is gst-launch running? \n");
    return -1;
  }

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
  ssize_t     nwritten;
  nwritten = write(list_s, data, size);
  return nwritten;
}

/*
 * Copyright (C) 2012-2013
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
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
    printf("Inet_pton error occured.\n");
    return -1;
  }

  if( connect(list_s, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
  {
    printf("Error : PPRZ<->GST Connect Failed?\n");
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

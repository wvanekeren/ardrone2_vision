/*
 * tcp_socket.h
 *
 *  Created on: Nov 26, 2013
 *      Author: mavlab
 */

#ifndef TCP_SOCKET_H_
#define TCP_SOCKET_H_


#include <unistd.h>             /*  for ssize_t data type  */


#define PORT         (2002)


// Open - Close
int initSocket(void) ;
int closeSocket(void);

// Read - Write
int Read_msg_socket(char * data, unsigned int size);
ssize_t Write_msg_socket(char * data, unsigned int size);


#endif /* TCP_SOCKET_H_ */

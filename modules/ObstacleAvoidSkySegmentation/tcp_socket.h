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

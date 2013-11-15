
////////////////////////////////////////////////
// Paparazzi communication interface

#include <pthread.h>    // pthread_create
#include <gst/gst.h>    // gprint

#include "socket.h"
#include "video_message_structs.h"

struct gst2ppz_message_struct gst2ppz;
struct ppz2gst_message_struct ppz2gst;

////////////////////////////////////////////////
// OWN INIT

void *TCP_thread( void *ptr);
void paparazzi_message_server_start(void);
void paparazzi_message_send(void);

void paparazzi_message_server_start(void)
{
  //initialise socket:
	if (tcp_port>0)
  {
		//start seperate threat to connect
		//seperate threat is needed because otherwise big delays can exist in the init or chain function
		//causing the gst to crash

		pthread_t th1;
		int th1_r;
		pthread_create(&th1,NULL,TCP_thread,&th1_r);
	}
}

unsigned int socketIsReady;

void *TCP_thread( void *ptr)
{
	g_print("Waiting for connection on port %d\n",tcp_port);

	socketIsReady = initSocket(tcp_port);
  if (!socketIsReady)
  {
		g_print("Error initialising connection\n");
	}
  else
  {
		g_print("Connected!\n");
	}


	while(1)
  {
		int res = Read_msg_socket((char *) &ppz2gst,sizeof(ppz2gst));
		if	(res>0)
    {
      // Message is automatically available to plugin
		}
    else
    {
			g_print("Nothing received: %d\n",res);
			usleep(100000);
		}
	}

}

void paparazzi_message_send()
{
	if (tcp_port>0)
	{ 	//if network was enabled by user
		if (socketIsReady)
 		{
			Write_msg_socket((char *) &gst2ppz, sizeof(gst2ppz));
		}
	}
}



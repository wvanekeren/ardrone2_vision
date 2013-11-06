
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
unsigned int counter;

void paparazzi_message_server_start(void)
{
	counter =0;

  //initialise socket:
	if (tcpport>0)
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
	g_print("Waiting for connection on port %d\n",tcpport);

	socketIsReady = initSocket(tcpport);
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
			int tmp;
			tmp = (int)counter; // TODO: fix - (int)ppz2gst.heading;
			// TODO FIXME g_print("Current counter: %d, Received counter: %d, diff: %d\n",counter, ppz2gst.heading, tmp);
			//ppz2gst.heading = 6;
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
	if (tcpport>0)
	{ 	//if network was enabled by user
		if (socketIsReady)
 		{
			Write_msg_socket((char *) &gst2ppz, sizeof(gst2ppz));
		}
	}
  counter++;
}



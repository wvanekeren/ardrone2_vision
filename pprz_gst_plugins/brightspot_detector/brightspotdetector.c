
#include <pthread.h>    // pthread_create
#include <unistd.h>     // usleep
#include <string.h>     // memset
#include <gst/gst.h>    // gprint

#include "brightspotdetector.h"
#include "cv_brightspot.h"

////////////////////////////////////////////////
// Plugin inputs
unsigned int imgWidth, imgHeight;
unsigned int tcpport;
unsigned char threshtune;

////////////////////////////////////////////////
// Paparazzi

#include "socket.h"
#include "video_message_structs.h"

struct gst2ppz_message_struct gst2ppz;
struct ppz2gst_message_struct ppz2gst;

////////////////////////////////////////////////
// OWN INIT

void *TCP_threat( void *ptr);
unsigned int counter;

void my_plugin_init()
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
		pthread_create(&th1,NULL,TCP_threat,&th1_r);
	}
}

unsigned int socketIsReady;

void *TCP_threat( void *ptr)
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
			tmp = (int)counter - (int)ppz2gst.heading;
			g_print("Current counter: %d, Received counter: %d, diff: %d\n",counter, ppz2gst.heading, tmp);
			ppz2gst.heading = 6;
		}
    else 
    {
			g_print("Nothing received: %d\n",res);
			usleep(100000);
		}
	}

}

////////////////////////////////////////////////
// PLUGIN CHAIN FUNCTION

void my_plugin_run(unsigned char* frame)
{
  signed int blobP[8];
  memset(blobP, 0, sizeof(blobP[0]) * 8);
	unsigned int max_idx, max_idy;

  // Actual Payload
	brightspotDetector(frame,blobP,&max_idx,&max_idy);
  //g_print("Max_idx: %d Max_idy: %d Counter: %d\n",max_idx,max_idy,counter);


	if (tcpport>0)
  { 	//if network was enabled by user
		if (socketIsReady)
    {
			gst2ppz.blob_x1 = blobP[0];
			gst2ppz.blob_y1 = blobP[1];
			gst2ppz.blob_x2 = blobP[2];
			gst2ppz.blob_y2 = blobP[3];
			gst2ppz.blob_x3 = blobP[4];
			gst2ppz.blob_y3 = blobP[5];
			gst2ppz.blob_x4 = blobP[6];
			gst2ppz.blob_y4 = blobP[7];
			gst2ppz.counter = counter;
			Write_msg_socket((char *) &gst2ppz, sizeof(gst2ppz));
		}
	}
	counter++;
}






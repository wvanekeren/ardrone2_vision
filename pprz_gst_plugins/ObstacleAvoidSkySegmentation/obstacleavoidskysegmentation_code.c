
#include <stdlib.h>

#include "obstacleavoidskysegmentation_code.h"
#include "skysegmentation.h"

// Settable by pluging
unsigned int imgWidth, imgHeight;
unsigned int tcpport;
unsigned char yourownvariable;

// Called by plugin

unsigned char * img_uncertainty;
int adjust_factor;
int counter;


void my_plugin_init(void)
{
  counter = 0;
	img_uncertainty= (unsigned char *) calloc(imgWidth*imgHeight*2,sizeof(unsigned char)); //TODO: find place to put: free(img_uncertainty);
}

void my_plugin_run(unsigned char *frame)
{
  int mode = 1;
  int pitch = 0; // ppz2gst.pitch/36
  int roll = 0;  // ppz2gst.roll/36

  if (mode==0)
	{
		segment_no_yco_AdjustTree(frame,img_uncertainty,adjust_factor);
	}
	else if (mode==1)
	{
		skyseg_interface_n(frame, img_uncertainty, adjust_factor, counter, pitch, roll);

		if (tcpport>0) { 	//if network was enabled by user
/*
			if (socketIsReady) {
				// gst2ppz.blob_x1 = blobP[0];
				gst2ppz.counter = counter;
				Write_msg_socket((char *) &gst2ppz, sizeof(gst2ppz));
			}
*/
		}
	}
}


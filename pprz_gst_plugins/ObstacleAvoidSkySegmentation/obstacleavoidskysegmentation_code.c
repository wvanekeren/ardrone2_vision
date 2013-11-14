
#include <stdlib.h>

#include "obstacleavoidskysegmentation_code.h"
#include "skysegmentation.h"

#include "paparazzi.h"

// Settable by pluging
unsigned int imgWidth, imgHeight;
unsigned int tcpport;
unsigned char yourownvariable;

// Called by plugin

unsigned char * img_uncertainty;
int adjust_factor;


void my_plugin_init(void)
{
	img_uncertainty= (unsigned char *) calloc(imgWidth*imgHeight*2,sizeof(unsigned char)); //TODO: find place to put: free(img_uncertainty);

  paparazzi_message_server_start();
}

void my_plugin_run(unsigned char *frame)
{
  int mode = 1;
  int pitch = ppz2gst.pitch/36;
  int roll = ppz2gst.roll/36;

  if (mode==0)
  {
    segment_no_yco_AdjustTree(frame,img_uncertainty,adjust_factor);
  }
	else if (mode==1)
	{
    // Run actual Image Analysis
    skyseg_interface_n(frame, img_uncertainty, adjust_factor, counter, pitch, roll);

    // Store the results
    // gst2ppz.blob_x1 = blobP[0];

    // Send to paparazzi
    paparazzi_message_send();
	}
}


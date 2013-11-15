
#include <stdio.h>
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
int adjustfactor = 0;


void my_plugin_init(void)
{
	img_uncertainty= (unsigned char *) calloc(imgWidth*imgHeight*2,sizeof(unsigned char)); //TODO: find place to put: free(img_uncertainty);

  //paparazzi_message_server_start();
}

void my_plugin_run(unsigned char *frame)
{
  int pitch = 0; //ppz2gst.pitch/36;
  int roll = 0; //ppz2gst.roll/36;

  // Run actual Image Analysis
  get_obstacle_bins_above_horizon(frame, img_uncertainty, adjustfactor, N_BINS, gst2ppz.obstacle_bins, gst2ppz.uncertainty_bins, pitch, roll);


				printf("*od*"); // protocol start for obstacle info
				for(int bin = 0; bin < N_BINS; bin++)
				{
				    printf("%d,", gst2ppz.obstacle_bins[bin]);
				}
//		    	printf("u");
//		    	for(bin = 0; bin < n_bin; bin++)
//		    	{
//		    	    printf("%d,", uncertainty[bin]);
//		    	}
		    	printf("s\n"); // protocol end
    // Send to paparazzi
  //  paparazzi_message_send();
}


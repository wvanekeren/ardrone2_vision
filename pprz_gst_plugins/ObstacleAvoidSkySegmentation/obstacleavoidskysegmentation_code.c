
#include <stdio.h>
#include <stdlib.h>

#include "obstacleavoidskysegmentation_code.h"
#include "skysegmentation.h"

#include "paparazzi.h"

// Settable by pluging
unsigned int imgWidth, imgHeight;
unsigned int tcp_port = 2200;
unsigned int adjust_factor = 0;
unsigned int verbose = 0;

// Local variables
static unsigned char * img_uncertainty;

// Called by plugin
void my_plugin_init(void)
{
  // Allocate second image
  img_uncertainty= (unsigned char *) calloc(imgWidth*imgHeight*2,sizeof(unsigned char)); //TODO: find place to put: free(img_uncertainty);

  // Init variables
  ppz2gst.pitch = 0;
  ppz2gst.pitch = 0;

  gst2ppz.counter = 0;

  // Start Socket Thread
  paparazzi_message_server_start();
}

void my_plugin_run(unsigned char *frame)
{
  // 12 bit FRAC in Radians -> Degrees
  int pitch = ppz2gst.pitch / 71.488686161687739470794373877294;
  int roll = ppz2gst.roll / 71.488686161687739470794373877294;

  // Run actual Image Analysis
  get_obstacle_bins_above_horizon(frame, img_uncertainty, adjust_factor, N_BINS, gst2ppz.obstacle_bins, gst2ppz.uncertainty_bins, pitch, roll);

  // Send to paparazzi
  gst2ppz.ID = 0x0001;
  gst2ppz.counter++;
  paparazzi_message_send();

  // Verbose
  if (verbose > 0)
  {
    printf("*od*%d*",  gst2ppz.counter); // protocol start for obstacle info
    for(int bin = 0; bin < N_BINS; bin++)
    {
      printf("%d,", gst2ppz.obstacle_bins[bin]);
    }
    if (verbose > 1)
    {
      printf("u");
      for(int bin = 0; bin < N_BINS; bin++)
      {
        printf("%d,", gst2ppz.uncertainty_bins[bin]);
      }
    }
    printf("s\n"); // protocol end
  }
}


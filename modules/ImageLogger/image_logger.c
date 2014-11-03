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


/*
 * Author: Wim 
 */

// flag 1 23/09/2014 13:27

// Own header
#include "image_logger.h"

// Semaphore to control memory acces
#include <semaphore.h>

// UDP RTP Images
#include "udp/socket.h"

// Timing
#include <sys/time.h>

// Calculations
#include <math.h>

// Threaded computer vision
#include <pthread.h>

// Paparazzi State: Attitude -> Vision
#include "state.h" // for attitude
#include "subsystems/ins/ins_int.h" // used for ins.sonar_z
#include "autopilot.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "firmwares/rotorcraft/navigation.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/stabilization.h"


//Values from optitrack system
#include "subsystems/gps.h"
// Video
#include "v4l/video.h"
#include "resize.h"

#include "encoding/jpeg.h"
#include "encoding/rtp.h"

// Own definitions
#define VIDEO_IN_W	320
#define VIDEO_IN_H	240
#define Fx		343.1211 // Camera focal length (px/rad)
#define Fy		348.5053 // Camera focal length (px/rad)

// Standard headers
#include <stdio.h>
#include <string.h>
#include <unistd.h> // added for usleep


// Downlink
//#ifndef DOWNLINK_DEVICE
//#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
//#endif
#include "messages.h"
#include "subsystems/datalink/downlink.h"


volatile uint8_t computervision_thread_has_results = 0;


// take image from flight plan
void saveThisImage(unsigned char *frame_buf, int width, int height, int counter, char *imgpath);

void image_logger_run(void) {
  

}



// COMPUTER VISION THREAD


// Timers
struct timeval start_time;
struct timeval end_time;

#define USEC_PER_SEC 1000000
volatile long time_elapsed (struct timeval *t1, struct timeval *t2);
volatile long time_elapsed (struct timeval *t1, struct timeval *t2)
{
	long sec, usec;
	sec = t2->tv_sec - t1->tv_sec;
	usec = t2->tv_usec - t1->tv_usec;
	if (usec < 0) {
	--sec;
	usec = usec + USEC_PER_SEC;
	}
	return sec*USEC_PER_SEC + usec;
}
void start_timer() {
	gettimeofday(&start_time, NULL);
}
long end_timer() {
	gettimeofday(&end_time, NULL);
	return time_elapsed(&start_time, &end_time);
}


pthread_t computervision_thread;
volatile uint8_t computervision_thread_status = 0;
volatile uint8_t computer_vision_thread_command = 1;



void *computervision_thread_main(void* data)
{
  
  printf("computervision_thread_main started\n");
  
  // Video Input
  struct vid_struct vid;
  vid.device = (char*)"/dev/video2";
  vid.w=320;
  vid.h=240;
  vid.n_buffers = 4;
  if (video_init(&vid)<0) {
    printf("Error initialising video\n");
    computervision_thread_status = -1;
    return 0;
  }
  
  // Video Grabbing
  struct img_struct* img_new = video_create_image(&vid);

  
  float dt	=0.0;
  float dt_total=0.0;
  long 	diffTime;
  int   img_counter2 = 0;
  
  

  
  
 

  char imgpath[512];
  
  FILE *fpinfo;
  // Check for available files

  sprintf(imgpath,"%s","/data/video/usb/");
  fpinfo=fopen("/data/video/usb/imginfo.csv", "w");  
  fprintf(fpinfo,"image id; dt\n");  
  
  printf("first line written\n");
  
  while (computer_vision_thread_command > 0)
  {
    
     
    
     usleep(1000* 500);

     // Grab image from camera
     video_grab_image(&vid, img_new);
     
     img_counter2++;
     printf("starting image writing\n");
     saveThisImage(img_new->buf,WIDTH,HEIGHT,img_counter2,imgpath);
     printf("image written\n");

     diffTime = end_timer();
     start_timer();
     dt = (float)(diffTime)/USEC_PER_SEC;
     
     // frames per second
     dt_total = dt_total+dt; // delta t since last calculated flow
     
     printf("dt [s]: %f\n",dt);
     fprintf(fpinfo,"%d;%f;\n",img_counter2,dt);
     
     
     computervision_thread_has_results++;
     
     
     if (img_counter2 > 25)
       computer_vision_thread_command = 0;
     
     
     
  }
  printf("Thread Closed\n");
  video_close(&vid);
  fclose(fpinfo);
  computervision_thread_status = -100;
  return 0;    
  }

void image_logger_start(void){
  
  computer_vision_thread_command = 1;
  int rc = pthread_create(&computervision_thread, NULL, computervision_thread_main, NULL);
  if(rc) {
    printf("ctl_Init: Return code from pthread_create(mot_thread) is %d\n", rc);
  }
}

void image_logger_stop(void)
{
  computer_vision_thread_command = 0;
}

void saveThisImage(unsigned char *frame_buf, int width, int height, int counter, char *imgpath) {
  
  // Check for available files
  char filename[100];
  sprintf(filename, "%s%05d.csv",imgpath , counter); 
  printf("%s\n",filename);
  
  int i;
  int j;
  
  FILE *fp;

  fp=fopen(filename, "w");

  for(i=0; i<height; i++)
  {
      for(j=0; j<width*2; j++) 
      {
	      fprintf(fp, "%u;",frame_buf[i * width * 2 + j]); 
      }
      fprintf(fp,"\n");
  }
  fclose(fp);
  
}
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
 * Runs Maxime's OpticFlow written for DSP completely on ARM side
 */

// flag 1 23/09/2014 13:27

// Own header
#include "opticflowarm_module.h"

// Semaphore to control memory acces
#include <semaphore.h>

// UDP RTP Images
#include "udp/socket.h"

// Timing
#include <time.h>

// Threaded computer vision
#include <pthread.h>

// Paparazzi State: Attitude -> Vision
#include "state.h" // for attitude
#include "subsystems/ins/ins_int.h" // used for ins.sonar_z
#include "autopilot.h"

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

// Standard headers
#include <stdio.h>
#include <string.h>
#include <unistd.h> // added for usleep


// Optical flow code
#include "opticflow_sobel_2.h"
#include "calcFlowXYZ.h"

// Downlink
//#ifndef DOWNLINK_DEVICE
//#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
//#endif
#include "messages.h"
#include "subsystems/datalink/downlink.h"

// Variables
// filtered optic_flow_velocity
float Vx_filt = 0.0, Vy_filt=0.0, Vz_filt=0.0;

static sem_t sem_results;

float OFXInt = 0;
float OFYInt = 0;
unsigned int pGainHover = 12;
unsigned int iGainHover = 6;

#define SAT	1500
unsigned char saturateX = 0, saturateY = 0;
unsigned char first = 1;

volatile struct Int32Eulers cmd_euler;
volatile uint8_t computervision_thread_has_results = 0;

void opticflow_module_run(void) {
  
	//if(autopilot_mode == AP_MODE_OPTIC_FLOW)
	//{
		// Read Latest Vision Module Results
		if (computervision_thread_has_results)
		{
			computervision_thread_has_results = 0;

			sem_wait(&sem_results);

			if(saturateX==0)
			{
				OFXInt += iGainHover*Vx_filt/10;
			}
			if(saturateY==0)
			{
				OFYInt += iGainHover*Vy_filt/10;
			}

			cmd_euler.phi = (pGainHover*Vx_filt + OFXInt)/10;
			cmd_euler.theta = (pGainHover*Vy_filt + OFYInt)/10;

			sem_post(&sem_results);

			saturateX = 0; saturateY = 0;
			if(cmd_euler.phi<-SAT){cmd_euler.phi = -SAT; saturateX = 1;}
			else if(cmd_euler.phi>SAT){cmd_euler.phi = SAT; saturateX = 1;}
			if(cmd_euler.theta<-SAT){cmd_euler.theta = -SAT; saturateY = 1;}
			else if(cmd_euler.theta>SAT){cmd_euler.theta = SAT;saturateY = 1;}
		}
		/*else
		{
			cmd_euler.phi = 0;
			cmd_euler.theta = 0;
		}*/

		if(first)
		{
			cmd_euler.psi = stateGetNedToBodyEulers_i()->psi;
			first = 0;
		}
		//stabilization_attitude_set_rpy_setpoint_i(&cmd_euler); // wait with this line until testing is justifiable
	//}
	//else
	//{
	//	OFXInt = 0;
	//	OFYInt = 0;
	//}
	DOWNLINK_SEND_OPTICFLOW_CTRL(DefaultChannel, DefaultDevice, &cmd_euler.phi, &cmd_euler.theta);

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
	gettimeofday (&start_time, NULL);
}
long end_timer() {
	gettimeofday (&end_time, NULL);
	return time_elapsed(&start_time, &end_time);
}

struct timeval start_time_rates;
struct timeval end_time_rates;

void start_timer_rates(void) {
	gettimeofday (&start_time_rates, NULL);
}
long end_timer_rates(void) {
	gettimeofday (&end_time_rates, NULL);
	return time_elapsed(&start_time_rates, &end_time_rates);
}

//Camera parameters
#define Fx		343.1211
#define Fy		348.5053

pthread_t computervision_thread;
volatile uint8_t computervision_thread_status = 0;
volatile uint8_t computer_vision_thread_command = 1;



void *computervision_thread_main(void* data)
{
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

  // Video Resizing
  #define DOWNSIZE_FACTOR   1
  uint8_t quality_factor = 50; // From 0 to 99 (99=high)
  uint8_t dri_jpeg_header = 0;
  int millisleep = 250;
  
  // image small is only used for transmitting to GCS
  struct img_struct small;
  small.w = vid.w / DOWNSIZE_FACTOR;
  small.h = vid.h / DOWNSIZE_FACTOR;
  small.buf = (uint8_t*)malloc(small.w*small.h*2);
  
  // Video Compression
  uint8_t* jpegbuf = (uint8_t*)malloc(vid.h*vid.w*2);

  // Network Transmit
  struct UdpSocket* vsock;
  vsock = udp_socket("192.168.1.255", 5000, 5001, FMS_BROADCAST);

  
  // declare and initialise parameters for calcFlowXYZ
  unsigned int profileX[320] = {}; 	// feature histograms
  unsigned int prevProfileX[320] = {};
  unsigned int profileY[240] = {};
  unsigned int prevProfileY[240] = {};
  
  unsigned int *histTempX,*histTempY; 	// hist Temp

  unsigned int threshold = 1000; 	// initial feature detection threshold
  
  unsigned int nbSkipped = 0;
  int Txp=0,Typ=0,Tzp=0;			// optical flow in percent px
  float Tx=0.0,Ty=0.0,Tz=0.0;			// optical flow in px
  
  unsigned int erreur;
  unsigned int percentDetected;
  
  float h=0.0;
  long timestamp;
  float FPS;
  
  struct FloatRates* body_rate;
  float dt=0.0;
  long diffTime;
  float p=0.0,		q=0.0; 			// originially, these variables were defined outside computer_vision_thread
  float p_temp=0.0,	q_temp=0.0;		// originially, these variables were defined outside computer_vision_thread
  float p_corr=0.0, 	q_corr=0.0;
  float Tx_corr = 0.0, 	Ty_corr=0.0;
  
  float Vx=0.0, Vy=0.0, Vz=0.0; 	// velocity in body frame
  float Vxprev=0.0, Vyprev=0.0, Vzprev=0.0;
  
  
  // temporary
  sem_post(&sem_results);
  
  
  while (computer_vision_thread_command > 0)
  {
    

    // Grab image from camera
    video_grab_image(&vid, img_new);

    /*// Send encoded image to GCS
    // Resize: device by 4
    resize_uyuv(img_new, &small, DOWNSIZE_FACTOR);
    
    // JPEG encode the image:
    uint32_t image_format = FOUR_TWO_TWO;  // format (in jpeg.h)
    uint8_t* end = encode_image (small.buf, jpegbuf, quality_factor, image_format, small.w, small.h, dri_jpeg_header);
    uint32_t size = end-(jpegbuf);

    printf("Sending an image ...%u\n",size);

    send_rtp_frame(
        vsock,            // UDP
        jpegbuf,size,     // JPEG
        small.w, small.h, // Img Size
        0,                // Format 422
        quality_factor,               // Jpeg-Quality
        dri_jpeg_header,                // DRI Header
        0              // 90kHz time increment
     );*/
     
    
     // CALCULATE FEATURE HISTOGRAMS
     percentDetected = ApplySobelFilter2(img_new, profileX, profileY, &threshold); 
     
     // CALCULATE FLOW
     // Tx/Ty/Tz are (probably) integers of 0.01 px
     erreur = calcFlowXYZ(&Txp,&Typ,&Tzp,profileX,prevProfileX,profileY,prevProfileY,&nbSkipped);
     
     // Convert from [percent px] to [px]
     Tx = Txp/100;
     Ty = Typ/100;
     // ------------------
     // CALCULATE VELOCITY
     // ------------------
     
     // first, wait to proceed
     sem_wait(&sem_results);
     
     // delta t
     diffTime = end_timer();
     start_timer();
     dt = (float)(diffTime)/USEC_PER_SEC;
     
     // frames per second
     FPS = 1/dt;
          
     // MICRO ROTATION (adapted code)			
     // current body rates
     body_rate = stateGetBodyRates_f();
     p_temp = body_rate->p;
     q_temp = body_rate->q;     
     
     //micro roation
     p_corr = p_temp*dt; // this is actually not a rate but an angle (RAD)
     q_corr = q_temp*dt; // this is actually not a rate but an angle (RAD)
     //p_corr = p; q_corr = q;
     //p = 0; q = 0;
     
     // calculate corrected flow with current roll/pitch movements (in px)
     Tx_corr = (float)Tx - p_corr*(float)Fx;
     Ty_corr = (float)Ty - q_corr*(float)Fy;
     
     // actual height [m]
     h = (float)ins_impl.sonar_z/1000; // sonar_z is an integer with unit [mm]
     
     // ACTUAL VELOCITY CALCULATION
     // velocity calculation from optic flow
     Vx = h*FPS*(float)Tx_corr/(Fx); // [m/s]
     Vy = h*FPS*(float)Ty_corr/(Fy); // [m/s]
     Vz = h*FPS*(float)Tz; 
     
     // filter V
     Vx_filt = 0.16*Vx + 0.84*Vx_filt;
     Vy_filt = 0.16*Vy + 0.84*Vy_filt;

     /* original calculations to calculate flow (Txp is the integer from calcFlowXYZ)
			Tx = (float)(Txp) - p_corr*(float)(Fx);
			Ty = (float)(Typ) - q_corr*(float)(Fy);
#if USE_OPTITRACK_Z
			Tx = (FPS*gps.optitrack_z*Tx)/(10*Fx);
			Ty = (FPS*gps.optitrack_z*Ty)/(10*Fy);
#else
			Tx = (FPS*ins_impl.sonar_z*Tx)/(10*Fx);
			Ty = (FPS*ins_impl.sonar_z*Ty)/(10*Fy);
#endif

			Tx_filt = 0.16*Tx + 0.84*Tx_filt;
			Ty_filt = 0.16*Ty + 0.84*Ty_filt;

     */
     sem_post(&sem_results);
     
     // DOWNLINK
     DOWNLINK_SEND_OF_DEBUG(DefaultChannel,DefaultDevice,&Tx,&Ty,&Tz, &Vx, &Vy, &Vz, &Vx_filt, &Vy_filt, &Vz_filt, &p_corr, &q_corr, &percentDetected, &erreur, &FPS, &h);
    
     // report new results
     computervision_thread_has_results++;
     
  }
  printf("Thread Closed\n");
  video_close(&vid);
  computervision_thread_status = -100;
  return 0;    
  }
  
  


void opticflow_module_start(void)
{
  
  computer_vision_thread_command = 1;
  int rc = pthread_create(&computervision_thread, NULL, computervision_thread_main, NULL);
  if(rc) {
    printf("ctl_Init: Return code from pthread_create(mot_thread) is %d\n", rc);
  }
}

void opticflow_module_stop(void)
{
  computer_vision_thread_command = 0;
}




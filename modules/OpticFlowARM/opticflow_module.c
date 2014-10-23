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
#include "opticflow_module.h"

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

// Standard headers
#include <stdio.h>
#include <string.h>
#include <unistd.h> // added for usleep


// Optical flow code
#include "opticflow_sobelfilter.h"
#include "calcflow.h"

// Downlink
//#ifndef DOWNLINK_DEVICE
//#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
//#endif
#include "messages.h"
#include "subsystems/datalink/downlink.h"

// Variables
// filtered optic_flow_velocity

float Vx_ctrl = 0.0, Vy_ctrl=0.0, Vz_ctrl=0.0;

float OFXInt = 0;
float OFYInt = 0;

// default settings
uint32_t pGainHover = 500;
uint32_t iGainHover = 5;
uint32_t dGainHover = 300;
unsigned int window = 12;

#define SAT	1500 // as int32 angle, so angle in rad = 1500/(1<<INT32_ANGLE_FRAC) = 1500/(2^12) = 0.36 rad (21 deg)
unsigned char saturateX = 0, saturateY = 0;
unsigned char first = 1;
int32_t cmd_phi0 	= 0;
int32_t cmd_theta0 	= 0;
int32_t cmd_psi0 	= 0;

volatile struct Int32Eulers cmd_euler;
volatile uint8_t computervision_thread_has_results = 0;

// acceleration in horizontal plane
struct NedCoor_f* A_ned;
void UpdateAccel(void);
float Ahx = 0.0;
float Ahy = 0.0;
float Ahx_m = 0.0;
float Ahy_m = 0.0;

// Complementary filter
uint8_t first_compl = 1;
float Vx_err_m = 0.0;
float Vy_err_m = 0.0;
float Vx_err_int_m = 0.0;
float Vy_err_int_m = 0.0;
float Ahx_bias_m = 0.0;
float Ahy_bias_m = 0.0;
float Vx_err_f = 0.0;
float Vy_err_f = 0.0;
float Vx_err_int_f = 0.0;
float Vy_err_int_f = 0.0;
float Ahx_bias_f = 0.0;
float Ahy_bias_f = 0.0;
float Vx_compl_m = 0.0;
float Vy_compl_m = 0.0;
float Vx_compl_f = 0.0;
float Vy_compl_f = 0.0;
float pGainCompl = 40;
float iGainCompl = 6.28;

// optic flow
float Vx_filt = 0.0, 		Vy_filt=0.0, 		Vz_filt=0.0;
float Vx_corr_rate=0.0, 	Vy_corr_rate=0.0;
float Vx_corr_angle=0.0, 	Vy_corr_angle=0.0;
float Vx_filt_corr_rate=0.0, 	Vy_filt_corr_rate=0.0;
float Vx_filt_corr_angle=0.0, Vy_filt_corr_angle=0.0;


// body velocity
struct FloatVect3 V_body;
void UpdateAutopilotBodyVel(void);


void opticflow_module_run(void) {
  
  UpdateAccel(); // update acceleration measurements for derivative control
  
	if(autopilot_mode == AP_MODE_OPTIC_FLOW) {
	  opticflow_control_hover_run();
	}
	else {
	  opticflow_control_hover_stop();
	}
}

void opticflow_control_hover_run(void) {
  
  

  if(first)
  {
	  
	  // last known setpoints are the trim setpoints:
	  cmd_phi0 	= stab_att_sp_euler.phi;
	  cmd_theta0 	= stab_att_sp_euler.theta;
	  cmd_psi0  	= stab_att_sp_euler.psi;
	  
	  // psi command is current heading
	  cmd_euler.psi = stateGetNedToBodyEulers_i()->psi;
	  first = 0;
  }	  	
  // Read Latest Vision Module Results
  if (computervision_thread_has_results)
  {
	  
	    
	  computervision_thread_has_results = 0;

	  if(saturateX==0)
	  {
		  OFYInt -= iGainHover*Vy_ctrl; // minus sign: positive Y velocity = negative phi command
	  }
	  if(saturateY==0)
	  {
		  OFXInt += iGainHover*Vx_ctrl;
	  }

	  cmd_euler.phi 	= cmd_phi0   + (pGainHover*-Vy_ctrl + OFYInt + dGainHover*-Ahy); // minus sign: positive Y velocity = negative phi command
	  cmd_euler.theta 	= cmd_theta0 + (pGainHover* Vx_ctrl + OFXInt + dGainHover*Ahx);
	  cmd_euler.psi 	= cmd_psi0;
	  
	  saturateX = 0; saturateY = 0;
	  
	  // if saturation limits are reached, integrator should not integrate extra
	  if(cmd_euler.phi < -SAT) {
	    cmd_euler.phi = -SAT; 
	    saturateX = 1; 
	  }
	  else if(cmd_euler.phi > SAT) {
	    cmd_euler.phi = SAT; 
	    saturateX = 1;
	  }
	  if(cmd_euler.theta < -SAT) { 
	    cmd_euler.theta = -SAT; 
	    saturateY = 1;
	  }
	  else if(cmd_euler.theta > SAT) { 
	    cmd_euler.theta = SAT;
	    saturateY = 1; 
	  }
	  
  }

  // for downlink only
  int32_t dl_cmd_phi = cmd_euler.phi;
  int32_t dl_cmd_theta = cmd_euler.theta;
  int32_t dl_cmd_psi = cmd_euler.psi;

  stabilization_attitude_set_rpy_setpoint_i(&cmd_euler); // wait with this line until testing is justifiable
  DOWNLINK_SEND_OF_CTRL(DefaultChannel, DefaultDevice, &dl_cmd_phi, &dl_cmd_theta, &dl_cmd_psi, &cmd_phi0, &cmd_theta0, &cmd_psi0, &pGainHover, &iGainHover, &dGainHover);
}

void opticflow_control_hover_stop(void) {
  // when optic flow inactive, reset integrator
  first=1;
  OFXInt = 0;
  OFYInt = 0;
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

struct timeval start_time_rates;
struct timeval end_time_rates;

void start_timer_rates(void) {
	gettimeofday(&start_time_rates, NULL);
}
long end_timer_rates(void) {
	gettimeofday(&end_time_rates, NULL);
	return time_elapsed(&start_time_rates, &end_time_rates);
}



pthread_t computervision_thread;
volatile uint8_t computervision_thread_status = 0;
volatile uint8_t computer_vision_thread_command = 1;




void UpdateAccel(void) {
  
    
    // acceleration measurement
    A_ned = stateGetAccelNed_f();
    
    
    
    
    // attitude
    float psi = stateGetNedToBodyEulers_f()->psi;
    
    // horizontal acceleration in body-x, body-y
    Ahx_m = cosf(psi)*A_ned->x  + sinf(psi)*A_ned->y;
    Ahy_m = -sinf(psi)*A_ned->x + cosf(psi)*A_ned->y;
    
    // filter
    float alpha_a = 0.05;
    Ahx = alpha_a*Ahx_m + (1-alpha_a)*Ahx;
    Ahy = alpha_a*Ahy_m + (1-alpha_a)*Ahy;
}

void ComplementaryFilter_run(float *dt);
void ComplementaryFilter_run(float *dt) {
    
    if (first_compl) {
      first_compl = 0;
      Vx_compl_m = Vx_filt_corr_angle;
      Vy_compl_m = Vx_filt_corr_angle;      
      Vx_compl_f = Vx_filt_corr_angle;
      Vy_compl_f = Vx_filt_corr_angle;
    }
    
    // raw accelerometer
    Vx_err_m = Vx_compl_m - Vx_filt_corr_angle;
    Vy_err_m = Vy_compl_m - Vx_filt_corr_angle;

    Vx_err_int_m += iGainCompl*Vx_err_m*(*dt);
    Vy_err_int_m += iGainCompl*Vy_err_m*(*dt);

    Ahx_bias_m = pGainCompl*Vx_err_m + Vx_err_int_m;
    Ahy_bias_m = pGainCompl*Vy_err_m + Vy_err_int_m;  
  
    Vx_compl_m = Vx_compl_m + (Ahx_m-Ahx_bias_m)*(*dt);
    Vy_compl_m = Vy_compl_m + (Ahx_m-Ahy_bias_m)*(*dt); 
    
    // filtered accelerometer
    Vx_err_f = Vx_compl_f - Vx_filt_corr_angle;
    Vy_err_f = Vy_compl_f - Vx_filt_corr_angle;

    Vx_err_int_f += iGainCompl*Vx_err_f*(*dt);
    Vy_err_int_f += iGainCompl*Vy_err_f*(*dt);

    Ahx_bias_f = pGainCompl*Vx_err_f + Vx_err_int_f;
    Ahy_bias_f = pGainCompl*Vy_err_f + Vy_err_int_f;      
    
    Vx_compl_f = Vx_compl_f + (Ahx  -Ahx_bias_f)*(*dt);
    Vx_compl_f = Vy_compl_f + (Ahx  -Ahy_bias_f)*(*dt);
}


void UpdateAutopilotBodyVel(void) {
     
     // Ned velocity
     struct NedCoor_f* V_ned;
     V_ned = stateGetSpeedNed_f();
     
     
     // attitude
     struct FloatQuat* BodyQuaternions = stateGetNedToBodyQuat_f();
     
     struct FloatRMat Rmat_Ned2Body;
     
     // rotation matrix from quaternions_filt_corr
     FLOAT_RMAT_OF_QUAT(Rmat_Ned2Body,*BodyQuaternions);
     
     struct FloatVect3 vect_ned;
     vect_ned.x = V_ned->x;
     vect_ned.y = V_ned->y;
     vect_ned.z = V_ned->z;

     // multiply
     FLOAT_RMAT_VECT3_MUL(V_body, Rmat_Ned2Body, vect_ned);  
}

//Camera parameters
#define Fx		343.1211
#define Fy		348.5053


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

  // declare and initialise parameters for calcflow
  unsigned int profileX[320] = {}; 	// feature histograms
  unsigned int prevProfileX[320] = {};
  unsigned int profileY[240] = {};
  unsigned int prevProfileY[240] = {};
  

  
  unsigned int threshold = 1000; 	// initial feature detection threshold
  
  unsigned int curskip = 0;
  unsigned int framesskip = 5;
  unsigned int lowflow_counter = 0;
  int 	Txp=0,     Typ=0,     Tzp=0;			// optical flow in percent px
  int 	prevTxp=0, prevTyp=0, prevTzp=0;
  float Tx=0.0,	   Ty=0.0,    Tz=0.0;			// optical flow in px
  
  
  unsigned int erreur;
  unsigned int percentDetected;
  
  float FPS;
  
  struct FloatRates* body_rate;
  struct FloatEulers* body_angle;
  
  float phi_corr = 0.0;
  float phi_temp = 0.0;
  float phi_temp_prev = 0.0;
  float theta_corr = 0.0;
  float theta_temp = 0.0;
  float theta_temp_prev = 0.0;
  
  float dt=0.0;
  float dt_total=0.0;
  long diffTime;
  float p_temp=0.0,	q_temp=0.0;		// originially, these variables were defined outside computer_vision_thread
  float p_corr=0.0, 	q_corr=0.0;
  
  
  float Vx=0.0,    Vy=0.0,    Vz=0.0; 	// velocity in body frame
  
  
  float Tx_slow=0.0;
  float Ty_slow=0.0;
  float Tn_slow=0.0;
  float Tx_corr_rate = 0.0, 	Ty_corr_rate=0.0;
  float Tx_corr_angle = 0.0, 	Ty_corr_angle=0.0;

  
  
  // flow velocity filter
  float alpha_v = 0.16;
  
  // sonar height
  float h_sonar_prev = 0.0;
  float h_sonar = 0.0;
  float h_sonar_m = 0.0;
  float h_sonar_raw = 0.0;
  float h_gps_corr = 0.0;
  float h = 0.0;
  float sonar_scaling = 0.68; // rough estimate of the scaling 
  float alpha_h = 0.5;
  
  // sending Hist Data
  uint8_t msg_id = 0;
  uint8_t profileYpart1[120] = {}; // for downlink
  uint8_t profileYpart2[120] = {};
  uint8_t part_id = 0;
  int img_counter = 0;

  // Autopilot state
  float state_phi = 0.0;
  float state_theta = 0.0;
  float state_psi = 0.0;
  
  float state_vx = 0.0;
  float state_vy = 0.0;
  float state_vz = 0.0;
  
  float state_p = 0.0;
  float state_q = 0.0;
  float state_r = 0.0;  
  
  float state_rpm1 = 0.0;
  float state_rpm2 = 0.0;
  float state_rpm3 = 0.0;
  float state_rpm4 = 0.0;
  
  
  
  while (computer_vision_thread_command > 0)
  {
    
    
     // Grab image from camera
     video_grab_image(&vid, img_new);
     
      
     
     img_counter++;
     if (img_counter==30) {
       printf("starting image writing\n");
       //send_this_image(img_new);
       img_counter=0;
       printf("image written\n");
     }
     
    // --------------
    // CALCULATE FLOW
    // --------------
    // Txp/Typ/Tzp are integers of 0.01 px
     
    // skip?
    curskip++;
    if(curskip < framesskip) {
      Txp = prevTxp;
      Typ = prevTyp;
      Tzp = prevTzp;
          
    }
    // or use calculated values
    else {
      // calculate feature histograms
      percentDetected = ApplySobelFilter2(img_new, profileX, profileY, &threshold);
      //percentDetected = ApplySobelFilter2(img_new, profile3X, profile3Y, &threshold);
      
      // calculate flow from current and previous histograms
      erreur = calcFlow(&Txp,&Typ,&Tzp,profileX,prevProfileX,profileY,prevProfileY,&curskip,&framesskip,&window);
      
      
      prevTxp = Txp;
      prevTyp = Typ;
      prevTzp = Tzp;
        
      
      curskip=0; // set curskip to zero if you have just calculated new flow
      memcpy(prevProfileX,profileX,WIDTH*sizeof(unsigned int));
      memcpy(prevProfileY,profileY,HEIGHT*sizeof(unsigned int));	  
    }     
     
     
     // Convert from [percent px] to [px] (this is also a cast from int to float)
     // watch out!! from image axes to body axes conversion!!
     Tx = (float)Typ/FLOWFACT/(framesskip+1); 
     Ty = -(float)Txp/FLOWFACT/(framesskip+1);
     
    
     // slow flow for frameskip control
     float alpha_T = 0.05;
     Tx_slow = alpha_T*(float)Txp/FLOWFACT + (1-alpha_T)*Tx_slow; // divide by 100 because of the per-cent scaling of the flow
     Ty_slow = alpha_T*(float)Typ/FLOWFACT + (1-alpha_T)*Ty_slow;
     Tn_slow = sqrt(Tx_slow*Tx_slow+Ty_slow*Ty_slow);

     // delta t (for FPS and for micro-rotation)
     diffTime = end_timer();
     start_timer();
     dt = (float)(diffTime)/USEC_PER_SEC;
     
     // frames per second
     dt_total = dt_total+dt; // delta t since last calculated flow
     if (curskip==0) {
      FPS = ((float)framesskip+1)/dt_total; // literally frames-per-second
      dt_total=0;
     }
          
     // MICRO ROTATION
     
     // method 1: calculate with body rates
     // current body rates
     body_rate = stateGetBodyRates_f();
     
     p_corr = (body_rate->p)*dt;
     q_corr = (body_rate->q)*dt;

     // method 2: calculate microroation with body angle difference
     if (curskip==0) {
       body_angle 	= stateGetNedToBodyEulers_f();
       phi_temp 	= body_angle->phi;
       theta_temp 	= body_angle->theta;
       
       phi_corr 	= (phi_temp-phi_temp_prev)/(framesskip+1); // correction angle
       theta_corr 	= (theta_temp-theta_temp_prev)/(framesskip+1); // correction angle
     
       phi_temp_prev 	= phi_temp;
       theta_temp_prev 	= theta_temp;
     }


     // calculate corrected flow with current roll/pitch movements (in px)
     // watch out: Fy,Fx are in image axes!
     
     // flow correction for microrotatino
     Tx_corr_rate = (float)Tx - q_corr*(float)Fy;
     Ty_corr_rate = (float)Ty + p_corr*(float)Fx;
     Tx_corr_angle = (float)Tx - theta_corr*(float)Fy;
     Ty_corr_angle = (float)Ty + phi_corr*(float)Fx;     
      
     
     //send a LOT of hist data
     if (msg_id > 99)
       msg_id = 1;	
     else
       msg_id++;
     
     for (int i=0;i<240;i++) {
       
       if (i<120) {
	 profileYpart1[i] = profileY[i];
       }
       else {
	 profileYpart2[i-120] = profileY[i];
       }
     }

     part_id=1;
     DOWNLINK_SEND_OF_HIST(DefaultChannel,DefaultDevice,&msg_id,&part_id,&Tx, &Typ, &erreur, &framesskip, 120,profileYpart1);// warning from this line (truncation of integer in profileX)
     part_id=2;
     DOWNLINK_SEND_OF_HIST(DefaultChannel,DefaultDevice,&msg_id,&part_id,&Tx, &Typ, &erreur, &framesskip, 120,profileYpart2);// warning from this line (truncation of integer in profileX)
     
     
     // framesskip control
     #define TMIN 	5
     #define TMIN_STEPS 20
     #define MAX_SKIP 	9
     // only adjust framesskip after just calculating new flow
     if (curskip==0) { 
       
	// if flow is equal to max window
	if (abs(Typ)==window*FLOWFACT || abs(Txp)==window*FLOWFACT) {   
	  if (framesskip > 5) {
	    framesskip=framesskip-3;
	  }
	  else if (framesskip>0){
	    framesskip--;
	    
	  }
	}
	// if filtered flow stays low
	if (Tn_slow < TMIN && framesskip < MAX_SKIP) {
	  lowflow_counter++;
	  if (lowflow_counter > TMIN_STEPS/(framesskip+1)) {
	    lowflow_counter=0;
	    framesskip++;
	  }
	  
	}
     }

     
     // SONAR HEIGHT [m]
     h_sonar_prev = h_sonar;
     h_sonar_raw = (float)ins_impl.sonar_z/1000;//*sonar_scaling;// sonar_z is an integer with unit [mm]
     
     // delete outliers
     if (abs(h_sonar-h_sonar_prev)>3)
       h_sonar = h_sonar_prev;
     else
       h_sonar = h_sonar_raw;
     
     // corrected gps height
     h_gps_corr = stateGetPositionEnu_f()->z - 0.26; // 0.32 is the standard optitrack offset, but should be rechecked if used
     
     // which height do you use?
     h = h_sonar_raw;

     // ------------------
     // CALCULATE VELOCITY
     // ------------------

     // velocity calculation from optic flow
     // watch out: Fy,Fx are in image axes
     
     // V not corrected for microrotation
     Vx = h*FPS*(float)Tx/(Fy); // [m/s]
     Vy = h*FPS*(float)Ty/(Fx); // [m/s]
     Vz = h*FPS*(float)Tz; 
     
     // V corrected for microrotation
     Vx_corr_rate = h*FPS*(float)Tx_corr_rate/(Fy);
     Vy_corr_rate = h*FPS*(float)Ty_corr_rate/(Fx);
     Vx_corr_angle = h*FPS*(float)Tx_corr_angle/(Fy);
     Vy_corr_angle = h*FPS*(float)Ty_corr_angle/(Fx);     
     
     
     // FILTERED VELOCITY
     
     // filter strength
     if (threshold<300) {
       alpha_v = 0.08;
     }
     else if (threshold<600) {
       alpha_v = 0.12;
     }
     else {
       alpha_v = 0.16;
     }
     
     // uncorrected, filtered velocity
     Vx_filt = alpha_v*Vx + (1-alpha_v)*Vx_filt;
     Vy_filt = alpha_v*Vy + (1-alpha_v)*Vy_filt;
     
     // corrected, filtered velocity
     Vx_filt_corr_rate = alpha_v*Vx_corr_rate + (1-alpha_v)*Vx_filt_corr_rate;
     Vy_filt_corr_rate = alpha_v*Vy_corr_rate + (1-alpha_v)*Vy_filt_corr_rate;
     Vx_filt_corr_angle = alpha_v*Vx_corr_angle + (1-alpha_v)*Vx_filt_corr_angle;
     Vy_filt_corr_angle = alpha_v*Vy_corr_angle + (1-alpha_v)*Vy_filt_corr_angle;     
     
     // Which velocity do you use for the control?
     Vx_ctrl = Vx_filt_corr_angle;
     Vy_ctrl = Vy_filt_corr_angle;
 
     // Complementary filter with accelerometer
     ComplementaryFilter_run(&dt);
     
     
     // AUTOPILOT BODY VELOCITY
     UpdateAutopilotBodyVel();

     // Autopilot state
     
     state_phi = stateGetNedToBodyEulers_f()->phi;
     state_theta = stateGetNedToBodyEulers_f()->theta;
     state_psi = stateGetNedToBodyEulers_f()->psi;
     
     state_vx = V_body.x;
     state_vy = V_body.y;
     state_vz = -V_body.z;
     
     state_p = stateGetBodyRates_f()->p;
     state_q = stateGetBodyRates_f()->q;
     state_r = stateGetBodyRates_f()->r;
     
     // DOWNLINK
     DOWNLINK_SEND_OF_VELOCITIES(DefaultChannel,DefaultDevice,&Vx,&Vy,&Vz,&(V_body.x),&(V_body.y),&(V_body.z),&Vx_corr_rate,&Vy_corr_rate,&Vx_corr_angle,&Vy_corr_angle,&Vx_compl_m,&Vx_compl_m,&Vx_compl_f,&Vx_compl_f);
     DOWNLINK_SEND_OF_DEBUG(DefaultChannel,DefaultDevice,&Tx,&Ty,&Tz, &Vx, &Vy, &Vz, &Vx_ctrl, &Vy_ctrl, &Vz_ctrl, &p_corr, &q_corr, &percentDetected, &threshold, &erreur, &FPS, &h_sonar, &h_sonar_raw, &h, &msg_id, &framesskip, &window, &autopilot_mode, &Ahx_m, &Ahy_m);
     
     
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

void send_this_image(struct img_struct* img_new);
void send_this_image(struct img_struct* img_new) {

    // Video Compression
    uint8_t* jpegbuf = (uint8_t*)malloc(img_new->h*img_new->w*2);

    // Network Transmit
    struct UdpSocket* vsock;
    vsock = udp_socket("192.168.1.255", 5000, 5001, FMS_BROADCAST);  
  
    // Video Resizing
    #define DOWNSIZE_FACTOR   1
    uint8_t quality_factor  = 99; // From 0 to 99 (99=high)
    uint8_t dri_jpeg_header = 0;
  
    
    
    // JPEG encode the image:
    uint32_t image_format = FOUR_TWO_TWO;  // format (in jpeg.h)
    uint8_t* end = encode_image (img_new->buf, jpegbuf, quality_factor, image_format, img_new->w, img_new->h, dri_jpeg_header);
    uint32_t size = end-(jpegbuf);

    printf("Sending an image ...%u\n",size);
    
        send_rtp_frame(
        vsock,            // UDP
        jpegbuf,size,     // JPEG
        img_new->w, img_new->h, // Img Size
        0,                // Format 422
        quality_factor,               // Jpeg-Quality
        dri_jpeg_header,                // DRI Header
        0              // 90kHz time increment
     );
    
}

  
  




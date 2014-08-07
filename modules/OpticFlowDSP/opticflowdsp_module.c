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
#include <time.h>
#include <semaphore.h>
#include <pthread.h>

//DSP headers
#include "dbapi.h"
#include "dsp.h"

// Own header
#include "opticflowdsp_module.h"

// UDP Message with GST vision
#include "udp/socket.h"
#include "video_message_structs.h"

// Paparazzi State: Attitude -> Vision
#include "state.h" // for attitude
#include "subsystems/ins/ins_int.h"
#include "autopilot.h"

//Values from optiktrack system
#include "subsystems/gps.h"

// Downlink
#include "messages.h"
#include "subsystems/datalink/downlink.h"

#include "firmwares/rotorcraft/autopilot.h"

#define VIDEO_IN_W	320
#define VIDEO_IN_H	240

#define DOWNSIZE_FACTOR   1

#define VIDEO_OUT_W	 VIDEO_IN_W
#define VIDEO_OUT_H	 VIDEO_IN_H

#define SIZEBUFF2DSP	VIDEO_IN_W*VIDEO_IN_H*2
#define SIZEBUFF2MPU	VIDEO_OUT_W*VIDEO_OUT_H*2

struct UdpSocket *sock;
struct gst2ppz_message_struct gst2ppz;
struct ppz2gst_message_struct ppz2gst;

static sem_t sem_rates, sem_results, sem_optitrack, sem_acc;
volatile struct Int32Eulers cmd_euler;

void opticflowdsp_module_init(void)
{
	INT_EULERS_ZERO(cmd_euler);
}

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

volatile uint8_t computervision_thread_has_results = 0;

float OFXInt = 0;
float OFYInt = 0;

#define SAT		1500
unsigned char saturateX = 0, saturateY = 0;
unsigned char first = 1;

//Low-pass filterAP_MODE_OPTIC_FLOW
#define Fc		1
#define Fs		60
//S[n] = A*E[n] + B*S[n-1]
#define A	(float)((2*M_PI*Fc)/(2*M_PI*Fc + Fs))
#define B	(float)(Fs/(2*M_PI*Fc + Fs))

int lost = 0;

//Optic Flow results
float Tx,Ty;
int threshold;
//Filtered speed
float vx = 0,vy = 0;
float Tx_filt=0,Ty_filt=0;
unsigned int OF_P_HOVER = 12;
unsigned int OF_I_HOVER = 6;


void opticflowdsp_module_run(void)
{
	if(autopilot_mode == AP_MODE_OPTIC_FLOW)
	{
		// Read Latest Vision Module Results
		if (computervision_thread_has_results)
		{
			computervision_thread_has_results = 0;

			sem_wait(&sem_results);

			if(saturateX==0)
			{
				OFXInt += OF_I_HOVER*Tx_filt/10;
			}
			if(saturateY==0)
			{
				OFYInt += OF_I_HOVER*Ty_filt/10;
			}

			cmd_euler.phi = (OF_P_HOVER*Tx_filt + OFXInt)/10;
			cmd_euler.theta = (OF_P_HOVER*Ty_filt + OFYInt)/10;

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
		stabilization_attitude_set_rpy_setpoint_i(&cmd_euler);
	}
	else
	{
		OFXInt = 0;
		OFYInt = 0;
	}
	DOWNLINK_SEND_OPTICFLOW_CTRL(DefaultChannel, DefaultDevice, &cmd_euler.phi, &cmd_euler.theta);
}

/*** ANGULAR RATES	***/
struct timeval start_time_rates;
struct timeval end_time_rates;

void start_timer_rates(void) {
	gettimeofday (&start_time_rates, NULL);
}
long end_timer_rates(void) {
	gettimeofday (&end_time_rates, NULL);
	return time_elapsed(&start_time_rates, &end_time_rates);
}

float p=0,q=0;
float p_temp=0,q_temp=0;

//A ajouter dans stateSetBodyRates_ dans state.h
void opticFlowPropagateRates_i(struct Int32Rates* body_rate)
{
	sem_wait(&sem_rates);

	long diffTime = end_timer_rates();
	start_timer_rates();

	p += p_temp*(float)(diffTime)/USEC_PER_SEC;
	p_temp = (float)(body_rate->p)/(1<<INT32_RATE_FRAC);

	q += q_temp*(float)(diffTime)/USEC_PER_SEC;
	q_temp = (float)(body_rate->q)/(1<<INT32_RATE_FRAC);

	sem_post(&sem_rates);
}

void opticFlowPropagateRates_f(struct FloatRates* body_rate)
{
	sem_wait(&sem_rates);

	long diffTime = end_timer_rates();
	start_timer_rates();

	p += p_temp*(float)(diffTime)/USEC_PER_SEC;
	p_temp = (float)(body_rate->p);

	q += q_temp*(float)(diffTime)/USEC_PER_SEC;
	q_temp = (float)(body_rate->q);

	sem_post(&sem_rates);
}

/***	ACCELERATION	***/
//Acceleration in ECEF never used
struct timeval start_time_acc;
struct timeval end_time_acc;

void start_timer_acc(void) {
	gettimeofday (&start_time_acc, NULL);
}
long end_timer_acc(void) {
	gettimeofday (&end_time_acc, NULL);
	return time_elapsed(&start_time_acc, &end_time_acc);
}

float Ax_body = 0,Ay_body = 0;
struct NedCoor_i accel_i;
struct NedCoor_f accel_f;

float heading = 0;

//for complementary filter
float integraleX = 0, integraleY = 0;
float ax_corr = 0, ay_corr = 0;
struct timeval start_time_cmpl;
struct timeval end_time_cmpl;

//Fc = 1Hz
#define I 	40
#define K	6.28

void start_timer_cmpl(void) {
	gettimeofday (&start_time_cmpl, NULL);
}
long end_timer_cmpl(void) {
	gettimeofday (&end_time_cmpl, NULL);
	return time_elapsed(&start_time_cmpl, &end_time_cmpl);
}

void OpticFlowPropagateAcceleration_i(struct NedCoor_i* ned_accel )
{
	sem_wait(&sem_acc);

	float diffTime = (float)(end_timer_acc())/USEC_PER_SEC;
	start_timer_acc();

	//YAW unrotation
	heading = stateGetNedToBodyEulers_f()->psi;
	float cos_psi = cos(heading);
	float sin_psi = sin(heading);

	float Ax = (ned_accel->x*cos_psi + ned_accel->y*sin_psi)/(1<<INT32_ACCEL_FRAC);
	float Ay = (ned_accel->y*cos_psi - ned_accel->x*sin_psi)/(1<<INT32_ACCEL_FRAC);

	//Change into optic flow coordinates
	Ax_body = -100*Ay;
	Ay_body = 100*Ax;

	//Integration to compute velocities
	vx += (Ax_body-ax_corr)*diffTime;
	vy += (Ay_body-ay_corr)*diffTime;

	sem_post(&sem_acc);
}

void OpticFlowPropagateAcceleration_f(struct NedCoor_f* ned_accel )
{
	sem_wait(&sem_acc);
	float diffTime = (float)(end_timer_acc())/USEC_PER_SEC;
	start_timer_acc();

	//YAW unrotation
	heading = stateGetNedToBodyEulers_f()->psi;
	float cos_psi = cos(heading);
	float sin_psi = sin(heading);

	float Ax = ned_accel->x*cos_psi + ned_accel->y*sin_psi;
	float Ay = ned_accel->y*cos_psi - ned_accel->x*sin_psi;

	//Change into optic flow coordinates and convert from m/s in cm/s
	Ax_body = -100*Ay;
	Ay_body = 100*Ax;

	//Integration to compute velocities
	vx += (Ax_body-ax_corr)*diffTime;
	vy += (Ay_body-ay_corr)*diffTime;

	sem_post(&sem_acc);
}
/***	OPTITRACK	***/
int optitrack_x, optitrack_y, optitrack_z;
signed int optitrack_x_prev, optitrack_y_prev;
float x_ref_speed, y_ref_speed;

struct timeval start_time_optitrack;
struct timeval end_time_optitrack;

void start_timer_optitrack(void) {
	gettimeofday (&start_time_optitrack, NULL);
}
long end_timer_optitrack(void) {
	gettimeofday (&end_time_optitrack, NULL);
	return time_elapsed(&start_time_optitrack, &end_time_optitrack);
}

void OptiFlowNotifyNewOptitrackPos(int x, int y, int z)
{
	sem_wait(&sem_optitrack);

	long diffTime = end_timer_optitrack();
	start_timer_optitrack();

	optitrack_x = x;
	optitrack_y = y;
	optitrack_z = z;

	x_ref_speed = USEC_PER_SEC*(float)(optitrack_x - optitrack_x_prev)/(float)diffTime;		//Speed in mm/s
	y_ref_speed = USEC_PER_SEC*(float)(optitrack_y - optitrack_y_prev)/(float)diffTime;		//Speed in mm/s
	optitrack_x_prev = optitrack_x;
	optitrack_y_prev = optitrack_y;

	sem_post(&sem_optitrack);
}

/**************************************************************************************************
*										COMPUTER VISION THREAD
 *************************************************************************************************/
struct timeval start_time;
struct timeval end_time;

void start_timer() {
	gettimeofday (&start_time, NULL);
}
long end_timer() {
	gettimeofday (&end_time, NULL);
	return time_elapsed(&start_time, &end_time);
}

// Video
#include "v4l/video.h"
#include "resize.h"

#undef DOWNLINK_VIDEO
#undef PRINT_OF_RESULTS

#ifdef DOWNLINK_VIDEO
#include "encoding/jpeg.h"
#include "encoding/rtp.h"
#endif

//Camera parameters
#define Fx		343.1211
#define Fy		348.5053

pthread_t computervision_thread;
volatile uint8_t computervision_thread_status = 0;
volatile uint8_t computer_vision_thread_command = 0;

signed int camZOffset,camHeight;

void *computervision_thread_main(void* data)
{
	int firstMessage = 1;
	int debut_cmpl = 1;
	int status;
	struct VISION_TASK visionTask;
	long timestamp;
	float FPS;
	signed int Txp,Typ;
	float p_corr, q_corr;
	long diffTime;

	/* Messaging used for GPP/DSP synchronization */
	struct DSP_MSG msgToDsp;
	struct DSP_MSG msgFromDsp;

	//Image structures initailization
	struct img_struct* img_new = malloc(sizeof(struct img_struct));
	img_new->w = VIDEO_IN_W;
	img_new->h = VIDEO_IN_H;
	//img_new->buf = (unsigned char*)(malloc(2*VIDEO_IN_W*VIDEO_IN_H*sizeof(char)));

	struct img_struct* img_dsp = malloc(sizeof(struct img_struct));
	img_dsp->w = VIDEO_OUT_W;
	img_dsp->h = VIDEO_OUT_H;

	//Filtre complémentaire
	float erreurX = 0, erreurY = 0;

#ifdef PRINT_OF_RESULTS
	unsigned char cnt = 0;
#endif

	printf("DSP Init...\n");

	//if(system("sysctl -w kernel.shmmax=134217728")<0){printf("Error on shared memory allocation\n");}
	//if(system("sysctl -w kernel.shmall=134217728")<0){printf("Error on shared memory allocation\n");}
	if(system("/data/video/dsp/cexec.out -T /data/video/dsp/ddspbase_tiomap3430.dof64P")<0){printf("Error on loading DSP program\n");}

	if(DSPVisionOpen(&visionTask, img_new, img_dsp)!=0)
	{
		printf("Error initialising DSP\n");
		computervision_thread_status = -1;
		return 0;
	}

	// Video Input
	struct vid_struct vid;
	vid.device = (char*)"/dev/video2"; // video1 = front camera; video2 = bottom camera
	vid.w=320; // front camera = 1280; bottom camera = 320
	vid.h=240;  // front camera = 720; bottom camera = 240
	vid.n_buffers = 4;
	if (video_init(&vid)<0) {
		printf("Error initialising video\n");
		computervision_thread_status = -1;
		return 0;
	}

#ifdef DOWNLINK_VIDEO

	// Video Compression
	uint8_t* jpegbuf = (uint8_t*)malloc(vid.h*vid.w*2);

	// Network Transmit
	struct UdpSocket* vsock;
	vsock = udp_socket("192.168.1.255", 5000, 5001, FMS_BROADCAST);

#endif

	p_temp = 0;
	q_temp = 0;

	//Onboard logging
	FILE *logOF = fopen("/data/video/dsp/log", "w");
	char test = '0';
	fwrite(&test,sizeof(char),1,logOF);
	fclose(logOF);

#if OPTITRACK_POS
	optitrack_x_prev = gps.optitrack_x;
	optitrack_y_prev = gps.optitrack_y;
#endif

	while (computer_vision_thread_command > 0)
	{
		start_timer();

		video_grab_image(&vid, img_new);

		//Compute angular rates
		sem_wait(&sem_rates);
		diffTime = end_timer_rates();
		start_timer_rates();
		p += p_temp*(float)(diffTime)/USEC_PER_SEC;
		q += q_temp*(float)(diffTime)/USEC_PER_SEC;
		p_corr = p; q_corr = q;
		p = 0; q = 0;
		sem_post(&sem_rates);

		DSPProcessor_FlushMemory(visionTask.hProcessor, (PVOID)(visionTask.bufferSend),SIZEBUFF2DSP,0);
		msgToDsp.dwCmd = VISION_WRITEREADY;
		msgToDsp.dwArg1 = (DWORD)SIZEBUFF2DSP / g_dwDSPWordSize;
		status = DSPNode_PutMessage(visionTask.hNode, &msgToDsp,  DSP_FOREVER);
		if (DSP_FAILED(status)) {
			fprintf(stdout, "DSPProcessor_PutMessage failed. Status = 0x%x\n", (UINT)status);
		}

		usleep(10000);

		//Read back
		status = DSPNode_GetMessage(visionTask.hNode, &msgFromDsp, DSP_FOREVER);
		if (DSP_FAILED(status)) {
			fprintf(stdout, "DSPProcessor_GetMessage failed. Status = 0x%x\n", (UINT)status);
		}
		// Go ahead and flush here
		DSPProcessor_InvalidateMemory(visionTask.hProcessor, (PVOID)(visionTask.bufferReceive),SIZEBUFF2MPU);

#ifdef DOWNLINK_VIDEO
		// JPEG encode the image:
		uint32_t quality_factor = 60; // quality factor from 1 (high quality) to 8 (low quality)
		uint8_t dri_header = 0;
		uint32_t image_format = FOUR_TWO_TWO;  // format (in jpeg.h)
		uint8_t* end = encode_image (small.buf, jpegbuf, quality_factor, image_format, small.w, small.h, dri_header);
		uint32_t size = end-(jpegbuf);

		printf("Sending an image ...%u\n",size);
		uint32_t delta_t_per_frame = 0; // 0 = use drone clock
		send_rtp_frame(vsock, jpegbuf,size, small.w, small.h,0, quality_factor, dri_header, delta_t_per_frame);
#endif
		computervision_thread_has_results++;

		timestamp = end_timer();
		FPS = (float) 1000000/(float)timestamp;
		Txp = *((int*)(visionTask.bufferReceive));
		Typ = *((int*)(visionTask.bufferReceive+sizeof(int)));
		threshold = msgFromDsp.dwArg2;

		sem_wait(&sem_results);
		sem_wait(&sem_optitrack);
		sem_wait(&sem_acc);
		//if(ins_impl.sonar_z<1000 || autopilot_mode != AP_MODE_ATTITUDE_Z_HOLD)
		//Use optic flow when camera is above 1 meter from the ground
#if USE_OPTITRACK_Z
		if(optitrack_z<1000 || (autopilot_mode != AP_MODE_ATTITUDE_Z_HOLD && autopilot_mode != AP_MODE_OPTIC_FLOW))
#else
		if(ins_impl.sonar_z<1000 || (autopilot_mode != AP_MODE_ATTITUDE_Z_HOLD && autopilot_mode != AP_MODE_OPTIC_FLOW))
#endif
		{
			Tx = 0;	 Ty = 0;
			vx = 0;	 vy = 0;
			start_timer_cmpl();
			debut_cmpl = 1;
		}
		else
		{
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

			if(debut_cmpl)
			{
				vx = Tx;
				vy = Ty;
				debut_cmpl = 0;
				start_timer_cmpl();
			}
			else
			{
				//Run complementary filter
				diffTime = end_timer_cmpl();
				start_timer_cmpl();

				erreurX = vx - Tx;
				erreurY = vy - Ty;

				float temp = (float)diffTime/USEC_PER_SEC;

				integraleX += I*erreurX*temp;
				integraleY += I*erreurY*temp;

				ax_corr = K*erreurX + integraleX;
				ay_corr = K*erreurY + integraleY;

			}
		}

#ifdef PRINT_OF_RESULTS
		if(cnt++==5)
		{
			printf("X:%d Y:%d, th:%d\n",Txp, Typ, threshold);
			printf("dt = %d, FPS = %f\n\n",timestamp, FPS);
			cnt = 0;
		}
#endif
		if(firstMessage)
		{
#if OPTITRACK_POS
			//DOWNLINK_SEND_OPTICFLOWDSP(DefaultChannel, DefaultDevice, &Tx_filt, &Ty_filt, &vx, &vy, &threshold, &FPS, &optitrack_z, &ins_impl.sonar_z);
			DOWNLINK_SEND_OPTICFLOWDSP(DefaultChannel, DefaultDevice, &Tx_filt, &Ty_filt, &threshold, &FPS, &optitrack_x, &optitrack_y, &optitrack_z, &ins_impl.sonar_z);
			//DOWNLINK_SEND_OPTICFLOW(DefaultChannel, DefaultDevice, &Tx, &Ty, &vx, &vy, &threshold, &FPS, &gps.ecef_vel.x, &gps.ecef_vel.y, &gps.ecef_vel.z, &optitrack_z, &ins_impl.sonar_z, &heading, &Ax_body, &Ay_body);
			//DOWNLINK_SEND_OPTICFLOW(DefaultChannel, DefaultDevice, &vx, &vy, &threshold, &FPS, &gps.ecef_vel.x, &gps.ecef_vel.y, &gps.ecef_vel.z, &optitrack_z, &ins_impl.sonar_z);
			//DOWNLINK_SEND_OPTICFLOW(DefaultChannel, DefaultDevice, &Tx, &Ty, &threshold, &FPS, &gps.ecef_vel.x, &gps.ecef_vel.y, &gps.ecef_vel.z, &optitrack_z, &ins_impl.sonar_z);
#else
			DOWNLINK_SEND_OPTICFLOW(DefaultChannel, DefaultDevice, &Tx, &Ty, &threshold, &FPS, &gps.ecef_vel.x, &gps.ecef_vel.y, &gps.ecef_vel.z, &ins_impl.sonar_z);
#endif
			//firstMessage = 0;
		}
		sem_post(&sem_results);
		sem_post(&sem_optitrack);
		sem_post(&sem_acc);
	}
	printf("Thread Closed\n");
	video_close(&vid);
	DSPVisionClose(&visionTask);
	computervision_thread_status = -100;
	fclose(logOF);
	return 0;
}

void opticflowdsp_module_start(void)
{
	computer_vision_thread_command = 1;
	int rc = pthread_create(&computervision_thread, NULL, computervision_thread_main, NULL);
	if(rc) {
		printf("ctl_Init: Return code from pthread_create(opticflowdsp_thread) is %d\n", rc);
	}
	sem_init(&sem_rates, 1, 1);
	sem_init(&sem_results, 1, 1);
	sem_init(&sem_optitrack, 1, 1);
	sem_init(&sem_acc, 1, 1);
}

void opticflowdsp_module_stop(void)
{
	computer_vision_thread_command = 0;
	sem_destroy(&sem_rates);
	sem_destroy(&sem_results);
	sem_destroy(&sem_optitrack);
	sem_destroy(&sem_acc);
}

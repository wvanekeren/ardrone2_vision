/*
 * Copyright (C) 2012-2013 Kevin van Hecke
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

/**
 * @file subsystems/video/video_ardrone2.c
 * Video implementation for ardrone2.
 *
 * Use the tcp output of a custom GStreamer framework plugin to receive
 * telemetry based on video
 */

#include "video_ardrone2.h"
#include "tcp_socket.h"

#include <stdio.h>
#include "video_message_structs_sky.h"
#include "subsystems/gps/gps_ardrone2.h"
#include "subsystems/imu/imu_ardrone2_raw.h"


#include "state.h" // for altitude
#include "math/pprz_algebra_int.h"


#include <time.h>

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif
#include "messages.h"
#include "subsystems/datalink/downlink.h"

//#include "subsystems/radio_control.h"
#include "boards/ardrone/navdata.h"


struct VideoARDrone video_impl;
struct gst2ppz_message_struct gst2ppz;
struct ppz2gst_message_struct ppz2gst;


void video_init(void) {


}

//TODO: rename to video process? Receive does not cover full contents, as also updated alt. values are send to gst framework
void video_receive(void) {




  //read the data from the video tcp socket

  if (Read_msg_socket((char *) &gst2ppz,sizeof(gst2ppz))>=0) {
    video_impl.counter = gst2ppz.counter;

    //received new optical flow output:
    //int roll = gst2ppz.optic_flow_x;
    //int pitch = gst2ppz.optic_flow_y;

    //printf("Optic flow: %d, %d\n", roll,pitch);

    //DOWNLINK_SEND_VIDEO_TELEMETRY( DefaultChannel, DefaultDevice, &gst2ppz.blob_x1, &gst2ppz.blob_y1,&gst2ppz.blob_x2, &gst2ppz.blob_y2,&gst2ppz.blob_x3, &gst2ppz.blob_y3,&gst2ppz.blob_x4, &gst2ppz.blob_y4);




  }

  struct Int32Eulers* att = stateGetNedToBodyEulers_i();
  ppz2gst.counter++;
  ppz2gst.ID = 0x0003;
  ppz2gst.roll = att->phi;
  ppz2gst.pitch = att->theta;
  //ppz2gst.alt = navdata_getHeight();
  Write_msg_socket((char *) &ppz2gst,sizeof(ppz2gst));

  //printf("Roll: %d, Pitch: %d, height: %d\n",att->phi,att->theta,alt);

}




void video_start(void)
{

  //init and start the GST framework
  //for now this is being done by the makefile.omap from ppz center upload button
  //the following code does not work properly:
  //	int status = system("/data/video/kevin/initvideoall.sh");
  //as it waits until script is done (which never happens)
  //-> no init is needed, framework is started automatically

  //init the socket
  initSocket();


}

void video_stop(void)
{
  printf( "Closing video socket %d", closeSocket());
}



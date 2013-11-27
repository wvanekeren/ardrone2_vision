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


#include "obstacle_avoid.h"
#include "tcp_socket.h"

#include <stdio.h>

#include "../../pprz_gst_plugins/ObstacleAvoidSkySegmentation/video_message_structs.h"

#include "subsystems/gps/gps_ardrone2.h"
#include "subsystems/imu/imu_ardrone2_raw.h"


#include "state.h" // for altitude
#include "math/pprz_algebra_int.h"


//#include <time.h>

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

#define VISION_DATA_SIZE 20

uint8_t vision_data[VISION_DATA_SIZE] = { 10, 30, 20, 40,
            30, 50, 40, 60,
            50, 70, 60, 80,
            50, 70, 60, 80,
            50, 70, 60, 80,
};


void video_receive(void) {
/*
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
  Write_msg_socket((char *) &ppz2gst,sizeof(ppz2gst));
*/


  static uint8_t nr = 0;
  vision_data[nr] ++;
  nr ++;
  if (nr >= VISION_DATA_SIZE)
    nr = 0;

  RunOnceEvery(10,DOWNLINK_SEND_PAYLOAD(DefaultChannel, DefaultDevice, VISION_DATA_SIZE, vision_data));

}


void video_start(void)
{
  // Start GST plugiun
  // Normal video 320
  //system("/opt/arm/gst/bin/gst-launch v4l2src device=/dev/video1 ! videorate ! 'video/x-raw-yuv,framerate=15/1' ! videoscale ! video/x-raw-yuv, width=320, height=240 !dspmp4venc ! rtpmp4vpay config-interval=2 ! udpsink host=192.168.1.255 port=5000 &");

  // Sky Segment 160
  system("/opt/arm/gst/bin/gst-launch v4l2src device=/dev/video1 ! videorate ! 'video/x-raw-yuv,framerate=15/1' ! videoscale ! video/x-raw-yuv, width=160, height=120 ! obstacleavoidskysegmentation adjust_factor=5 verbose=0 tcp_port=2000  ! dspmp4venc ! rtpmp4vpay config-interval=2 ! udpsink host=192.168.1.255 port=5000 &");

  // Sky Segment 160 DSP 320
  //system("/opt/arm/gst/bin/gst-launch v4l2src device=/dev/video1 ! videorate ! 'video/x-raw-yuv,framerate=15/1' ! videoscale ! video/x-raw-yuv, width=160, height=120 ! obstacleavoidskysegmentation adjust_factor=5 verbose=0 tcp_port=2000 ! videoscale ! video/x-raw-yuv, width=320, height=240  ! dspmp4venc ! rtpmp4vpay config-interval=2 ! udpsink host=192.168.1.255 port=5000 &");

  // Start TCP Server
  //initSocket();
  //printf( "Opening gst<->pprz socket %d", initSocket());
}

void video_stop(void)
{
  // Stop TCP Server
  //printf( "Closing gst<->pprz socket %d", closeSocket());

  // Stop GST-Plugin
  system("kill -9 `pidof gst-launch-0.10` &");
}



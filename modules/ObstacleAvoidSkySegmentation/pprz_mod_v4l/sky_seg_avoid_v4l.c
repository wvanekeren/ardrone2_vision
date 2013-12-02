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


// Own header
#include "obstacle_avoid.h"

// Vision Result
#include "../../pprz_gst_plugins/ObstacleAvoidSkySegmentation/video_message_structs.h"

// Navigate Based On Vision
#include "avoid_navigation.h"

// Paparazzi State: Attitude -> Vision
#include "state.h" // for attitude

// Send Images to ground
#include "../../gst_plugin_framework/socket.h"


#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif
#include "messages.h"
#include "subsystems/datalink/downlink.h"

#include "boards/ardrone/navdata.h"


struct UdpSocket *sock;
struct gst2ppz_message_struct gst2ppz;
struct ppz2gst_message_struct ppz2gst;
int obstacle_avoid_adjust_factor;


void video_init(void) {
  // Give unique ID's to messages TODO: check that received messages are correct (not from an incompatable gst plugin)
  ppz2gst.ID = 0x0003;
  gst2ppz.ID = 0x0004;
  obstacle_avoid_adjust_factor = 5;

  // Open UDP socket
  sock = udp_socket("192.168.1.1", 2001, 2000, FMS_UNICAST);

  // Navigation Code
  init_avoid_navigation();
}


void video_receive(void) {

  // Send Attitude To GST Module
  struct Int32Eulers* att = stateGetNedToBodyEulers_i();
  ppz2gst.counter++; // 512 Hz
  ppz2gst.roll = att->phi;
  ppz2gst.pitch = att->theta;
  ppz2gst.adjust_factor = obstacle_avoid_adjust_factor;


  // Read Latest GST Module Results
  int ret = udp_read(sock, (unsigned char *) &gst2ppz, sizeof(gst2ppz));
  if (ret >= sizeof(gst2ppz))
  {
    run_avoid_navigation_onvision();

    // Send ALL vision data to the ground
    DOWNLINK_SEND_PAYLOAD(DefaultChannel, DefaultDevice, N_BINS, gst2ppz.obstacle_bins);
  }
  else
  {
    // Play annimation
    static uint8_t nr = 0;
    gst2ppz.obstacle_bins[nr] ++;
    nr ++;
    if (nr >= N_BINS)
      nr = 0;
  }
}


void video_start(void)
{
}

void video_stop(void)
{
}




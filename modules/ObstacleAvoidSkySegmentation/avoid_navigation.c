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

// Own Header
#include "avoid_navigation.h"

// Paparazzi Data
#include "state.h"

// Vision Data
#include "../../pprz_gst_plugins/ObstacleAvoidSkySegmentation/video_message_structs.h"

// Interact with navigation
#include "navigation.h"

// Know waypoint numbers and blocks
#include "generated/flight_plan.h"

struct AvoidNavigationStruct avoid_navigation_data;

void init_avoid_navigation()
{
  avoid_navigation_data.mode = 0;
}

void run_avoid_navigation_climb_until_clear(void);
void run_avoid_navigation_move_target_waypoint(void);

void run_avoid_navigation_onvision(void)
{
  switch (avoid_navigation_data.mode)
  {
  case 1:     // climb until clear
    run_avoid_navigation_climb_until_clear();
    break;
  default:    // do nothing
    break;
  }

}

/////////////////////////////////////////////////////////////////////////////////////////
//
//  HELPER FUNCTIONS



static uint8_t average_bin(void)
{
  uint16_t avg = 0;
  for (int i=0; i < N_BINS; i++)
    avg += gst2ppz.obstacle_bins[i];
  avg /= N_BINS;
  return avg;
}

static uint8_t all_bins_less_than(uint8_t thres)
{
  for (int i=0; i < N_BINS; i++)
    if (gst2ppz.obstacle_bins[i] >= thres)
      return FALSE;
  return TRUE;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
//  AVOID FUNCTIONS

void run_avoid_navigation_climb_until_clear(void)
{
  if (all_bins_less_than(10))
  {
    // Stop climbing
    avoid_navigation_data.mode = 0;
  }
  else
  {
    // On each video frame

    // nav_heading = atan(vy,vx) // INT32_ANGLE_FRAC
    navigation_SetFlightAltitude(flight_altitude + 0.1f);
  }
}

void run_avoid_navigation_move_target_waypoint(void)
{
  uint8_t avg = average_bin();
  if (avg > 10)
  {
    // There is danger!!
  }
}





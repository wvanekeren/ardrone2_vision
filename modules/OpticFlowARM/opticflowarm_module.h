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

/**
 * @file VIEW_VIDEO.h
 */

#ifndef VIEW_VIDEO_H
#define VIEW_VIDEO_H

// Settings
extern unsigned int OF_P_HOVER;
extern unsigned int OF_I_HOVER;


// Module functions
extern void opticflow_module_run(void);
extern void opticflow_module_start(void);
extern void opticflow_module_stop(void);
void *computervision_thread_main(void* data);

void start_timer(void);
long end_timer(void);
void start_timer_rates(void);
long end_timer_rates(void);


#endif /* VIEW_VIDEO_H */

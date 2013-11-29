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


#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>

#include "video.h"
#include "jpeg.h"

#include "../gst_plugin_framework/socket.h"

#define DOWNSIZE_FACTOR   2

void resize_uyuv(struct img_struct* input, struct img_struct* output, int downsample)
{
  uint8_t *source = input->buf;
  uint8_t *dest = output->buf;

  int pixelskip = downsample-1;
  for (int y=0;y<output->h;y++)
  {
    for (int x=0;x<output->w;x+=2)
    {
      // YUYV
      *dest++ = *source++; // U
      *dest++ = *source++; // Y
      // now skip 3 pixels
      source+=(pixelskip+1)*2;
      *dest++ = *source++; // U
      *dest++ = *source++; // V
      source+=(pixelskip-1)*2;
    }
    // skip 3 rows
    source += pixelskip * input->w * 2;
  }
}


int main(int argc,char ** argv)
{
  struct UdpSocket* sock;
  long len;
  printf("Starting video test program!\n");


  // Video Input
  struct vid_struct vid;
  vid.device = (char*)"/dev/video1";
  vid.w=1280;
  vid.h=720;
  len = 1280*720*2; // width * height * nbytes_perpixel
  vid.n_buffers = 4;
  if (video_Init(&vid)<0) {
    printf("Error initialising video\n");
    return 0;
  }

  // Video Grabbing
  struct img_struct* img_new = video_CreateImage(&vid);

  // Video Resizing
  struct img_struct small;
  small.w = vid.w / DOWNSIZE_FACTOR;
  small.h = vid.h / DOWNSIZE_FACTOR;
  small.buf = (uint8_t*)malloc(small.w*small.h*2);

  // Video Compression
  uint8_t* jpegbuf = (uint8_t*)malloc(vid.h*vid.w*2);

  // Network Transmit
  //#define FMS_UNICAST 0
  //#define FMS_BROADCAST 1
  sock = udp_socket("192.168.1.2", 5000, 5001, FMS_UNICAST);

  while (1) {

    //aquire image
    printf("Aquiring an image ...\n");
    video_GrabImage(&vid, img_new);


    // Resize: device by 4
    resize_uyuv(img_new, &small, DOWNSIZE_FACTOR);

    // JPEG encode the image:
    uint32_t quality_factor = 3; // quality factor from 1 (high quality) to 8 (low quality)
    uint32_t image_format = FOUR_TWO_TWO;  // format (in jpeg.h)
    uint8_t* end = encode_image (small.buf, jpegbuf+10, quality_factor, image_format, small.w, small.h);
    uint32_t size = end-jpegbuf-10;
    printf("Sending an image ...%u\n",size);

    //send image
    // SVS Surveyor Jpeg UDP format
    uint8_t* p = (uint8_t*) & size;
    jpegbuf[0]='#';
    jpegbuf[1]='#';
    jpegbuf[2]='I';
    jpegbuf[3]='M';
    jpegbuf[4]='J';
    jpegbuf[5]='7'; // 1=(40,30) 2=(128,96) 3=(160,120) 5=(320,240) 7=(640,480) 9=(1280,1024);
    jpegbuf[6]=p[0];
    jpegbuf[7]=p[1];
    jpegbuf[8]=p[2];
    jpegbuf[9]=0x00;
    udp_write(sock, (char*) jpegbuf, size+10);

  }

  video_Close(&vid);

  return 0;
}

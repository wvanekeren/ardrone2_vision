#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include "./video/video.h"
#include "socket.h"

int main(int argc,char ** argv)
{
	long len;
	printf("Starting video test program!\n");
	

	struct vid_struct vid;
	vid.device = (char*)"/dev/video1";
	vid.w=1280;
	vid.h=720;
	len = 1280*720 *2; // width * height * nbytes_perpixel
	vid.n_buffers = 4;
	if (video_Init(&vid)<0) {
		printf("Error initialising video\n");
		return 0;
	}

	struct img_struct* img_new = video_CreateImage(&vid);
	
	
	if (!initSocket()) { 
		printf("Error initialising connection\n");
		return 0;	
  }

    while (1) {
		
		//aquire image
		printf("Aquiring an image ...\n");
		video_GrabImage(&vid, img_new);
		
		//send image	
		printf("Sending an image ...%ld\n",len);
		Writeline_socket(img_new, len);
		
		sleep(1);
		
    }

	video_Close(&vid);
	
    return 0;
}
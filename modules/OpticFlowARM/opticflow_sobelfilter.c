/*
 * Author: Maxime
 * Edited and improved by Wim, November 2014
 */

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "image.h"
#include "opticflow_sobelfilter.h"
#include "opticflow_module.h"

#define LONG_MAX	2147483647
#define GNOISE 		15 // noise gradient (needed in ApplySobelfilter5)

// imgIn is in U-Y-V-Y format (exactly that order)
// http://en.wikipedia.org/wiki/YUV
unsigned int ApplySobelFilter2(struct img_struct* imgIn, unsigned int *profileX, unsigned int *profileY, unsigned int *threshold)
// http://en.wikipedia.org/wiki/Sobel_operator
{
    int nbPointsDetected = 0;
    int nbPixelsTested = 0;
    int percentDetected;

	int i,j;
	int Gx,Gy;
	long Gtot;

	unsigned short widthStep;
	unsigned char* imageData;
		
	widthStep = 2*WIDTH;
	imageData = imgIn->buf;
	
	unsigned char *img = imageData+1;

	unsigned char *in =(img + widthStep); 		// "current" line of the image
	unsigned char *inPrevLine = img; 		// previous line of the image
	unsigned char *inNextLine = (img + 2*widthStep);// next line of the image

	unsigned int *histSobelX = profileX+1; // initialise the position of the histogram X and Y
	unsigned int *histSobelY = profileY+1;
	
	memset(profileX,0,WIDTH*sizeof(unsigned int)); // initialise profileX with 0
	memset(profileY,0,HEIGHT*sizeof(unsigned int));

	// for all lines in the image (start with 1)
	for(j=1;j<HEIGHT-1;j++) //for(j=1;j<height-1;j++) 
	{
		histSobelX = profileX+1; // initialise the position of the histogram X
		
		// over the entire width of the image
		for(i=1;i<WIDTH-1;i++)
		{
		    
		    // convolution filter
		    Gx = -*(inPrevLine);    
		    Gy = -*(inPrevLine+=2);
		    Gy -= 2*(*(inPrevLine+=2));
		    Gx += *inPrevLine;
		    Gy -= *(inPrevLine-=2);
		    
		    Gx -= 2*(*(in+=2));
		    in+=2;
		    Gx += 2*(*(in-=2));
		    
		    Gx -= *inNextLine;
		    Gy += *(inNextLine+=2);
		    Gy += 2*(*(inNextLine+=2));
		    Gx += *inNextLine;
		    Gy += *(inNextLine-=2);
		    
		    Gtot = Gx*Gx + Gy*Gy;
		    // feature detected?
		    if(Gtot>*threshold)
		    {
			nbPointsDetected++;
			// add a 1 on the current x and y location of the histogram
			*histSobelX += 1; 
			*histSobelY += 1;
		    }
		    nbPixelsTested++;
		    
		    histSobelX++;
		 
		}
		in += 4;
		inPrevLine += 4;
		inNextLine += 4;
		histSobelY++;
		
	}
	// result is 2 histograms with over its elements a counting of the features

    
    // threshold control
    percentDetected = (100*nbPointsDetected)/nbPixelsTested;
    
    // too much detected
    if(percentDetected>20)
    {
    	if(*threshold<100)
    		*threshold+=10;
        else 
		*threshold+=100;
    }
    
    // too less detected
    else if(percentDetected<10)
    {
    	if(*threshold>100)
	        *threshold-=100;
	    else if(*threshold>10)
	    	*threshold -= 10;
    }
    return percentDetected;
}



unsigned int ApplySobelFilter4a(struct img_struct* imgIn, unsigned int *profileX, unsigned int *profileY, unsigned int *threshold)
// http://en.wikipedia.org/wiki/Sobel_operator
// Variation on ApplySobelFilter2: corrected convolution filter (convolution matrix turned out to be wrong at ApplySobelFilter2)
{
    int nbPointsDetected = 0;
    int nbPixelsTested = 0;
    int percentDetected;

    int i,j;
    int Gx,Gy;
    long Gtot;


    unsigned short widthStep;
    unsigned char* imageData;
	    
    widthStep = 2*WIDTH;
    imageData = imgIn->buf;
    
    unsigned char *img = imageData+1;

    unsigned char *in =(img + widthStep); 		// "current" line of the image
    unsigned char *inPrevLine = img; 		// previous line of the image
    unsigned char *inNextLine = (img + 2*widthStep);// next line of the image

    unsigned int *histSobelX = profileX+1; // initialise the position of the histogram X and Y
    unsigned int *histSobelY = profileY+1;
    
    memset(profileX,0,WIDTH*sizeof(unsigned int)); // initialise profileX with 0
    memset(profileY,0,HEIGHT*sizeof(unsigned int));

    // for all lines in the image (start with 1)
    for(j=1;j<HEIGHT-1;j++) //for(j=1;j<height-1;j++) 
    {
	histSobelX = profileX+1; // initialise the position of the histogram X
	
	// over the entire width of the image
	for(i=1;i<WIDTH-1;i++)
	{
	    
	    Gx = 0;
	    Gy = 0;
	    // convolution filter
	    // inPrevline
	    Gx = Gx - 1 * (*inPrevLine);
	    Gy = Gy - 1 * (*inPrevLine);
	    
	    // inPrevLine +1
	    inPrevLine+=2;
	    Gx = Gx + 0 * (*inPrevLine);
	    Gy = Gy - 2 * (*inPrevLine);  
	    
	    // inPrevLine +2
	    inPrevLine+=2;
	    Gx = Gx + 1 * (*inPrevLine);
	    Gy = Gy - 1 * (*inPrevLine);  
	    
	    // in
	    Gx = Gx - 2 * (*in);
	    Gy = Gy + 0 * (*in);
	    
	    // in +1 
	    in+=2;
	    Gx = Gx + 0 * (*in);
	    Gy = Gy + 0 * (*in);  
	    
	    // in + 2
	    in+=2;
	    Gx = Gx + 2 * (*in);
	    Gy = Gy + 0 * (*in);  
	    
	    
	    // in Nextline
	    Gx = Gx - 1 * (*inNextLine);
	    Gy = Gy + 1 * (*inNextLine);
	    
	    // in Nextline +1
	    inNextLine+=2;
	    Gx = Gx + 0 * (*inNextLine);
	    Gy = Gy + 2 * (*inNextLine);
	      
	    // in Nextline +2
	    inNextLine+=2;
	    Gx = Gx + 1 * (*inNextLine);
	    Gy = Gy + 1 * (*inNextLine);
	    
	    Gtot = Gx*Gx + Gy*Gy;
	    
	    
	    // feature detected?
	    if(Gtot>*threshold)
	    {
		nbPointsDetected++;
		// add a 1 on the current x and y location of the histogram
		*histSobelX += 1; 
		*histSobelY += 1;
	    }

	    nbPixelsTested++;
	    
	    histSobelX++; // move to next entry of the x histogram
	    
	    // Move to next pixel
	    inPrevLine-=2;
	    in-=2;
	    inNextLine-=2;
	  
	}
	in += 4;
	inPrevLine += 4;
	inNextLine += 4;
	histSobelY++; // move to the next entry of the x histogram
	    
    }
    // result is 2 histograms with over its elements a sum of the convolution
    // threshold control
    percentDetected = (100*nbPointsDetected)/nbPixelsTested;
    
    // too much detected
    if(percentDetected>20)
    {
    	if(*threshold<100)
    		*threshold+=10;
        else 
		*threshold+=100;
    }
    
    // too less detected
    else if(percentDetected<10)
    {
    	if(*threshold>100)
	        *threshold-=100;
	    else if(*threshold>10)
	    	*threshold -= 10;
    }
    return percentDetected;

}





void ApplySobelFilter5(struct img_struct* imgIn, unsigned int *profileX, unsigned int *profileY)
// http://en.wikipedia.org/wiki/Sobel_operator
// Variation on ApplySobelFilter4:
// - Gtot returned directly in the histogram, no threshold
// - Gtot as sqrt(Gx^2 + Gy^2) instead of Gtot = Gx^2 + Gy^2
// - no return variable
{


	int i,j;
	int Gx,Gy;
	long Gtot;
	float sqrtGtot;
	
	unsigned short widthStep;
	unsigned char* imageData;
		
	widthStep = 2*WIDTH;
	imageData = imgIn->buf;
	
	unsigned char *img = imageData+1;

	unsigned char *in =(img + widthStep); 		// "current" line of the image
	unsigned char *inPrevLine = img; 		// previous line of the image
	unsigned char *inNextLine = (img + 2*widthStep);// next line of the image
	
	unsigned int *histSobelX = profileX+1; // initialise the position of the histogram X and Y
	unsigned int *histSobelY = profileY+1;
	
	memset(profileX,0,WIDTH*sizeof(unsigned int)); // initialise profileX with 0
	memset(profileY,0,HEIGHT*sizeof(unsigned int));

	// for all lines in the image (start with 1)
	for(j=1;j<HEIGHT-1;j++) //for(j=1;j<height-1;j++) 
	{
		histSobelX = profileX+1; // initialise the position of the histogram X
		
		// over the entire width of the image
		for(i=1;i<WIDTH-1;i++)
		{
		    
		    Gx = 0;
		    Gy = 0;
		    // convolution filter
		    // inPrevline
		    Gx = Gx - 1 * (*inPrevLine);
		    Gy = Gy - 1 * (*inPrevLine);
		    
		    // inPrevLine +1
		    inPrevLine+=2;
		    Gx = Gx + 0 * (*inPrevLine);
		    Gy = Gy - 2 * (*inPrevLine);  
		    
		    // inPrevLine +2
		    inPrevLine+=2;
		    Gx = Gx + 1 * (*inPrevLine);
		    Gy = Gy - 1 * (*inPrevLine);  
		    
		    // in
		    Gx = Gx - 2 * (*in);
		    Gy = Gy + 0 * (*in);
		    
		    // in +1 
		    in+=2;
		    Gx = Gx + 0 * (*in);
		    Gy = Gy + 0 * (*in);  
		    
		    // in + 2
		    in+=2;
		    Gx = Gx + 2 * (*in);
		    Gy = Gy + 0 * (*in);  
		    
		    
		    // in Nextline
		    Gx = Gx - 1 * (*inNextLine);
		    Gy = Gy + 1 * (*inNextLine);
		    
		    // in Nextline +1
		    inNextLine+=2;
		    Gx = Gx + 0 * (*inNextLine);
		    Gy = Gy + 2 * (*inNextLine);
		      
		    // in Nextline +2
		    inNextLine+=2;
		    Gx = Gx + 1 * (*inNextLine);
		    Gy = Gy + 1 * (*inNextLine);
		    
		    Gtot = Gx*Gx + Gy*Gy;
		    
		    sqrtGtot = sqrt(Gtot);
		    
// 		    printf("i,j: %d, %d",i,j);
// 		    printf("Gtot: %d\n",Gtot);
		    
		    // add Gtot on the current x and y location of the histogram
		    *histSobelX += sqrtGtot; 
		    *histSobelY += sqrtGtot;
// 		    printf("new HistX: %d\n",*histSobelX);
// 		    printf("new HistY: %d\n",*histSobelY);
		    
		    histSobelX++; // move to next entry of the x histogram
		    
		    // Move to next pixel
		    inPrevLine-=2;
		    in-=2;
		    inNextLine-=2;
		 
		}
		in += 4;
		inPrevLine += 4;
		inNextLine += 4;
		histSobelY++; // move to the next entry of the x histogram
		
	}
	// result is 2 histograms with over its elements a sum of the convolution


// 	printf("hist y: \n");
// 	for (i=0;i<HEIGHT;i++) {
// 	printf("%d\n",*(histSobelY+i));
// 	}
// 
// 	printf("hist x: \n");
// 	for (i=0;i<WIDTH;i++) {
// 	printf("%d\n",*(histSobelX+i));
//  	}
}

void ApplySobelFilter6(struct img_struct* imgIn, unsigned int *profileX, unsigned int *profileY)
// http://en.wikipedia.org/wiki/Sobel_operator
// variation on ApplySobelFilter4/5: 
// - return Gtot instead of a thresholded value of 1 or 0;
// - also, Gtot = sqrt(Gx^2 and Gy^2) instead of Gtot = Gx^2 + Gy^2 --> heavier calculations
// - furthermore: only return Gtot if it is above the NOISE threshold (set as define GNOISE)
// - no return variable

{


	int i,j;
	int Gx,Gy;
	long Gtot;

	unsigned short widthStep;
	unsigned char* imageData;
		
	widthStep = 2*WIDTH;
	imageData = imgIn->buf;
	
	unsigned char *img = imageData+1;

	unsigned char *in =(img + widthStep); 		// "current" line of the image
	unsigned char *inPrevLine = img; 		// previous line of the image
	unsigned char *inNextLine = (img + 2*widthStep);// next line of the image

	unsigned int *histSobelX = profileX+1; // initialise the position of the histogram X and Y
	unsigned int *histSobelY = profileY+1;
	
	memset(profileX,0,WIDTH*sizeof(unsigned int)); // initialise profileX with 0
	memset(profileY,0,HEIGHT*sizeof(unsigned int));

	// for all lines in the image (start with 1)
	for(j=1;j<HEIGHT-1;j++) //for(j=1;j<height-1;j++) 
	{
		histSobelX = profileX+1; // initialise the position of the histogram X
		
		// over the entire width of the image
		for(i=1;i<WIDTH-1;i++)
		{
		    
		    Gx = 0;
		    Gy = 0;
		    // convolution filter
		    // inPrevline
		    Gx += Gx - 1 * (*inPrevLine);
		    Gy += Gy - 1 * (*inPrevLine);
		    
		    // inPrevLine +1
		    inPrevLine+=2;
		    Gx = Gx + 0 * (*inPrevLine);
		    Gy = Gy - 2 * (*inPrevLine);  
		    
		    // inPrevLine +2
		    inPrevLine+=2;
		    Gx = Gx + 1 * (*inPrevLine);
		    Gy = Gy - 1 * (*inPrevLine);  
		    
		    // in
		    Gx = Gx - 2 * (*in);
		    Gy = Gy + 0 * (*in);
		    
		    // in +1 
		    in+=2;
		    Gx = Gx + 0 * (*in);
		    Gy = Gy + 0 * (*in);  
		    
		    // in + 2
		    in+=2;
		    Gx = Gx + 2 * (*in);
		    Gy = Gy + 0 * (*in);  
		    
		    
		    // in Nextline
		    Gx = Gx - 1 * (*inNextLine);
		    Gy = Gy + 1 * (*inNextLine);
		    
		    // in Nextline +1
		    inNextLine+=2;
		    Gx = Gx + 0 * (*inNextLine);
		    Gy = Gy + 2 * (*inNextLine);
		      
		    // in Nextline +2
		    inNextLine+=2;
		    Gx = Gx + 1 * (*inNextLine);
		    Gy = Gy + 1 * (*inNextLine);
		    
		    Gtot = Gx*Gx + Gy*Gy;
		    Gtot = sqrt(Gtot);
		    

		    if(Gtot < GNOISE) {
		      Gtot=0;
		    }
		    
		    // add gradient tot histograms
		    *histSobelX += Gtot; 
		    *histSobelY += Gtot;
	    
		    histSobelX++; // move to next entry of the x histogram
		    
		    // Move to next pixel
		    inPrevLine-=2;
		    in-=2;
		    inNextLine-=2;
		 
		}
		in += 4;
		inPrevLine += 4;
		inNextLine += 4;
		histSobelY++; // move to the next entry of the x histogram
		
	}
	// result is 2 histograms with over its elements a sum of the convolution

}


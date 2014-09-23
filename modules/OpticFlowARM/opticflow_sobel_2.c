/*
 * Author: Maxime
 * Edited by Wim to run on the ARM
 */

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "image.h"
#include "opticflow_sobel_2.h"

#define RESOLUTION	10
#define LONG_MAX	2147483647

//imgIn is in YUYV format
unsigned int ApplySobelFilter2(struct img_struct* imgIn, unsigned int *profileX, unsigned int *profileY, unsigned int *threshold)
{
    int nbPointsDetected = 0;
    int nbPixelsTested = 0;
    int percentDetected;

	int i,j;
	int Gx,Gy;
	long Gtot;

	unsigned short width;
	unsigned short height;
	unsigned short widthStep;
	unsigned char* imageData;
		
	width = 320;//*((unsigned int *)(imgIn->buf)); (just guessed width must be image width and not something else but it works...)
	height = 240;//*((unsigned int *)(imgIn->buf+4));
	widthStep = 2*width;
	imageData = imgIn->buf;
	
	unsigned char *img = imageData+1;

	unsigned char *in =(img + widthStep); 		// "current" line of the image
	unsigned char *inPrevLine = img; 		// previous line of the image
	unsigned char *inNextLine = (img + 2*widthStep);// next line of the image

	unsigned int *histSobelX = profileX+1; // initialise the position of the histogram X and Y
	unsigned int *histSobelY = profileY+1;
	
	memset(profileX,0,width*sizeof(unsigned int)); // initialise profileX with 0
	memset(profileY,0,height*sizeof(unsigned int));

	// for all lines in the image (start with 1)
	for(j=1;j<height-1;j++) //for(j=1;j<height-1;j++) 
	{
		histSobelX = profileX+1; // initialise the position of the histogram X
		
		// over the entire width of the image
		for(i=1;i<width-1;i++)
		{
		    
		    // some weird algorithm calculates the correspondence with neighbouring pixels
		    
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

char TrackZone_2(unsigned int *act, unsigned int *prev, unsigned int start, unsigned int end, signed int *depl)
{
	//Déplacement max:+/-10 pixels
	int i,j;
	signed int E=0,sigma=0;
	long min = LONG_MAX;
	int depl_temp;
	long erreur[2*RESOLUTION+1];
	for(i=-RESOLUTION;i<=RESOLUTION;i++)
	{
		erreur[i+RESOLUTION] = 0;
		for(j=start;j<=end;j++)
		{
			erreur[i+RESOLUTION] += abs(act[j+i] - prev[j]);
		}
	}

	//Valeurs statistiques des erreurs
	for(i=0;i<2*RESOLUTION+1;i++)
	{
		E += erreur[i];
	}
	E/=2*RESOLUTION+1;

	for(i=0;i<21;i++)
	{
		sigma += (erreur[i]-E)*(erreur[i]-E);
	}
	sigma /= 2*RESOLUTION+1;
	sigma = sqrt(sigma);

	for(i=0;i<2*RESOLUTION+1;i++)
	{
		if(erreur[i]<min)
		{
			min = erreur[i];
			depl_temp = i-10;
		}
	}
	if(min>(E-sigma))
	{
		return -1;
	}
	*depl = depl_temp;
	return 0;
}

int CalcOF3_2(unsigned int *act, unsigned int *prev, unsigned int length)
{
	int T=0,T_temp;
	//Calcul de la moyenne de la norme des gradients
	int i,j;
	float Gm=0;
	unsigned int start,end,zone=0;
	signed int nbZones = 0;
	float borne;
	int *buff;
	
	for(i=1;i<length;i++)
	{
		Gm += abs(prev[i]-prev[i-1]);
	}
	Gm /= (length-1);

	//Recherche des zones
	//Pour la sauvegarde des valeurs, chaque zone à une taille supérieur ou égale à 10
	buff = (int*)(malloc((length/RESOLUTION)*sizeof(int)));
	borne = Gm;
	for(i=2*RESOLUTION;i<length-2*RESOLUTION-1;i++)
	{
		if(abs(prev[i]-prev[i-1])>borne && zone==0)
		{
			start = i;
			zone = 1;
		}
		else if(zone)
		{
			if((abs(prev[i]-prev[i-1])<borne && i-start>=10) || i-start>20 )
			{
				end = i;
				zone = 0;
				if(TrackZone_2(act,prev,start,end,&T_temp)!=-1)
				{
					buff[nbZones++] = T_temp;
				}
			}
		}
	}
	memcpy(prev,act,length*sizeof(unsigned int));
	if(nbZones == 0){ free(buff); return -255;}

	//Filtrage médian pour avoir le résultat
	for(i=0;i<nbZones;i++)
	{
		for(j=i;j<nbZones-1;j++)
		{
			if(buff[i]>buff[j+1])
			{
				buff[i] ^= buff[j+1];
				buff[j+1] ^= buff[i];
				buff[i] ^= buff[j+1];
			}
		}
	}

	T =  buff[nbZones/2];	
	free(buff);
	return T;
}

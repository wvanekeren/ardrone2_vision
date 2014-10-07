/*
 * Author: Maxime
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#include "calcFlowXYZ.h"


unsigned int histTempX[320];
//extern unsigned int *histTempX;
unsigned int histTempY[240];
//extern unsigned int *histTempY;

int prevTx = 0;
int prevTy = 0;
int prevTz = 0;

unsigned int errorHists(unsigned int *hist1, unsigned int *hist2, unsigned int start, unsigned int end)
{
	int j;
	unsigned int erreur = 0;
	for(j=start;j<end;j++)
	{
		erreur += (hist1[j]-hist2[j])*(hist1[j]-hist2[j]);
	}
	erreur *= 512;
	return erreur/(end-start);
}

unsigned int calcFlowXYZ(int *Tx_min, int *Ty_min, int *Tz_min, unsigned int *histX, unsigned int *prevHistX, unsigned int *histY, unsigned int *prevHistY, unsigned int *curskip, unsigned int *framesskip)
{
// x,y in image plane!! image plane is rotated 90 degrees with respect to body frame: x_body = y_image, y_body =  -x_image
// in this function, the flow Tx,Ty,Tz is in percent, so Tx = 100*flow;  
  
    int Tx=0,Ty=0,Tz=0;
    //int Tz_inf,Tz_sup; // unused
    int TyPrev=-255;
    short start=-1,end=0;
    unsigned int erreur=0,erreurX=0,erreurY=0;
    unsigned int min=99999999; // 99 million
    short first=1;
    int idx_shifted;
    short i;
    
    int width = 320;
    int halfwidth = width/2;
    int height = 240;
    int halfheight = height/2;
    
    //short X,Y; // unused
	#define FACT	100
	/*
	Tz_inf = prevTz-10;
	if(Tz_inf<-30)
	{
		Tz_inf = -30;
	}
	Tz_sup = prevTz+10;
	if(Tz_sup>30)
	{
		Tz_sup = 30;
	}*/
	

    // try for all possible Tx,Ty,Tz around Tz=0,Tx=prevTx,Ty=prevTy
    for(Tz=0;Tz<=0;Tz++)
    {
    	// for Ty = 
	for(Ty=0-1200;Ty<=0+1200;Ty+=100)
    	{
    		for(Tx=0-1200;Tx<=0+1200;Tx+=100)
    		{
				// set histTempX to zero
				start = -1;
				memset(histTempX,0,320*sizeof(unsigned int));
				for(i=-halfwidth;i<halfwidth;i++)
				{
					//idxGros = i*(100+Tz)-Tx;				  
					//idx = idxGros/100 + halfwidth;
					idx_shifted = halfwidth+i*(1+Tz/100)-Tx/100;
					if(idx_shifted>=0 && idx_shifted<320)
					{
						if(start==-1)
							start = idx_shifted;
						end = idx_shifted;
						
						// on Temporary Hist X, fill it with the correct values of previous Hist X on the correct locations (x-shift in px)
						histTempX[halfwidth+i] = prevHistX[idx_shifted];
					}
				}
				
				// Calculate, with the given Tx,Ty,Tz, the error with the current Hist
				erreurX = errorHists(histTempX,histX,start,end+1);

    			if(TyPrev != Ty)
    			{
					start = -1;
					memset(histTempY,0,240*sizeof(unsigned int));
					for(i=-halfheight;i<halfheight;i++)
					{
						//idxGros = i*(100+Tz)-Ty;
						//idx = idxGros/100 + 120;
						idx_shifted = halfheight+i*(1+Tz/100)-Ty/100;
						if(idx_shifted>=0 && idx_shifted<height)
						{
							if(start==-1)
								start = idx_shifted;
							end = idx_shifted;
							histTempY[halfheight+i] = prevHistY[idx_shifted];
						}
					}
					erreurY = errorHists(histTempY,histY,start,end+1);
					TyPrev = Ty;
    			}

				erreur = erreurX + erreurY;

				if(first)
				{
					min = erreur;
					*Tx_min = Tx;
					*Ty_min = Ty;
					*Tz_min = Tz;
					first = 0;
				}
				else if(erreur<min)
				{
					min = erreur;
					*Tx_min = Tx;
					*Ty_min = Ty;
					*Tz_min = Tz;
				}
    		}
    	}
    }
    
    // skip?
    if(*Tz_min==0 && (*curskip)++<*framesskip) // skip=0 is tested, the normal 60 FPS is reached.
    {
    	*Tx_min = prevTx;
    	*Ty_min = prevTy;
    	*Tz_min = prevTz;
    }
    
    // or use calculated values
    else
    {
		prevTx = *Tx_min;
		prevTy = *Ty_min;
		prevTz = *Tz_min;
		*curskip=0;
		memcpy(prevHistX,histX,320*sizeof(unsigned int));
		memcpy(prevHistY,histY,240*sizeof(unsigned int));
		

		
    }
    
    

    
    return min;	
}


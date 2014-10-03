/*
 * Author: Maxime
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <limits.h>

#include "calcFlowXYZ.h"

#define width 	320
#define height 	240
unsigned int histTempX[width];
unsigned int histTempY[height];

unsigned int errorHists(unsigned int *hist1, unsigned int *hist2, unsigned int idx_start, unsigned int idx_end)
{
	int j;
	unsigned int erreur = 0;
	for(j=idx_start;j<idx_end;j++)
	{
		erreur += (hist1[j]-hist2[j])*(hist1[j]-hist2[j]);
	}
	erreur *= 512;
	return erreur/(idx_end-idx_start);
}

unsigned int calcFlowXYZ(int *Tx_result, int *Ty_result, int *Tz_result, unsigned int *histX, unsigned int *prevHistX, unsigned int *histY, unsigned int *prevHistY, unsigned int *skip)
{
// x,y in image plane!! image plane is rotated 90 degrees with respect to body frame: x_body = y_image, y_body =  -x_image
// in this function, the flow Tx,Ty,Tz is in percent, so Tx = 100*flow;  

// skip is not used in this version  
    
    int i;
  
    int halfwidth 	 = width/2;
    int halfheight 	 = height/2;
  
    short first 	 = 0;
    int idx_start 	 = -1;
    int idx_end 	 = -1;
    unsigned int min_err = INT_MAX;
    
    int shift		= -1;
    int idx_normal	= -1;
    int idx_shifted	= -1;
    
    int lastTy = INT_MAX;
    
    memset(histTempX,0,width*sizeof(unsigned int));
    memset(histTempY,0,height*sizeof(unsigned int));
    
    int Tx=0;
    int Ty=0;
    int Tz=0;
    
    unsigned int errorX = INT_MAX;
    unsigned int errorY = INT_MAX;
    unsigned int errorTotal = INT_MAX;
    
    
    for (Tz=0;Tz<=0;Tz++) {
      for (Ty=-1000;Ty<=1000;Ty+=100) {
	for (Tx=-1000;Tx<=1000;Tx+=100) {
	// for every combination of Tz,Ty,Tx, do:
	  
	  // calculate error x:
	  
	  // reset temp variables
	  idx_start=-1;
	  memset(histTempX,0,width*sizeof(unsigned int));
	  
	  for (i=-halfwidth;i<halfwidth;i++) {
	    shift = i*Tz/100 - Tx/100;
	    
	    idx_normal 		= i+halfwidth;
	    idx_shifted 	= idx_normal+shift;
	    
	    if (idx_shifted >= 0 && idx_shifted < width) {
	      if (idx_start==-1)
		idx_start = idx_shifted;
	      idx_end = idx_shifted;
	      
	      histTempX[idx_normal] = prevHistX[idx_shifted];
	    }
	  }
	  
	  
	  errorX = errorHists(histTempX,histX,idx_start,idx_end+1);
	  
	  // calculate error y:
	  if (Ty!=lastTy) { // within the Tx-loop, we don't have to recalculate errorY
	  
	  // reset temp variables
	  idx_start=-1;
	  memset(histTempY,0,height*sizeof(unsigned int));
	  
	  for (i=-halfheight;i<halfheight;i++) {
	    shift = i*Tz/100 - Ty/100;
	    
	    idx_normal 		= i+halfheight;
	    idx_shifted 	= idx_normal+shift;
	    
	    if (idx_shifted >= 0 && idx_shifted < width) {
	      if (idx_start==-1)
		idx_start = idx_shifted;
	      idx_end = idx_shifted;
	      
	      histTempY[idx_normal] = prevHistY[idx_shifted];
	    }
	  }
	  
	  errorY = errorHists(histTempY,histY,idx_start,idx_end+1);
	  
	  lastTy=Ty;
	  }
	  
	  
	  // total error:
	  errorTotal = errorY + errorX;
	  
	  // is it a new minimum?
	  if (first) {
	      min_err = errorTotal;
	      *Tx_result  = Tx;
	      *Ty_result  = Ty;
	      *Tz_result  = Tz;
	      first   = 0;
	  }
	  else if (errorTotal < min_err) {
	      min_err = errorTotal;
	      *Tx_result  = Tx;
	      *Ty_result  = Ty;
	      *Tz_result  = Tz;
	  }
	  
	 
	  
	} // end for every Tx,Ty,Tz
      }
    }
    
    // set Hist to prevHist for next calculation
    memcpy(prevHistX,histX,320*sizeof(unsigned int));
    memcpy(prevHistY,histY,240*sizeof(unsigned int));	  
	  
    return min_err;    
      
}


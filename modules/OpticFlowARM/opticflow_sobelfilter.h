/*
 * Author: Maxime
 * Edited by Wim to run on the ARM
 * See opticflow_sobelfilter.c for more info/comments
 */

#ifndef _OPTICFLOW_SOBEL_H_
#define _OPTICFLOW_SOBEL_H_
void OFCreateBuffs(unsigned int startAddress);
void ApplySobelFilter(struct img_struct* imgIn, unsigned int *profileX, unsigned int *profileY, unsigned int *threshold);

// old convolution method (wrong convolution matrix):
unsigned int ApplySobelFilter2(struct img_struct* imgIn, unsigned int *profileX, unsigned int *profileY, unsigned int *threshold);

// new convolution method (corrected convolution matrix):
// 1: old threshold method
unsigned int ApplySobelFilter4(struct img_struct* imgIn, unsigned int *profileX, unsigned int *profileY, unsigned int *threshold);
// 2: no threshold, just summing up Gtot
void ApplySobelFilter5(struct img_struct* imgIn, unsigned int *profileX, unsigned int *profileY); // --> Gtot directly, and sqrt)
// 3: noise threshold for Gtot, summing up Gtot
void ApplySobelFilter6(struct img_struct* imgIn, unsigned int *profileX, unsigned int *profileY); // --> Gtot directly, sqrt, and GNOISE threshold

// other
char TrackZone_2(unsigned int *act, unsigned int *prev, unsigned int start, unsigned int end, signed int *depl);
int CalcOF3_2(unsigned int *act, unsigned int *prev, unsigned int length);
#endif

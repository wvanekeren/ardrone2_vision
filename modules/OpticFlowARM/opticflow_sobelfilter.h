/*
 * Author: Maxime
 * Edited by Wim to run on the ARM
 */

#ifndef _OPTICFLOW_SOBEL_H_
#define _OPTICFLOW_SOBEL_H_
void OFCreateBuffs(unsigned int startAddress);
void ApplySobelFilter(struct img_struct* imgIn, unsigned int *profileX, unsigned int *profileY, unsigned int *threshold);
unsigned int ApplySobelFilter2(struct img_struct* imgIn, unsigned int *profileX, unsigned int *profileY, unsigned int *threshold);
unsigned int ApplySobelFilter3(struct img_struct* imgIn, unsigned int *profileX, unsigned int *profileY);
unsigned int ApplySobelFilter4(struct img_struct* imgIn, unsigned int *profileX, unsigned int *profileY, unsigned int *threshold);
char TrackZone_2(unsigned int *act, unsigned int *prev, unsigned int start, unsigned int end, signed int *depl);
int CalcOF3_2(unsigned int *act, unsigned int *prev, unsigned int length);
#endif

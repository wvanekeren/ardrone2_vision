#ifndef SKYSEGMENTATION
#define SKYSEGMENTATION

extern unsigned int imgWidth, imgHeight;


/***
 *    \brief:   Sky Segmentation: find ground/sky pixels
 *              no_yco: No y-coordinate in decission tree
 *
 *              frame_buf = input image -> output of the classification
 *              frame_buf2 = output of the certainty of the classification
 *              adjust_factor = -3 , -2, -1, [0], 1, 2, 3  (adjust: find more or less ground)
 */

extern void segment_no_yco_AdjustTree(unsigned char *frame_buf, unsigned char *frame_buf2, int adjust_factor);




void skyseg_interface_n(unsigned char *frame_buf, unsigned char *frame_buf2, char adjust_factor, unsigned int counter, int pitch, int roll);

#endif /* SKYSEGMENTATION */

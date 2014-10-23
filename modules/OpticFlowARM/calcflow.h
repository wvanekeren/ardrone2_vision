/*
 * Author: Maxime
 */
unsigned int errorHists(unsigned int *hist1, unsigned int *hist2, unsigned int start, unsigned int end);
unsigned int calcFlow(int *Tx_min, int *Ty_min, int *Tz_min, unsigned int *histX, unsigned int *prevHistX, unsigned int *histY, unsigned int *prevHistY, unsigned int *curskip, unsigned int *framesskip, unsigned int *window);


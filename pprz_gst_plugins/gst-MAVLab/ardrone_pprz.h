
#include <gst/gst.h>

extern gint adjust_factor;
extern unsigned int imgWidth, imgHeight;
extern int mode;
extern unsigned int tcpport;

void my_plugin_init(void);
void my_plugin_run(unsigned char* img);

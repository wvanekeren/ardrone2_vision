

// Settable by pluging
extern unsigned int imgWidth, imgHeight;
extern unsigned int tcpport;
extern int adjustfactor;

// Called by plugin
void my_plugin_init(void);
void my_plugin_run(unsigned char *frame);


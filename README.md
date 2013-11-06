ardrone2_vision
===============

ARDrone2 Onboard Image Processing


 - ardrone2_gstreamer:	compiled library and includes for gstreamer on ardrone
 - gst_framework:		common files for all plugins
 - pprz_gst_plugins:	working directory

to create a new plugin, run the script create_new_plugin.py <PlugInName>. This will create a new directory and add a compiling pass-through plugin. The files are:

 - gst_PLUGINNAME_plugin.c/h -> the gstreamer interface, including settables
 - PLUGINNAME_code.c/h -> an empty project communicating with paparazzi to add your own code


ardrone2_vision
===============

ARDrone2 Onboard Image Processing

Directories:
------------


 - ardrone2_gstreamer:	compiled library and includes for gstreamer on ardrone
 - gst_framework:		common files for all plugins
 - pprz_gst_plugins:	working directory

Using:
-----

 - **git clone https://github.com/tudelft/ardrone2_vision** ./paparazzi/sw/ext/ardrone2_vision
 - **make** (will also download submodule, and makes the gstreamer plugins with your custom code)
 - **make drone** (called from ardrone2_gstreamer folder: only needed 1 time: will put the framework on your drone)
put the libPlugin.so on the drone, and start gstreamer using command as in sourcecode

 - (if first make fails at the end) **make install** (will install scratchbox2 and qemu if you didn't have them)
 - (if make is still not working) gedit ./ardrone2_gstreamer/Makefile edit the path to your ardrone crosscompiler and make install again to setup sb2

Start New Plugin:
---------------

to create a new plugin, run the script **create_new_plugin.py <PlugInName>**. This will create a new directory and add a compiling pass-through plugin. The files are:

 - gst_PLUGINNAME_plugin.c/h -> the gstreamer interface, including settables
 - PLUGINNAME_code.c/h -> an empty project communicating with paparazzi to add your own code




all:
	git submodule sync
	git submodule init
	git submodule update
	make -C ./ardrone2_gstreamer
	sb2 make -C ./gst-MAVLab clean all
	sb2 make -C ./gst-plugin_example clean all

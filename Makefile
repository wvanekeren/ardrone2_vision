

all:
	git submodule sync
	git submodule init
	git submodule update
	make -C ./ardrone2_gstreamer
	sb2 make -C ./pprz_gst_plugins/gst-MAVLab clean all
	sb2 make -C ./pprz_gst_plugins/BrightSpotDetector clean all
	sb2 make -C ./pprz_gst_plugins/ObstacleAvoidSkySegmentation clean all

clean:
	make -C ./ardrone2_gstreamer clean
	sb2 make -C ./pprz_gst_plugins/gst-MAVLab clean
	sb2 make -C ./pprz_gst_plugins/BrightSpotDetector clean
	sb2 make -C ./pprz_gst_plugins/ObstacleAvoidSkySegmentation clean

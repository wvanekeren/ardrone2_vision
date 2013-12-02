include ../../Makefile.include

CROSS_COMPILE ?=
CC := $(CROSS_COMPILE)gcc


CFLAGS := -O2 -ggdb -Wall -Wextra -Wno-unused-parameter -Wmissing-prototypes -ansi
LDFLAGS := -Wl,--no-undefined -Wl,--as-needed

GST_CFLAGS := -pthread 
GST_LIBS := -pthread -lrt 

override CFLAGS += -std=c99 -D_GNU_SOURCE -DGST_DISABLE_DEPRECATED
override CFLAGS += -mcpu=cortex-a8 -mfpu=neon -mfloat-abi=softfp -ftree-vectorize
override CFLAGS += -I$(DIR_GSTREAMER)
override CFLAGS += -I$(DIR_CV) -I$(DIR_LIB)

D = $(DESTDIR)


gst_plugin := grab.x


OBJECTS =  	testrtp.o \
		$(DIR_LIB)/v4l/video.o \
		$(DIR_LIB)/udp/socket.o \
		$(DIR_CV)/encoding/jpeg.o \
		$(DIR_CV)/encoding/rtp.o

$(gst_plugin): $(OBJECTS)


$(gst_plugin): override CFLAGS += $(GST_CFLAGS)
$(gst_plugin): override LIBS += $(GST_LIBS)

targets += $(gst_plugin)

upload:
	sb2 make -C ./ all && $(DRONE_TOOL) upload_paparazzi ./$(gst_plugin) vision

all: $(targets) $(bins)

# pretty print
ifndef V
QUIET_CC    = @echo '   CC         '$@;
QUIET_LINK  = @echo '   LINK       '$@;
QUIET_CLEAN = @echo '   CLEAN      '$@;
endif

%.so: override CFLAGS += -fPIC

%.o:: %.c $(GST_H)
	$(QUIET_CC)$(CC) $(CFLAGS) -MMD -MP -o $@ -c $<

$(bins):
	$(QUIET_LINK)$(CC) $(LDFLAGS) $^ $(LIBS) -o $@

%.x::
	$(QUIET_LINK)$(CC) $(LDFLAGS) $^ $(LIBS) -o $@

%.a::
	$(QUIET_LINK)$(AR) rcs $@ $^

clean:
	echo $(PROCESSOR)
	$(QUIET_CLEAN)$(RM) -v $(targets) $(bins) *.o *.d $(OBJECTS)

-include *.d tidsp/*.d

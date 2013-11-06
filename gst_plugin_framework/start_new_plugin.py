#!/usr/bin/env python

import sys
import os

if (len(sys.argv) <= 1) :
    print("Error: Please specify a plugin name\n")
    sys.exit()

name = str(sys.argv[1])
print("Creating new GStreamer pluging called: '" + name + "'")


# Function definition is here
def copy_and_convert( fn_in, fn_out ):
    infile = open(fn_in)
    outfile = open(fn_out, 'w')

    replacements = {'%%%pluginname%%%':name.lower(), '%%%Pluginname%%%':name, '%%%PLUGINNAME%%%':name.upper()}

    for line in infile:
        for src, target in replacements.iteritems():
            line = line.replace(src, target)
        outfile.write(line)
    infile.close()
    outfile.close
    return;


dirname = "../pprz_gst_plugins/" + name + "/"
os.makedirs(dirname)

copy_and_convert('./template/gst_plugin_template.h', dirname + 'gst_' + name.lower() + '_plugin.h')
copy_and_convert('./template/gst_plugin_template.c', dirname + 'gst_' + name.lower() + '_plugin.c')
copy_and_convert('./template/plugin_code.h', dirname + name.lower() + '_code.h')


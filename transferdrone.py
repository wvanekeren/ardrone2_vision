#!/usr/bin/env python

import sys
import telnetlib

HOST = "192.168.1.1"
user = raw_input("Enter your remote account: ")

tn = telnetlib.Telnet(HOST)
tn.read_until("#")
tn.write("ls\n")
tn.write("exit\n")

print tn.read_all()

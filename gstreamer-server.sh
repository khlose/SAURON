#!/bin/bash
raspivid -t 999999 -h 300 -w 300 -fps 20 -hf -vf -b 2000000 -o - | gst-launch-1.0 -v fdsrc ! h264parse ! queue ! rtph264pay config-interval=1 pt=96 ! gdppay ! tcpserversink host=$1 port=$2

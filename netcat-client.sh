#!/bin/bash
nc -l 5777 | mplayer -fps 200 -demuxer h264es -

#nc -l 5777 | python detect-track-motion.py

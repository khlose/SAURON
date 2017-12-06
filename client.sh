#!/bin/bash
nc -l 5777 | mplayer -fps 20 -demuxer h264es -

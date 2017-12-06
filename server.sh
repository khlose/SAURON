#!/bin/bash
raspivid -t 0 -w 300 -h 300 -hf -fps 20 -o - | nc $1 5777

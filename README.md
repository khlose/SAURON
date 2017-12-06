<h1>SAURON</h1>
pipreqs generates requirements.txt
need to enable camera via 
sudo raspi-config

also need 
python-picamera

opencv installation guide
https://www.pyimagesearch.com/2015/07/27/installing-opencv-3-0-for-both-python-2-7-and-python-3-on-your-raspberry-pi-2/


if you fail to open video make sure that you hve libav-tools installed and if it still fails, try sudo modprobe bcm2835-v4l2

<h2>netcat approach</h2>
To stream to mplayer:
<br>on client(laptop):nc -l 2222 | mplayer -fps 200 -demuxer h264es -
<br>on server(pi):/opt/vc/bin/raspivid -t 0 -w 300 -h 300 -hf -fps 20 -o - | nc <IP-OF-THE-CLIENT> 2222
<br>from https://raspberrypi.stackexchange.com/questions/27082/how-to-stream-raspivid-to-linux-and-osx-using-gstreamer-vlc-or-netcat
<br> no idea how to use it in a python script + opencv though, the delay seems decent

<h2>gstreamer approach</h2>
<br>
Might have to try gstreamer, install via<br>
sudo apt-get install gstreamer1.0-tools
then run two scripts, gstreamer-client on laptop, gstreamer-server on pi. First arg is pi's ip second is your preferred port

opencv still doesn't work for either case :\

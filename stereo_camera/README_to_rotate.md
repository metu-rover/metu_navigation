# Steps

1- Enable Device: `sudo modprobe v4l2loopback exclusive_caps=1`

2- Write: `ffmpeg -f v4l2 -i /dev/video2 -vf "transpose=2,transpose=2,format=yuyv422" -f v4l2 /dev/video1^C`

# Notes

- Change yuv420p --> yuyv422 in order to run camera on ROS
- transpose=2 means rotate 180 degree.
- [Link](https://unix.stackexchange.com/questions/408187/rotate-webcam-as-well-as-screen-portrait-mode)
- [Answer link](https://unix.stackexchange.com/a/490040)


# AP_Camera examples


## GStreamer examples

The following examples demonstrate how to manipulate GStreamer pipelines to
capture and display video streams from a UDP or RTSP source. They are useful
for providing test images to camera algorithms, or forwarding streams from
simulators such as Gazebo to ground control stations.  

For further details see:

- [GStreamer tutorials](https://gstreamer.freedesktop.org/documentation/tutorials/index.html?gi-language=c)


### Install dependencies

#### Ubuntu

```bash
sudo apt install libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl gstreamer1.0-tools gstreamer1.0-plugins-ugly libgstrtspserver-1.0-dev
```

#### macOS

```bash
brew install gstreamer opencv
```

### `gst_rtsp_server.py`

The example provides a number of test images as an RTSP stream that may be accessed at:

- rtsp://127.0.0.1:8554/camera
- rtsp://127.0.0.1:8554/ball
- rtsp://127.0.0.1:8554/snow


Start the RTSP server:

```bash
python ./gst_rtsp_server.py
```

Display the RTSP stream:

```bash
gst-launch-1.0 rtspsrc location=rtsp://localhost:8554/camera latency=50 ! decodebin ! autovideosink
```

### `gst_rtsp_to_wx.py`

The example displays an RTSP stream in a wxPython application.

Start the RTSP server:

```bash
python ./gst_rtsp_server.py
```

Display the RTSP stream in wxPython

```bash
python ./gst_rtsp_to_wx.py
```

### `gst_udp_to_rtsp.py`

Convert a UDP stream to RTSP. This example is configured to use the same UDP
pipeline provided by the ArduPilot Gazebo [`GstCameraPlugin`](https://github.com/ArduPilot/ardupilot_gazebo?tab=readme-ov-file#3-streaming-camera-video). By converting the
UDP stream to RTSP is may be consumed by multiple applications
(e.g. a camera tracking algorithm and a GCS).

Create a UDP video stream

```bash
gst-launch-1.0 -v videotestsrc ! 'video/x-raw,format=I420,width=640,height=480,framerate=50/1' ! queue ! videoconvert ! x264enc bitrate=800 speed-preset=6 tune=4 key-int-max=10 ! rtph264pay ! udpsink host=127.0.0.1 port=5600
```

Convert to RTSP:

```bash
python ./gst_udp_to_rtsp.py
```

Display the RTSP stream:

```bash
gst-launch-1.0 rtspsrc location=rtsp://localhost:8554/camera latency=50 ! decodebin ! autovideosink
```

### `gst_udp_to_wx.py`

The example displays a UDP video stream in a wxPython application.

Create a UDP video stream:

```bash
gst-launch-1.0 -v videotestsrc ! 'video/x-raw,format=I420,width=640,height=480,framerate=50/1' ! queue ! videoconvert ! x264enc bitrate=800 speed-preset=6 tune=4 key-int-max=10 ! rtph264pay ! udpsink host=127.0.0.1 port=5600
```

Display the UDP video stream in wxPython:

```bash
python ./gst_udp_to_wx.py
```

# Real-Time Object Tracking with Gimbal Control (Simulation)

## Overview

## Requirements and Installations
1. We assume you have already setup Ardupilot with SITL and gazebo simulation
2. Install Gstreamer

```
sudo apt update
```

Install build tools and Python development libraries and GStreamer related plugins
```

sudo apt install -y build-essential cmake git pkg-config libgtk-3-dev libavcodec-dev libavformat-dev libswscale-dev

sudo apt install -y python3-dev python3-numpy python3-pip

sudo apt install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
                    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
                    gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools
```

3. default opencv from pip should be uninstalled and again built with gstreamer support (If already did this go to next step)
```
git clone https://github.com/opencv/opencv.git
```

```
git clone https://github.com/opencv/opencv_contrib.git
```

```
cd opencv
mkdir build
cd build
```

Build Configurations
```
cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
      -D PYTHON3_EXECUTABLE=$(which python3) \
      -D PYTHON3_INCLUDE_DIR=$(python3 -c "from distutils.sysconfig import get_python_inc; print(get_python_inc())") \
      -D PYTHON3_LIBRARY=$(python3 -c "from distutils.sysconfig import get_config_var; print(get_config_var('LIBDIR'))") \
      -D PYTHON3_PACKAGES_PATH=$(python3 -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())") \
      -D WITH_GSTREAMER=ON \
      -D WITH_FFMPEG=ON \
      -D BUILD_opencv_python3=ON \
      -D BUILD_EXAMPLES=OFF ..

```

```
make -j$(nproc)
```

```
sudo make install
sudo ldconfig
```



## Description
There are two main Python scripts:

1. **`tracking.py`**: This script handles video streaming and object tracking. It uses OpenCV to process video frames, applies an object tracking algorithm, and sends commands to the gimbal to adjust its orientation based on the object's position in the frame.

2. **`send_camera_information.py`**: This script communicates with the UAV's autopilot system using MAVLink. It sends the necessary camera and gimbal information to ensure the UAV's camera is correctly oriented.


## Runing the scripts
just run

  ```bash run_tracking.sh```

![](https://github.com/rvp-group/ca2lib/wiki/images/ca2lib_title.png)
Ca2Lib is an open-source `LiDAR-Camera[RGB]` calibration toolbox.

# 🚀 Usage

To calibrate your camera(s), first generate a [target configuration file](todo), gather your sensors/bag and run:
```bash
rosrun ca2lib stereo_recorder \
    -l <topic_camera_left> \
    -r <topic_camera_right> \
    -t <target.yaml> \
    -o <output folder>
```

After enough views of the calibration target are acquired, run the calibration

```bash
rosrun ca2lib calibrate_camera \
    -i <images folder> \
    -t <target.yaml> \
    -o <cam_results.yaml>
```

Onto the extrinsics!

To calibrate the LiDAR relative pose, follow the calibration guide [Link Soon]() and run the following node

```bash
rosrun ca2lib calibrate_lidar_camera \
    -c <camera_topic> \
    -i <cam_intrinsics.yaml> \
    -l <cloud_topic> \
    -t <target.yaml> \
    -o <lidar_in_cam.yaml>
```

Finally, visualize your setup reprojections:

```bash
rosrun ca2lib visualize_lidar_in_camera \
    -c <camera_topic> \
    -i <cam_intrinsics.yaml> \
    -l <cloud_topic>
    -e <lidar_in_cam.yaml>
```

# 👷 Setup

We suggest running `ca2lib` using Docker. Simply `cd` into the repository and build the Docker image:

```bash
git clone git@github.com:rvp-group/ca2lib.git
cd ca2lib
./docker/build.sh
```

To finally start a container, simply:

```bash
./docker/run.sh <DATA-DIR>
```

## Build from source

Assuming you already have ROS Noetic installed (if not, refer to [this guide](https://wiki.ros.org/noetic/Installation)), install the requirements:

```bash
sudo apt-get install -y \
    libspdlog-dev
```

Clone the repository inside your catkin workspace and build the package.

```bash
cd catkin_ws/src
git clone git@github.com:rvp-group/ca2lib.git
cd ..
catkin build ca2lib
```


# 📰 News

22/02/2024: Version 0.0.0 is released!

# 📖 Cite us
If you use any of this code here the reference
```
@article{giac2024ca2lib,
    author = {Giacomini, Emanuele and Brizi, Leonardo and Di Giammarino, Luca and Salem, Omar and Perugini, Patrizio and Grisetti, Giorgio},
    title = {Ca2Lib: Simple and Accurate LiDAR-RGB Calibration Using Small Common Markers},
    journal = {Sensors},
    volume = {24},
    year = {2024},
    number = {3},
    doi = {10.3390/s24030956}
}
```

# Copyright

Copyright (c) 2023, R(obot) V(ision and) P(erception)

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

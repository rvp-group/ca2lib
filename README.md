![](https://github.com/rvp-group/ca2lib/wiki/images/ca2lib_title.png)
Ca2Lib is an open-source `LiDAR-Camera[RGB]` calibration toolbox.

# 🚀 Usage

You can follow our guide [here](https://github.com/rvp-group/ca2lib/wiki/Usage)

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

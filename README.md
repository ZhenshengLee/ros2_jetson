# ros2_jetson

- ros2 foxy src confuguration in jetson ubuntu18.04
- tested in jetpack 4.4
- no dependency on jetson container service

## setup

- internet is needed
- replace `URL https://github.com` with `URL https://hub.fastgit.org` for connection issue.
- `GIT_REPOSITORY https://github.com` with `GIT_REPOSITORY https://hub.fastgit.org`
- `curl https://github.com` with `curl https://hub.fastgit.org`

```sh
# install ROS packages
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
                        libbullet-dev \
                        libpython3-dev \
                        python3-colcon-common-extensions \
                        python3-flake8 \
                        python3-pip \
                        python3-pytest-cov \
                        python3-setuptools \
                        python3-vcstool \
                        libasio-dev \
                        libtinyxml2-dev \
                        libcunit1-dev \
# python3-rosdep， python3-rosinstall-generator与ros1有依赖问题

# ecal
sudo apt-get install git cmake doxygen graphviz build-essential \
                    zlib1g-dev qt5-default libhdf5-dev libprotobuf-dev
                    libprotoc-dev protobuf-compiler libcurl4-openssl-dev

# for galactic cyclonedds
sudo apt-get install -y --no-install-recommends \
                        libbison-dev

python3 -m pip install -U \
            argcomplete \
            flake8-blind-except \
            flake8-builtins \
            flake8-class-newline \
            flake8-comprehensions \
            flake8-deprecated \
            flake8-docstrings \
            flake8-import-order \
            flake8-quotes \
            pytest-repeat \
            pytest-rerunfailures \
            pytest
```

```sh
# installation is in /usr/local
cd ./common
vcs import src < common.repos
# manually install iceoryx
# manually install fastdds
# manually install yaml-cpp
```

```sh
cd ./foxy
vcs import src < ros2.repos
# vcs import --force src < ./ros2-dev.repos
sudo su
./build.sh
# install it into /opt/ros/
ldconfig
```

## removed packages

- connext and rmw
- unneeded msgs
- ros1 bridge
- dummy, demo and example

## added packages

- ecal
- iceoryx
- image based pipline
- pointcloud based pipline
- cuda based pipline

## deployment

- docker based binary deployment

## test

```sh
# optional choose rmw
# export RMW_IMPLEMENTATION=rmw_iceoryx_cpp
# export RMW_IMPLEMENTATION=rmw_ecal_dynamic_cpp
# export RMW_IMPLEMENTATION=rmw_ecal_proto_cpp
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/foxy-dev/setup.bash
ros2 launch demo_nodes_cpp talker_listener.launch.py
```

# ros2_jetson

- ros2 foxy src confuguration in jetson ubuntu18.04
- tested in jetpack 4.4
- no dependency on jetson container service
- internet is needed

## install ros2-dashing

too many dependencies, install ros2-dashing first

```sh
sudo apt-key add ./ros.asc
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu/ $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install ros-dashing-desktop

sudo apt-get update
sudo apt-get install -y --no-install-recommends \
        curl \
        wget \
        gnupg2 \
        lsb-release \
        libncurses5-dev \
        uuid-dev libacl1-dev liblzo2-dev \
        libssl-dev \
        libeigen3-dev \
        libxaw7-dev \
        libxrandr-dev \
        liblog4cxx-dev \
        sip-dev \
        libfreetype6-dev \
        libogre-1.9-dev \
        lttng-tools lttng-modules-dkms liblttng-ust-dev \
```

## install debs

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
                        rapidjson-dev \
# python3-rosdep， python3-rosinstall-generator与ros1有依赖问题

# ecal
sudo apt-get install git cmake doxygen graphviz build-essential \
                    zlib1g-dev qt5-default libhdf5-dev libprotobuf-dev \
                    libprotoc-dev protobuf-compiler libcurl4-openssl-dev \

# for galactic cyclonedds
sudo apt-get install -y bison libbison-dev
# --reinstall if failed

# for ros2-tracing
sudo apt-add-repository ppa:lttng/stable-2.12
sudo apt update
sudo apt-get install lttng-tools lttng-modules-dkms liblttng-ust-dev python3-lttngust python3-babeltrace

sudo usermod -aG tracing $USER
reboot

sudo -H python3 -m pip install -U \
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
            pytest \
            -i  https://pypi.tuna.tsinghua.edu.cn/simple \

sudo -H python3 -m pip install -U \
        sip  \
        lark_parser \
        -i  https://pypi.tuna.tsinghua.edu.cn/simple

# for galactic
sudo -H python3 -m pip install -U \
        importlib-resources  \
        -i  https://pypi.tuna.tsinghua.edu.cn/simple
```

## compile some deps

```sh
# installation is in /usr/local
cd ./common
vcs import src < common.repos
# manually install yaml-cpp
# install cpp-toml
# manually install iceoryx
~~# manually install fastdds~~
```

## patch and compile

- replace `URL https://github.com` with `URL https://github.com.cnpmjs.org` for connection issue.
- `GIT_REPOSITORY https://github.com` with `GIT_REPOSITORY https://github.com.cnpmjs.org`
- `curl https://github.com` with `curl https://github.com.cnpmjs.org`

for foxy release

```sh
cd ./foxy
vcs import src < ros2.repos
# for events executor in foxy
# vcs import --force src < ./ros2-dev.repos
sudo su
./build.sh
# install it into /opt/ros/
ldconfig
```

for galactic release

```sh
cd ./galactic
# vcs import src < ros2.repos
vcs import --force src < ./ros2-dev.repos
vcs import --force src < ./basic.repos
vcs import --force src < ./localization.repos
vcs import --force src < ./perception.repos
vcs import --force src < ./planning.repos
vcs import --force src < ./safety.repos
sudo su
./build.sh
# install it into /opt/ros/
ldconfig

sudo su
# tar binary
tar -zcvf ./ros2-binary-x86-galactic-dev.tar.gz /opt/ros/galactic-dev
tar -zcvf ./ros2-binary-x86-galactic.tar.gz /opt/ros/galactic
tar -zcvf ./ros2-binary-aarch64-galactic.tar.gz /opt/ros/galactic

# tar src files
# in root
tar -zcvf  ./ros2_jetson-src-2022-galactic-dev.tar.gz  ./ros2_jetson
tar -zcvf  ./ros2_jetson-src-2022-galactic.tar.gz  ./ros2_jetson

# tar build-cache
tar -zcvf  ./build-cache-x86-galactic-dev.tar.gz $HOME/.colcon/build/x86_64/galactic-dev/
tar -zcvf  ./build-cache-x86-galactic.tar.gz $HOME/.colcon/build/x86_64/galactic/
tar -zcvf  ./build-cache-aarch64-galactic.tar.gz $HOME/.colcon/build/aarch64/galactic/
```

## compile with cache

**first you should install ros2-dashing, debs and compile some deps**

in another host you can compile with cache to acc the speed

```sh
# untar in another machine
sudo su
tar -xvf ./build-cache-x86-galactic-dev.tar.gz -C /
tar -xvf ./build-cache-x86-galactic.tar.gz -C /

cd ./galactic
./build.sh
```

## deployment with binary

**first you should install ros2-dashing, debs and compile some deps**

with binary, simply untar in /opt/ros/galactic

```sh
tar -xvf ./ros2-binary-x86-galactic-dev.tar.gz -C /
tar -xvf ./ros2-binary-x86-galactic.tar.gz -C /
```

## deployment with docker

- docker based binary deployment

```sh
# 1. build docker image based on original image by nvidia
./build_docker.sh

# 2. docker into this container with root
./docker_jetson_into_root.sh
# and install sudo and lsb-release
sed -i 's/ports.ubuntu.com/mirrors.tuna.tsinghua.edu.cn/' /etc/apt/sources.list
apt update && apt upgrade -y
apt install sudo lsb-release -y
# optinal apt clean all

# 3. build ros2_dep
# upgrade your cmake to 3.18.2
# following guides in install_ros2_dep.sh

# 4. export to a new image
docker export 31bccc730393 > ga_jetson_r32.4.4_zs.tar

# 5. reuse the new image
docker import ga_jetson_r32.4.4_zs.tar ga_jetson_r32.4.4:dep
./jetson_start.sh
./jetson_into.sh

# 6. use the new image to build ros2 binary
sudo su
./build.sh

# 7. tar and copy the binary to host
sudo su
./package.sh
```

note:

- docker container is sharing `/root/.colcon/` with host, so colcon build files are not in the image.
- while `/opt/` is the private folder of container.
- the ros2 cannot be run in this docker container but canbe run in the jetson target.

## test

```sh
# export ENV with custom ros2
export ROS_DOMAIN_ID=42
export ROS_VERSION=2
export ROS_PYTHON_VERSION=3
export ROS_DISTRO=galactic

# optional choose rmw
# export RMW_IMPLEMENTATION=rmw_iceoryx_cpp
# export RMW_IMPLEMENTATION=rmw_ecal_dynamic_cpp
# export RMW_IMPLEMENTATION=rmw_ecal_proto_cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# if in 20.04
source /opt/ros/galactic-dev/setup.bash
# if in 18.04
source /opt/ros/galactic/setup.bash
# rm -rf /opt/ros/galactic and rebuild if encounter any source issues
ros2 launch demo_nodes_cpp talker_listener.launch.py
```

## build-farm

You can cross-compile certain packages in `build_farm` with `ga_ros.sh`.

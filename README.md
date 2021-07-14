# ros2_jetson

- ros2 foxy src confuguration in jetson ubuntu18.04
- tested in jetpack 4.4
- no dependency on jetson container service

## setup

- internet is needed
- replace `URL https://github.com` with `URL https://hub.fastgit.org` for connection issue.
- `GIT_REPOSITORY https://github.com` with `GIT_REPOSITORY https://hub.fastgit.org`

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
cd ./foxy
sudo ./build.sh
```

## removed packages

- connext and rmw
- fastrtps and rmw
- unneeded msgs
- ros1 bridge
- dummy, demo and example

## added packages

- ecal
- iceoryx

## todo

- cross compile
- deb deployment
- docker deployment

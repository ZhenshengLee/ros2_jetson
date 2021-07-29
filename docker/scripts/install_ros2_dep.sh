apt update && apt install locales
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add the ROS 2 apt repository
apt-get update
apt-get install -y --no-install-recommends \
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
# rm -rf /var/lib/apt/lists/*

# so many other dependencies
apt install ros-dashing-desktop

# sudo apt install build-essential automake libtool libfreetype6-dev libfreeimage-dev
# libzzip-dev libxrandr-dev libxaw7-dev freeglut3-dev libgl1-mesa-dev libglu1-mesa-dev libpoco-dev libtbb-dev doxygen libcppunit-dev

apt-key add /ros2_jetson/ros.asc
sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# install development packages
apt-get update
apt-get install -y --no-install-recommends \
		build-essential \
		cmake \
		git \
		libbullet-dev \
		libpython3-dev \
		python3-colcon-common-extensions \
		python3-flake8 \
		python3-pip \
		python3-pytest-cov \
		python3-rosdep \
		python3-setuptools \
		python3-vcstool \
		python3-rosinstall-generator \
		libasio-dev \
		libtinyxml2-dev \
		libcunit1-dev
# rm -rf /var/lib/apt/lists/*

# install some pip packages needed for testing
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
		pytest \
        -i  https://pypi.tuna.tsinghua.edu.cn/simple \

python3 -m pip install -U \
		sip  \
		lark_parser \
        -i  https://pypi.tuna.tsinghua.edu.cn/simple

# for galactic
python3 -m pip install -U \
		importlib-resources  \
        -i  https://pypi.tuna.tsinghua.edu.cn/simple

# ecal
apt-get install git cmake doxygen graphviz build-essential \
                    zlib1g-dev qt5-default libhdf5-dev libprotobuf-dev \
                    libprotoc-dev protobuf-compiler libcurl4-openssl-dev \

# rviz ogre
# https://hub.fastgit.org/aseprite/freetype2/archive/refs/tags/VER-2-6-5.tar.gz
# https://github.com/madler/zlib/archive/refs/tags/v1.2.11.tar.gz
#!/usr/bin/env bash

# 检测是16.04还是18.04
case "$(lsb_release -r | cut -f2)" in
    20.04)
      GA_ROS_DISTRO="foxy-dev"
      ;;
    18.04)
    GA_ROS_DISTRO="foxy"
      ;;
    *)
    error "Unsupported ubuntu distro"
    error "Please use Linux, we recommend Ubuntu 18.04."
    exit 1
esac

export ROS_DISTRO="foxy"

MACHINE_ARCH=$(uname -m)
JOB_ARG="--parallel-workers 10"
if [ "$MACHINE_ARCH" == 'aarch64' ]; then
  JOB_ARG="--parallel-workers 6"
fi
LOG_ARG=" --log-base $HOME/.colcon/log/$MACHINE_ARCH/$GA_ROS_DISTRO "
COLCON_ARG=" --build-base $HOME/.colcon/build/$MACHINE_ARCH/$GA_ROS_DISTRO --install-base /opt/ros/$GA_ROS_DISTRO --merge-install"
# COLCON_ARG=" --build-base ./colcon/build --install-base ./colcon/install --merge-install"
CMAKE_OPT=" --cmake-args -DBUILD_TESTING=OFF "

# echo "rm -rf /opt/ros/$GA_ROS_DISTRO"
# rm -rf /opt/ros/$GA_ROS_DISTRO

echo "colcon ${LOG_ARG} build ${COLCON_ARG}   ${JOB_ARG}  ${CMAKE_OPT} -DCMAKE_BUILD_TYPE=Release"
colcon ${LOG_ARG} build ${COLCON_ARG}   ${JOB_ARG} ${CMAKE_OPT} -DCMAKE_BUILD_TYPE=Release
# --cmake-force-configure
# colcon ${LOG_ARG} build ${COLCON_ARG}   ${JOB_ARG} ${CMAKE_OPT} -DCMAKE_BUILD_TYPE=Release
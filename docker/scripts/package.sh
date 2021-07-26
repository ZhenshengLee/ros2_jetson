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

echo "tar -zcvf /opt/ros/${GA_ROS_DISTRO}"
tar -zcvf /ros2_jetson/docker/opt/ros/${GA_ROS_DISTRO}.tar.gz /opt/ros/${GA_ROS_DISTRO}


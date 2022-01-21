#!/usr/bin/env bash

# 检测是16.04还是18.04
case "$(lsb_release -r | cut -f2)" in
    20.04)
      GA_ROS_DISTRO="galactic-dev"
      ;;
    18.04)
    GA_ROS_DISTRO="galactic"
      ;;
    *)
    error "Unsupported ubuntu distro"
    error "Please use Linux, we recommend Ubuntu 18.04."
    exit 1
esac

cmd=$(which tmux) # tmux path
session=zswork # session name

if [ -z $cmd ]; then
    echo "You need to tmux."
    exit 1
fi

$cmd has -t $session

if [ $? != 0 ]; then
	$cmd new -s $session -d -n git

	$cmd neww -n loc -t $session -d
	$cmd neww -n odd -t $session -d
	$cmd neww -n shm -t $session -d
    $cmd neww -n gazebo -t $session -d
	$cmd neww -n temp -t $session -d

    $cmd splitw -v -p 50 -t $session:git
    $cmd splitw -h -p 50 -t $session:git

	$cmd splitw -v -p 50 -t $session:loc
	$cmd splitw -h -p 50 -t $session:loc
	$cmd splitw -h -p 50 -t $session:loc

    $cmd splitw -v -p 50 -t $session:odd
	$cmd splitw -h -p 50 -t $session:odd
	$cmd splitw -h -p 50 -t $session:odd

    $cmd splitw -v -p 50 -t $session:shm
	$cmd splitw -h -p 50 -t $session:shm
	$cmd splitw -h -p 50 -t $session:shm

    $cmd splitw -v -p 50 -t $session:gazebo
	$cmd splitw -h -p 50 -t $session:gazebo
	$cmd splitw -h -p 50 -t $session:gazebo

    $cmd select-window -t $session:loc
    $cmd select-layout tiled
    for _pane in $(tmux list-panes -F '#P'); do
        tmux send-keys -t ${_pane} "source /opt/ros/$GA_ROS_DISTRO/setup.bash" ENTER
    done

    $cmd select-window -t $session:odd
    $cmd select-layout tiled
    for _pane in $(tmux list-panes -F '#P'); do
        tmux send-keys -t ${_pane} "source /opt/ros/$GA_ROS_DISTRO/setup.bash" ENTER
    done

    $cmd select-window -t $session:shm
    $cmd select-layout tiled
    for _pane in $(tmux list-panes -F '#P'); do
        tmux send-keys -t ${_pane} "source /opt/ros/$GA_ROS_DISTRO/setup.bash" ENTER
    done

    $cmd select-window -t $session:gazebo
    $cmd select-layout tiled
    for _pane in $(tmux list-panes -F '#P'); do
        tmux send-keys -t ${_pane} "source /opt/ros/$GA_ROS_DISTRO/setup.bash" ENTER
    done

    $cmd select-window -t $session:git

else

echo "Tmux session:$session exist, attach to."
$cmd att -t $session

fi

$cmd att -t $session

exit 0

#kill
#tmux kill-session -t zswork

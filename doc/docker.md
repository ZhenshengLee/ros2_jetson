# docker

## install

### install docker-ce


```
sudo apt install apt-transport-https ca-certificates curl software-properties-common
# curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
# sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu bionic stable"
curl -fsSL https://mirrors.ustc.edu.cn/docker-ce/linux/ubuntu/gpg | sudo apt-key add -
sudo add-apt-repository "deb [arch=amd64] https://mirrors.ustc.edu.cn/docker-ce/linux/ubuntu $(lsb_release -cs) stable"
apt-cache policy docker-ce
sudo apt install docker-ce docker-compose
sudo systemctl status docker
# 配置
如果您想在运行docker命令时避免键入sudo ，请将您的用户名添加到docker组：
sudo groupadd docker
sudo usermod -aG docker ${USER}
要应用新的组成员身份，请注销服务器并重新登录，或键入以下内容：
su - ${USER}
 系统将提示您输入用户密码以继续。
通过键入以下内容确认您的用户现已添加到docker组：
id -nG
如果您需要将用户添加到您未登录的docker组，请使用以下命令明确声明该用户名：
sudo usermod -aG docker yx
# 使用
docker [option] [command] [arguments]
# 查看所有命令
docker
```

### 安装nvidia-container-toolkit
```
# 添加源
$ distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
$ curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
$ curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
# 安装并重启docker
$ sudo apt update && sudo apt install -y nvidia-container-toolkit
$ sudo systemctl restart docker
```

## development

```
docker import - ga_team/ga_all:18.04 < ga_all_1804.tar
./ga_all_start.sh -l
./ga_all_into.sh
```

docker image包含组件
```
cmake3.18.2
cuda10.0
ros-melodic
gtsam4.0-/usr/local/
opencv4.1-cuda-/usr/local/
pcl-1.9-cuda-/usr/local/
open3d-gpu-/opt/open3d/open3d-gpu/
```

后续可以制作数据镜像, data volume

## 测试

```
# 在docker 内部
roslaunch gpuac_launch rslidar_loc_docker.launch
# 在主机
rosbag play --clock 2020-09-02-18-00-39.bag --topics /imu/data /rslidar_points /fix
roslaunch gpuac_launch rslidar_loc_rviz.launch
```

## 保存新镜像

```
docker export 50d941249065 > ga_all_1804.tar
```

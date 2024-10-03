# 塔洛斯社团考核

*土木（布院）-24-李芊墨*

## 使用教程（*ubuntu*）

### 安装*ros2*&应用ros2环境变量

```bash
sudo apt update -y && \
    sudo apt install -y --no-install-recommends \
        tzdata \
        build-essential \
        curl \
        gnupg2 \
        lsb-release \
        locales \
        software-properties-common \
        x11-apps && \
    sudo ln -fs /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    sudo dpkg-reconfigure --frontend noninteractive tzdata && \
    sudo locale-gen en_US en_US.UTF-8 && \
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null && \

sudo apt update -y && \

sudo apt upgrade -y && \

sudo apt install -y --no-install-recommends \
        ros-humble-desktop \
        ros-dev-tools \
        ros-humble-turtlesim \
        '~nros-humble-rqt*' && \

sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*

echo "source /opt/ros/humble/setup.bash" | sudo tee -a /etc/bash.bashrc

source /opt/ros/humble/setup.bash
```



### 克隆仓库&更改目录

`git clone https://github.com/Qume2005/talos-recruitment.git`

`cd ./talos-recruitment`



### 构建包

`colcon build`



### 应用项目工作空间环境变量

`source install/setup.bash`



### 启动*turtlesim_node*

`ros2 run turtlesim turtlesim_node`



### 启动*turtle_controller*

`ros2 run moving turtle_controller`




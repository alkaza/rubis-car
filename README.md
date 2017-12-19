# Guidelines
```
git clone https://github.com/alkaza/rubis-car

cp -avr ~/rubis-car/race ~/catkin_ws/src

cd ~/catkin_ws
catkin_make race
source devel/setup.bash
catkin_make

cp -avr ~/rubis-car/Arduino/Firmware/Libraries/RobotEQ ~/Arduino/libraries

cd ~/Arduino/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .

roscore
rosrun race talker.py
rosrun rosserial_python serial_node.py /dev/ttyACM0
rosrun race keyboard.py
```

# Install L4T 21.3 (from host PC)
## Download L4T 21.3 release package and sample file system from
```
wget http://developer.download.nvidia.com/embedded/L4T/r21_Release_v3.0/Tegra124_Linux_R21.3.0_armhf.tbz2
wget http://developer.download.nvidia.com/embedded/L4T/r21_Release_v3.0/Tegra_Linux_Sample-Root-Filesystem_R21.3.0_armhf.tbz2
```
## Untar the files and assemble the rootfs:
```
tar xpf Tegra124_Linux_R21.3.0_armhf.tbz2
cd Linux_for_Tegra/rootfs/
sudo tar xpf ../../Tegra_Linux_Sample-Root-Filesystem_R21.3.0_armhf.tbz2
cd ../
sudo ./apply_binaries.sh
```
## Flash the rootfs onto the system's internal eMMC.
```
sudo ./flash.sh jetson-tk1 mmcblk0p1
```

# Jetson TK1
## Install
### build-essential
```
sudo get update && apt-get install build-essential
```
### gcc 5
```
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt-get update
sudo apt-get install gcc-5 g++-5
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 60 --slave /usr/bin/g++ g++ /usr/bin/g++-5
```
### git
```
sudo apt-get install git
```

## Grinch Kernel
```
git clone https://github.com/jetsonhacks/installGrinch.git
cd installGrinch
./installGrinch.sh 
```

## ROS setup
### Install ROS
```
git clone https://github.com/jetsonhacks/installROS.git
cd installROS
./updateRepositories.sh
./installROS.sh
```
### Create a ROS Workspace
```
source /opt/ros/indigo/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
echo $ROS_PACKAGE_PATH
```
### Install ROS packages
#### rosserial_arduino
```
sudo apt-get install ros-indigo-angles
sudo apt-get install ros-indigo-rosserial-arduino
sudo apt-get install ros-indigo-rosserial
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200
rostopic echo chatter
```
#### teleop_twist_keyboard (optional)
```
sudo apt-get install ros-indigo-teleop-twist-keyboard
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
#### urg_node
```
sudo apt-get install ros-indigo-roslint
sudo apt-get install ros-indigo-urg-node
rosrun urg_node urg_node _ip_address:=192.168.1.11
rostopic echo /scan
```

## Arduino setup
### Install Arduino IDE
```
wget http://arduino.cc/download.php?f=/arduino-1.8.5-linuxarm.tgz
tar -xvf arduino-1.8.5-linuxarm.tgz
cd arduino-1.8.5
./install.sh
```

### Setup ros_lib
```
cd ~/Arduino/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
```

# Network setup
## Hokuyo 
```
Method: Manual
Connection Name: Hokuyo
IP Address 192.168.1.15
Subnet Mask 255.255.255.0
Default Gateway 192.168.1.1
```
## Jetson TK1
### Open ~/.bashrc
```
sudo gedit ~/.bashrc
```
### Add lines:
```
export ROS_IP=192.168.0.206
```
### To revert this, replace it back by:
```
export ROS_HOSTNAME=localhost
```
### Open /etc/hosts
```
sudo gedit /etc/hosts
```
### Add the line:
```
192.168.1.160 alena
```
## Remote system
### Open ~/.bashrc
```
sudo gedit ~/.bashrc
```
### Add lines:
```
export ROS_MASTER_URI=http://192.168.0.206:11311
export ROS_IP=192.168.0.160
```
### To revert this, replace it back by:
```
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311
```
### Open /etc/hosts
```
sudo gedit /etc/hosts
```
### Add the line:
```
192.168.1.206 ubuntu
```

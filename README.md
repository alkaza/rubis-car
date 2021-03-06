# Guidelines
## Get source code
```
git clone https://github.com/alkaza/rubis-car
cp -avr ~/rubis-car/race ~/catkin_ws/src
cp -avr ~/rubis-car/Arduino/Firmware/libraries/RobotEQ ~/Arduino/libraries
```
## Launch keyboard control
```
roscore
rosrun race talker.py
rosrun rosserial_python serial_node.py /dev/ttyACM0
rosrun race keyboard.py
rosrun race kill.py
```
## Launch driving straight
```
roscore
rosrun urg_node urg_node _ip_address:=192.168.1.11
rosrun race talker.py
rosrun rosserial_python serial_node.py /dev/ttyACM0
rosrun race control.py
rosrun race dist_finder.py
rosrun race kill.py
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
* Put your system into "reset recovery mode" by holding down the RECOVERY button and press RESET button once on the main board.
* Ensure your Linux host system is connected to the target device through the USB cable for flashing.
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
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600
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
cd ~/catkin_ws
catkin_make race
source devel/setup.bash
catkin_make
cd ~/Arduino/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
```

# Network setup
## Hokuyo 
_Note: device IP 192.168.1.11_ 
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
export ROS_IP={Jetson TK1 IP}
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
{Remote system IP} {User name}
```
## Remote system
### Open ~/.bashrc
```
sudo gedit ~/.bashrc
```
### Add lines:
```
export ROS_MASTER_URI=http://{Jetson TK1 IP}:11311
export ROS_IP={Remote system IP}
```
_Note: only run roscore on Jetson TK1_
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
{Jetson TK1 IP} {User name}
```

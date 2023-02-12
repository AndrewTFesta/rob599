# mkdir ~/ros_install
# cd ~/ros_install
# sudo apt update
# sudo apt upgrade
# wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_noetic.sh
# chmod 755 ./install_ros_noetic.sh 
# bash ./install_ros_noetic.sh

sudo apt-get install ros-noetic-joy
sudo apt-get install ros-noetic-teleop-twist-joy
sudo apt-get install ros-noetic-teleop-twist-keyboard
sudo apt-get install ros-noetic-laser-proc
sudo apt-get install ros-noetic-rgbd-launch
sudo apt-get install ros-noetic-rosserial-arduino 
sudo apt-get install ros-noetic-rosserial-python
sudo apt-get install ros-noetic-rosserial-client
sudo apt-get install ros-noetic-rosserial-msgs
sudo apt-get install ros-noetic-amcl
sudo apt-get install ros-noetic-map-server
sudo apt-get install ros-noetic-move-base
sudo apt-get install ros-noetic-urdf
sudo apt-get install ros-noetic-xacro
sudo apt-get install ros-noetic-compressed-image-transport
sudo apt-get install ros-noetic-rqt*
sudo apt-get install ros-noetic-rviz
sudo apt-get install ros-noetic-gmapping
sudo apt-get install ros-noetic-navigation
sudo apt-get install ros-noetic-interactive-markers

sudo apt install ros-noetic-dynamixel-sdk
sudo apt install ros-noetic-turtlebot3-msgs
sudo apt install ros-noetic-turtlebot3

# sudo apt update
# sudo apt install libudev-dev
# cd ~/catkin_ws/src
# git clone -b develop https://github.com/ROBOTIS-GIT/ld08_driver.git
# cd ~/catkin_ws/src/turtlebot3 && git pull
# rm -r turtlebot3_description/ turtlebot3_teleop/ turtlebot3_navigation/ turtlebot3_slam/ turtlebot3_example/
# cd ~/catkin_ws && catkin_make

# echo 'export LDS_MODEL=LDS-02' >> ~/.bashrc
# source ~/.bashrc

# sudo dpkg --add-architecture armhf
# sudo apt-get update
# sudo apt-get install libc6:armhf

# export OPENCR_PORT=/dev/ttyACM0
# export OPENCR_MODEL=burger_noetic
# rm -rf ./opencr_update.tar.bz2

# mkdir ~/ros_install
# cd ~/ros_install
# wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS1/latest/opencr_update.tar.bz2 
# tar -xvf opencr_update.tar.bz2

# cd ./opencr_update
# ./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr

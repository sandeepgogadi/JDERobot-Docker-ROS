sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key 67170598AF249743

sudo apt-add-repository "deb http://zeroc.com/download/apt/ubuntu$(lsb_release -rs) stable main"
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv 5E6DA83306132997

sudo sh -c 'cat<<EOF>/etc/apt/sources.list.d/jderobot.list
# for ubuntu 16.04 LTS (64 bit)

deb [arch=amd64] http://jderobot.org/apt xenial main
EOF'

sudo apt-key adv --keyserver keyserver.ubuntu.com --recv 24E521A4

sudo apt update

sudo apt install jderobot
sudo apt install jderobot-gazebo-assets

source ~/.bashrc

sudo apt install jderobot-cameraview jderobot-cameraserver 

sudo apt install jderobot-uav-viewer-python

sudo apt install jderobot-webtools

source ~/.bashrc

sudo apt update && sudo apt upgrade

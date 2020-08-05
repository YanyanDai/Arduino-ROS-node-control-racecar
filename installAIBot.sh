# Make the jetsonbot catkin workspace
./setupCatkinWorkspace.sh AIBot
cd ~/AIBot
sudo apt-get install arduino
sudo apt-get install ros-melodic-joy -y
cd src
#git clone https://github.com/jetsonhacks/jetsoncar_teleop.git
cd ..
catkin_make

# Copy Arduino code 
cd ~/InstallAIBot
cp -r Arduino\ Firmware/* '/home/nvidia/sketchbook'
sudo apt-get install ros-melodic-rosserial-arduino ros-melodic-rosserial ros-melodic-angles -y
cd ~/sketchbook/libraries
rm -rf ros_lib
source ~/AIBot/devel/setup.bash
rosrun rosserial_arduino make_libraries.py ~/sketchbook/libraries
cd ~/InstallAIBot

# InstallAIBot
# Arduino ROS node control racecar

Installation:
git clone https://github.com/YanyanDai/Arduino-ROS-node-control-racecar.git InstallAIBot

cd InstallAIBot

./installAIBot.sh

Test:
1. Open Arduino file (sketchbook/AIBot/AIBot.ino)
--import ros_lib library
--choose board and port
--upload to arduino

2. Open terminal
roscore

3. Open terminal
rosrun rosserial_python serial_node.py /dev/ttyACM0

4. check rostopic list

5. publish topic
rostopic pub /aibot/cmd_vel geometry_msgs/Twist "linear:
        x: 0.5
        y: 0.0
        z: 0.0
angular:
        x: 0.0
        y: 0.0
        z: 0.0" 
        
PS: AIBot (racecar) linear velocity range: 
    Forware: 0.5 ~ 1 m/s, Backward:-0.5 ~ -1 m/s, Stop: 0
   
   AIBot (racecar) angular velocity range: 
    -0.5~0.5 rad



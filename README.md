# Robotics-Project
Making a simple government turtlebot navigate through a given environment. This is the README for utilizing the this repo's code and successfully launching the project.

# Prerequirements
- Fully installed ROS Kinetic version
- TurtleBot 3 Kinetic Packages
```bash
sudo apt update
sudo apt install ros-kinetic-turtlebot-gazebo
sudo apt install ros-kinetic-turtlebot-navigation
sudo apt install ros-kinetic-turtlebot-teleop
```
-----------------
### Launch ROS abd source environments
1. Check ROS version
```bash
rosversion -d #should return 'kinetic'
```
2. Source environments
```bash
source /opt/ros/kinetic/setup.bash
source /opt/ros/kinetic/setup.bash
source ~/yumi_ws/devel/setup.bash
```
Alternatively add these to your `.bashrc` file 
--------------------
### Unpacking, Preparing and Compiling
1. Unpack 
```bash 
tar -zxvf robot_assignment_ws.tar.gz
```
2. Compile
```bash
rm -rf robot_assignment_ws.tar.gz
cd robot_assignment_ws
rm -rf build devel install #Pravesh pre-built the environment so we need to remove that first
cd ~/robot_assignment_ws/src
catkin_create_pkg surveillance_bot rospy std_msgs geometry_msgs sensor_msgs nav_msgs # Creates a package called surveillance_bot
mkdir -p surveillance_bot/launch #Creates a launch folder
nano surveillance_bot/launch/gmapping.launch #Paste the xml file, save and exit
cd ~/robot_assignment_ws 
catkin_make
```
3. Run
```bash
source devel/setup #add this to your .bashrc file
./startWorld
```
### Mapping Phase - Driving the robot manually
1. 
```bash
sudo apt install ros-kinetic-teleop-twist-keyboard
```
2. Terminal 1 
```bash
clear
./startWorld
```
3. Terminal 2 - navigator node
```bash
# environment variables should be automatically loaded
roslaunch surveillance_bot gmapping.launch
```
<img width="1436" height="1084" alt="image" src="https://github.com/user-attachments/assets/893f6276-2de8-48d8-9805-fe5c52d31fbc" />

4. Terminal 3
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/cmd_vel_mux/input/teleop
```
5. In the terminal, use the following keys to map the robot around the environment in the way you'd like. 

(add picture of the environment )

6. Save the map in Terminal 4
```bash
rosrun map_server map_saver -f my_map
mkdir ~/robot_assignment_ws/src/surveillance_bot/maps
cp my_map.yaml ~/robot_assignment_ws/src/surveillance_bot/maps
cp my_map.pgm ~/robot_assignment_ws/src/surveillance_bot/maps
```
This creates:
- my_map.pgm -> grayscale occupancy grid
- my_map.yaml -> resolution, origin, thresholds
7. Close all nodes/ terminals to prepare for Navigation and Testing.
------------------
### Navigation Phase 
1. In Terminal 1
```bash
cd ~/robot_assignment_ws/ 
./startWorld
```
2. In Terminal 2 - we will save the nav node code and run the navigation 
```bash
nano ~/robot_assignment_ws/src/surveillance_bot/src/navigator_node.py #copy the code from the repo
chmod +x ~/robot_assignment_ws/src/surveillance_bot/src/navigator_node.py
nano ~/robot_assignment_ws/src/surveillance_bot/launch/nav.launch #copy the code from the repo
roslaunch surveillance_bot nav.launch
```
3. Here you will be prompted to type your coordinates, the robot will move, the PRM path is then followed!

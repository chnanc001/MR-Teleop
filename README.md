# MR-Teleop
TO INSTALL DVRK STUFF:
source /opt/ros/melodic/setup.bash 
mkdir ~/catkin_ws                  
cd ~/catkin_ws                     
wstool init src                    
catkin init                        
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release # all code should be compiled in release mode
cd src                             
wstool merge https://raw.githubusercontent.com/jhu-dvrk/dvrk-ros/devel/dvrk_ros.rosinstall
wstool up       
catkin build --summary  
source ~/catkin_ws/devel/setup.bash 
cd catkin_ws/src/cisst-saw
git clone https://github.com/jhu-saw/sawSocketStreamer.git
cd sawSocketStreamer/
git checkout devel 
catkin build 

TO START ROSCORE:
source ~/catkin_ws/devel/setup.bash
roscore

TO RUN SIMULATION:    
source ~/catkin_ws/devel/setup.bash 
roscd dvrk_config/
cd socket-streamer/
rosrun dvrk_robot dvrk_console_json -j  ../console/console-PSM1_KIN_SIMULATED.json -m manager-socket-streamer-PSM1.json

TO RUN REAL PSM1:    
source ~/catkin_ws/devel/setup.bash 
roscd dvrk_config/
cd socket-streamer/
rosrun dvrk_robot dvrk_console_json -j ../jhu-daVinci/console-PSM1.json -m manager-socket-streamer-PSM1.json

TO RUN ARM SIMULATION:
source ~/catkin_ws/devel/setup.bash 
roslaunch dvrk_robot dvrk_arm_rviz_only.launch arm:=PSM1



---- RUNNING OMNI WITH PSM1 ----

WITH SIMULATED PSM1:    - operator and clutch buttons on omni
source ~/catkin_ws/devel/setup.bash
roscd dvrk_config/
cd console/
rosrun dvrk_robot dvrk_console_json -j console-MTMR-Omni-PSM1_KIN_SIMULATED-Teleop.json

WITH REAL PSM1:    - operator and clutch buttons are foot pedals
source ~/catkin_ws/devel/setup.bash
roscd dvrk_config/
cd jhu-daVinci/
rosrun dvrk_robot dvrk_console_json -j console-Omni-PSM1-udp.json 

WITH REAL PSM1 AND dVRK SENDING MSGS:
source ~/catkin_ws/devel/setup.bash
roscd dvrk_config/
cd socket-streamer/
rosrun dvrk_robot dvrk_console_json -j ../jhu-daVinci/console-Omni-PSM1-udp.json -m manager-socket-streamer-PSM1.json

TO RUN ARM SIMULATION:
source ~/catkin_ws/devel/setup.bash 
roslaunch dvrk_robot dvrk_arm_rviz_only.launch arm:=PSM1



---- RUNNING MTM WITH PSM1 ----

TO RUN MTM PROCESS:
source ~/catkin_ws/devel/setup.bash
roscd dvrk_config/
cd jhu-dVRK-Si/
rosrun dvrk_robot dvrk_console_json -j console-MTMR-PSM1_ROS-Teleop.json

TO RUN REAL PSM1 PROCESS:
source ~/catkin_ws/devel/setup.bash 
roscd dvrk_config/
cd socket-streamer/
rosrun dvrk_robot dvrk_console_json -j ../jhu-daVinci/console-PSM1.json -m manager-socket-streamer-PSM1.json

**make sure you are running the console for the actual PSM
*make sure the neighboring computer is disconnected from the controller box

-- USEFUL NOTES --
can lock rotation of the psm via GUI or through config file (.json that are used to run)
can adjust scaling through GUI or config file
can see buttons status on GUI


(if get some error about power, try the following 2:)

TO CLOSE RELAYS (over udp):
qlacommand -pudp -c close-relays


qlacommand -c reset-encoder-preload -pudp


REBOOT CONTROLLERS:
qlacommand -c reboot



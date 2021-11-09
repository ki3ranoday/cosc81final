# cosc81final
Final Project for the FA 21 COSC 81/281 Principles of Robotics class.



## How to install GMapping
1. run `docker-compose exec ros bash` then `sudo apt install ros-melodic-gmapping`
2. then run `rosrun gmapping slam_gmapping scan:=base_scan` if you are working in simulation or `rosrun gmapping slam_gmapping scan:=scan` if working on the real robot
3. this will listen for laser readings and publish the occupancy grid as it comes in

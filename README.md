# cosc81final
Final Project for the FA 21 COSC 81/281 Principles of Robotics class.



## How to install GMapping
1. run `docker-compose exec ros bash` then `sudo apt install ros-melodic-gmapping`
2. then run `rosrun gmapping slam_gmapping scan:=base_scan _xmax:=10 _ymax:=10 _delta:=0.1` if you are working in simulation or `rosrun gmapping slam_gmapping scan:=scan _xmax:=10 _ymax:=10 _delta:=0.1` if working on the real robot
3. this will listen for laser readings and publish the occupancy grid as it comes in

## How to run the Change Detector
For detailed instructions, go [here](https://www.overleaf.com/project/5da4b7b87a7dd90001b4618a)
1. If working on the real robot, connect to the robot over ssh:
   1. `docker-compose up --build`
   2. `docker-compose exec ros bash` then `husarnet daemon`
   3. move the code from your computer to the rosbot by running `docker-compose exec ros bash` then `scp -r cosc81final/*.py husarion@husarion:~/cosc81final/`
   4. connect to the robot with `docker-compose exec ros bash` then `ssh husarion@husarion`
   5. Then in the ssh connection:
      1. `screen -R core` then `roslaunch rosbot_ekf all.launch`
      2. `screen -R sensors` then `roslaunch rosbot_description rosbot_hardware.launch`
      3. `screen -R gmapping` then `rosrun gmapping slam_gmapping scan:=scan _xmax:=10 _ymax:=10 _delta:=0.1`
      4. `screen -R python` then `python cosc81final/change_detector.py scan`

## Visualizing on docker
1. In a docker bash terminal, export these three commands:

```
   export ROS_MASTER_URI=http://husarion:11311
   export ROS_IPV6=on
   export ROS_HOSTNAME=
```
1. The hostname should be: kieran, jonah, nick or ryan depending on which computer we are running from


# Drone-Rescue-Mission
Intro to Robotics Final Project 

This project involved the Parrot Bebop 2 drone and a motion catpure system for accurate flight commands. The project was to have the drone map its envrionment through AR tags, run an A* path planning algorithim to find the landing destination, and then follow that discrete path through an inverse kinematics feedback loop.

## Dependencies

* [ROS Kinetic](http://wiki.ros.org/kinetic) - The backbone for flight conrols and connection
* [AR Track Alvar](http://wiki.ros.org/ar_track_alvar) - The AR tags for mapping the room instead of object detection.
* [ROS vrpn client](http://wiki.ros.org/vrpn_client_ros) - For connecting to the motion capture system
* [Parrot bebop_autonomy](https://bebop-autonomy.readthedocs.io/en/latest/) - Used to control the drone and send it proper commands


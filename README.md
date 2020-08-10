# Flipkart Robotics Challenge-Intelligent Picking ( Team- Excalibur)
The problem statement tries to replicate the quintessential warehouse problem of picking in which the
participants are supposed to build their own robot hardware and software (collectively, a “Robot”) that is
capable of doing general tasks of picking items from a pick area and place them into a cell in th drop/stow
area.

## Project Overview
* The robot consists of - MASTER and SLAVE(in case the distance between pick up and drop area is large).
* The master robot has Single Board Computer(SBC) + microcontroller ; slave has a single microcontroller board.
* The SBC is in communication with PC through TCP/IP protocol.ROS would be installed on both the SBC and the PC, but all nodes would be configured to use the same master, via     ROS_MASTER_URI which would be on SBC.
* The execution of the robot is as follows-

  Grasping: Once we place it at the edge of the pick area, (using the 2D camera) the images and point cloud (depth camera) of the object to be picked would be taken. Using         Grasping Point Detection Algorithm grasping point would be detected.The The image will also be fed to the CNN network which object it is.

  Arm movements: Once we get the grasping point, motion planning for the gripper to reach that position is performed by the MoveIt package. The feedback from the force             sensors would be used to control the grip force applied by a servo to pick the object. Once the object gets picked, MoveIt package would be used to place it in the basket of a   slave robot. 
  
  Localization and Navigation: Using the internal sensors(IMUs, motor encoders, and laser scan) for state estimation and updation,SLAM would be performed slave bot before the     robot can starts with picking the items. Once a map of the environment is ready, the robot will navigate to the pick area(using DWA algorithm).
  After these operations Master bot would navigate to the drop area, followed by slave bot.This communication is also achieved using SSH.
  
  Stowing: As soon as the robot reaches the edge of the drop area, a 2D image would be taken. Image manipulation would be done(using OpenCV and CV_Bridge package) to get the       coordinates of each small square grid and navigation would be performed to that point thereafter placing the object one by one in all of the small square grids.  



## Team
1.Dikshant
2.Shreeya Shrikant Athaley
3.Leena Chaudhari
4.Navjit Debnath
5.Tejal Ashwini Barnwal







#include <ros/ros.h>


#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

void openGripper(trajectory_msgs::JointTrajectory& posture)
{

  posture.joint_names.resize(4);
  posture.joint_names[0]="base_gear1_joint";
  posture.joint_names[1]="finger_gear1_joint";
  posture.joint_names[2]="base_gear2_joint";
  posture.joint_names[3]="finger_gear2_joint";

  posture.points.resize(1);
  posture.points[0].positions.resize(4);
  posture.points[0].positions[0]= 1.19;
  posture.points[0].positions[1]= -1.23;
  posture.points[0].positions[2]= 1.19;
  posture.points[0].positions[3]= -1.23;
  posture.points[0].time_from_start = ros::Duration(7);

}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{

  posture.joint_names.resize(4);
  posture.joint_names[0]="base_gear1_joint";
  posture.joint_names[1]="finger_gear1_joint";
  posture.joint_names[2]="base_gear2_joint";
  posture.joint_names[3]="finger_gear2_joint";

  posture.points.resize(1);
  posture.points[0].positions.resize(4);
  posture.points[0].positions[0]= -0.53;
  posture.points[0].positions[1]= 0.43;
  posture.points[0].positions[2]= -0.53;
  posture.points[0].positions[3]= 0.43;
  posture.points[0].time_from_start = ros::Duration(7);
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
  // BEGIN_SUB_TUTORIAL pick1
  // Create a vector of grasps to be attempted, currently only creating single grasp.
  // This is essentially useful when using a grasp generator to generate and test multiple grasps.
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);
  std::cout << "A GRASP OBJECT IS BEEN DEFINED: \n" ;
  // Setting grasp pose
  // ++++++++++++++++++++++
  // This is the pose of panda_link8. |br|
  // From panda_link8 to the palm of the eef the distance is 0.058, the cube starts 0.01 before 5.0 (half of the length
  // of the cube). |br|
  // Therefore, the position for panda_link8 = 5 - (length of cube/2 - distance b/w panda_link8 and palm of eef - some
  // extra padding)
  std::cout << "GRASP_POSE IS STARTING TO BE DEFINED: \n" ;
  grasps[0].grasp_pose.header.frame_id = "gripper_holder_link"; //GRIPPER_HOLDER
  std::cout << "GRASP_POSE FRAME DECLARED: \n" ;
  tf2::Quaternion orientation;
  orientation.setRPY(-M_PI / 2, 0, 0);
  std::cout << "GRASP_POSE ORIENTATION IS SET: \n" ;
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  grasps[0].grasp_pose.pose.position.x = 0;
  grasps[0].grasp_pose.pose.position.y = 0;
  grasps[0].grasp_pose.pose.position.z = 0.063;
  std::cout << "GRASP_POSE IS COMPLETELY DEFINED WRT BASE_LINK: \n" ;

  // Setting pre-grasp approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  std::cout << "PRE GRASP APPROACH IS STARTING TO BE DEFINED: \n" ;
  grasps[0].pre_grasp_approach.direction.header.frame_id = "base_link";
  std::cout << "PRE GRASP APPROACH FRAME DECLARED: \n" ;
  /* Direction is set as positive x axis */
  std::cout << "PRE GRASP APPROACH DIRECTION IS STARTING TO BE DEFINED: \n" ;
  grasps[0].pre_grasp_approach.direction.vector.z = -1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.00005;
  grasps[0].pre_grasp_approach.desired_distance = 0.0000564154;
  std::cout << "PRE GRASP APPROACH COMPLETELY DEFINED: \n" ;
  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  std::cout << "POST GRASP APPROACH IS STARTING TO BE DEFINED: \n" ;
  grasps[0].post_grasp_retreat.direction.header.frame_id = "base_link";
  std::cout << "POST GRASP APPROACH FRAME DECLARED: \n" ;
  /* Direction is set as positive z axis */
  std::cout << "POST GRASP APPROACH DIRECTION IS STARTING TO BE DEFINED: \n" ;
  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.00005;
  grasps[0].post_grasp_retreat.desired_distance = 0.0000564154;
  std::cout << "POST GRASP APPROACH COMPLETELY DEFINED: \n" ;

  // Setting posture of eef before grasp
  // +++++++++++++++++++++++++++++++++++
  std::cout << "PRE GRASP POSTURE TO BE DEFINED: \n" ;
  openGripper(grasps[0].pre_grasp_posture);
  std::cout << "PRE GRASP POSTURE IS COMPLETELY DEFINED : \n" ;
  // END_SUB_TUTORIAL

  // BEGIN_SUB_TUTORIAL pick2
  // Setting posture of eef during grasp
  // +++++++++++++++++++++++++++++++++++

  std::cout << "GRASP POSTURE TO BE DEFINED: \n" ;
  closedGripper(grasps[0].grasp_posture);
  std::cout << "GRASP POSTURE IS COMPLETELY DEFINED: \n" ;
  // END_SUB_TUTORIAL

  // BEGIN_SUB_TUTORIAL pick3
  // Set support surface as table1.
  std::cout << "TO BE PICKED FROM TABLE1: \n" ;
  move_group.setSupportSurfaceName("table1");
  // Call pick to pick up the object using the grasps given
  std::cout << "MOVE_GROUP PICK IS STARTING: \n" ;
  move_group.pick("object", grasps);
  std::cout << "MOVE_GROUP PICK IS COMPLETED: \n" ;
  // END_SUB_TUTORIAL
}

void place(moveit::planning_interface::MoveGroupInterface& group)
{
  // BEGIN_SUB_TUTORIAL place
  // TODO(@ridhwanluthra) - Calling place function may lead to "All supplied place locations failed. Retrying last
  // location in
  // verbose mode." This is a known issue and we are working on fixing it. |br|
  // Create a vector of placings to be attempted, currently only creating single place location.
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  // Setting place location pose
  // +++++++++++++++++++++++++++
  place_location[0].place_pose.header.frame_id = "base_link";
  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, 0);
  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

  /* While placing it is the exact location of the center of the object. */
  place_location[0].place_pose.pose.position.x = -0.00596;
  place_location[0].place_pose.pose.position.y = 0.849;
  place_location[0].place_pose.pose.position.z = 0.616;

  // Setting pre-place approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].pre_place_approach.direction.header.frame_id = "base_link";
  /* Direction is set as negative z axis */
  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.0;
  place_location[0].pre_place_approach.desired_distance = 0.0000001;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].post_place_retreat.direction.header.frame_id = "base_link";
  /* Direction is set as negative y axis */
  place_location[0].post_place_retreat.direction.vector.z = 1.0;
  place_location[0].post_place_retreat.min_distance = 0.1;
  place_location[0].post_place_retreat.desired_distance = 0.25;

  // Setting posture of eef after placing object
  // +++++++++++++++++++++++++++++++++++++++++++
  /* Similar to the pick case */
  openGripper(place_location[0].post_place_posture);

  // Set support surface as table2.
  group.setSupportSurfaceName("table2");
  // Call place to place the object using the place locations given.
  group.place("object", place_location);
  // END_SUB_TUTORIAL
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  // BEGIN_SUB_TUTORIAL table1
  //
  // Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold 3 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(3);

  // Add the first table where the cube will originally be kept.
  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "base_link";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.2;
  collision_objects[0].primitives[0].dimensions[1] = 0.4;
  collision_objects[0].primitives[0].dimensions[2] = 0.5;

  /* Define the pose of the table. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.5;
  collision_objects[0].primitive_poses[0].position.y = 0;
  collision_objects[0].primitive_poses[0].position.z = 0.25;
  // END_SUB_TUTORIAL

  collision_objects[0].operation = collision_objects[0].ADD;

  // BEGIN_SUB_TUTORIAL table2
  // Add the second table where we will be placing the cube.
  collision_objects[1].id = "table2";
  collision_objects[1].header.frame_id = "base_link";

  /* Define the primitive and its dimensions. */
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.3;
  collision_objects[1].primitives[0].dimensions[1] = 0.4;
  collision_objects[1].primitives[0].dimensions[2] = 0.1;

  /* Define the pose of the table. */
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0;
  collision_objects[1].primitive_poses[0].position.y = 0.8;
  collision_objects[1].primitive_poses[0].position.z = 0.05;
  // END_SUB_TUTORIAL

  collision_objects[1].operation = collision_objects[1].ADD;

  // BEGIN_SUB_TUTORIAL object
  // Define the object that we will be manipulating
  collision_objects[2].header.frame_id = "base_link";
  collision_objects[2].id = "object";

  /* Define the primitive and its dimensions. */
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.15;
  collision_objects[2].primitives[0].dimensions[1] = 0.1;
  collision_objects[2].primitives[0].dimensions[2] = 0.15;

  /* Define the pose of the object. */
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = 0.5;
  collision_objects[2].primitive_poses[0].position.y = 0;
  collision_objects[2].primitive_poses[0].position.z = 0.575;
  // END_SUB_TUTORIAL

  collision_objects[2].operation = collision_objects[2].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "apna_arm_pick_place");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::cout << "node is intialized: \n" ;
  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  std::cout << "planning_scene_interace \n" ;
  moveit::planning_interface::MoveGroupInterface group("flip_arm");
  std::cout << "group is set \n";
  group.setPlanningTime(70.0);
  std::cout << "planning time set \n";
  addCollisionObjects(planning_scene_interface);
  std::cout << "collision object added \n";
  // Wait a bit for ROS things to initialize
  ros::WallDuration(1.0).sleep();

  pick(group);
  std::cout << "object is getting pickedup \n";
  ros::WallDuration(1.0).sleep();

  place(group);
  std::cout << "object is getting placed \n";
  ros::waitForShutdown();
  return 0;
}

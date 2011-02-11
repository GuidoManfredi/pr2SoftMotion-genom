#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
// Arms pose
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
// Head pose
#include <pr2_controllers_msgs/PointHeadAction.h>
// Torso pose
#include <pr2_controllers_msgs/SingleJointPositionAction.h>
// Grippers pose
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
// Base pose
//#include <geometry_msgs/Twist.h>
//#include <tf/transform_listener.h>


typedef actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction > ArmClient;
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::SingleJointPositionAction> TorsoClient;
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;

class Pr2Pose
{
private:
  // node
  ros::NodeHandle nh;
 
  // action clients
  ArmClient* r_arm_client;
  ArmClient* l_arm_client;
  TorsoClient* torso_client;
  GripperClient* r_gripper_client;
  GripperClient* l_gripper_client;
  // publishers
  ros::Publisher head_pub;
  //ros::Publisher cmd_vel_pub;
  // listeners
  //tf::TransformListener listenerOdom;

public:
  // Initialize the action clients and wait for action servers to come up
  Pr2Pose(ros::NodeHandle &nodeHandle) 
  {
    nh = nodeHandle;

    // tell the action client that we want to spin threads by default
    r_arm_client = new ArmClient("r_arm_controller/joint_trajectory_action", true);
    l_arm_client = new ArmClient("l_arm_controller/joint_trajectory_action", true);
    torso_client= new TorsoClient("torso_controller/position_joint_action", true);
    r_gripper_client = new GripperClient("r_gripper_controller/gripper_action", true);
    l_gripper_client = new GripperClient("l_gripper_controller/gripper_action", true);

    // wait for action server to come up
    bool ready= false;
    while(!ready) 
    {
      if(!r_arm_client->waitForServer(ros::Duration(5.0)))
        ROS_INFO("Waiting for right arm action server");
      else if(!l_arm_client->waitForServer(ros::Duration(5.0)))
        ROS_INFO("Waiting for left arm action server");
      else if(!torso_client->waitForServer(ros::Duration(5.0)))
        ROS_INFO("Waiting for torso action server");      
      else if(!r_gripper_client->waitForServer(ros::Duration(5.0)))
        ROS_INFO("Waiting for right gripper action server");      
      else if(!l_gripper_client->waitForServer(ros::Duration(5.0)))
        ROS_INFO("Waiting for left gripper action server");      
      else
        ready= true;
    }
    head_pub = nh.advertise<trajectory_msgs::JointTrajectory>("head_traj_controller/command", 2);
    //cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  }

  // Clean up
  ~Pr2Pose()
  {
    delete r_arm_client;
    delete l_arm_client;
    delete torso_client;
    delete r_gripper_client;
    delete l_gripper_client;
  }

  /**
  * La variable pose est un tableau de 19 doubles organise comme suit:
  * pose[0]= "torso"
  * pose[1]= "head_pan"
  * pose[2]= "head_tilt"
  * pose[3]= "tilt_laser"                 NON UTILISE
  * pose[4]= "r_shoulder_pan_joint"
  * pose[5]= "r_shoulder_lift_joint"
  * pose[6]= "r_upper_arm_roll_joint"
  * pose[7]= "r_elbow_flex_joint"
  * pose[8]= "r_forearm_roll_joint"
  * pose[9]= "r_wrist_flex_joint"
  * pose[10]= "r_wrist_roll_joint"
  * pose[11]= "r_gripper"
  * pose[12]= "r_gripper2"                NON UTILISE
  * pose[13]= "l_shoulder_pan_joint"
  * pose[14]= "l_shoulder_lift_joint"
  * pose[15]= "l_upper_arm_roll_joint"
  * pose[16]= "l_elbow_flex_joint"
  * pose[17]= "l_forearm_roll_joint"
  * pose[18]= "l_wrist_flex_joint"
  * pose[19]= "l_wrist_roll_joint"
  * pose[20]= "l_gripper"
  * pose[21]= "l_gripper2"                NON UTILISE
  */
  void sendTrajectory(double* pose, double* velocity, double* acc)
  {
    pr2_controllers_msgs::SingleJointPositionGoal torso_goal= torsoGoal(pose[0]);
    //headGoal use (pose[1], pose[2])
    pr2_controllers_msgs::JointTrajectoryGoal r_arm_goal= armGoal(pose[4],pose[5],pose[6],pose[7],pose[8],pose[9],pose[10],
                                                                  velocity[4],velocity[5],velocity[6],velocity[7],velocity[8],velocity[9],velocity[10],
                                                                  acc[4],acc[5],acc[6],acc[7],acc[8],acc[9],acc[10],'r');
    pr2_controllers_msgs::Pr2GripperCommandGoal r_gripper_goal= gripperGoal(pose[11]);
    pr2_controllers_msgs::JointTrajectoryGoal l_arm_goal= armGoal(pose[13],pose[14],pose[15],pose[16],pose[17],pose[18],pose[19],
                                                                  velocity[13],velocity[14],velocity[15],velocity[16],velocity[17],velocity[18],velocity[19],
                                                                  acc[13],acc[14],acc[15],acc[16],acc[17],acc[18],acc[19],'l');
    pr2_controllers_msgs::Pr2GripperCommandGoal l_gripper_goal= gripperGoal(pose[20]);

/*
    r_arm_goal.trajectory.header.stamp = ros::Time::now();
    l_arm_goal.trajectory.header.stamp = ros::Time::now();
    head_goal.trajectory.header.stamp = ros::Time::now();
    torso_goal.trajectory.header.stamp = ros::Time::now();
    r_gripper_goal.trajectory.header.stamp = ros::Time::now();
    l_gripper_goal.trajectory.header.stamp = ros::Time::now();
*/

    r_arm_client->sendGoal(r_arm_goal);
    l_arm_client->sendGoal(l_arm_goal);
    torso_client->sendGoal(torso_goal);
    r_gripper_client->sendGoal(r_gripper_goal);
    l_gripper_client->sendGoal(l_gripper_goal);
    //headGoal(pose[1], pose[2]);
    //driveBaseTo(0.0, 0.0, 0.0);
  }

  pr2_controllers_msgs::JointTrajectoryGoal armGoal(double q1, double q2, double q3, double q4, double q5, double q6, double q7,
                                                    double vq1, double vq2, double vq3, double vq4, double vq5, double vq6, double vq7,
                                                    double aq1, double aq2, double aq3, double aq4, double aq5, double aq6, double aq7,
                                                    char arm)
  {
    // goal variable
    pr2_controllers_msgs::JointTrajectoryGoal goal;

    // Joint names
    if(arm == 'r'){
      goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
      goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
      goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
      goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
      goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
      goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
      goal.trajectory.joint_names.push_back("r_wrist_roll_joint");
    }
    else if(arm == 'l'){
      goal.trajectory.joint_names.push_back("l_shoulder_pan_joint");
      goal.trajectory.joint_names.push_back("l_shoulder_lift_joint");
      goal.trajectory.joint_names.push_back("l_upper_arm_roll_joint");
      goal.trajectory.joint_names.push_back("l_elbow_flex_joint");
      goal.trajectory.joint_names.push_back("l_forearm_roll_joint");
      goal.trajectory.joint_names.push_back("l_wrist_flex_joint");
      goal.trajectory.joint_names.push_back("l_wrist_roll_joint");
    }

    goal.trajectory.points.resize(1);

    // Positions
    int ind = 0;
    goal.trajectory.points[ind].positions.resize(7);
    goal.trajectory.points[ind].positions[0] = q1;
    goal.trajectory.points[ind].positions[1] = q2;
    goal.trajectory.points[ind].positions[2] = q3;
    goal.trajectory.points[ind].positions[3] = q4;
    goal.trajectory.points[ind].positions[4] = q5;
    goal.trajectory.points[ind].positions[5] = q6;
    goal.trajectory.points[ind].positions[6] = q7;

    // Velocities
    goal.trajectory.points[ind].velocities.resize(7);
/*
    goal.trajectory.points[ind].velocities[0] = vq1;
    goal.trajectory.points[ind].velocities[1] = vq2;
    goal.trajectory.points[ind].velocities[2] = vq3;
    goal.trajectory.points[ind].velocities[3] = vq4;
    goal.trajectory.points[ind].velocities[4] = vq5;
    goal.trajectory.points[ind].velocities[5] = vq6;
    goal.trajectory.points[ind].velocities[6] = vq7;
*/
    // Accelerations
    goal.trajectory.points[ind].accelerations.resize(7);
/*
    goal.trajectory.points[ind].accelerations[0] = aq1;
    goal.trajectory.points[ind].accelerations[1] = aq2;
    goal.trajectory.points[ind].accelerations[2] = aq3;
    goal.trajectory.points[ind].accelerations[3] = aq4;
    goal.trajectory.points[ind].accelerations[4] = aq5;
    goal.trajectory.points[ind].accelerations[5] = aq6;
    goal.trajectory.points[ind].accelerations[6] = aq7;
*/

    // To be reached 0.01 second after starting along the trajectory
    //goal.trajectory.points[ind].time_from_start = ros::Duration(0.01);

    //we are done; return the goal
    return goal;
  }

  pr2_controllers_msgs::SingleJointPositionGoal torsoGoal(double height){

    pr2_controllers_msgs::SingleJointPositionGoal goal;
    goal.position = height;  
    //goal.min_duration = ros::Duration(1.0);
    goal.max_velocity = 1.0;
    
    return goal;
  }

  pr2_controllers_msgs::Pr2GripperCommandGoal gripperGoal(double position){
    pr2_controllers_msgs::Pr2GripperCommandGoal goal;
    goal.command.position = position;
    goal.command.max_effort = -1.0;  // Do not limit effort (negative)
    
    return goal;
  }

  void headGoal(double a, double b)
  { 
    // Trajectory
    trajectory_msgs::JointTrajectory traj;

    // Joint names
    traj.joint_names.push_back("head_pan_joint");
    traj.joint_names.push_back("head_tilt_joint");

    traj.points.resize(1);

    // Positions
    int ind = 0;
    traj.points[ind].positions.resize(2);
    traj.points[ind].positions[0] = a;
    traj.points[ind].positions[1] = b;

    traj.points[ind].velocities.resize(2);
    for (size_t j = 0; j < 2; ++j)
    {
      traj.points[ind].velocities[j] = 0.1;
    }
    // To be reached 1 second after starting along the trajectory
    traj.points[ind].time_from_start = ros::Duration(1.0);

    head_pub.publish(traj); 
  }  
/*
  bool driveBaseTo(double x, double y, double w)
  {
    //wait for the listener to get the first message
    listenerOdom.waitForTransform("base_footprint", "odom_combined", 
                               ros::Time(0), ros::Duration(1.0));
    
    //we will record transforms here
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    //record the starting transform from the odometry to the base frame
    listenerOdom.lookupTransform("base_footprint", "odom_combined", 
                              ros::Time(0), start_transform);
    
    //we will be sending commands of type "twist"
    geometry_msgs::Twist base_cmd;
    //the command will be to go forward at 0.25 m/s
    base_cmd.linear.y = base_cmd.angular.z = 0;
    base_cmd.linear.x = 0.25;
    
    ros::Rate rate(10.0);
    bool done = false;
    while (!done && nh.ok())
    {
      //send the drive command
      cmd_vel_pub.publish(base_cmd);
      rate.sleep();
      //get the current transform
      try
      {
        listenerOdom.lookupTransform("base_footprint", "odom_combined", 
                                  ros::Time(0), current_transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
        break;
      }
      //see how far we've traveled
      tf::Transform relative_transform = 
        start_transform.inverse() * current_transform;
      if( relative_transform.getOrigin().x() == x
        && relative_transform.getOrigin().y() == y
        && relative_transform.getOrigin().w() == w)
        done = true;
    }
    if (done) return true;
    return false;
  }
*/
};

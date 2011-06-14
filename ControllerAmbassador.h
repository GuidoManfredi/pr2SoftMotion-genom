#ifndef PR2SM_CONTROLLERAMBASSADOR
#define PR2SM_CONTROLLERAMBASSADOR

#include "ros/ros.h"
#include "pr2_controllers_msgs/JointTrajectoryControllerState.h"
#include "std_msgs/Float64.h"

#include "pr2_soft_controller/SM_TRAJ_STR_ROS.h"

#include <portLib.h>
#include "server/pr2SoftMotionHeader.h"
#include "pr2SoftMotionConst.h"
#include "pr2SoftMotionAuxFunctions.h"

#include "softMotion/softMotion.h"
#include "softMotion/softMotionStruct.h"
#include "softMotion/softMotionStructGenom.h"

#include <vector>
#include <string>

class ControllerAmbassador
{
public:
  ControllerAmbassador(PR2SM_ROBOT_PART_ENUM robotPart, ros::NodeHandle* nh);
  ~ControllerAmbassador();
  bool trackQ(SM_TRAJ_STR *sm_traj_str, int *report);
  bool gotoQ(PR2SM_QSTR *qGoto, int *report);
  bool monitorTraj();
  void publishTimeScale();
  void setRatios(double accVel, double jerkAcc);

protected:
  void computeGoto(PR2SM_QSTR *qGoto, int *report);
  bool loadTraj(SM_TRAJ_STR* smTraj, int debut, int fin);
  void sendTraj();
  void saveTimeCB(const pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr& msg);
  void savePoseCB(const sensor_msgs::JointStateConstPtr& msg);
  void setMaxVelVect(); 

  PR2SM_ROBOT_PART_ENUM robotPart_;
  std::string controllerNames_;
  ros::Publisher timescale_pub_;
  ros::Publisher command_pub_;
  ros::Subscriber state_sub_;
  ros::Subscriber joint_state_sub_;

  POSTER_ID posterId_;
  int currentTrajId_;
  pr2_soft_controller::SM_TRAJ_STR_ROS smTrajROS_;
  SM_TRAJ_STR* smTraj_;
  double time_from_start_;
  double currTime_;
  double motionDuration_;
  int debut_, fin_;
  int nbJoints_;
  double* vect_current_pose_;
  double* vect_J_max_;
  double* vect_V_max_;
  double* vect_A_max_;
  double accVelRatio_;
  double jerkAccRatio_;
};


#endif


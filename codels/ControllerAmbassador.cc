#include "ControllerAmbassador.h"

ControllerAmbassador::ControllerAmbassador(PR2SM_ROBOT_PART_ENUM robotPart, ros::NodeHandle* nh)
{
  robotPart_ = robotPart;

  switch (robotPart_){
  case BASE:
    ac = new actionlib::SimpleActionClient<soft_move_base::SoftMoveBaseAction>("soft_move_base", true);
    timescale_pub_= nh->advertise<std_msgs::Float64>("soft_move_base/timescale", 1);
    debut_ = 0;
    fin_ = 5;
    break; 
  case TORSO:
    command_pub_= nh->advertise<pr2_soft_controller::SM_TRAJ_STR_ROS>("torso_soft_controller/command", 1);
    timescale_pub_= nh->advertise<std_msgs::Float64>("torso_soft_controller/timescale", 1);
    state_sub_= nh->subscribe("torso_soft_controller/state", 1, &ControllerAmbassador::saveTimeCB, this);
    debut_ = 6;
    fin_ = 6;
    break;
  case HEAD:
    command_pub_= nh->advertise<pr2_soft_controller::SM_TRAJ_STR_ROS>("head_soft_controller/command", 1);
    timescale_pub_= nh->advertise<std_msgs::Float64>("head_soft_controller/timescale", 1);
    state_sub_= nh->subscribe("head_soft_controller/state", 1, &ControllerAmbassador::saveTimeCB, this);
    debut_ = 7;
    fin_ = 8;
    break;
  case RARM:
    command_pub_= nh->advertise<pr2_soft_controller::SM_TRAJ_STR_ROS>("r_arm_soft_controller/command", 1);
    timescale_pub_= nh->advertise<std_msgs::Float64>("r_arm_soft_controller/timescale", 1);
    state_sub_= nh->subscribe("r_arm_soft_controller/state", 1, &ControllerAmbassador::saveTimeCB, this);
    debut_ = 10;
    fin_ = 16;
    break;
  case LARM:
    command_pub_= nh->advertise<pr2_soft_controller::SM_TRAJ_STR_ROS>("l_arm_soft_controller/command", 1);
    timescale_pub_= nh->advertise<std_msgs::Float64>("l_arm_soft_controller/timescale", 1);
    state_sub_= nh->subscribe("l_arm_soft_controller/state", 1, &ControllerAmbassador::saveTimeCB, this);
    debut_ = 19;
    fin_ = 25;
    break;
  case PR2SYN:
    command_pub_= nh->advertise<pr2_soft_controller::SM_TRAJ_STR_ROS>("pr2_soft_controller/command", 1);
    timescale_pub_= nh->advertise<std_msgs::Float64>("pr2_soft_controller/timescale", 1);
    state_sub_= nh->subscribe("pr2_soft_controller/state", 1, &ControllerAmbassador::saveTimeCB, this);
    debut_ = 6;
    fin_ = 27;
    break;
  default:
    printf("ERROR: Unknown robot part. No publisher/subscriber created.\n");
  }

  if(robotPart_ == BASE)
    joint_state_sub_ = nh->subscribe("amcl_pose", 1, &ControllerAmbassador::saveBasePoseCB, this);
  else
    joint_state_sub_ = nh->subscribe("joint_states", 1, &ControllerAmbassador::savePoseCB, this);

  currentTrajId_= -1;
  // to be set by findPoster
  posterId_ = NULL;
  time_from_start_ = 0;
  motionDuration_ = 0;

  nbJoints_= fin_ - debut_ + 1;
  vect_current_pose_= (double*)malloc(nbJoints_*sizeof(double));
  vect_J_max_= (double*)malloc(nbJoints_*sizeof(double));
  vect_A_max_= (double*)malloc(nbJoints_*sizeof(double)); 
  vect_V_max_= (double*)malloc(nbJoints_*sizeof(double)); 
  smTraj_ = (SM_TRAJ_STR*)malloc(sizeof(SM_TRAJ_STR));
  
  // compute and set max vel/acc/jerk vectors
  setRatios(1,3);  // default values
}

bool ControllerAmbassador::trackQ(SM_TRAJ_STR *sm_traj_str, int *report)
{
  // load a trajectory from a SM_TRAJ_STR
  if(! loadTraj(sm_traj_str, debut_, fin_) ) 
    return false;
  
  // and send it to the concerned controllers
  sendTraj();

  return true;
}

bool ControllerAmbassador::gotoQ(PR2SM_QSTR *qGoto, int *report)
{
  //setMaxVelVect();
  // compute a trajectory from current
  // position to goal
  computeGoto(qGoto, report);
  // load the former trajectory
  loadTraj(smTraj_, 0, nbJoints_-1);
  // send it to the concerned controller
  sendTraj();

  return true;
}

bool ControllerAmbassador::monitorTraj()
{
  //Publish timescale in case it changed
  publishTimeScale();

  ros::spinOnce();

  if(robotPart_ == BASE)
  {
    if( ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED )
      return true;
  }
  else
  {
    if( fabs(time_from_start_ - motionDuration_) < 0.000001)
      return true; 
  }
    
  return false;
}

void ControllerAmbassador::publishTimeScale()
{
  std_msgs::Float64 timescale;
  //ROS_INFO("using robotPart:%d", robotPart_);
  //minus 1 because ENUMs the ENUM PR2(position 0) is never
  // called.
  timescale.data= SDI_F->timeScale.timescale[robotPart_-1];
  timescale_pub_.publish(timescale);
}

void ControllerAmbassador::setRatios(double accVel, double jerkAcc)
{
  accVelRatio_= accVel;
  jerkAccRatio_= jerkAcc;
  setMaxVelVect();
}

void ControllerAmbassador::computeGoto(PR2SM_QSTR *qGoto, int *report)
{
  double* qGotod= (double*)malloc(PR2SM_NBJOINT*sizeof(double));
  QStr2doubles(qGoto, qGotod);
  
  SM_TRAJ traj;
  std::vector<SM_COND> IC; 
  std::vector<SM_COND> FC;
  std::vector<SM_LIMITS> limits;
  SM_COND cond;
  SM_LIMITS limit; 

  ros::spinOnce();

  if(qGoto->relatif != 0){
    for(int i=debut_, n=0; i<debut_+nbJoints_; ++i, ++n){
      cond.x =  vect_current_pose_[n];
      cond.v =  0.0;
      cond.a = 0.0;
      IC.push_back(cond);

      cond.x = vect_current_pose_[n] + qGotod[i];
      cond.v = 0.0;
      cond.a = 0.0;
      FC.push_back(cond);

      limit.maxJerk = vect_J_max_[n];
      limit.maxAcc  = vect_A_max_[n];
      limit.maxVel  = vect_V_max_[n];
      limits.push_back(limit);
    }
  }
  else {
    for(int i=debut_, n=0; i<debut_+nbJoints_; ++i, ++n) {
      cond.x = vect_current_pose_[n];
      cond.v = 0.0;
      cond.a = 0.0;
      IC.push_back(cond);

      cond.x = qGotod[i];
      cond.v = 0.0;
      cond.a = 0.0;
      FC.push_back(cond);

      limit.maxJerk = vect_J_max_[n];
      limit.maxAcc  = vect_A_max_[n];
      limit.maxVel  = vect_V_max_[n];
      limits.push_back(limit);
    }
  }

  // Compute soft motion between initial and final conditions
  traj.computeTraj(IC, FC, limits, SM_TRAJ::SM_INDEPENDANT);
  traj.convertToSM_TRAJ_STR(smTraj_);
}

bool ControllerAmbassador::loadTraj(SM_TRAJ_STR *smTraj, int debut, int fin)
{
  if(!smTraj){
    printf("Error: No trajectory. Aborting.\n");
    return false;
  }

  ros::spinOnce();

  //fix initial conditions for circular dofs
  // namely wrist and forearm for each arm
  if (robotPart_ == RARM )
  {
    smTraj->traj[14].seg[0].ic_x =  vect_current_pose_[14-debut];
    smTraj->qStart[14] =  vect_current_pose_[14-debut];
    smTraj->traj[16].seg[0].ic_x =  vect_current_pose_[16-debut];
    smTraj->qStart[16] =  vect_current_pose_[16-debut];
  }
  else if( robotPart_ == LARM )
  {
    smTraj->traj[23].seg[0].ic_x =  vect_current_pose_[23-debut];
    smTraj->qStart[23] =  vect_current_pose_[23-debut];
    smTraj->traj[25].seg[0].ic_x =  vect_current_pose_[25-debut];
    smTraj->qStart[22] =  vect_current_pose_[25-debut]; 
  }
  
  // compute duration. This is ugly but we have to make sure the duration
  //has been computed.
  SM_TRAJ tmpMotion;
  tmpMotion.importFromSM_TRAJ_STR( smTraj );
  /* also needed for circular dof ... */
  tmpMotion.computeTimeOnTraj();

  /* Check the initial configuration of the trajectory */
  std::vector<SM_COND> cond;
  tmpMotion.getMotionCond(0.0 , cond);
  for(int i=0; i< nbJoints_; i++) {
    if(fabs(vect_current_pose_[i] - cond[i+debut].x) >  vect_V_max_[i]*0.1) {
      if(robotPart_ != BASE)
      {
	printf("ERROR pr2SoftMotion loadTraj. \
		Invalid init position for robotpart %d joint %d: \
		val=%f current %f initTraj %f threshold %f",
		robotPart_, i, fabs(vect_current_pose_[i] - cond[i+debut].x),
		vect_current_pose_[i], cond[i+debut].x, vect_V_max_[i]*0.1);
	printf("Aborting.\n");
	return false;
      } 
      else
      {
	printf("WARNING pr2SoftMotion loadTraj. \
		Invalid init position, BASE\n");
      }
    }
  }
  
  //save the trajectory duration
  motionDuration_ = tmpMotion.getDuration();
  tmpMotion.convertToSM_TRAJ_STR( smTraj );

  if(SDI_F->motionIsAllowed == GEN_TRUE) {
    // copy of the softmotion trajectory into the ros trajectory
 
    currentTrajId_= smTraj->trajId;

    smTrajROS_.trajId= currentTrajId_;
    //we use only a subset of the total joints set
    smTrajROS_.nbAxis= nbJoints_;
    smTrajROS_.timePreserved= smTraj->timePreserved;
    smTrajROS_.qStart.resize(smTrajROS_.nbAxis);
    smTrajROS_.qGoal.resize(smTrajROS_.nbAxis);
    smTrajROS_.jmax.resize(smTrajROS_.nbAxis);
    smTrajROS_.amax.resize(smTrajROS_.nbAxis);
    smTrajROS_.vmax.resize(smTrajROS_.nbAxis);
    smTrajROS_.traj.resize(smTrajROS_.nbAxis);
    // we need the joints trajectory between debut and fin
    for(int i=debut, n=0; i<fin+1; ++i, ++n){
        smTrajROS_.qStart[n] = smTraj->qStart[i];
        smTrajROS_.qGoal[n] = smTraj->qGoal[i];
        smTrajROS_.jmax[n] = smTraj->jmax[n];
        smTrajROS_.amax[n] = smTraj->amax[n];
        smTrajROS_.vmax[n] = smTraj->vmax[n];
        smTrajROS_.traj[n].nbSeg= smTraj->traj[i].nbSeg;
        smTrajROS_.traj[n].unsused= smTraj->traj[i].unsused;
        smTrajROS_.traj[n].seg.resize(smTrajROS_.traj[n].nbSeg);
       for(int j=0; j<smTraj->traj[i].nbSeg; ++j){
          smTrajROS_.traj[n].seg[j].lpId= smTraj->traj[i].seg[j].lpId;
          smTrajROS_.traj[n].seg[j].unused= 0;
          smTrajROS_.traj[n].seg[j].timeOnTraj= smTraj->traj[i].seg[j].timeOnTraj;
          smTrajROS_.traj[n].seg[j].time= smTraj->traj[i].seg[j].time;
          smTrajROS_.traj[n].seg[j].ic_a= smTraj->traj[i].seg[j].ic_a;
          smTrajROS_.traj[n].seg[j].ic_v= smTraj->traj[i].seg[j].ic_v;
          smTrajROS_.traj[n].seg[j].ic_x= smTraj->traj[i].seg[j].ic_x;
          smTrajROS_.traj[n].seg[j].jerk= smTraj->traj[i].seg[j].jerk;
        }
    }
  }


  return true;
}

void ControllerAmbassador::sendTraj()
{
  if(robotPart_ == BASE)
  {
    soft_move_base::SoftMoveBaseGoal goal;
    goal.traj = smTrajROS_;
    ac->sendGoal(goal);
  }
  else
  {
    command_pub_.publish(smTrajROS_); 
  }
}

void ControllerAmbassador::saveTimeCB(const pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr& msg)
{ 
  time_from_start_= msg->actual.time_from_start.toSec();
}

// Saves the current pose of the robot in a vector
void ControllerAmbassador::saveBasePoseCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  vect_current_pose_[0]= msg->pose.pose.position.x;			// base
  vect_current_pose_[1]= msg->pose.pose.position.y;			// base
  vect_current_pose_[2]= 0.0;						// base
  
  vect_current_pose_[3]= 0.0;						// base
  vect_current_pose_[4]= 0.0;						// base
  vect_current_pose_[5]= tf::getYaw(msg->pose.pose.orientation);	// base
}

// Saves the current pose of the robot in a vector
void ControllerAmbassador::savePoseCB(const sensor_msgs::JointStateConstPtr& msg)
{
  switch (robotPart_){
    case TORSO:
      vect_current_pose_[0]=  msg->position[12];  // torso
      break;
    case HEAD:
      vect_current_pose_[0]=  msg->position[13];  // head pan
      vect_current_pose_[1]=  msg->position[14];  // head tilt
      break;
    case RARM:
      vect_current_pose_[0]=  msg->position[17];  // r shoulder pan
      vect_current_pose_[1]=  msg->position[18];  // r shoulder lift
      vect_current_pose_[2]=  msg->position[16];  // r upper arm roll
      vect_current_pose_[3]=  msg->position[20];  // r elbow flex
      vect_current_pose_[4]=  msg->position[19];  // r forearm roll
      vect_current_pose_[5]=  msg->position[21];  // r wrist flex
      vect_current_pose_[6]=  msg->position[22];  // r wrist roll
      break;
    case LARM:
      vect_current_pose_[0]=  msg->position[29];  // l shoulder pan
      vect_current_pose_[1]=  msg->position[30];  // l shoulder lift
      vect_current_pose_[2]=  msg->position[28];  // l upper arm roll
      vect_current_pose_[3]=  msg->position[32];  // l elbow flex
      vect_current_pose_[4]=  msg->position[31];  // l forearm roll
      vect_current_pose_[5]=  msg->position[33];  // l wrist flex
      vect_current_pose_[6]=  msg->position[34];  // l wrist roll
      break;
    case PR2SYN:
      vect_current_pose_[0]=  msg->position[12];  // torso
      vect_current_pose_[1]=  msg->position[13];  // head pan
      vect_current_pose_[2]=  msg->position[14];  // head tilt
      vect_current_pose_[3]=  msg->position[15];  // laser
      vect_current_pose_[4]=  msg->position[17];  // r shoulder pan
      vect_current_pose_[5]=  msg->position[18];  // r shoulder lift
      vect_current_pose_[6]=  msg->position[16];  // r upper arm roll
      vect_current_pose_[7]=  msg->position[20];  // r elbow flex
      vect_current_pose_[8]=  msg->position[19];  // r forearm roll
      vect_current_pose_[9]=  msg->position[21];  // r wrist flex
      vect_current_pose_[10]=  msg->position[22];  // r wrist roll
      vect_current_pose_[11]=  msg->position[23];  // r gripper
      vect_current_pose_[12]=  msg->position[24];  // r gripper false
      vect_current_pose_[13]=  msg->position[29];  // l shoulder pan
      vect_current_pose_[14]=  msg->position[30];  // l shoulder lift
      vect_current_pose_[15]=  msg->position[28];  // l upper arm roll
      vect_current_pose_[16]=  msg->position[32];  // l elbow flex
      vect_current_pose_[17]=  msg->position[31];  // l forearm roll
      vect_current_pose_[18]=  msg->position[33];  // l wrist flex
      vect_current_pose_[19]=  msg->position[34];  // l wrist roll
      vect_current_pose_[20]=  msg->position[35];  // l gripper
      vect_current_pose_[21]=  msg->position[36];  // l gripper false
      break;
    default:
      printf("ERROR: SavePoseCB: Unknown robot part. GotoQ will certainly fail.\n");
  }
}

void ControllerAmbassador::setMaxVelVect()
{
  switch (robotPart_){
    case BASE:
      vect_V_max_[0] =	  10;//SDI_F->speedLimit * BASE_LIN_X_MAXVEL	      ;
      vect_V_max_[1] =	  10;//SDI_F->speedLimit * BASE_LIN_Y_MAXVEL	      ;
      vect_V_max_[2] =	  10;//SDI_F->speedLimit * BASE_LIN_Z_MAXVEL	      ;
      vect_V_max_[3] =	  10;//SDI_F->speedLimit * BASE_ROT_X_MAXVEL	      ;
      vect_V_max_[4] =	  10;//SDI_F->speedLimit * BASE_ROT_Y_MAXVEL	      ;
      vect_V_max_[5] =	  10;//SDI_F->speedLimit * BASE_ROT_Z_MAXVEL	      ;

    case TORSO:
      vect_V_max_[0]=     SDI_F->speedLimit * TORSO_MAXVEL            ;
      break;
    case HEAD:
      vect_V_max_[0]=     SDI_F->speedLimit * HEAD_PAN_MAXVEL         ;
      vect_V_max_[1]=     SDI_F->speedLimit * HEAD_TILT_MAXVEL        ;
      break;
    case RARM:
      vect_V_max_[0]=     SDI_F->speedLimit * R_SHOULDER_PAN_MAXVEL   ;
      vect_V_max_[1]=     SDI_F->speedLimit * R_SHOULDER_LIFT_MAXVEL  ;
      vect_V_max_[2]=     SDI_F->speedLimit * R_UPPER_ARM_ROLL_MAXVEL ;
      vect_V_max_[3]=     SDI_F->speedLimit * R_ELBOW_FLEX_MAXVEL     ;
      vect_V_max_[4]=     SDI_F->speedLimit * R_FOREARM_ROLL_MAXVEL   ;
      vect_V_max_[5]=     SDI_F->speedLimit * R_WRIST_FLEX_MAXVEL     ;
      vect_V_max_[6]=     SDI_F->speedLimit * R_WRIT_ROLL_MAXVEL      ;
      break;
    case LARM:
      vect_V_max_[0]=     SDI_F->speedLimit * L_SHOULDER_PAN_MAXVEL   ;
      vect_V_max_[1]=     SDI_F->speedLimit * L_SHOULDER_LIFT_MAXVEL  ;
      vect_V_max_[2]=     SDI_F->speedLimit * L_UPPER_ARM_ROLL_MAXVEL ;
      vect_V_max_[3]=     SDI_F->speedLimit * L_ELBOW_FLEX_MAXVEL     ;
      vect_V_max_[4]=     SDI_F->speedLimit * L_FOREARM_ROLL_MAXVEL   ;
      vect_V_max_[5]=     SDI_F->speedLimit * L_WRIST_FLEX_MAXVEL     ;
      vect_V_max_[6]=     SDI_F->speedLimit * L_WRIST_ROLL_MAXVEL     ;
      break;
    case PR2SYN:
      vect_V_max_[0]=  SDI_F->speedLimit * TORSO_MAXVEL            ;
      vect_V_max_[1]=  SDI_F->speedLimit * HEAD_PAN_MAXVEL         ;
      vect_V_max_[2]=  SDI_F->speedLimit * HEAD_TILT_MAXVEL        ;
      vect_V_max_[3]=  SDI_F->speedLimit * LASER_TILT_MAXVEL       ;
      vect_V_max_[4]=  SDI_F->speedLimit * R_SHOULDER_PAN_MAXVEL   ;
      vect_V_max_[5]=  SDI_F->speedLimit * R_SHOULDER_LIFT_MAXVEL  ;
      vect_V_max_[6]=  SDI_F->speedLimit * R_UPPER_ARM_ROLL_MAXVEL ;
      vect_V_max_[7]=  SDI_F->speedLimit * R_ELBOW_FLEX_MAXVEL     ;
      vect_V_max_[8]=  SDI_F->speedLimit * R_FOREARM_ROLL_MAXVEL   ;
      vect_V_max_[9]=  SDI_F->speedLimit * R_WRIST_FLEX_MAXVEL     ;
      vect_V_max_[10]= SDI_F->speedLimit * R_WRIT_ROLL_MAXVEL      ;
      vect_V_max_[11]= SDI_F->speedLimit * R_GRIPPER_MAXVEL        ;
      vect_V_max_[12]= SDI_F->speedLimit * R_GRIPPER_FALSE_MAXVEL  ;
      vect_V_max_[13]= SDI_F->speedLimit * L_SHOULDER_PAN_MAXVEL   ;
      vect_V_max_[14]= SDI_F->speedLimit * L_SHOULDER_LIFT_MAXVEL  ;
      vect_V_max_[15]= SDI_F->speedLimit * L_UPPER_ARM_ROLL_MAXVEL ;
      vect_V_max_[16]= SDI_F->speedLimit * L_ELBOW_FLEX_MAXVEL     ;
      vect_V_max_[17]= SDI_F->speedLimit * L_FOREARM_ROLL_MAXVEL   ;
      vect_V_max_[18]= SDI_F->speedLimit * L_WRIST_FLEX_MAXVEL     ;
      vect_V_max_[19]= SDI_F->speedLimit * L_WRIST_ROLL_MAXVEL     ;
      vect_V_max_[20]= SDI_F->speedLimit * L_GRIPPER_MAXVEL        ;
      vect_V_max_[21]= SDI_F->speedLimit * L_GRIPPER_FALSE_MAXVEL  ;
      break;
    default:
      printf("ERROR: SetMaxVelVect: Unknown robot part.\n");
  }

  for(int i=0; i< nbJoints_; ++i) {
    vect_A_max_[i] = accVelRatio_ * vect_V_max_[i];
    vect_J_max_[i] = jerkAccRatio_ * vect_A_max_[i];
  }
}

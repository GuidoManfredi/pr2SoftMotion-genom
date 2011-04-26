/*
 * Copyright (c) 2011 LAAS/CNRS                       --  Feb 2011
 * All rights reserved.
 *
 * Redistribution and use  in source  and binary  forms,  with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice, this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice,  this list of  conditions and the following disclaimer in
 *      the  documentation  and/or  other   materials provided  with  the
 *      distribution.
 *
 * THIS  SOFTWARE IS PROVIDED BY  THE  COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND  ANY  EXPRESS OR IMPLIED  WARRANTIES,  INCLUDING,  BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES  OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR  PURPOSE ARE DISCLAIMED. IN  NO EVENT SHALL THE COPYRIGHT
 * HOLDERS OR      CONTRIBUTORS  BE LIABLE FOR   ANY    DIRECT, INDIRECT,
 * INCIDENTAL,  SPECIAL,  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF  SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE   OF THIS SOFTWARE, EVEN   IF ADVISED OF   THE POSSIBILITY OF SUCH
 * DAMAGE.
 */

/**
 ** pr2SoftMotionMainCodels.cc
 **
 ** Codels called by execution task pr2SoftMotionMain
 **
 ** Author:  Xavier BROQUERE
 ** Modified by: Guido MANFREDI
 ** Date: Feb 2011
 **
 **/

#include <portLib.h>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include <tf/transform_listener.h>

#include "server/pr2SoftMotionHeader.h"

#include "pr2_soft_controller/SM_TRAJ_STR_ROS.h"

#include "softMotion/softMotion.h"
#include "softMotion/softMotionStruct.h"
#include "softMotion/softMotionStructGenom.h"

static POSTER_ID pr2Trackposter = NULL; /* the poster to load */ 
static SM_TRAJ currentMotion; /* the softMotion trajectory */
static pr2_soft_controller::SM_TRAJ_STR_ROS smTrajROS; //the last sent trajectory

static double currTime; /* the time parameter along the trajectory */
static SM_COND timeScaleCond; /* the kinematic state of the time parameter */

static ros::Subscriber joint_state_listener;
static ros::Publisher traj_pub;
static ros::Publisher timeScale_pub;
static ros::NodeHandle* nh;

static int currentTrajId;
static double currentDuration;

void doubles2QStr(double* src, PR2SM_QSTR& dst);
void QStr2doubles(PR2SM_QSTR& dst, double* src);
void savePoseCB(const sensor_msgs::JointStateConstPtr& msg);
tf::TransformListener & getTf();
int setMaxVelVect();

static double vect_J_max[PR2SM_NBJOINT];
static double vect_A_max[PR2SM_NBJOINT];
static double vect_V_max[PR2SM_NBJOINT];

/********      Variables for moveHead    ********/
// Contains the current pose of the robot joints
static double vect_current_pose[PR2SM_NBJOINT];
//Listen for transforms between robot frames
static std::string pan_link_= "head_pan_link";
static std::string tilt_link_= "head_tilt_link";
static std::string pan_parent_;
static tf::Stamped<tf::Point> target_in_pan_;
static std::vector<double> q_goal(2);  // [pan, tilt]

/*------------------------------------------------------------------------
 *
 * pr2SoftMotionMainEnd  --  Termination codel
 *
 * Description: 
 * 
 * Returns:    OK or ERROR
 */

STATUS
pr2SoftMotionMainEnd(void)
{
  return OK;
}

/*------------------------------------------------------------------------
 *
 * pr2SoftMotionMainPerm  --  1st codel of permanent activity (called BEFORE other activities)
 *
 * Description: 
 * 
 * Reports:      OK
 * 
 * Returns:    OK or ERROR
 */

STATUS
pr2SoftMotionMainPerm(int *report)
{
  if(SDI_F->isInit == GEN_FALSE) {
    return ETHER;
  }
  return OK;
}
/*------------------------------------------------------------------------
 * Init
 *
 * Description: 
 *
 * Reports:      OK
 *              S_pr2SoftMotion_CANNOT_INIT_ReS
 *              S_pr2SoftMotion_CANNOT_CREATE_ROS_NODE
 *              S_pr2SoftMotion_CANNOT_SUSCRIBE_TO_TOPIC
 */

/* pr2SoftMotionInitMain  -  codel EXEC of Init
   Returns:  EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
pr2SoftMotionInitMain(int *report)
{
  char* argv[] = { "pr2SoftMotionNode",
                    "",
                    NULL};
  int argc = 2;

  SDI_F->timeScale = 1.0;
  SDI_F->motionIsAllowed = GEN_TRUE;
  SDI_F->speedLimit = 1;
  SDI_F->accelerationVelRatio = 1; // Unsused
  SDI_F->jerkAccelerationRatio = 3;

  setMaxVelVect();
  
  //Ros init
  printf("Init ros node...");
  ros::init(argc, argv, "pr2SoftMotionNode");
  printf("...OK\n");

  nh= new ros::NodeHandle(); 

  //ros publishers init
  printf("Init publishers...");
  traj_pub = nh->advertise<pr2_soft_controller::SM_TRAJ_STR_ROS>("pr2_soft_controller/command", 1);  
  timeScale_pub = nh->advertise<std_msgs::Float64>("pr2_soft_controller/timescale", 1);
  joint_state_listener = nh->subscribe("joint_states", 1, savePoseCB);
  printf("...OK\n");

  currentTrajId= -1;

  SDI_F->isInit = GEN_TRUE;

  if(pr2SoftMotionSM_TRAJ_STRPosterFind ("mhpArmTraj", &pr2Trackposter) != OK) {
      *report = S_pr2SoftMotion_POSTER_NOT_FOUND;
      printf("Init failed: no poster found.\n");
      return END;
    }

  return ETHER;
}

/*------------------------------------------------------------------------
 * TrackQ
 *
 * Description: 
 *
 * Reports:      OK
 *              S_pr2SoftMotion_NOT_INITIALIZED
 *              S_pr2SoftMotion_POSTER_NOT_FOUND
 *              S_pr2SoftMotion_POSTER_READ_ERROR
 *              S_pr2SoftMotion_FILE_NOT_FOUND
 *              S_pr2SoftMotion_ERROR_INIT_POSITION
 *              S_pr2SoftMotion_Q_LIMITED
 *              S_pr2SoftMotion_BAD_MODE;
 */

/* pr2SoftMotionTrackQStart  -  codel START of TrackQ
   Returns:  START EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
pr2SoftMotionTrackQStart(PR2SM_TRACK_STR *trackStr, int *report)
{
  return EXEC;
}

/* pr2SoftMotionTrackQMain  -  codel EXEC of TrackQ
   Returns:  EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
pr2SoftMotionTrackQMain(PR2SM_TRACK_STR *trackStr, int *report)
{
  static int previousState = 0;
  SM_TRAJ_STR smTraj;

  /********************** POSTER READING ***********************/
  switch(trackStr->trackMode) {
    case PR2SM_TRACK_FILE:
      printf("Reading from file \n");
      currentMotion.load(trackStr->posterName.name, NULL);      
      currentMotion.computeTimeOnTraj();
    currentDuration= currentMotion.getDuration();
      currentMotion.convertToSM_TRAJ_STR(&smTraj);
      break;

    case PR2SM_TRACK_POSTER:
      if(pr2SoftMotionSM_TRAJ_STRPosterRead (pr2Trackposter, &smTraj) != OK) {
        *report = S_pr2SoftMotion_POSTER_READ_ERROR;
        printf("ERROR: %s poster read failed", trackStr->posterName.name);
        return ETHER;
      }
      if(smTraj.nbAxis != PR2SM_NBJOINT) {
	if(previousState == 1){
          printf("ERROR: pr2SM::TrackQ traj.nbAxis (%d) != %d \n", smTraj.nbAxis, PR2SM_NBJOINT);
          previousState = 0;
        }
        return ETHER;
      } else { 
        if(previousState == 0){
          printf("INFO: pr2SM::TrackQ there are %d axes in the trajectory \n", smTraj.nbAxis);
          previousState = 1;
        }
      }
      break;
    default:
      *report = S_pr2SoftMotion_BAD_MODE;
      return ETHER;

    // compute duration
    currentMotion.importFromSM_TRAJ_STR( &smTraj ); 
    currentMotion.computeTimeOnTraj();
    currentDuration= currentMotion.getDuration();
    currentMotion.convertToSM_TRAJ_STR(&smTraj);
  }
  /**********************************************************/


  if(SDI_F->motionIsAllowed == GEN_TRUE) {
    // copy of the softmotion trajectory into the ros trajectory 
    //currentTrajId= smTraj.trajId;
    
    // update currentTrajId
    smTrajROS.trajId= ++currentTrajId;
    smTrajROS.nbAxis= smTraj.nbAxis;
    smTrajROS.timePreserved= smTraj.timePreserved;	
    smTrajROS.qStart.resize(smTrajROS.nbAxis);
    smTrajROS.qGoal.resize(smTrajROS.nbAxis);
    smTrajROS.traj.resize(smTrajROS.nbAxis);
    for(int i=0; i<smTraj.nbAxis; ++i){
        smTrajROS.qStart[i] = smTraj.qStart[i];
        smTrajROS.qGoal[i] = smTraj.qGoal[i];
        smTrajROS.traj[i].nbSeg= smTraj.traj[i].nbSeg;
        smTrajROS.traj[i].unsused= smTraj.traj[i].unsused;
        smTrajROS.traj[i].seg.resize(smTrajROS.traj[i].nbSeg);
       for(int j=0; j<smTraj.traj[i].nbSeg; ++j){
          smTrajROS.traj[i].seg[j].lpId= smTraj.traj[i].seg[j].lpId;
          smTrajROS.traj[i].seg[j].unused= smTraj.traj[i].seg[j].unused;
          smTrajROS.traj[i].seg[j].timeOnTraj= smTraj.traj[i].seg[j].timeOnTraj;
          smTrajROS.traj[i].seg[j].time= smTraj.traj[i].seg[j].time;
          smTrajROS.traj[i].seg[j].ic_a= smTraj.traj[i].seg[j].ic_a;
          smTrajROS.traj[i].seg[j].ic_v= smTraj.traj[i].seg[j].ic_v;
          smTrajROS.traj[i].seg[j].ic_x= smTraj.traj[i].seg[j].ic_x;
          smTrajROS.traj[i].seg[j].jerk= smTraj.traj[i].seg[j].jerk;
        }
    }
  
    traj_pub.publish(smTrajROS);
    std_msgs::Float64 timescale;
    timescale.data= SDI_F->timeScale;
    timeScale_pub.publish(timescale);

    sleep(currentDuration);

  } else {
    printf("Motion not allowed\n");
    return ETHER;
  }
  return ETHER;
}

/* pr2SoftMotionTrackQEnd  -  codel END of TrackQ
   Returns:  END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
pr2SoftMotionTrackQEnd(PR2SM_TRACK_STR *trackStr, int *report)
{
  return ETHER;
}


void doubles2QStr(double* src, PR2SM_QSTR* dst)
{
  dst->torso= src[0];
  dst->head_pan= src[1];
  dst->head_tilt= src[2];
  dst->laser_tilt= src[3];
  dst->r_shoulder_pan= src[4];
  dst->r_shoulder_lift= src[5];
  dst->r_upper_arm_roll= src[6];
  dst->r_elbow_flex= src[7];
  dst->r_forearm_roll= src[8];
  dst->r_wrist_flex= src[9];
  dst->r_writ_roll= src[10];
  dst->r_gripper= src[11];
  dst->r_gripper_false= src[12];
  dst->l_shoulder_pan= src[13];
  dst->l_shoulder_lift= src[14];
  dst->l_upper_arm_roll= src[15];
  dst->l_elbow_flex= src[16];
  dst->l_forearm_roll= src[17];
  dst->l_wrist_flex= src[18];
  dst->l_wrist_roll= src[19];
  dst->l_gripper= src[20];
  dst->l_gripper_false= src[21]; 
}

void QStr2doubles(PR2SM_QSTR* src, double* dst)
{
  dst[0]= src->torso;
  dst[1]= src->head_pan;
  dst[2]= src->head_tilt;
  dst[3]= src->laser_tilt;
  dst[4]= src->r_shoulder_pan;
  dst[5]= src->r_shoulder_lift;
  dst[6]= src->r_upper_arm_roll;
  dst[7]= src->r_elbow_flex;
  dst[8]= src->r_forearm_roll;
  dst[9]= src->r_wrist_flex;
  dst[10]= src->r_writ_roll;
  dst[11]= src->r_gripper;
  dst[12]= src->r_gripper_false;
  dst[13]= src->l_shoulder_pan;
  dst[14]= src->l_shoulder_lift;
  dst[15]= src->l_upper_arm_roll;
  dst[16]= src->l_elbow_flex;
  dst[17]= src->l_forearm_roll;
  dst[18]= src->l_wrist_flex;
  dst[19]= src->l_wrist_roll;
  dst[20]= src->l_gripper;
  dst[21]= src->l_gripper_false;
}


int setMaxVelVect() {
  vect_V_max[0]=  SDI_F->speedLimit * TORSO_MAXVEL            ; 
  vect_V_max[1]=  SDI_F->speedLimit * HEAD_PAN_MAXVEL         ; 
  vect_V_max[2]=  SDI_F->speedLimit * HEAD_TILT_MAXVEL        ; 
  vect_V_max[3]=  SDI_F->speedLimit * LASER_TILT_MAXVEL       ; 
  vect_V_max[4]=  SDI_F->speedLimit * R_SHOULDER_PAN_MAXVEL   ; 
  vect_V_max[5]=  SDI_F->speedLimit * R_SHOULDER_LIFT_MAXVEL  ; 
  vect_V_max[6]=  SDI_F->speedLimit * R_UPPER_ARM_ROLL_MAXVEL ; 
  vect_V_max[7]=  SDI_F->speedLimit * R_ELBOW_FLEX_MAXVEL     ; 
  vect_V_max[8]=  SDI_F->speedLimit * R_FOREARM_ROLL_MAXVEL   ; 
  vect_V_max[9]=  SDI_F->speedLimit * R_WRIST_FLEX_MAXVEL     ; 
  vect_V_max[10]= SDI_F->speedLimit * R_WRIT_ROLL_MAXVEL      ; 
  vect_V_max[11]= SDI_F->speedLimit * R_GRIPPER_MAXVEL        ; 
  vect_V_max[12]= SDI_F->speedLimit * R_GRIPPER_FALSE_MAXVEL  ; 
  vect_V_max[13]= SDI_F->speedLimit * L_SHOULDER_PAN_MAXVEL   ; 
  vect_V_max[14]= SDI_F->speedLimit * L_SHOULDER_LIFT_MAXVEL  ; 
  vect_V_max[15]= SDI_F->speedLimit * L_UPPER_ARM_ROLL_MAXVEL ; 
  vect_V_max[16]= SDI_F->speedLimit * L_ELBOW_FLEX_MAXVEL     ; 
  vect_V_max[17]= SDI_F->speedLimit * L_FOREARM_ROLL_MAXVEL   ; 
  vect_V_max[18]= SDI_F->speedLimit * L_WRIST_FLEX_MAXVEL     ; 
  vect_V_max[19]= SDI_F->speedLimit * L_WRIST_ROLL_MAXVEL     ; 
  vect_V_max[20]= SDI_F->speedLimit * L_GRIPPER_MAXVEL        ; 
  vect_V_max[21]= SDI_F->speedLimit * L_GRIPPER_FALSE_MAXVEL  ; 



  vect_A_max[0]=  SDI_F->speedLimit * TORSO_MAXACC            ; 
  vect_A_max[1]=  SDI_F->speedLimit * HEAD_PAN_MAXACC         ; 
  vect_A_max[2]=  SDI_F->speedLimit * HEAD_TILT_MAXACC        ; 
  vect_A_max[3]=  SDI_F->speedLimit * LASER_TILT_MAXACC       ; 
  vect_A_max[4]=  SDI_F->speedLimit * R_SHOULDER_PAN_MAXACC   ; 
  vect_A_max[5]=  SDI_F->speedLimit * R_SHOULDER_LIFT_MAXACC  ; 
  vect_A_max[6]=  SDI_F->speedLimit * R_UPPER_ARM_ROLL_MAXACC ; 
  vect_A_max[7]=  SDI_F->speedLimit * R_ELBOW_FLEX_MAXACC     ; 
  vect_A_max[8]=  SDI_F->speedLimit * R_FOREARM_ROLL_MAXACC   ; 
  vect_A_max[9]=  SDI_F->speedLimit * R_WRIST_FLEX_MAXACC     ; 
  vect_A_max[10]= SDI_F->speedLimit * R_WRIT_ROLL_MAXACC      ; 
  vect_A_max[11]= SDI_F->speedLimit * R_GRIPPER_MAXACC        ; 
  vect_A_max[12]= SDI_F->speedLimit * R_GRIPPER_FALSE_MAXACC  ; 
  vect_A_max[13]= SDI_F->speedLimit * L_SHOULDER_PAN_MAXACC   ; 
  vect_A_max[14]= SDI_F->speedLimit * L_SHOULDER_LIFT_MAXACC  ; 
  vect_A_max[15]= SDI_F->speedLimit * L_UPPER_ARM_ROLL_MAXACC ; 
  vect_A_max[16]= SDI_F->speedLimit * L_ELBOW_FLEX_MAXACC     ; 
  vect_A_max[17]= SDI_F->speedLimit * L_FOREARM_ROLL_MAXACC   ; 
  vect_A_max[18]= SDI_F->speedLimit * L_WRIST_FLEX_MAXACC     ; 
  vect_A_max[19]= SDI_F->speedLimit * L_WRIST_ROLL_MAXACC     ; 
  vect_A_max[20]= SDI_F->speedLimit * L_GRIPPER_MAXACC        ; 
  vect_A_max[21]= SDI_F->speedLimit * L_GRIPPER_FALSE_MAXACC  ; 

  for(int i=0; i< PR2SM_NBJOINT; i++) {
    //vect_A_max[i] = SDI_F->accelerationVelRatio * vect_V_max[i];
    vect_J_max[i] = SDI_F->jerkAccelerationRatio * vect_A_max[i];
  }  
}


int smConvertSM_MOTIONtoSM_TRAJ( SM_MOTION_MONO motion[], int nbJoints, SM_TRAJ &traj, int *report) {
  SM_STATUS resp;
  SM_COND IC[nbJoints][SM_NB_SEG];
  double  Time[nbJoints][SM_NB_SEG];
  double  Jerk[nbJoints][SM_NB_SEG];
  SM_SEG seg;
  std::vector<SM_SEG> traj_i;
  std::vector<double> I(3);
  std::vector<double> T(SM_NB_SEG);
  std::vector<double> J(SM_NB_SEG);
  std::vector<double> t(1);
  std::vector<double> a(1);
  std::vector<double> v(1);
  std::vector<double> x(1);

  for (int i = 0; i < nbJoints; i++)
    {

      I[0] = motion[i].IC.a;
      I[1] = motion[i].IC.v;
      I[2] = motion[i].IC.x;

      T[0] = motion[i].Times.Tjpa;
      T[1] = motion[i].Times.Taca;
      T[2] = motion[i].Times.Tjna;
      T[3] = motion[i].Times.Tvc;
      T[4] = motion[i].Times.Tjnb;
      T[5] = motion[i].Times.Tacb;
      T[6] = motion[i].Times.Tjpb;

      J[0] =   motion[i].Dir*motion[i].jerk.J1;
      J[1] =   0.0;
      J[2] = - motion[i].Dir*motion[i].jerk.J1;
      J[3] =   0.0;
      J[4] = - motion[i].Dir*motion[i].jerk.J1;
      J[5] =   0.0;
      J[6] =   motion[i].Dir*motion[i].jerk.J1;

      Time[i][0] = T[0];
      Time[i][1] = T[1];
      Time[i][2] = T[2];
      Time[i][3] = T[3];
      Time[i][4] = T[4];
      Time[i][5] = T[5];
      Time[i][6] = T[6];

      Jerk[i][0] = J[0];
      Jerk[i][1] = J[1];
      Jerk[i][2] = J[2];
      Jerk[i][3] = J[3];
      Jerk[i][4] = J[4];
      Jerk[i][5] = J[5];
      Jerk[i][6] = J[6];

      IC[i][0].a = I[0];
      IC[i][0].v = I[1];
      IC[i][0].x = I[2];
      t[0] = 0.0;
      for (int smp = 0; smp < SM_NB_SEG ; smp++) {
	resp = sm_AVX_TimeVar(I, T, J, t, a, v, x);
	IC[i][smp].a = a[0];
	IC[i][smp].v = v[0];
	IC[i][smp].x = x[0];     
	if (resp != SM_OK) {
	  printf("ERROR: Q interpolation failed (sm_AVX_TimeVar funcion)\n");
	  *report = S_pr2SoftMotion_SOFTMOTION_ERROR;
	  return ETHER;
	}
	t[0] += Time[i][smp];
      }
    }

  traj.clear();
  for (int i = 0; i < nbJoints; i++) {
    traj_i.clear();
    for (int j = 0; j < SM_NB_SEG; j++) {
      seg.IC = IC[i][j];
      seg.time = Time[i][j];
      seg.jerk = Jerk[i][j];
      traj_i.push_back(seg);     
    }
    traj.traj.push_back(traj_i);
    traj.qStart.push_back(motion[i].IC.x);
    traj.qGoal.push_back(motion[i].FC.x);
  }
  traj.computeTimeOnTraj();

  return 0;
}




/*------------------------------------------------------------------------
 * GotoQ
 *
 * Description: 
 *
 * Reports:      OK
 *              S_pr2SoftMotion_NOT_INITIALIZED
 *              S_pr2SoftMotion_SOFTMOTION_ERROR
 */

/* pr2SoftMotionGotoQStart  -  codel START of GotoQ
   Returns:  START EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
pr2SoftMotionGotoQStart(PR2SM_QSTR *qGoto, int *report)
{
  setMaxVelVect();
  return EXEC;
}

/* pr2SoftMotionGotoQMain  -  codel EXEC of GotoQ
   Returns:  EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
pr2SoftMotionGotoQMain(PR2SM_QSTR *qGoto, int *report)
{
  double* qGotod= (double*)malloc(PR2SM_NBJOINT*sizeof(double));
  QStr2doubles(qGoto, qGotod);

  SM_TRAJ traj;
  SM_MOTION_MONO motion[PR2SM_NBJOINT];

  ros::spinOnce();

  memset(motion, 0, PR2SM_NBJOINT*sizeof(SM_MOTION_MONO));

  for(int i=0; i<PR2SM_NBJOINT; ++i) {

    motion[i].IC.x = vect_current_pose[i];
    motion[i].IC.v = 0.0 ;
    motion[i].IC.a = 0.0 ;

    motion[i].FC.x = qGotod[i];
    motion[i].FC.v = 0.0 ;
    motion[i].FC.a = 0.0 ;

  }

  // Compute soft motion between initial and final conditions
  SM_STATUS resp =   sm_ComputeSoftMotionPointToPoint_gen(PR2SM_NBJOINT, vect_J_max, vect_A_max, vect_V_max, motion);
  if (resp != SM_OK) {
    printf("ERROR: Q interpolation failed (sm_ComputeSoftMotionPointToPoint_gen funcion)\n");
    *report = S_pr2SoftMotion_SOFTMOTION_ERROR;
    return ETHER;
  }

  smConvertSM_MOTIONtoSM_TRAJ(motion, PR2SM_NBJOINT, traj, report);

  SM_TRAJ_STR smTraj;
  traj.convertToSM_TRAJ_STR(&smTraj);

  // copy of the softmotion trajectory into the ros trajectory 
  smTrajROS.trajId= ++currentTrajId;
  smTrajROS.nbAxis= smTraj.nbAxis;
  smTrajROS.timePreserved= smTraj.timePreserved;
  smTrajROS.qStart.resize(smTrajROS.nbAxis);
  smTrajROS.qGoal.resize(smTrajROS.nbAxis);
  smTrajROS.traj.resize(smTrajROS.nbAxis);
  for(int i=0; i<smTraj.nbAxis; ++i){
      smTrajROS.qStart[i] = smTraj.qStart[i];
      smTrajROS.qGoal[i] = smTraj.qGoal[i];
      smTrajROS.traj[i].nbSeg= smTraj.traj[i].nbSeg;
      smTrajROS.traj[i].unsused= smTraj.traj[i].unsused;
      smTrajROS.traj[i].seg.resize(smTrajROS.traj[i].nbSeg);
     for(int j=0; j<smTraj.traj[i].nbSeg; ++j){
        smTrajROS.traj[i].seg[j].lpId= smTraj.traj[i].seg[j].lpId;
        smTrajROS.traj[i].seg[j].unused= smTraj.traj[i].seg[j].unused;
        smTrajROS.traj[i].seg[j].timeOnTraj= smTraj.traj[i].seg[j].timeOnTraj;
        smTrajROS.traj[i].seg[j].time= smTraj.traj[i].seg[j].time;
        smTrajROS.traj[i].seg[j].ic_a= smTraj.traj[i].seg[j].ic_a;
        smTrajROS.traj[i].seg[j].ic_v= smTraj.traj[i].seg[j].ic_v;
        smTrajROS.traj[i].seg[j].ic_x= smTraj.traj[i].seg[j].ic_x;
        smTrajROS.traj[i].seg[j].jerk= smTraj.traj[i].seg[j].jerk;
      }
  }

  traj_pub.publish(smTrajROS);
  std_msgs::Float64 timescale;
  timescale.data= SDI_F->timeScale;
  timeScale_pub.publish(timescale);

  return ETHER;
}

/* pr2SoftMotionGotoQEnd  -  codel END of GotoQ
   Returns:  END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
pr2SoftMotionGotoQEnd(PR2SM_QSTR *qGoto, int *report)
{
  /* ... add your code here ... */
  return ETHER;
}

// Saves the current pose of the robot in a vector
void savePoseCB(const sensor_msgs::JointStateConstPtr& msg)
{
  vect_current_pose[0]=  msg->position[12];  // torso
  vect_current_pose[1]=  msg->position[13];  // head pan
  vect_current_pose[2]=  msg->position[14];  // head tilt
  vect_current_pose[3]=  msg->position[15];  // laser tilt
  vect_current_pose[6]=  msg->position[16];  // r upper arm roll
  vect_current_pose[4]=  msg->position[17];  // r shoulder pan
  vect_current_pose[5]=  msg->position[18];  // r shoulder lift
  vect_current_pose[8]=  msg->position[19];  // forearm roll
  vect_current_pose[7]=  msg->position[20];  // r elbow flex
  vect_current_pose[9]=  msg->position[21];  // wrist flex
  vect_current_pose[10]=  msg->position[22];  // wrist roll
  vect_current_pose[11]=  msg->position[23];  // gripper
  vect_current_pose[12]=  msg->position[24];  // gripper false
  vect_current_pose[15]=  msg->position[28];
  vect_current_pose[13]=  msg->position[29];
  vect_current_pose[14]=  msg->position[30];
  vect_current_pose[17]=  msg->position[31];
  vect_current_pose[16]=  msg->position[32];
  vect_current_pose[18]=  msg->position[33];
  vect_current_pose[19]=  msg->position[34];
  vect_current_pose[20]=  msg->position[35];
  vect_current_pose[21]=  msg->position[36];
}

ACTIVITY_EVENT
pr2SoftMotionMoveHeadStart(PR2SM_xyzHead *xyzHead, int *report)
{
  tf::TransformListener& localTf= getTf();

  // Before we do anything, we need to know the name of the pan_link's parent.
  if (pan_parent_.empty())
  {
    for (int i = 0; i < 10; ++i)
    {
      try {
        localTf.getParent(pan_link_, ros::Time(), pan_parent_);
        break;
      }
      catch (const tf::TransformException &ex) {}
      ros::Duration(0.5).sleep();
    }
  }
  // Checking if a parent name has been retrieved successfully
  if (pan_parent_.empty())
  {
    ROS_ERROR("Could not get parent of %s in the TF tree", pan_link_.c_str());
    return ETHER;
  }
  

  return EXEC;
}


ACTIVITY_EVENT
pr2SoftMotionMoveHeadMain(PR2SM_xyzHead *xyzHead, int *report)
{
    tf::TransformListener& localTf= getTf();

    // Transforms the target point into the pan and tilt links.
    geometry_msgs::PointStamped target;
    target.header.frame_id= "base_link";
    target.header.stamp= ros::Time::now();
    target.point.x = xyzHead->x;
    target.point.y = xyzHead->y; 
    target.point.z = xyzHead->z;

    bool ret1 = false, ret2 = false;
    try {
      ros::Time now = ros::Time::now();
      std::string error_msg;

      printf("parent: %s target frame: %s \n", pan_parent_.c_str(), target.header.frame_id.c_str());

      ret1 = localTf.waitForTransform(pan_parent_, target.header.frame_id, target.header.stamp,
                                 ros::Duration(1.0), ros::Duration(0.01), &error_msg);
      ret2 = localTf.waitForTransform(pan_link_, target.header.frame_id, target.header.stamp,
                                   ros::Duration(1.0), ros::Duration(0.01), &error_msg);

      // Transforms the target into the pan and tilt frames
      tf::Stamped<tf::Point> target_point, target_in_tilt;
      tf::pointStampedMsgToTF(target, target_point);
      localTf.transformPoint(pan_parent_, target_point, target_in_pan_);
      localTf.transformPoint(pan_link_, target_point, target_in_tilt);

      // Computes the desired joint positions.
      q_goal[0] = atan2(target_in_pan_.y(), target_in_pan_.x());
      q_goal[1] = atan2(-target_in_tilt.z(),
                        sqrt(pow(target_in_tilt.x(),2) + pow(target_in_tilt.y(),2)));
    }
    catch(const tf::TransformException &ex)
    {
      ROS_ERROR("Transform failure (%d,%d): %s", ret1, ret2, ex.what());
      return ETHER;
    }

   printf("Sending pan: %f / tilt: %f \n", q_goal[0], q_goal[1]);

return ETHER;
}

ACTIVITY_EVENT
pr2SoftMotionMoveHeadEnd(PR2SM_xyzHead *xyzHead, int *report)
{
  // ... add your code here ...
  return ETHER;
}

// This function is needed because the constructor of the transformlistener
// would crash if called before ros::init()
tf::TransformListener & getTf() {
    static tf::TransformListener tf_;
    return tf_;
}

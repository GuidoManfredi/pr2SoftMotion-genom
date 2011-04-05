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
 ** Date: Feb 2011
 **
 **/

#include <portLib.h>

#include "ros/ros.h"
#include "pr2_soft_controller/SM_TRAJ_STR_ROS.h"
#include "std_msgs/Float64.h"

#include "server/pr2SoftMotionHeader.h"


#include "softMotion/softMotion.h"
#include "softMotion/softMotionStruct.h"
#include "softMotion/softMotionStructGenom.h"

static POSTER_ID pr2Trackposter = NULL; /* the poster to load */ 
static SM_TRAJ currentMotion; /* the softMotion trajectory */
pr2_soft_controller::SM_TRAJ_STR_ROS smTrajROS; //the last sent trajectory

static double currTime; /* the time parameter along the trajectory */
static SM_COND timeScaleCond; /* the kinematic state of the time parameter */

static ros::Publisher traj_pub;
static ros::Publisher timeScale_pub;
static ros::NodeHandle* nh;



void doubles2QStr(double* src, PR2SM_QSTR& dst);
void QStr2doubles(PR2SM_QSTR& dst, double* src);

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
 *              S_pr2SoftMotion_CANNOT_INIT_ROS
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

  //Ros init
  printf("Init ros node...");
  ros::init(argc, argv, "pr2SoftMotionNode");
  printf("...OK\n");

  nh= new ros::NodeHandle(); 

  //ros publishers init
  printf("Init publishers...");
  traj_pub = nh->advertise<pr2_soft_controller::SM_TRAJ_STR_ROS>("pr2_soft_controller/command", 1);  
  timeScale_pub = nh->advertise<std_msgs::Float64>("pr2_soft_controller/timescale", 1);
  printf("...OK\n");

  SDI_F->isInit = GEN_TRUE;
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
  SM_TRAJ_STR smTraj;

  /********************** POSTER READING ***********************/
  switch(trackStr->trackMode) {
    case PR2SM_TRACK_FILE:
      currentMotion.load(trackStr->posterName.name, NULL);      
      currentMotion.computeTimeOnTraj();
      currentMotion.convertToSM_TRAJ_STR(&smTraj);
      break;

    case PR2SM_TRACK_POSTER:
      if(pr2SoftMotionSM_TRAJ_STRPosterFind (trackStr->posterName.name, &pr2Trackposter) != OK) {
        *report = S_pr2SoftMotion_POSTER_NOT_FOUND;
        return ETHER;
      }
      if(pr2SoftMotionSM_TRAJ_STRPosterRead (pr2Trackposter, &smTraj) != OK) {
        *report = S_pr2SoftMotion_POSTER_READ_ERROR;
        printf("ERROR: %s poster read failed", trackStr->posterName.name);
        return ETHER;
      }
      if(smTraj.nbAxis != PR2SM_NBJOINT) {
        printf("ERROR: pr2SM::TrackQ traj.nbAxis (%d) != %d \n", smTraj.nbAxis, PR2SM_NBJOINT);
        return ETHER;
      } else { 
        printf("INFO: pr2SM::TrackQ there are %d axes in the trajectory \n", smTraj.nbAxis);
      }
      break;
    default:
      *report = S_pr2SoftMotion_BAD_MODE;
      return ETHER;

    // compute duration
    currentMotion.importFromSM_TRAJ_STR( &smTraj ); 
    currentMotion.computeTimeOnTraj();
    currentMotion.convertToSM_TRAJ_STR(&smTraj);
  }
  /**********************************************************/


  if(SDI_F->motionIsAllowed == GEN_TRUE) {
    // copy of the softmotion trajectory into the ros trajectory 
    smTrajROS.trajId= smTraj.trajId;
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
  } else {
    printf("Motion not allowed\n");
    return END;
  }
  return EXEC;
}

/* pr2SoftMotionTrackQEnd  -  codel END of TrackQ
   Returns:  END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
pr2SoftMotionTrackQEnd(PR2SM_TRACK_STR *trackStr, int *report)
{
  printf("INFO: Motion Terminated\n");
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
  /* ... add your code here ... */
  return ETHER;
}

/* pr2SoftMotionGotoQMain  -  codel EXEC of GotoQ
   Returns:  EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
pr2SoftMotionGotoQMain(PR2SM_QSTR *qGoto, int *report)
{
  /* ... add your code here ... */
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





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
#include "softMotion/softMotion.h"
#include "ControllerAmbassador.h"
#include "GripperSensorMonitor.h"

static POSTER_ID pr2Trackposter = NULL; /* the poster with a trajectyory to execute */ 
static POSTER_ID posterHumDistId = NULL; /* the poster with the human distance */
static SM_TRAJ currentMotion; /* the softMotion trajectory */

static ros::NodeHandle* nh;
static ControllerAmbassador* baseAmbassador; 
static ControllerAmbassador* headAmbassador; 
static ControllerAmbassador* torsoAmbassador; 
static ControllerAmbassador* rArmAmbassador; 
static ControllerAmbassador* lArmAmbassador; 
static ControllerAmbassador* pr2SynAmbassador; 

static ros::Publisher pan_head_pub;
static ros::Publisher tilt_head_pub;

static GripperSensorMonitor* r_gripperSensorMonitor;

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
    return OK;
  }
  if(posterHumDistId && SDI_F->humanDistMode == GEN_TRUE) {
    if(pr2SoftMotionGENHUM_HUMAN_DISTANCEPosterRead(posterHumDistId,
						    &SDI_F->humanDist) == ERROR) {
      *report =  S_pr2SoftMotion_CANNOT_READ_POSTER;
    }
    // there are 5 groups
    for(int i=0; i<5; ++i) {
      SDI_F->timeScale.timescale[i] = SDI_F->humanDist.costDistRobToHum;
    }

    //printf("SDI_F->humanDist.costDistRobToHum %f\n",SDI_F->humanDist.costDistRobToHum);

  }
  //baseAmbassador->publishTimeScale();
  torsoAmbassador->publishTimeScale();
  headAmbassador->publishTimeScale();
  rArmAmbassador->publishTimeScale();
  lArmAmbassador->publishTimeScale();

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

  for(int i=0; i<5; ++i) {
    SDI_F->timeScale.timescale[i] = 1.0;
  }
  SDI_F->motionIsAllowed = GEN_TRUE;
  SDI_F->speedLimit = 1;
  SDI_F->accelerationVelRatio = 1; // Unsused
  SDI_F->jerkAccelerationRatio = 3;
  SDI_F->humanDistMode = GEN_FALSE;

  //Ros init
  printf("Init ros node...");
  ros::init(argc, argv, "pr2SoftMotionNode");
  printf("...OK\n");

  nh= new ros::NodeHandle(); 

  baseAmbassador = new ControllerAmbassador(BASE, nh);
  headAmbassador = new ControllerAmbassador(HEAD, nh);
  torsoAmbassador = new ControllerAmbassador(TORSO, nh);
  rArmAmbassador = new ControllerAmbassador(RARM, nh);
  lArmAmbassador = new ControllerAmbassador(LARM, nh);
  pr2SynAmbassador = new ControllerAmbassador(PR2SYN, nh);

  r_gripperSensorMonitor = new GripperSensorMonitor(nh);
  SDI_F->sensorTresholds.grabAcc= 4.0;
  SDI_F->sensorTresholds.grabSlip= 0.05;
  SDI_F->sensorTresholds.releaseAcc= 4.0;
  SDI_F->sensorTresholds.releaseSlip= 0.05;

  SDI_F->isInit = GEN_TRUE;

  pan_head_pub= nh->advertise<std_msgs::Float64>("pan_head_soft_controller/command", 1);
  tilt_head_pub= nh->advertise<std_msgs::Float64>("tilt_head_soft_controller/command", 1);

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
  static int previousState = 0, motionOk = 0;
  SM_TRAJ_STR smTraj;
  PR2SM_QSTR qInit;

  /********************** POSTER READING ***********************/
  switch(trackStr->trackMode) {
    case PR2SM_TRACK_FILE:
      printf("Reading from file \n");
      currentMotion.load(trackStr->posterName.name, NULL);      
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
      }
      else { 
        if(previousState == 0){
          printf("INFO: pr2SM::TrackQ there are %d axes in the trajectory \n", smTraj.nbAxis);
          previousState = 1;
          motionOk = 0;
        }
      }
      break;
    default:
      *report = S_pr2SoftMotion_BAD_MODE;
      return ETHER;
  }
  /**********************************************************/

  if(SDI_F->motionIsAllowed == GEN_TRUE) {
    switch(trackStr->robotPart){
      case BASE:
	baseAmbassador->trackQ(&smTraj, report);
        break;
      case TORSO:
        torsoAmbassador->trackQ(&smTraj, report);
        break;
      case HEAD:
        headAmbassador->trackQ(&smTraj, report);
        break;
      case RARM:
        rArmAmbassador->trackQ(&smTraj, report);
        break;
      case LARM:
        lArmAmbassador->trackQ(&smTraj, report);
        break;
      case PR2SYN:
        pr2SynAmbassador->trackQ(&smTraj, report);
        break;
      case PR2:
        torsoAmbassador->trackQ(&smTraj, report);
        headAmbassador->trackQ(&smTraj, report);
        rArmAmbassador->trackQ(&smTraj, report);
        lArmAmbassador->trackQ(&smTraj, report); 
        break;
      case ARMS:
        rArmAmbassador->trackQ(&smTraj, report);
        lArmAmbassador->trackQ(&smTraj, report);
        break;
      case PR2NOHEAD:
        torsoAmbassador->trackQ(&smTraj, report);
        rArmAmbassador->trackQ(&smTraj, report);
        lArmAmbassador->trackQ(&smTraj, report);
        break;
      case PR2FULL:
        baseAmbassador->trackQ(&smTraj, report);
        torsoAmbassador->trackQ(&smTraj, report);
        headAmbassador->trackQ(&smTraj, report);
        rArmAmbassador->trackQ(&smTraj, report);
        lArmAmbassador->trackQ(&smTraj, report); 
        break;
      default:
        printf("Error: unknown robot part. Motion cancelled.\n");
        return ETHER; 
    }
  } else {
    printf("Motion not allowed\n");
    return ETHER;
  }

  return EXEC;
}

/* pr2SoftMotionTrackQMain  -  codel EXEC of TrackQ
   Returns:  EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
pr2SoftMotionTrackQMain(PR2SM_TRACK_STR *trackStr, int *report)
{
  bool finished= false;

  switch(trackStr->robotPart){
    case BASE:
      finished= baseAmbassador->monitorTraj();
      break;
    case HEAD:
      finished= headAmbassador->monitorTraj();
      break;
    case TORSO:
      finished= torsoAmbassador->monitorTraj();
      break;
    case RARM:
      finished= rArmAmbassador->monitorTraj();
      break;
    case LARM:
      finished= lArmAmbassador->monitorTraj();
      break;
    case PR2SYN:
      finished= pr2SynAmbassador->monitorTraj();
      break;
    case PR2: 
      // we wait for all parts to finish
      finished= headAmbassador->monitorTraj() && 
                torsoAmbassador->monitorTraj() &&
                rArmAmbassador->monitorTraj() &&
                lArmAmbassador->monitorTraj();
      break;
    case ARMS:
      // we wait for all parts to finish
      finished= rArmAmbassador->monitorTraj() &&
                lArmAmbassador->monitorTraj();
      break;
    case PR2NOHEAD:
      // we wait for all parts to finish
      finished= rArmAmbassador->monitorTraj() &&
                lArmAmbassador->monitorTraj() &&
		torsoAmbassador->monitorTraj() ;
      break;
    case PR2FULL: 
      //we wait for all parts to finish
      finished= baseAmbassador->monitorTraj() &&  
                headAmbassador->monitorTraj() && 
                torsoAmbassador->monitorTraj() &&
                rArmAmbassador->monitorTraj() &&
                lArmAmbassador->monitorTraj();
      //finished= true;
      break;

    default:
      printf("Error: unknown robot part. Motion cancelled.\n");
      return ETHER;
  }

  //end loop conditions
  if(finished)
    return END;
  else
    return EXEC;
}

/* pr2SoftMotionTrackQEnd  -  codel END of TrackQ
   Returns:  END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
pr2SoftMotionTrackQEnd(PR2SM_TRACK_STR *trackStr, int *report)
{
  printf("INFO: Motion finished.\n");
  return ETHER;
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
  switch(qGoto->robotPart){
    case BASE:
      baseAmbassador->setRatios(SDI_F->accelerationVelRatio, SDI_F->jerkAccelerationRatio);
      baseAmbassador->gotoQ(qGoto, report);
      break;
    case TORSO:
      torsoAmbassador->setRatios(SDI_F->accelerationVelRatio, SDI_F->jerkAccelerationRatio);
      torsoAmbassador->gotoQ(qGoto, report);
      break;
    case HEAD:
      headAmbassador->setRatios(SDI_F->accelerationVelRatio, SDI_F->jerkAccelerationRatio);
      headAmbassador->gotoQ(qGoto, report);
      break;
    case RARM:
      rArmAmbassador->setRatios(SDI_F->accelerationVelRatio, SDI_F->jerkAccelerationRatio);
      rArmAmbassador->gotoQ(qGoto, report);
      break;
    case LARM:
      lArmAmbassador->setRatios(SDI_F->accelerationVelRatio, SDI_F->jerkAccelerationRatio);
      lArmAmbassador->gotoQ(qGoto, report);
      break;
    case PR2SYN:
      pr2SynAmbassador->setRatios(SDI_F->accelerationVelRatio, SDI_F->jerkAccelerationRatio);
      pr2SynAmbassador->gotoQ(qGoto, report);
      break;
    case PR2:
      torsoAmbassador->setRatios(SDI_F->accelerationVelRatio, SDI_F->jerkAccelerationRatio);
      torsoAmbassador->gotoQ(qGoto, report);
      headAmbassador->setRatios(SDI_F->accelerationVelRatio, SDI_F->jerkAccelerationRatio);
      headAmbassador->gotoQ(qGoto, report);
      rArmAmbassador->setRatios(SDI_F->accelerationVelRatio, SDI_F->jerkAccelerationRatio);
      rArmAmbassador->gotoQ(qGoto, report);
      lArmAmbassador->setRatios(SDI_F->accelerationVelRatio, SDI_F->jerkAccelerationRatio);
      lArmAmbassador->gotoQ(qGoto, report);
      break;
    case ARMS:
      rArmAmbassador->setRatios(SDI_F->accelerationVelRatio, SDI_F->jerkAccelerationRatio);
      rArmAmbassador->gotoQ(qGoto, report);
      lArmAmbassador->setRatios(SDI_F->accelerationVelRatio, SDI_F->jerkAccelerationRatio);
      lArmAmbassador->gotoQ(qGoto, report);
      break;
    case PR2NOHEAD:
      torsoAmbassador->setRatios(SDI_F->accelerationVelRatio, SDI_F->jerkAccelerationRatio);
      torsoAmbassador->gotoQ(qGoto, report);
      rArmAmbassador->setRatios(SDI_F->accelerationVelRatio, SDI_F->jerkAccelerationRatio);
      rArmAmbassador->gotoQ(qGoto, report);
      lArmAmbassador->setRatios(SDI_F->accelerationVelRatio, SDI_F->jerkAccelerationRatio);
      lArmAmbassador->gotoQ(qGoto, report);
      break;
    default:
      printf("Error: unknown robot part. Motion cancelled.\n");
  }
  return EXEC;
}

/* pr2SoftMotionGotoQMain  -  codel EXEC of GotoQ
   Returns:  EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
pr2SoftMotionGotoQMain(PR2SM_QSTR *qGoto, int *report)
{
  //Timescale is published again (manually) in case it has changed
  switch(qGoto->robotPart){
    case BASE:
      baseAmbassador->publishTimeScale();
      break;
    case TORSO:
      torsoAmbassador->publishTimeScale();
      break;
    case HEAD:
      headAmbassador->publishTimeScale();
      break;
    case RARM:
      rArmAmbassador->publishTimeScale();
      break;
    case LARM:
      lArmAmbassador->publishTimeScale();
      break;
    case PR2SYN:
      pr2SynAmbassador->publishTimeScale();
      break;
    case PR2:
      torsoAmbassador->publishTimeScale();
      headAmbassador->publishTimeScale();
      rArmAmbassador->publishTimeScale();
      lArmAmbassador->publishTimeScale();
      break;
    case ARMS:
      rArmAmbassador->publishTimeScale();
      lArmAmbassador->publishTimeScale();
      break;
    case PR2NOHEAD:
      torsoAmbassador->publishTimeScale();
      rArmAmbassador->publishTimeScale();
      lArmAmbassador->publishTimeScale();
      break;
    default:
      printf("Error: unknown robot part. Motion cancelled.\n");
  } 

  return END;
}

/* pr2SoftMotionGotoQEnd  -  codel END of GotoQ
   Returns:  END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
pr2SoftMotionGotoQEnd(PR2SM_QSTR *qGoto, int *report)
{
    bool finished= false;

  switch(qGoto->robotPart){
    case BASE:
      finished= baseAmbassador->monitorTraj();
    case HEAD:
      finished= headAmbassador->monitorTraj();
      break;
    case TORSO:
      finished= torsoAmbassador->monitorTraj();
      break;
    case RARM:
      finished= rArmAmbassador->monitorTraj();
      break;
    case LARM:
      finished= lArmAmbassador->monitorTraj();
      break;
    case PR2SYN:
      finished= pr2SynAmbassador->monitorTraj();
      break;
    case PR2: 
      // we choose the slower joint, so the torso
      finished= headAmbassador->monitorTraj() && 
                torsoAmbassador->monitorTraj() &&
                rArmAmbassador->monitorTraj() &&
                lArmAmbassador->monitorTraj();
      break;
    case PR2NOHEAD:
      // we choose the slower joint, so the torso
      finished=torsoAmbassador->monitorTraj() &&
                rArmAmbassador->monitorTraj() &&
                lArmAmbassador->monitorTraj();
      break;
    case ARMS:
      finished= rArmAmbassador->monitorTraj() &&
                lArmAmbassador->monitorTraj();
      break;
    default:
      printf("Error: unknown robot part. Motion cancelled.\n");
      return ETHER;
  }

  //end loop conditions
  if(finished)
    return ETHER;
  else
    return END;

}

/*------------------------------------------------------------------------
 * MoveHead
 *
 * Description: 
 *
 * Reports:      OK
 *              S_pr2SoftMotion_NOT_INITIALIZED
 *              S_pr2SoftMotion_SOFTMOTION_ERROR
 */
/* pr2SoftMotionMoveHeadStart  -  codel START of MoveHead
   Returns:  START EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
pr2SoftMotionMoveHeadStart(PR2SM_xyzHead *xyzHead, int *report)
{
  tf::TransformListener listener;
  std_msgs::Float64 pan, tilt;
  //transforming point into robot frame
  geometry_msgs::PointStamped goal;
  geometry_msgs::PointStamped goalRobotFrame;
  geometry_msgs::PointStamped goalPanFrame;

  goal.header.stamp= ros::Time(0);
  goal.header.frame_id= xyzHead->frame;
  goal.point.x= xyzHead->x;
  goal.point.y= xyzHead->y;
  goal.point.z= xyzHead->z;

  try
  {
    //wait for the listener to get the first message
    listener.waitForTransform(xyzHead->frame, "base_footprint", ros::Time(0), ros::Duration(1.0));
    listener.waitForTransform(xyzHead->frame, "head_pan_link", ros::Time(0), ros::Duration(1.0));
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
    return ETHER;
  }

  listener.transformPoint("base_footprint", goal, goalRobotFrame);
  listener.transformPoint("head_pan_link", goal, goalPanFrame);

  pan.data= atan2(goalRobotFrame.point.y, goalRobotFrame.point.x);
  tilt.data= atan2(-goalPanFrame.point.z, sqrt(pow(goalPanFrame.point.x,2)+pow(goalPanFrame.point.y,2)));

  //printf("%f %f\n", pan, tilt);
  pan_head_pub.publish(pan);
  tilt_head_pub.publish(tilt);

  return EXEC;
}

/* pr2SoftMotionMoveHeadMain  -  codel START of MoveHead
   Returns:  START EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
pr2SoftMotionMoveHeadMain(PR2SM_xyzHead *xyzHead, int *report)
{

return END;
}

/* pr2SoftMotionMoveHeadEnd  -  codel START of MoveHead
   Returns:  START EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
pr2SoftMotionMoveHeadEnd(PR2SM_xyzHead *xyzHead, int *report)
{
  // ... add your code here ...
  return ETHER;
}

/*------------------------------------------------------------------------
 * GetPanTiltFromXYZ
 *
 * Description: 
 *
 * Reports:      OK
 *              S_pr2SoftMotion_NOT_INITIALIZED
 *              S_pr2SoftMotion_SOFTMOTION_ERROR
 */

/* pr2SoftMotionGetPanTiltFromXYZMain  -  codel START of GetPanTiltFromXYZ
   Returns:  START EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
pr2SoftMotionGetPanTiltFromXYZMain(PR2SM_xyzHead *xyzHead, PR2SM_PanTilt *panTilt ,int *report)
{

  tf::TransformListener listener;
  //transforming point into robot frame
  geometry_msgs::PointStamped goal;
  geometry_msgs::PointStamped goalRobotFrame;
  geometry_msgs::PointStamped goalPanFrame;

  goal.header.stamp= ros::Time(0);
  goal.header.frame_id= xyzHead->frame;
  goal.point.x= xyzHead->x;
  goal.point.y= xyzHead->y;
  goal.point.z= xyzHead->z;

  try
  {
    //wait for the listener to get the first message
    listener.waitForTransform(xyzHead->frame, "base_footprint", ros::Time(0), ros::Duration(1.0));
    listener.waitForTransform(xyzHead->frame, "head_pan_link", ros::Time(0), ros::Duration(1.0));
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
    return ETHER;
  }

  listener.transformPoint("base_footprint", goal, goalRobotFrame);
  listener.transformPoint("head_pan_link", goal, goalPanFrame);

  panTilt->pan= atan2(goalRobotFrame.point.y, goalRobotFrame.point.x);
  panTilt->tilt= atan2(-goalPanFrame.point.z, sqrt(pow(goalPanFrame.point.x,2)+pow(goalPanFrame.point.y,2)));

  return ETHER;
}

/*------------------------------------------------------------------------
 * HeadTrack
 *
 * Description: 
 *
 * Reports:      OK
 *              S_pr2SoftMotion_NOT_INITIALIZED
 *              S_pr2SoftMotion_SOFTMOTION_ERROR
 */
/* pr2SoftMotionHeadTrackStart  -  codel START of HeadTrack
   Returns:  START EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
pr2SoftMotionHeadTrackStart(PR2SM_xyzHead *xyzHead, int *report)
{

  return EXEC;
}

/* pr2SoftMotionMoveHeadMain  -  codel START of MoveHead
   Returns:  START EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
pr2SoftMotionHeadTrackMain(PR2SM_xyzHead *xyzHead, int *report)
{
  tf::TransformListener listener;
  std_msgs::Float64 pan, tilt;
  //transforming point into robot frame
  geometry_msgs::PointStamped goal;
  geometry_msgs::PointStamped goalRobotFrame;
  geometry_msgs::PointStamped goalPanFrame;

  goal.header.stamp= ros::Time(0);
  goal.header.frame_id= xyzHead->frame;
  goal.point.x= xyzHead->x;
  goal.point.y= xyzHead->y;
  goal.point.z= xyzHead->z;

  try
  {
    //wait for the listener to get the first message
    listener.waitForTransform(xyzHead->frame, "base_footprint", ros::Time(0), ros::Duration(1.0));
    listener.waitForTransform(xyzHead->frame, "head_pan_link", ros::Time(0), ros::Duration(1.0));
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
    return ETHER;
  }

  listener.transformPoint("base_footprint", goal, goalRobotFrame);
  listener.transformPoint("head_pan_link", goal, goalPanFrame);

  pan.data = atan2(goalRobotFrame.point.y, goalRobotFrame.point.x);
  tilt.data = atan2(-goalPanFrame.point.z, sqrt(pow(goalPanFrame.point.x,2)+pow(goalPanFrame.point.y,2)));

  //printf("%f %f\n", pan, tilt);
  pan_head_pub.publish(pan);
  tilt_head_pub.publish(tilt);

  return EXEC;

}

/* pr2SoftMotionMoveHeadEnd  -  codel START of MoveHead
   Returns:  START EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
pr2SoftMotionHeadTrackEnd(PR2SM_xyzHead *xyzHead, int *report)
{
  // ... add your code here ...
  return ETHER;
}


/*------------------------------------------------------------------------
 * GripperGrabRelease
 *
 * Description: 
 *
 *
 * Reports:      OK
 *              S_pr2SoftMotion_NOT_INITIALIZED
 *              S_pr2SoftMotion_SOFTMOTION_ERROR
 */


/* pr2SoftMotionGripperGrabReleaseStart  -  codel START of MoveHead
   Returns:  START EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
pr2SoftMotionGripperGrabReleaseStart(PR2SM_gripperGrabRelease *mode, int *report)
{
  switch(*mode)
  {
    case RELEASE:
      printf("Ready to release.\n");
      r_gripperSensorMonitor->detect(SDI_F->sensorTresholds.releaseAcc, SDI_F->sensorTresholds.releaseSlip);
      break;
    case GRAB:
      printf("Ready to grab.\n");
      r_gripperSensorMonitor->detect(SDI_F->sensorTresholds.grabAcc, SDI_F->sensorTresholds.grabSlip);
      break;
    case OPEN:
      printf("Openning gripper.\n");
      break;
    case CLOSE:
      r_gripperSensorMonitor->findTwoContacts();
      printf("Closing gripper.\n");
      break;
    case CANCEL:
      printf("Canceling all gripper actions.\n");
      r_gripperSensorMonitor->stopAll();
      break;
    default:
      printf("Error: Unrecognized gripper mode\n");
  }

  return EXEC;
}


/* pr2SoftMotionGripperGrabReleaseMain  -  codel START of MoveHead
   Returns:  START EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
pr2SoftMotionGripperGrabReleaseMain(PR2SM_gripperGrabRelease *mode, int *report)
{
  switch(*mode)
  {
    case RELEASE:
      if(r_gripperSensorMonitor->detectWait()){
        *mode= OPEN;
      }
      break;
    case GRAB:
      if(r_gripperSensorMonitor->detectWait()){
        r_gripperSensorMonitor->findTwoContacts();
        *mode= CLOSE;
      }
      break;
    case OPEN:
      r_gripperSensorMonitor->open();
      return ETHER;
      break;
    case CLOSE:
      if(r_gripperSensorMonitor->findTwoContactsWait()){
        r_gripperSensorMonitor->hold(SDI_F->sensorTresholds.holdForce);
        return ETHER;
      }
      break;
    case CANCEL:
      return ETHER;
      break;
    default:
      printf("Error: Unrecognized gripper mode\n");
      return ETHER;
  }

return EXEC;
}

/* pr2SoftMotionGripperGrabReleaseEnd  -  codel START of GrabRelease
   Returns:  START EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
pr2SoftMotionGripperGrabReleaseEnd(PR2SM_gripperGrabRelease *mode, int *report)
{
  // ... add your code here ...
  return ETHER;
}

/*------------------------------------------------------------------------
 * SetHumanDistancePoster
 *
 * Description: 
 *
 * Reports:      OK
 *              S_pr2SoftMotion_CANNOT_FIND_POSTER
 *              S_pr2SoftMotion_CANNOT_READ_POSTER
 */

/* pr2SoftMotionSetHumanDistancePosterMain  -  codel EXEC of SetHumanDistancePoster
   Returns:  EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
pr2SoftMotionSetHumanDistancePosterMain(GEN_STRING128 *humanDistPosterName, int *report)
{
  posterHumDistId = NULL;
  /* Find the external poster and try it by reading once */
 if(pr2SoftMotionGENHUM_HUMAN_DISTANCEPosterFind(humanDistPosterName->name,
                                      &posterHumDistId) == ERROR) {
    *report =  S_pr2SoftMotion_CANNOT_FIND_POSTER;
  }
  else if(pr2SoftMotionGENHUM_HUMAN_DISTANCEPosterRead(posterHumDistId,
                                           &SDI_F->humanDist) == ERROR) {
    *report =  S_pr2SoftMotion_CANNOT_READ_POSTER;
  }
  else {

    *report = OK;
    printf("pr2SoftMotionSetHumanDistancePosterMain poster connected \n");
  }
  printf("pr2SoftMotionSetHumanDistancePosterMain OK\n");
  return ETHER;
}


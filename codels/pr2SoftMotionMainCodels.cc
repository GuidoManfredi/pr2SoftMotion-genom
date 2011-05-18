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
#include "softMotion/softMotionStruct.h"
#include "softMotion/softMotionStructGenom.h"

#include "ControllerAmbassador.h"

static POSTER_ID pr2Trackposter = NULL; /* the poster to load */ 
static SM_TRAJ currentMotion; /* the softMotion trajectory */

static ros::NodeHandle* nh;
ControllerAmbassador* rArmAmbassador; 
ControllerAmbassador* lArmAmbassador; 
ControllerAmbassador* panHeadAmbassador; 
ControllerAmbassador* tiltHeadArmAmbassador;
ControllerAmbassador* pr2Ambassador;


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

  rArmAmbassador = new ControllerAmbassador(PR2SM_RARM, nh);
  lArmAmbassador = new ControllerAmbassador(PR2SM_LARM, nh);
  panHeadAmbassador = new ControllerAmbassador(PR2SM_HEADPAN, nh);
  tiltHeadArmAmbassador = new ControllerAmbassador(PR2SM_HEADTILT, nh);
  pr2Ambassador = new ControllerAmbassador(PR2SM_PR2, nh);

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
  static int previousState = 0, motionOk = 0;
  SM_TRAJ_STR smTraj;

  /********************** POSTER READING ***********************/
  switch(trackStr->trackMode) {
    case PR2SM_TRACK_FILE:
      printf("Reading from file \n");
      currentMotion.load(trackStr->posterName.name, NULL);      
      currentMotion.computeTimeOnTraj();
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

  // TODO make sure the time computation is done before
  currentMotion.importFromSM_TRAJ_STR( &smTraj ); 
  currentMotion.computeTimeOnTraj();
  currentMotion.convertToSM_TRAJ_STR(&smTraj);

  if(SDI_F->motionIsAllowed == GEN_TRUE) {
    switch(trackStr->robotPart){
      case PR2SM_RARM:
        rArmAmbassador->trackQ(smTraj)
        break;
      case PR2SM_LARM:
        lArmAmbassador->trackQ(smTraj)
        break;
      case PR2SM_HEADPAN:
        panHeadAmbassador->trackQ(smTraj)
        break;
      case PR2SM_HEADTILT:
        tiltHeadAmbassador->trackQ(smTraj)
        break;
      case PR2SM_TORSO:
        pr2Ambassador->trackQ(smTraj)
        break;
      default:
        ;
    }
  } else {
    printf("Motion not allowed\n");
    return ETHER;
  }
  return END;
}

/* pr2SoftMotionTrackQEnd  -  codel END of TrackQ
   Returns:  END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
pr2SoftMotionTrackQEnd(PR2SM_TRACK_STR *trackStr, int *report)
{
  printf("INFO: Motion Terminated\n");
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
  setMaxVelVect();
  return EXEC;
}

/* pr2SoftMotionGotoQMain  -  codel EXEC of GotoQ
   Returns:  EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
pr2SoftMotionGotoQMain(PR2SM_QSTR *qGoto, int *report)
{


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
  

  return EXEC;
}


/* pr2SoftMotionMoveHeadMain  -  codel START of MoveHead
   Returns:  START EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
pr2SoftMotionMoveHeadMain(PR2SM_xyzHead *xyzHead, int *report)
{

return ETHER;
}

/* pr2SoftMotionMoveHeadEnd  -  codel START of MoveHead
   Returns:  START EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
pr2SoftMotionMoveHeadEnd(PR2SM_xyzHead *xyzHead, int *report)
{
  // ... add your code here ...
  return ETHER;
}




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

#include "server/pr2SoftMotionHeader.h"


#include "softMotion/softMotion.h"
#include "softMotion/softMotionStruct.h"
#include "softMotion/softMotionStructGenom.h"

static POSTER_ID pr2Trackposter = NULL; 
static SM_TRAJ currentMotion; /* the softMotion trajectory */
static double currTime; /* the time parameter along the trajectory */
static SM_COND timeScaleCond; /* the kinematic state of the time parameter */

static double pr2Jnts[PR2SM_NBJOINT]; /* the target */

static int pr2SMComputeSmoothedTimeScale(double timeScale, SM_LIMITS limitsGoto, SM_COND* timeScaleCond);

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
  SDI_F->timeScale = 1.0;
  SDI_F->motionIsAllowed = GEN_TRUE;

  /* Create here the ros node */
  /* .........................*/

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
  SM_TRAJ_STR smTraj;

  switch(trackStr->trackMode) {
  case PR2SM_TRACK_FILE:
    printf("load file\n");
    currentMotion.load(trackStr->posterName.name, NULL);
    printf("file loaded\n");
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
    currentMotion.clear();
    currentMotion.importFromSM_TRAJ_STR(&smTraj);
    //currentMotion.print();
    break;
  default:
    *report = S_pr2SoftMotion_BAD_MODE;
    return ETHER;
  }
  currentMotion.computeTimeOnTraj();
  currentMotion.print();
  currTime = 0.0;
  return EXEC;
}

/* pr2SoftMotionTrackQMain  -  codel EXEC of TrackQ
   Returns:  EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
pr2SoftMotionTrackQMain(PR2SM_TRACK_STR *trackStr, int *report)
{
  if(SDI_F->motionIsAllowed == GEN_TRUE) {

    std::vector<SM_COND> cond;
    currentMotion.getMotionCond(currTime, cond);
    for (int i = 0; i < PR2SM_NBJOINT; i++) {
      pr2Jnts[i] = cond[i].x;
    }
    /* Send here the new target position */
    /* ..................................*/


    /* Compute the next timeStep */    
    SM_LIMITS limitsGoto;
    limitsGoto.maxJerk = 9.0;
    limitsGoto.maxAcc  = 3.0;
    limitsGoto.maxVel  = 1.0;
    pr2SMComputeSmoothedTimeScale(SDI_F->timeScale, limitsGoto, &timeScaleCond);

    currTime = currTime + (timeScaleCond.v * PR2SM_PERIOD);
    if(currTime<0.0 || currTime >=  currentMotion.getDuration()) {
      return END;
    }
  } else {
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

int pr2SMComputeSmoothedTimeScale(double timeScale, SM_LIMITS limitsGoto, SM_COND* timeScaleCond) {
  double dc = 0.0;
  SM_COND FC;
  SM_STATUS resp;
  SM_TIMES T_Jerk;
  int dir = 0;
  double timeTrajDuration = 0.0;
  std::vector<double> I(3);
  std::vector<double> T(SM_NB_SEG);
  std::vector<double> J(SM_NB_SEG);
  std::vector<double> t(1);
  std::vector<double> a(1);
  std::vector<double> v(1);
  std::vector<double> x(1);

  FC.a = 0.0;
  FC.v = timeScale;
  FC.x = 1.0;

  timeScaleCond->x = 0.0;

  if(ABS(FC.v - timeScaleCond->v) > 0.001) {
    sm_CalculOfCriticalLength(*timeScaleCond, FC, limitsGoto, &dc);
    if (isnan(dc)) {
      printf("isnan dc\n");
      printf("IC.a = %f    IC.v = %f    IC.x = %f\n", timeScaleCond->a, timeScaleCond->v, timeScaleCond->x);
      printf("FC.a = %f    FC.v = %f    FC.x = %f\n",FC.a, FC.v, FC.x);
    }
    FC.x = dc;    	    
    /* compute the motion */
    resp = sm_ComputeSoftMotion(*timeScaleCond, FC, limitsGoto, &T_Jerk, &dir);
    if (resp != SM_OK) {
      printf("ERROR sm_ComputeSoftMotion\n");
    }
    /* get the  position at the next tick */       
    I[0] = timeScaleCond->a;
    I[1] = timeScaleCond->v;
    I[2] = timeScaleCond->x; 
    T[0] = T_Jerk.Tjpa;
    T[1] = T_Jerk.Taca;
    T[2] = T_Jerk.Tjna;
    T[3] = T_Jerk.Tvc;
    T[4] = T_Jerk.Tjnb;
    T[5] = T_Jerk.Tacb;
    T[6] = T_Jerk.Tjpb;
    J[0] =   dir*limitsGoto.maxJerk;
    J[1] =   0.0;
    J[2] = - dir*limitsGoto.maxJerk;
    J[3] =   0.0;
    J[4] = - dir*limitsGoto.maxJerk;
    J[5] =   0.0;
    J[6] =   dir*limitsGoto.maxJerk;

    timeTrajDuration = 0.0;
    for (int p=0; p < SM_NB_SEG; p++) {
      timeTrajDuration += T[p];
    }
    if(timeTrajDuration < PR2SM_PERIOD) {
      t[0] = (double)timeTrajDuration; 
    } else {
      t[0] = (double)PR2SM_PERIOD; 
    }
    resp = sm_AVX_TimeVar(I, T, J, t, a, v, x);
    timeScaleCond->x = x[0];
    timeScaleCond->v  = v[0];
    timeScaleCond->a  = a[0];
  } else {
    timeScaleCond->v = timeScale;
  }
  return 0;
}

/**
 ** pr2SoftMotionCntrlTaskCodels.cc
 **
 ** Codels used by the control task pr2SoftMotionCntrlTask
 **
 ** Author: 
 ** Date: 
 **
 **/

#include <portLib.h>
#include <ros/ros.h>
#include "server/pr2SoftMotionHeader.h"




/*------------------------------------------------------------------------
 * pr2SoftMotionSetTimeScaleCntrl  -  control codel of CONTROL request SetTimeScale
 *
 * Description:    
 * Report: OK
 *                 S_pr2SoftMotion_WRONG_VALUE
 *
 * Returns:    OK or ERROR
 */
STATUS 
pr2SoftMotionSetTimeScaleCntrl(PR2SM_timeScale* timeScale, int *report)
{
  for(int i=0; i<5; ++i){
    if(timeScale->timescale[i] > 1.0) {
      SDI_F->timeScale.timescale[i] = 1.0;
      printf("WARN: Wrong timeScale value, set it to 1.0\n");
      *report = S_pr2SoftMotion_WRONG_VALUE;
      return OK;
    }
    if(timeScale->timescale[i] < -1.0) {
      SDI_F->timeScale.timescale[i] = -1.0;
      printf("WARN: Wrong timeScale value, set it to -1.0\n");
      *report = S_pr2SoftMotion_WRONG_VALUE;
      return OK;
    }
  }
  return OK;
}


/*------------------------------------------------------------------------
 * pr2SoftMotionHoldSmthgCntrl  -  control codel of CONTROL request HoldSmthg
 *
 * Description:    
 * Report: OK
 *                 S_pr2SoftMotion_WRONG_VALUE
 *
 * Returns:    OK or ERROR
 */

STATUS
pr2SoftMotionHoldSmthgCntrl(int *report)
{
  ros::spinOnce();

  return OK;
}

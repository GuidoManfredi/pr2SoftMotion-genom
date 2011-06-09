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


#ifndef _PR2SMSTRUCT_H
#define _PR2SMSTRUCT_H

#include "pr2SoftMotionConst.h"

typedef enum PR2SM_ROBOT_PART_ENUM {
  PR2SM_PR2,
  PR2SM_PR2SYN,
  PR2SM_TORSO,
  PR2SM_HEAD,
  PR2SM_RARM,
  PR2SM_LARM
} PR2SM_ROBOT_PART_ENUM;

typedef enum PR2SM_gripperGrabRelease {
  PR2SM_GRAB,
  PR2SM_RELEASE,
  PR2SM_OPEN,
  PR2SM_CLOSE
} PR2SM_gripperGrabRelease;

typedef enum PR2SM_TRACK_MODE_ENUM {
  PR2SM_TRACK_FILE,
  PR2SM_TRACK_POSTER
} PR2SM_TRACK_MODE_ENUM;

typedef struct PR2SM_TRACK_STR {
  GEN_STRING128       posterName;
  PR2SM_TRACK_MODE_ENUM trackMode;
  PR2SM_ROBOT_PART_ENUM robotPart;
} PR2SM_TRACK_STR;

typedef struct PR2SM_QSTR{
  PR2SM_ROBOT_PART_ENUM robotPart;
  int relatif; // we use a int for 64 bits compatibility
  double torso;
  double head_pan;
  double head_tilt;
  double laser_tilt;
  double r_shoulder_pan;
  double r_shoulder_lift;
  double r_upper_arm_roll;
  double r_elbow_flex;
  double r_forearm_roll;
  double r_wrist_flex;
  double r_writ_roll;
  double r_gripper;
  double r_gripper_false;
  double l_shoulder_pan;
  double l_shoulder_lift;
  double l_upper_arm_roll;
  double l_elbow_flex;
  double l_forearm_roll;
  double l_wrist_flex;
  double l_wrist_roll;
  double l_gripper;
  double l_gripper_false;
} PR2SM_QSTR;

typedef struct PR2SM_xyzHead {
  double x;
  double y;
  double z;
  char frame[56];
} PR2SM_xyzHead;

typedef struct PR2SM_gripperSensorTresh {
  double grabAcc;
  double grabSlip;
  double releaseAcc;
  double releaseSlip;
  double holdForce;
} PR2SM_gripperSensorTresh;
#endif

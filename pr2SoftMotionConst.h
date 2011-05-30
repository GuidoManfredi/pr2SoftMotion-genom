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


#ifndef _PR2SMCONST_H
#define _PR2SMCONST_H

#define PR2SM_PERIOD_TIC 10 /* 1 tic = 10 ms */
#define PR2SM_PERIOD 0.1  /* s */

#define PR2SM_NBJOINT 22
#define PR2SM_ARM_NBJOINTS 7
#define PR2SM_GRIPPER_NBJOINTS 2
#define PR2SM_HEAD_NBJOINTS 2
#define PR2SM_TORSO_NBJOINTS 1
#define PR2SM_LASER_NBJOINTS 1

#define  TORSO_MAXVEL               0.013             
#define  HEAD_PAN_MAXVEL            6
#define  HEAD_TILT_MAXVEL           5
#define  LASER_TILT_MAXVEL          10

#define  TORSO_MAXACC               1.0             
#define  HEAD_PAN_MAXACC            6
#define  HEAD_TILT_MAXACC           5
#define  LASER_TILT_MAXACC          10

// Maximum velocities from the URDF file 
// #define  R_SHOULDER_PAN_MAXVEL      2.088
// #define  R_SHOULDER_LIFT_MAXVEL     2.082
// #define  R_UPPER_ARM_ROLL_MAXVEL    3.27
// #define  R_ELBOW_FLEX_MAXVEL        3.3
// #define  R_FOREARM_ROLL_MAXVEL      3.3
// #define  R_WRIST_FLEX_MAXVEL        3.078
// #define  R_WRIT_ROLL_MAXVEL         3.6
// #define  R_GRIPPER_MAXVEL           0.9
// #define  R_GRIPPER_FALSE_MAXVEL     0.9
// 
// #define  L_SHOULDER_PAN_MAXVEL      2.088
// #define  L_SHOULDER_LIFT_MAXVEL     2.082
// #define  L_UPPER_ARM_ROLL_MAXVEL    3.27
// #define  L_ELBOW_FLEX_MAXVEL        3.3
// #define  L_FOREARM_ROLL_MAXVEL      3.3
// #define  L_WRIST_FLEX_MAXVEL        3.078
// #define  L_WRIST_ROLL_MAXVEL        3.6
// #define  L_GRIPPER_MAXVEL           0.9
// #define  L_GRIPPER_FALSE_MAXVEL     0.9


// Velocities used by the motion planner
#define  R_SHOULDER_PAN_MAXVEL      1.5
#define  R_SHOULDER_LIFT_MAXVEL     1.52
#define  R_UPPER_ARM_ROLL_MAXVEL    0.27
#define  R_ELBOW_FLEX_MAXVEL        1.5
#define  R_FOREARM_ROLL_MAXVEL      1.5
#define  R_WRIST_FLEX_MAXVEL        1.5
#define  R_WRIT_ROLL_MAXVEL         1.5
#define  R_GRIPPER_MAXVEL           0.9
#define  R_GRIPPER_FALSE_MAXVEL     0.9

#define  L_SHOULDER_PAN_MAXVEL      1.5
#define  L_SHOULDER_LIFT_MAXVEL     1.52
#define  L_UPPER_ARM_ROLL_MAXVEL    0.27
#define  L_ELBOW_FLEX_MAXVEL        1.5
#define  L_FOREARM_ROLL_MAXVEL      1.5
#define  L_WRIST_FLEX_MAXVEL        1.5
#define  L_WRIST_ROLL_MAXVEL        1.5
#define  L_GRIPPER_MAXVEL           0.9
#define  L_GRIPPER_FALSE_MAXVEL     0.9

// Acceleration used by the motion planner
#define  R_SHOULDER_PAN_MAXACC      1.0
#define  R_SHOULDER_LIFT_MAXACC     1.0
#define  R_UPPER_ARM_ROLL_MAXACC    1.0
#define  R_ELBOW_FLEX_MAXACC        1.0
#define  R_FOREARM_ROLL_MAXACC      1.0
#define  R_WRIST_FLEX_MAXACC        1.0
#define  R_WRIT_ROLL_MAXACC         1.0
#define  R_GRIPPER_MAXACC           1.0
#define  R_GRIPPER_FALSE_MAXACC     1.0

#define  L_SHOULDER_PAN_MAXACC      1.0
#define  L_SHOULDER_LIFT_MAXACC     1.0
#define  L_UPPER_ARM_ROLL_MAXACC    1.0
#define  L_ELBOW_FLEX_MAXACC        1.0
#define  L_FOREARM_ROLL_MAXACC      1.0
#define  L_WRIST_FLEX_MAXACC        1.0
#define  L_WRIST_ROLL_MAXACC        1.0
#define  L_GRIPPER_MAXACC           1.0
#define  L_GRIPPER_FALSE_MAXACC     1.0
#endif

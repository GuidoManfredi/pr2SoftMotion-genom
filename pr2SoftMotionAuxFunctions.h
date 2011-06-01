#ifndef PR2SM_AUXFUNCTIONS
#define PR2SM_AUXFUNCTIONS

#include "softMotion/softMotion.h"
#include "softMotion/softMotionStruct.h"
#include "softMotion/softMotionStructGenom.h"

#include <tf/transform_listener.h>
#include "sensor_msgs/JointState.h"
#include "pr2_controllers_msgs/QueryTrajectoryState.h"
#include "pr2_controllers_msgs/JointTrajectoryControllerState.h"

static void doubles2QStr(double* src, PR2SM_QSTR* dst)
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

static void QStr2doubles(PR2SM_QSTR* src, double* dst)
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

static int smConvertSM_MOTIONtoSM_TRAJ( SM_MOTION_MONO motion[], int nbJoints, SM_TRAJ &traj, int *report) {
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

#endif 

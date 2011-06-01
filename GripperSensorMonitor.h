#ifndef PR2SM_GRIPPERSENSORMONITOR
#define PR2SM_GRIPPERSENSORMONITOR

#include <ros/ros.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <pr2_gripper_sensor_msgs/PR2GripperFindContactAction.h>
#include <pr2_gripper_sensor_msgs/PR2GripperSlipServoAction.h>
#include <pr2_gripper_sensor_msgs/PR2GripperEventDetectorAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<pr2_gripper_sensor_msgs::PR2GripperSlipServoAction> SlipClient;
// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_gripper_sensor_msgs::PR2GripperFindContactAction> ContactClient;
// Our Action interface type, provided as a typedef for convenience                   
typedef actionlib::SimpleActionClient<pr2_gripper_sensor_msgs::PR2GripperEventDetectorAction> EventDetectorClient;
// Our Action interface type, provided as a typedef for convenience                   
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;
// Our Action interface type, provided as a typedef for convenience  

class GripperSensorMonitor{
  private:
    GripperClient* gripper_client_;  
    ContactClient* contact_client_;
    SlipClient* slip_client_;
    EventDetectorClient* event_detector_client_;

  public:
    GripperSensorMonitor();
    ~GripperSensorMonitor();

    bool grab(double accT, double slipT);
    bool release(double accT, double slipT);
    bool open();
    bool findTwoContacts();
    void slipServo();
    bool detect(double accT, double slipT);
};

#endif

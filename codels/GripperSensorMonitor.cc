#include "GripperSensorMonitor.h"

GripperSensorMonitor::GripperSensorMonitor(ros::NodeHandle* nh){

    //Initialize the client for the Action interface to the gripper controller
    //and tell the action client that we want to spin a thread by default
    gripper_client_ = new GripperClient("r_gripper_sensor_controller/gripper_action", true);
    contact_client_  = new ContactClient("r_gripper_sensor_controller/find_contact",true);
    slip_client_  = new SlipClient("r_gripper_sensor_controller/slip_servo",true);
    event_detector_client_  = new EventDetectorClient("r_gripper_sensor_controller/event_detector",true);
    force_client_  = new ForceClient("r_gripper_sensor_controller/force_servo",true);

    //wait for the gripper action server to come up 
    while(!gripper_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the r_gripper_sensor_controller/gripper_action action server to come up");
    }

    while(!contact_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the r_gripper_sensor_controller/find_contact action server to come up");
    }
    
    while(!slip_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the r_gripper_sensor_controller/slip_servo action server to come up");
    }    

    while(!event_detector_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the r_gripper_sensor_controller/event_detector action server to come up");
    } 

    while(!force_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the r_gripper_sensor_controller/force_servo action server to come up");
    }

  gripper_state_sub_= nh->subscribe("r_gripper_sensor_controller/contact_state", 1, &GripperSensorMonitor::contactStateCB, this);
  gripper_position_sub_= nh->subscribe("r_gripper_controller/state", 1, &GripperSensorMonitor::positionCB, this);

  //default values, they doesn't really  mean something.
  detectActive_= false;
  padContact_ = false;
  handClosed_ = false;
}

GripperSensorMonitor::~GripperSensorMonitor(){
    delete gripper_client_;
    delete contact_client_;
    delete slip_client_;
    delete event_detector_client_;
    delete force_client_;
}

void GripperSensorMonitor::stopAll(){

  if(detectActive_)
    event_detector_client_->cancelGoal();
}

//Open the gripper
bool GripperSensorMonitor::open(){
    pr2_controllers_msgs::Pr2GripperCommandGoal open;
    open.command.position = 0.09;    // position open (9 cm)
    open.command.max_effort = -1.0;  // unlimited motor effort

    ROS_INFO("Sending open goal");
    gripper_client_->sendGoal(open);    
}

  //Find two contacts on the robot gripper
void GripperSensorMonitor::findTwoContacts(){
    pr2_gripper_sensor_msgs::PR2GripperFindContactGoal findTwo;
    findTwo.command.contact_conditions = findTwo.command.BOTH;  // close until both fingers contact
    findTwo.command.zero_fingertip_sensors = true;   // zero fingertip sensor values before moving
 
    ROS_INFO("Sending find 2 contact goal");
    contact_client_->sendGoal(findTwo);
}

bool GripperSensorMonitor::findTwoContactsWait(){
    if(contact_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Contact found. Left: %d, Right: %d", contact_client_->getResult()->data.left_fingertip_pad_contact, 
               contact_client_->getResult()->data.right_fingertip_pad_contact);
      ROS_INFO("Contact force. Left: %f, Right: %f", contact_client_->getResult()->data.left_fingertip_pad_force, 
           contact_client_->getResult()->data.right_fingertip_pad_force);
      return true;
    }
    else{
      return false;
    }
}

void GripperSensorMonitor::detect(double accT, double slipT){
    //printf("%f %f\n", accT, slipT);

    pr2_gripper_sensor_msgs::PR2GripperEventDetectorGoal goal;
    goal.command.trigger_conditions = goal.command.FINGER_SIDE_IMPACT_OR_SLIP_OR_ACC;
    goal.command.acceleration_trigger_magnitude = (float)accT;  // set the contact acceleration to n m/s^2
    goal.command.slip_trigger_magnitude = (float)slipT;

    ROS_INFO("Waiting for contact with accT=%f and slipT=%f...", accT, slipT);
    event_detector_client_->sendGoal(goal);
    detectActive_ = true;
}

bool GripperSensorMonitor::detectWait(){
    if(event_detector_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Contact detected");
      ROS_INFO("cond met: %d, acc_met: %d, slip_met: %d", event_detector_client_->getResult()->data.trigger_conditions_met, event_detector_client_->getResult()->data.acceleration_event, event_detector_client_->getResult()->data.slip_event);
      detectActive_= false;
      return true;
    }
    else{
      return false;
    }
}

bool GripperSensorMonitor::hold(double holdForce){
  pr2_gripper_sensor_msgs::PR2GripperForceServoGoal squeeze;
  squeeze.command.fingertip_force = holdForce;   // hold with X N of force
    
  ROS_INFO("Sending hold goal");
  force_client_->sendGoal(squeeze);
  //wait only for 3 seconds in order not to block.
  force_client_->waitForResult(ros::Duration(3.0));
  if(force_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Stable force was achieved");
    return true;
  }
  else{
    ROS_INFO("Stable force was NOT achieved");
    return false;
  }
}

void GripperSensorMonitor::contactStateCB(const pr2_gripper_sensor_msgs::PR2GripperFindContactDataConstPtr& msg)
{
  if(msg->left_fingertip_pad_contact && msg->right_fingertip_pad_contact){
    //printf("Contact on pads\n");
    padContact_= true;
  }
  else{
    //printf("No contact on pads\n");
    padContact_= false;
  }

  if(padContact_ && !handClosed_)
    SDI_F->holdSmthg= GEN_TRUE;
  else
    SDI_F->holdSmthg= GEN_FALSE;
}

void  GripperSensorMonitor::positionCB(const pr2_controllers_msgs::JointControllerStateConstPtr& msg)
{
  // 0.0028 is the value when the gripper is closed 
  if( msg->process_value <= 0.0032){
    //printf("Hand closed\n");
    handClosed_ = true;
  }
  else{
    //printf("Hand still open\n");
    handClosed_ = false;
  }

  if(padContact_ && !handClosed_)
    SDI_F->holdSmthg= GEN_TRUE;
  else
    SDI_F->holdSmthg= GEN_FALSE;   
}

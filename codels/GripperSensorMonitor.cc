#include "GripperSensorMonitor.h"

GripperSensorMonitor::GripperSensorMonitor(){

    //Initialize the client for the Action interface to the gripper controller
    //and tell the action client that we want to spin a thread by default
    gripper_client_ = new GripperClient("r_gripper_sensor_controller/gripper_action", true);
    contact_client_  = new ContactClient("r_gripper_sensor_controller/find_contact",true);
    slip_client_  = new SlipClient("r_gripper_sensor_controller/slip_servo",true);
    event_detector_client_  = new EventDetectorClient("r_gripper_sensor_controller/event_detector",true);


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
}

GripperSensorMonitor::~GripperSensorMonitor(){
    delete gripper_client_;
    delete contact_client_;
    delete slip_client_;
    delete event_detector_client_;
}

//Open the gripper, find contact on both fingers, and go into slip-servo control mode
bool GripperSensorMonitor::grab(double grabAcc, double grabSlip){
  if( detect(grabAcc, grabSlip) )
    if( findTwoContacts() ){
      slipServo();
      return true;
    }
  
  return false;
}

// Look for side impact, finerpad slip, or contact acceleration signals and release the object once these occur
bool GripperSensorMonitor::release(double releaseAcc, double releaseSlip){
  if (detect(releaseAcc, releaseSlip) )
    return open();

  return false;
}

//Open the gripper
bool GripperSensorMonitor::open(){
    pr2_controllers_msgs::Pr2GripperCommandGoal open;
    open.command.position = 0.09;    // position open (9 cm)
    open.command.max_effort = -1.0;  // unlimited motor effort

    ROS_INFO("Sending open goal");
    gripper_client_->sendGoal(open);
    gripper_client_->waitForResult();
    if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("The gripper opened!");
      return true;
    }
    else{
      ROS_INFO("The gripper failed to open.");
      return false;
    }
}

  //Find two contacts on the robot gripper
bool GripperSensorMonitor::findTwoContacts(){
    pr2_gripper_sensor_msgs::PR2GripperFindContactGoal findTwo;
    findTwo.command.contact_conditions = findTwo.command.BOTH;  // close until both fingers contact
    findTwo.command.zero_fingertip_sensors = true;   // zero fingertip sensor values before moving
 
    ROS_INFO("Sending find 2 contact goal");
    contact_client_->sendGoal(findTwo);
    contact_client_->waitForResult(ros::Duration(5.0));
    if(contact_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Contact found. Left: %d, Right: %d", contact_client_->getResult()->data.left_fingertip_pad_contact, 
               contact_client_->getResult()->data.right_fingertip_pad_contact);
      ROS_INFO("Contact force. Left: %f, Right: %f", contact_client_->getResult()->data.left_fingertip_pad_force, 
           contact_client_->getResult()->data.right_fingertip_pad_force);
      return true;
    }
    else{
      ROS_INFO("The gripper did not find a contact.");\
      return false;
    }
}

  //Slip servo
void GripperSensorMonitor::slipServo(){
    pr2_gripper_sensor_msgs::PR2GripperSlipServoGoal slip_goal;

    ROS_INFO("Slip Servoing");
    slip_client_->sendGoal(slip_goal);
    if(slip_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("You Should Never See This Message!"); 
    else
      ROS_INFO("SlipServo Action returned without success.");
    
} 

bool GripperSensorMonitor::detect(double accT, double slipT){
    //printf("%f %f\n", accT, slipT);

    pr2_gripper_sensor_msgs::PR2GripperEventDetectorGoal goal;
    goal.command.trigger_conditions = goal.command.FINGER_SIDE_IMPACT_OR_SLIP_OR_ACC;
    goal.command.acceleration_trigger_magnitude = (float)accT;  // set the contact acceleration to n m/s^2
    goal.command.slip_trigger_magnitude = (float)slipT;

    ROS_INFO("Waiting for contact with accT=%f and slipT=%f...", accT, slipT);
    event_detector_client_->sendGoal(goal);
    event_detector_client_->waitForResult();
    if(event_detector_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Contact detected");
      ROS_INFO("cond met: %d, acc_met: %d, slip_met: %d", event_detector_client_->getResult()->data.trigger_conditions_met, event_detector_client_->getResult()->data.acceleration_event, event_detector_client_->getResult()->data.slip_event);
      return true;
    }
    else{
      ROS_INFO("Contact detection failure");
      return false;
    }
}

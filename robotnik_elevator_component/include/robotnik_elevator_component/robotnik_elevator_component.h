#ifndef _ROBOTNIK_ELEVATOR_COMPONENT_H_
#define _ROBOTNIK_ELEVATOR_COMPONENT_H_

#include <rcomponent/rcomponent.h>
#include <robotnik_elevator_interface_msgs/ElevatorState.h>
#include <robotnik_elevator_interface_msgs/SetElevatorControl.h>
#include <robotnik_elevator_interface_msgs/SetDoorState.h>
#include <robotnik_elevator_interface_msgs/GoToFloor.h>

#define DEFAULT_MAX_CONTROL_TIME_IN_IDLE	60.0
#define DEFAULT_FREQ	10.0


enum{
	ELEVATOR_CONTROL_OK = 0,
	ELEVATOR_RELEASE_OK = 1,
	ELEVATOR_OPEN_OK = 2,
	ELEVATOR_CLOSE_OK = 3,
	ELEVATOR_FLOOR_OK = 4,
	ELEVATOR_UNDER_CONTROL_BY_OTHER = -1,
	ELEVATOR_UNDER_CONTROL_BY_YOU = -2,
	ELEVATOR_NOT_UNDER_CONTROL = -3,
	ELEVATOR_WRONG_TOKEN = -4,
	ELEVATOR_UNKNOWN_STATUS = -5,
	ELEVATOR_ERROR_TAKING_CONTROL = -6,
	ELEVATOR_ERROR_RELEASING_CONTROL = -7,
	ELEVATOR_ERROR_OPEN = -8,
	ELEVATOR_ERROR_CLOSE = -9,
	ELEVATOR_ERROR_DOOR_STATE = -10,
	ELEVATOR_ERROR_FLOOR = -11,
	ELEVATOR_ERROR_INVALID_FLOOR = -12,
};	
	
class RobotnikElevatorComponent : public rcomponent::RComponent
{
protected:
 
  robotnik_elevator_interface_msgs::ElevatorState elevator_state;
  //! Publish the component state
  ros::Publisher elevator_state_publisher;
  ros::ServiceServer set_elevator_control_service_server, set_door_state_service_server, go_to_floor_service_server;  
  // Key to control the elevator. Has to be set in setElevatorControl service
  std::string control_token;
  
  // saves the time when the elevator starts the control
  ros::Time time_under_control;
  // saves the time of the last elevator action requested by the master
  ros::Time last_elevator_action;
  // defines the max time (seconds) of the elevator without receiving commands, in order to release the control
  double max_control_time_in_idle;
  
public:
  RobotnikElevatorComponent(ros::NodeHandle h)
    : rcomponent::RComponent(h)
  {
	  max_control_time_in_idle = DEFAULT_MAX_CONTROL_TIME_IN_IDLE;
	  desired_freq_ = DEFAULT_FREQ;
	  rosReadParams();
  }

  virtual ~RobotnikElevatorComponent()
  {
  }

  /* Rcomponent stuff */
  void initState();
  //void standbyState();
  //void readyState();
  //void allState();
  int rosSetup();
  void rosReadParams();
  //int setup();
  //int stop();
  //std::string getStateString();
  void rosPublish();
  //int start();
  
  bool setElevatorControlServiceServerCb(robotnik_elevator_interface_msgs::SetElevatorControl::Request& request, robotnik_elevator_interface_msgs::SetElevatorControl::Response& response);
  bool setDoorStateServiceServerCb(robotnik_elevator_interface_msgs::SetDoorState::Request& request, robotnik_elevator_interface_msgs::SetDoorState::Response& response);
  bool goToFloorServiceServerCb(robotnik_elevator_interface_msgs::GoToFloor::Request& request, robotnik_elevator_interface_msgs::GoToFloor::Response& response);
  

protected:
 
  void initElevatorState();
  void switchToElevatorStatus(std::string new_status);
  void switchToDoorStatus(std::string new_status);
  //! method called when someone takes the elevator control
  int takeControl(std::string control_token, std::string master_id);
  //! method called when someone releases the elevator control
  int releaseControl(std::string control_token, std::string master_id);
  //! returns the string for a type of return message
  std::string elevatorControlReturnMsgs(int msg_id);
  
  // Function to define for each specific elevator
  virtual int takeElevatorControl();
  virtual int releaseElevatorControl();
  virtual int goToFloor(int floor);
  virtual int openDoor();
  virtual int closeDoor();
  //! checks if it's a valid floor number
  virtual int checkValidFloor(int floor); 
  
  
};

#endif  //_ROBOTNIK_ELEVATOR_COMPONENT_H_

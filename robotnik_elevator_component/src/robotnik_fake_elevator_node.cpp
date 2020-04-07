#include <ros/ros.h>

#include <robotnik_elevator_component/robotnik_elevator_component.h>



class RobotnikFakeElevatorNode : public RobotnikElevatorComponent
{
public:
 
public:
  RobotnikFakeElevatorNode(ros::NodeHandle h)
    : RobotnikElevatorComponent(h)
  {
	  rosReadParams();
  }

  virtual ~RobotnikFakeElevatorNode()
  {
  }

  /* Rcomponent stuff */
  /*virtual void initState();
  virtual void standbyState();
  virtual void readyState();
  virtual void allState();
  virtual int rosSetup();
  virtual void rosReadParams();
  virtual int setup();
  virtual int stop();
  virtual std::string getStateString();
  virtual void rosPublish();
  
  virtual bool setElevatorControlServiceServerCb(robotnik_elevator_interface_msgs::SetElevatorControl::Request& request, robotnik_elevator_interface_msgs::SetElevatorControl::Response& response);
  virtual bool setDoorStateServiceServerCb(robotnik_elevator_interface_msgs::SetDoorState::Request& request, robotnik_elevator_interface_msgs::SetDoorState::Response& response);
  virtual bool goToFloorServiceServerCb(robotnik_elevator_interface_msgs::GoToFloor::Request& request, robotnik_elevator_interface_msgs::GoToFloor::Response& response);
  */
protected:
 
 /* robotnik_elevator_interface_msgs::ElevatorState elevator_state;
  
  //! Publish the component state
  ros::Publisher elevator_state_publisher;
  ros::ServiceServer set_elevator_control_service_server, set_door_state_service_server, go_to_floor_service_server;  
  
  void initElevatorState();
  void switchToElevatorStatus(std::string new_status);*/
 /* int takeElevatorControl();
  int releaseElevatorControl();
  int goToFloor(int floor);
  int openDoor();
  int closeDoor();*/
  
public:
  
	void standbyState(){
		switchToState(robotnik_msgs::State::READY_STATE);
		switchToElevatorStatus(robotnik_elevator_interface_msgs::ElevatorState::ELEVATOR_STATUS_IDLE);
	}
  
	int takeElevatorControl(){
		return 0;
	}

	int releaseElevatorControl(){
		return 0;
	}

	int goToFloor(int floor){
		elevator_state.current_floor = floor;
		elevator_state.target_floor = floor;
		return 0;
	}

	int openDoor(){
		elevator_state.door_status = robotnik_elevator_interface_msgs::ElevatorState::DOOR_STATUS_OPEN;
		return 0;
	}

	int closeDoor(){
		elevator_state.door_status = robotnik_elevator_interface_msgs::ElevatorState::DOOR_STATUS_CLOSE;		
		return 0;
	}
	
	void rosReadParams(){
	}

  
};


// MAIN
int main(int argc, char** argv)
{
  ros::init(argc, argv, "robotnik_elevator_node");

  ros::NodeHandle n;
  RobotnikFakeElevatorNode controller(n);

  controller.start();

  return (0);
}

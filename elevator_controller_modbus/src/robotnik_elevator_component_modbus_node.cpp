/**
 * This node is intended to communicate with the elevator controller modbus node
 * */

#include <ros/ros.h>

#include <robotnik_elevator_component/robotnik_elevator_component.h>
#include <robotnik_msgs/inputs_outputs.h>
#include <robotnik_msgs/set_digital_output.h>

using namespace robotnik_elevator_component;


class RobotnikModbusElevatorNode : public RobotnikElevatorComponent
{
public:
 
public:
  RobotnikModbusElevatorNode(ros::NodeHandle h)
    : RobotnikElevatorComponent(h)
  {
  }

  virtual ~RobotnikModbusElevatorNode()
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
  
  ros::Subscriber modbus_io_subscriber;
  ros::ServiceClient modbus_write_digital_output_service_client;
  unsigned int take_control_output_;
  unsigned int take_control_input_;
  
  /*
   floor_0_out: 1
	floor_1_out: 2
	floor_2_out: 3
	cabin_out: 4
	door_out: 5

	#inputs:
	floor_0_in: 1
	floor_1_in: 2
	floor_2_in: 3
	presence_in: 4
	door_state_in: 5
   * */
  
public:
  
    int rosSetup(){
		RobotnikElevatorComponent::rosSetup();
		
		modbus_io_subscriber = nh_.subscribe("elevator_controller_interface/input_output", 10,  &RobotnikModbusElevatorNode::modbusIOCallback, this);
		modbus_write_digital_output_service_client = nh_.serviceClient<robotnik_msgs::set_digital_output>("elevator_controller_interface/write_digital_output");

		return 0;
	}
	
	
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

    void modbusIOCallback(const robotnik_msgs::inputs_outputsConstPtr& message)
	{
	  RCOMPONENT_INFO("received io msg");
	}
  
};


// MAIN
int main(int argc, char** argv)
{
  ros::init(argc, argv, "robotnik_elevator_modbus_node");

  ros::NodeHandle n;
  RobotnikModbusElevatorNode controller(n);

  controller.start();

  return (0);
}

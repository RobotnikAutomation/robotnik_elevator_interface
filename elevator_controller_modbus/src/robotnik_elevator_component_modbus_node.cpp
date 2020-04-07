/**
 * This node is intended to communicate with the elevator controller modbus node
 * */

#include <ros/ros.h>

#include <robotnik_elevator_component/robotnik_elevator_component.h>
#include <robotnik_msgs/inputs_outputs.h>
#include <robotnik_msgs/set_digital_output.h>



class RobotnikModbusElevatorNode : public RobotnikElevatorComponent
{
public:
 
public:
  RobotnikModbusElevatorNode(ros::NodeHandle h)
    : RobotnikElevatorComponent(h)
  {
	  rosReadParams();
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
  int closeDoor();
  int checkValidFloor();
  * */
  
  ros::Subscriber modbus_io_subscriber;
  ros::ServiceClient modbus_write_digital_output_service_client;
  
  int take_control_output_;
  std::vector<int> door_control_; // [output, input]  
  std::map<int, std::vector<int>> served_floors_;  // number of floor -> [output, input]

   //! timeout to consider that the topic is not receiving
  double topic_timeout_;
  //! saves the ros time of the last msg received
  ros::Time modbus_io_last_msg_time_;
  
  
public:
  
    int rosSetup(){
		RobotnikElevatorComponent::rosSetup();
		
		modbus_io_subscriber = nh_.subscribe("elevator_controller_interface/input_output", 10,  &RobotnikModbusElevatorNode::modbusIOCallback, this);
		modbus_write_digital_output_service_client = nh_.serviceClient<robotnik_msgs::set_digital_output>("elevator_controller_interface/write_digital_output");
		
		modbus_io_last_msg_time_ = ros::Time(0);
		topic_timeout_ = 5.0;

		return 0;
	}
	
	void rosReadParams(){
		pnh_.param("take_control_output", take_control_output_, 4);
		if(take_control_output_ <= 0){
			RCOMPONENT_ERROR("take_control_output has to be > 0 (%d)", take_control_output_);
			exit(-1);
		}
		
		// TODO: read from params
		//indice es el piso : salida_floor y entrada_floor
		served_floors_[0]={1,1}; 
		served_floors_[1]={2,2}; 
		served_floors_[2]={3,3}; 
		
		door_control_={5,5};	 
		pnh_.param("door_control_output", door_control_[0], 5);
		pnh_.param("door_control_input", door_control_[1], 5);
		if(door_control_[0] <= 0 || door_control_[1] <= 0){
			RCOMPONENT_ERROR("door_control i/o have to be > 0. out, in = (%d, %d)", door_control_[0], door_control_[1]);
			exit(-1);
		}
		
	}
	
	void standbyState(){
		
		if(isReceivingIO())
			switchToState(robotnik_msgs::State::READY_STATE);
		else{
			ROS_ERROR("Not receiving I/O state");
			switchToState(robotnik_msgs::State::EMERGENCY_STATE);
		}
		//switchToElevatorStatus(robotnik_elevator_interface_msgs::ElevatorState::ELEVATOR_STATUS_IDLE);
	}
	
	void readyState(){
		
		if(not isReceivingIO()){
			ROS_ERROR("Not receiving I/O state");
			switchToState(robotnik_msgs::State::EMERGENCY_STATE);
		}
	}
	
	void emergencyState(){
		if(isReceivingIO())
			switchToState(robotnik_msgs::State::READY_STATE);	
	}
  
	int takeElevatorControl(){
		robotnik_msgs::set_digital_output srv;
		
		srv.request.output = take_control_output_;
		srv.request.value = true;
			
		if(not modbus_write_digital_output_service_client.call(srv)){
			RCOMPONENT_ERROR("Error calling service");
			return -1;
		}
		
		return 0;
	}

	int releaseElevatorControl(){
		robotnik_msgs::set_digital_output srv;
		
		srv.request.output = take_control_output_;
		srv.request.value = false;
			
		if(not modbus_write_digital_output_service_client.call(srv)){
			RCOMPONENT_ERROR("Error calling service");
			return -1;
		}
		
		return 0;
	}


	//! checks if it's a valid floor number
	//! 0 is ok, -1 invalid
	int checkValidFloor(int floor){
		
		std::map<int, std::vector<int>>::iterator it;
		it = served_floors_.find(floor);
		// The floor is available
		if (it == served_floors_.end())
			return -1;
			
	
		return 0;
	}

	
	int goToFloor(int floor){

		robotnik_msgs::set_digital_output srv;
		
		srv.request.output = served_floors_[floor][0];
		srv.request.value = true;
			
		if(not modbus_write_digital_output_service_client.call(srv)){
			RCOMPONENT_ERROR("Error calling service");
			return -1;
		}
		
		elevator_state.target_floor = floor;
		

		srv.request.output = served_floors_[floor][0];
		srv.request.value = false;
			
		if(not modbus_write_digital_output_service_client.call(srv)){
			RCOMPONENT_ERROR("Error calling service");
			return -1;
		}

		
		return 0;
	}
	

	int openDoor(){
		robotnik_msgs::set_digital_output srv;
		
		srv.request.output = door_control_[0];
		srv.request.value = true;
			
		if(not modbus_write_digital_output_service_client.call(srv)){
			RCOMPONENT_ERROR("Error calling service");
			return -1;
		}
		
		return 0;
	}

	int closeDoor(){
		robotnik_msgs::set_digital_output srv;
		
		srv.request.output = door_control_[0];
		srv.request.value = false;
			
		if(not modbus_write_digital_output_service_client.call(srv)){
			RCOMPONENT_ERROR("Error calling service");
			return -1;
		}
			
		return 0;
	}
	
    void modbusIOCallback(const robotnik_msgs::inputs_outputsConstPtr& message)
	{
		RCOMPONENT_INFO("received io msg");
		modbus_io_last_msg_time_ = ros::Time::now();
		processIO(*message);
	}
	
	
	// True if topic is receiving data
	bool isReceivingIO(){
		
		if (modbus_io_subscriber == 0 or modbus_io_subscriber.getNumPublishers() == 0)
		  return false;

		if ((ros::Time::now() - modbus_io_last_msg_time_).toSec() > topic_timeout_)
		  return false;

		return true;	
	};
	
	// Process the i/o state 
	void processIO(robotnik_msgs::inputs_outputs io){

		RCOMPONENT_INFO("processIO");
		
		//
		//elevator_state.door_status = robotnik_elevator_interface_msgs::ElevatorState::DOOR_STATUS_OPEN;
		//elevator_state.door_status = robotnik_elevator_interface_msgs::ElevatorState::DOOR_STATUS_CLOSE;	
		
		int size_inputs = io.digital_inputs.size();
		int size_outputs = io.digital_outputs.size();
		
		// check elevator_control
		take_control_output_ = 8;
		if( take_control_output_ > size_outputs){
			RCOMPONENT_WARN_THROTTLE(5, "take control output (%d) is greater than the number of received outputs (%d)", take_control_output_, size_outputs);
		}else{
			if(io.digital_outputs[take_control_output_ - 1]){
				elevator_state.under_control = true;
			}else
				elevator_state.under_control = false;
		}
		
		// check current floor 		
		for (std::map<int, std::vector<int>>::iterator it=served_floors_.begin(); it!=served_floors_.end(); ++it){
			//it->first (floor number)
			//it->second (io associated to the floor)
			int floor_number = it->first;
			int floor_input = it->second[1];
			if(floor_input > size_inputs){
				RCOMPONENT_WARN_THROTTLE(5, "input (%d) for floor (%d) is greater than the number of received inputs (%d)", floor_input, floor_number, size_inputs);
			}else{
				if(io.digital_inputs[floor_input - 1]){
					RCOMPONENT_INFO("floor_input: %d", floor_input);
					elevator_state.current_floor = floor_number;


				}	
			}
		}
		
		// check door status
		// output to open_door -> door_control_[0]
		// input to read door if opened -> door_control[1]
		int door_output = door_control_[0];
		int door_input = door_control_[1];
		
		if( door_output > size_outputs){
			RCOMPONENT_WARN_THROTTLE(5, "door control output (%d) is greater than the number of received outputs (%d)", door_output, size_outputs);
			switchToDoorStatus(robotnik_elevator_interface_msgs::ElevatorState::DOOR_STATUS_UNKNOWN);
		}else if( door_input > size_inputs){
			RCOMPONENT_WARN_THROTTLE(5, "door control input (%d) is greater than the number of received inputs (%d)", door_input, size_inputs);
			switchToDoorStatus(robotnik_elevator_interface_msgs::ElevatorState::DOOR_STATUS_UNKNOWN);
		}else{
			if(io.digital_inputs[door_input-1])
				switchToDoorStatus(robotnik_elevator_interface_msgs::ElevatorState::DOOR_STATUS_CLOSE);
			else if(io.digital_outputs[door_output-1])
				switchToDoorStatus(robotnik_elevator_interface_msgs::ElevatorState::DOOR_STATUS_OPEN);
			else
				switchToDoorStatus(robotnik_elevator_interface_msgs::ElevatorState::DOOR_STATUS_OPEN);
		}
		
		
		// check elevator status 
		if(not elevator_state.under_control){
			switchToElevatorStatus(robotnik_elevator_interface_msgs::ElevatorState::ELEVATOR_STATUS_IDLE);
		}else if(elevator_state.current_floor != elevator_state.target_floor){
			switchToElevatorStatus(robotnik_elevator_interface_msgs::ElevatorState::ELEVATOR_STATUS_MOVING);
		}
		
		//
		//io.digital_inputs
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

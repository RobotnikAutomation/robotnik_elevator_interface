#include <ros/ros.h>

#include <robotnik_elevator_component/robotnik_elevator_component.h>


/*!	\fn int RobotnikElevatorComponent::rosSetup()
 * 	\brief Setups all ROS' stuff
*/
int RobotnikElevatorComponent::rosSetup(){
	
	// Checks if has been initialized
  if (ros_initialized)
  {
    RCOMPONENT_INFO("Already initialized");

    return rcomponent::INITIALIZED;
  }

  // PUBLISHERS
  state_publisher = pnh_.advertise<robotnik_msgs::State>("state", 1);
  elevator_state_publisher = pnh_.advertise<robotnik_elevator_interface_msgs::ElevatorState>("elevator_state", 1);
 
  // Service Servers
  set_elevator_control_service_server = pnh_.advertiseService("set_elevator_control", &RobotnikElevatorComponent::setElevatorControlServiceServerCb, this);
  set_door_state_service_server = pnh_.advertiseService("set_door_state", &RobotnikElevatorComponent::setDoorStateServiceServerCb, this);
  go_to_floor_service_server = pnh_.advertiseService("go_to_floor", &RobotnikElevatorComponent::goToFloorServiceServerCb, this);

  ros_initialized = true;

  return rcomponent::OK;
	
}


void RobotnikElevatorComponent::rosReadParams(){	
	pnh_.param<std::string>("elevator_id", elevator_state.elevator_id, "Elevator-1");
}


bool RobotnikElevatorComponent::setElevatorControlServiceServerCb(robotnik_elevator_interface_msgs::SetElevatorControl::Request& request, robotnik_elevator_interface_msgs::SetElevatorControl::Response& response)
{
  RCOMPONENT_INFO("Received server");
  
  if(request.under_control){
	int ret = takeControl(request.control_token, request.master_id);
	
	if(ret == ELEVATOR_CONTROL_OK or ret == ELEVATOR_UNDER_CONTROL_BY_YOU)
		response.success = true;
	else
		response.success = false;
	response.message = elevatorControlReturnMsgs(ret);
  }else{
	  
	int ret = releaseControl(request.control_token, request.master_id);
	
	if(ret == ELEVATOR_CONTROL_OK)
		response.success = true;
	else
		response.success = false;
		
	response.message = elevatorControlReturnMsgs(ret);
  }
  

  return true;
}


bool RobotnikElevatorComponent::setDoorStateServiceServerCb(robotnik_elevator_interface_msgs::SetDoorState::Request& request, robotnik_elevator_interface_msgs::SetDoorState::Response& response){
	
   RCOMPONENT_INFO("Received server");
   
   if(!elevator_state.under_control){
	   response.success = false;
	   response.message = elevatorControlReturnMsgs(ELEVATOR_NOT_UNDER_CONTROL);
	   return true;
   }
   
   if(request.control_token != this->control_token){
	   response.success = false;
	   response.message = elevatorControlReturnMsgs(ELEVATOR_WRONG_TOKEN);
	   return true;	   
   }
	
	   
	if(request.state == robotnik_elevator_interface_msgs::ElevatorState::DOOR_STATUS_OPEN){
		if(openDoor() == 0){
			response.success = true;
			response.message = elevatorControlReturnMsgs(ELEVATOR_OPEN_OK);
		}else{
			response.success = false;
			response.message = elevatorControlReturnMsgs(ELEVATOR_ERROR_OPEN);
		}
	}
	else if(request.state == robotnik_elevator_interface_msgs::ElevatorState::DOOR_STATUS_CLOSE){
		if(closeDoor() == 0){
			response.success = true;
			response.message = elevatorControlReturnMsgs(ELEVATOR_CLOSE_OK);
		}else{
			response.success = false;
			response.message = elevatorControlReturnMsgs(ELEVATOR_ERROR_CLOSE);
		}
		
	}else{
		response.success = false;
		response.message = elevatorControlReturnMsgs(ELEVATOR_ERROR_DOOR_STATE);
	}

	last_elevator_action = ros::Time::now();
	
	return true;
}


bool RobotnikElevatorComponent::goToFloorServiceServerCb(robotnik_elevator_interface_msgs::GoToFloor::Request& request, robotnik_elevator_interface_msgs::GoToFloor::Response& response){
	RCOMPONENT_INFO("Received server");

	if(!elevator_state.under_control){
	   response.success = false;
	   response.message = elevatorControlReturnMsgs(ELEVATOR_NOT_UNDER_CONTROL);
	   return true;
	}

	if(request.control_token != this->control_token){
	   response.success = false;
	   response.message = elevatorControlReturnMsgs(ELEVATOR_WRONG_TOKEN);
	   return true;	   
	}
	
    if(checkValidFloor(request.floor) != 0){
		response.success = false;
		response.message = elevatorControlReturnMsgs(ELEVATOR_ERROR_INVALID_FLOOR);
		return true;
	}
	
	if(goToFloor(request.floor) == 0){
		response.success = true;
		response.message = elevatorControlReturnMsgs(ELEVATOR_FLOOR_OK);
	}else{
		response.success = false;
		response.message = elevatorControlReturnMsgs(ELEVATOR_ERROR_FLOOR);
	}
	
	last_elevator_action = ros::Time::now();
   
	return true;
}


void RobotnikElevatorComponent::rosPublish(){
	rcomponent::RComponent::rosPublish();
	
	elevator_state_publisher.publish(elevator_state);
	
}


/*!	\fn void RComponent::initState()
 *	\brief Actions performed on initial
 * 	Setups the component
*/
void RobotnikElevatorComponent::initState()
{
  // If component setup is successful goes to STANDBY (or READY) state
  if (setup() != rcomponent::ERROR)
  {
	  
	initElevatorState();
    switchToState(robotnik_msgs::State::STANDBY_STATE);
  }
}

/*! Initialization of the elevator state
 * */
void RobotnikElevatorComponent::initElevatorState(){
	elevator_state.current_floor = -1000;
	elevator_state.target_floor = -1000;
	elevator_state.under_control = false;
	elevator_state.master_id = "";
	elevator_state.elevator_status = robotnik_elevator_interface_msgs::ElevatorState::ELEVATOR_STATUS_UNKNOWN;
	elevator_state.door_status = robotnik_elevator_interface_msgs::ElevatorState::DOOR_STATUS_UNKNOWN;
	elevator_state.cabin_presence_free = true;
}


/*! Method called to change states 
 */
void RobotnikElevatorComponent::switchToElevatorStatus(std::string new_status){
  if (new_status == elevator_state.elevator_status)
    return;


  RCOMPONENT_INFO("%s -> %s", elevator_state.elevator_status.c_str(), new_status.c_str());
  elevator_state.elevator_status = new_status;
}

/*! Method called to change states 
 */
void RobotnikElevatorComponent::switchToDoorStatus(std::string new_status){
  if (new_status == elevator_state.door_status)
    return;


  RCOMPONENT_INFO("%s -> %s", elevator_state.door_status.c_str(), new_status.c_str());
  elevator_state.door_status = new_status;
}


//! method called when someone takes the elevator control
//! return 0 if OK,-1 otherwise
int RobotnikElevatorComponent::takeControl(std::string control_token, std::string master_id){
	
	 if(elevator_state.elevator_status == robotnik_elevator_interface_msgs::ElevatorState::ELEVATOR_STATUS_UNKNOWN)
		return ELEVATOR_UNKNOWN_STATUS;
		
	 if(elevator_state.under_control){
		 
		if(elevator_state.master_id == master_id)
			return ELEVATOR_UNDER_CONTROL_BY_YOU;
		else
			return ELEVATOR_UNDER_CONTROL_BY_OTHER;
	 }
	 
	 if(takeElevatorControl() == 0)
		elevator_state.under_control = true;
	 else
	    return ELEVATOR_ERROR_TAKING_CONTROL;
	    
	 elevator_state.master_id = master_id;
	 this->control_token = control_token;
	 
	 last_elevator_action = time_under_control = ros::Time::now();
	 
	 return ELEVATOR_CONTROL_OK;
	 
 }
 
//! method called when someone releases the elevator control
//! return 0 if OK,-1 otherwise
int RobotnikElevatorComponent::releaseControl(std::string control_token, std::string master_id){
	
	 if(elevator_state.under_control){
		if(control_token == this->control_token){
			
			if(releaseElevatorControl() == 0)
				elevator_state.under_control = false;
			else
				return ELEVATOR_ERROR_RELEASING_CONTROL;
			
			elevator_state.master_id = "";
			this->control_token = "";
			
			return ELEVATOR_RELEASE_OK;
		}else
			return ELEVATOR_WRONG_TOKEN;
			
	 }else
	    return ELEVATOR_NOT_UNDER_CONTROL;
	 
 }

//! returns the string for a type of return message
std::string RobotnikElevatorComponent::elevatorControlReturnMsgs(int msg_id){
	
	switch(msg_id){
		case ELEVATOR_CONTROL_OK:
			return std::string("OK: Elevator under control");
		break;
		case ELEVATOR_UNDER_CONTROL_BY_OTHER:
			return std::string("ERROR: Elevator is already under control");
		break;
		case ELEVATOR_UNDER_CONTROL_BY_YOU:
			return std::string("OK: Elevator is already under your control");
		break;
		case ELEVATOR_NOT_UNDER_CONTROL:
			return std::string("ERROR: Elevator is not controlled");
		break;
		case ELEVATOR_WRONG_TOKEN:
			return std::string("ERROR: The control token is incorrect");
		break;
		case ELEVATOR_RELEASE_OK:
			return std::string("OK: Elevator control released");
		break;
		case ELEVATOR_UNKNOWN_STATUS:
			return std::string("ERROR: Elevator status is unknown");
		break;
		case ELEVATOR_ERROR_TAKING_CONTROL:
			return std::string("ERROR: Elevator control request failed");
		break;
		case ELEVATOR_ERROR_RELEASING_CONTROL:
			return std::string("ERROR: Elevator release request failed");
		break;
		case ELEVATOR_OPEN_OK:
			return std::string("OK: Elevator door open");
		break;
		case ELEVATOR_CLOSE_OK:
			return std::string("OK: Elevator door close");
		break;
		case ELEVATOR_ERROR_OPEN:
			return std::string("ERROR: Elevator door open request failed");
		break;
		case ELEVATOR_ERROR_CLOSE:
			return std::string("ERROR: Elevator door close request failed");
		break;
		case ELEVATOR_ERROR_DOOR_STATE:
			return std::string("ERROR: Unknown door state request");
		break;
		case ELEVATOR_ERROR_INVALID_FLOOR:
			return std::string("ERROR: Invalid floor number");
		break;
		case ELEVATOR_FLOOR_OK:
			return std::string("OK: Elevator floor set");
		break;
		case ELEVATOR_ERROR_FLOOR:
			return std::string("ERROR: Elevator floor request failed");
		break;
		default:
			return std::string("UNKNOWN");
		break;
	}
}

// TO DEFINE

int RobotnikElevatorComponent::takeElevatorControl(){
	return 0;
}

int RobotnikElevatorComponent::releaseElevatorControl(){
	return 0;
}

int RobotnikElevatorComponent::goToFloor(int floor){
	return 0;
}

int RobotnikElevatorComponent::openDoor(){
	return 0;
}

int RobotnikElevatorComponent::closeDoor(){
	return 0;
}

//! checks if it's a valid floor number
//! 0 is ok, -1 invalid
int RobotnikElevatorComponent::checkValidFloor(int floor){
	return 0;
}

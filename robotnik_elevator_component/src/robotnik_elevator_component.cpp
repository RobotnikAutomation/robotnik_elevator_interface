#include <ros/ros.h>

#include <robotnik_elevator_component/robotnik_elevator_component.h>

/*!	\fn int RobotnikElevatorComponent::rosSetup()
 * 	\brief Setups all ROS' stuff
*/
int robotnik_elevator_component::RobotnikElevatorComponent::rosSetup(){
	
	// Checks if has been initialized
  if (ros_initialized)
  {
    RCOMPONENT_INFO("Already initialized");

    return rcomponent::INITIALIZED;
  }

  state_publisher = pnh_.advertise<robotnik_msgs::State>("state", 1);
  elevator_state_publisher = pnh_.advertise<robotnik_elevator_interface_msgs::ElevatorState>("elevator_state", 1);
  // state_publisher = pnh_.advertise<std_msgs::Empty>("state", 1);

  ros_initialized = true;

  return rcomponent::OK;
	
}

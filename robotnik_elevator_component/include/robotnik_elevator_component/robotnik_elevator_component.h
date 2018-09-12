#ifndef _ROBOTNIK_ELEVATOR_COMPONENT_H_
#define _ROBOTNIK_ELEVATOR_COMPONENT_H_

#include <rcomponent/rcomponent.h>
#include <robotnik_elevator_interface_msgs/ElevatorState.h>


namespace robotnik_elevator_component
{
class RobotnikElevatorComponent : public rcomponent::RComponent
{
public:
 
public:
  RobotnikElevatorComponent(ros::NodeHandle h, std::string name = "RobotnikElevarorComponent")
    : rcomponent::RComponent(h, name)
  {
  }

  virtual ~RobotnikElevatorComponent()
  {
  }

  /* Rcomponent stuff */
  virtual void initState();
  virtual void standbyState();
  virtual void readyState();
  virtual void allState();
  virtual int rosSetup();
  virtual void rosReadParams();
  virtual int setup();
  virtual int stop();
  virtual std::string getStateString();

protected:
 
  
  //! Publish the component state
  ros::Publisher elevator_state_publisher;
  
};
}

#endif  //_ROBOTNIK_ELEVATOR_COMPONENT_H_

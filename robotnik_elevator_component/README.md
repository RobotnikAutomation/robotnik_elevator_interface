# robotnik_elevator_component

It is a component library interface to be implemented by all the elevator controllers. The component inherits from [rcomponent|https://github.com/RobotnikAutomation/rcomponent] and defines the topics and services.

All the elevator components have to implement the following topics & services.

## topics


### Publishers

* state (robotnik_msgs/State)
  * Rcomponent standard state of the component
* elevator_state (robotnik_elevator_interface_msgs/ElevatorState)
  * State of the elevator
  * Example:
    ```
    current_floor: -1000
    target_floor: -1000
    under_control: False
    master_id: ''
    elevator_status: "idle"
    door_status: "unknown"
    cabin_presence_free: True
    ```

## Services

### Servers

* set_elevator_control (robotnik_elevator_interface_msgs/SetElevatorControl)
  * Example:
  ```
  rosservice call /elevator/set_elevator_control "under_control: true 
  master_id: 'fms'
  control_token: 'sadk3221'" 
  success: True
  message: "OK: Elevator under control"
  ```

* go_to_floor (robotnik_elevator_interface_msgs/GoToFloor)
  * Example:
  ```
  rosservice call /elevator/go_to_floor "floor: 2
  control_token: 'sadk3221'" 
  success: True
  message: "OK: Elevator floor set"
  ```

* set_door_state (robotnik_elevator_interface_msgs/SetDoorState)
  * Example:
  ```
  rosservice call /elevator/set_door_state "state: 'open'
  control_token: 'sadk3221'" 
  success: True
  message: "OK: Elevator door open"
  ```



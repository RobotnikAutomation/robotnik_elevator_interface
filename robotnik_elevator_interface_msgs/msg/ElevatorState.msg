string ELEVATOR_STATUS_IDLE=idle
string ELEVATOR_STATUS_UNKNOWN=unknown
string ELEVATOR_STATUS_MOVING=moving

string DOOR_STATUS_UNKNOWN=unknown
string DOOR_STATUS_OPENING=opening
string DOOR_STATUS_CLOSING=closing
string DOOR_STATUS_CLOSE=close
string DOOR_STATUS_OPEN=open

string elevator_id

int32 current_floor
int32 target_floor
# True if the elevator is under control
bool under_control
# id to identify who is controlling the elevator
string master_id
# elevator status
string elevator_status
# door status
string door_status
# flag active when no one is in the cabin
bool cabin_presence_free

uint8 state         # State enums ; state: 5
string state_info   # 打印目前获取的状态 ；state_info: "STATE_ROBOT_RUN"
int32 number        # number: 0

# State enums
uint8 STATE_ROBOT_NULL = 0
uint8 STATE_ROBOT_CONFIG = 1
uint8 STATE_ROBOT_ON = 2
uint8 STATE_ROBOT_START = 3
uint8 STATE_ROBOT_INIT = 4
uint8 STATE_ROBOT_RUN = 5
uint8 STATE_ROBOT_HALT = 6
uint8 STATE_ROBOT_STOP = 7
uint8 STATE_ROBOT_OFF = 8
uint8 STATE_ROBOT_ERR = 9
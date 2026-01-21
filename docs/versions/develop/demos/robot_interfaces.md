## robot/robot_state

**Description**

```tex
底层通信模块状态机发布接口用于实时发布底层通信模块的当前状态，包括运行状态、错误信息等。状态信息包括底层通信模块当前模式如待机、运行、暂停等
```

**ROS Type:** `Topic`

**Data Type:** `zj_robot/RobotState.msg`

**Version:**

- 1.0.0 : added

**CLI:**

```shell
rostopic echo /zj_humanoid/robot/robot_state
```

## robot/set_robot_state/stop

**Description**

```tex
设置状态机急停接口用于紧急停止底层通信模块的所有运动和状态机的运行。调用该接口后，底层通信模块会立即停止所有运动，并进入急停状态。急停状态需要手动调用状态机的运行服务才能解除，亦或重启底层通信模块
```

**ROS Type:** `Service`

**Data Type:** `std_srvs/Trigger.srv`

**Version:**

- 1.0.0 : added

**CLI:**

```shell
rosservice call /zj_humanoid/robot/set_robot_state/stop "{}"
```

## robot/set_robot_state/run

**Description**

```tex
设置状态机启动接口用于启动底层通信模块的状态机，使底层通信模块进入运行状态。底层通信模块正常开机默认此状态。调用该接口后，底层通信模块会从当前状态启动状态机，开始执行预设的任务。如果底层通信模块处于急停状态，可以调用此服务启动，注意松开外接物理急停
```

**ROS Type:** `Service`

**Data Type:** `std_srvs/Trigger.srv`

**Version:**

- 1.0.0 : added

**CLI:**

```shell
rosservice call /zj_humanoid/robot/set_robot_state/run "{}"
```

## robot/set_robot_state/restart

**Description**

```tex
设置状态机重启接口用于重启底层通信模块的状态机，恢复到初始（正常运行）状态。调用该接口后，底层通信模块会停止当前的所有任务，状态机重启并进入待机状态。中间会先跳转到软急停状态，再跳转到运行状态
```

**ROS Type:** `Service`

**Data Type:** `std_srvs/Trigger.srv`

**Version:**

- 1.0.0 : added

**CLI:**

```shell
rosservice call /zj_humanoid/robot/set_robot_state/restart "{}"
```

## robot/set_robot_state/OFF

**Description**

```tex
设置状态机关机接口用于关闭底层通信模块的状态机，使机器人进入关机状态。调用该接口后，底层通信模块会停止所有运动和任务，状态机关闭，机器人进入低功耗状态。关机后需要手动启动才能恢复
```

**ROS Type:** `Service`

**Data Type:** `std_srvs/Trigger.srv`

**Version:**

- 1.0.0 : added

**CLI:**

```shell
rosservice call /zj_humanoid/robot/set_robot_state/OFF "{}"
```

## robot/battery_info

**Description**

```tex
电池BMS获取接口用于实时获取机器人电池的状态信息，包括电压、电流、电量等。该接口用于监控电池状态，便于及时充电或更换电池。电池信息包括：电压、电流、剩余电量、温度、循环次数等
```

**ROS Type:** `Topic`

**Data Type:** `zj_robot/BatteryInfo.msg`

**Version:**

- 1.0.0 : added

**CLI:**

```shell
rostopic echo /zj_humanoid/robot/battery_info
```

## robot/joint_motor/errors

**Description**

```tex
机器人关节电机错误信息发布接口用于实时发布各关节电机的错误信息。该接口用于监控关节电机的运行状态，及时发现和处理电机故障。错误信息存储在sensor_msgs::JointState消息的position字段中，每个关节对应一个错误码
```

**ROS Type:** `Topic`

**Data Type:** `sensor_msgs/JointState.msg`

**Version:**

- 1.0.0 : added

**CLI:**

```shell
rostopic echo /zj_humanoid/robot/joint_motor/errors
```

## robot/joint_motor/temperatures

**Description**

```tex
机器人关节电机温度信息发布接口用于实时发布各关节电机的温度信息。该接口用于监控关节电机的温度，防止电机过热损坏。温度信息存储在sensor_msgs::JointState消息的position字段中，每个关节对应一个温度值（单位：℃）
```

**ROS Type:** `Topic`

**Data Type:** `sensor_msgs/JointState.msg`

**Version:**

- 1.0.0 : added

**CLI:**

```shell
rostopic echo /zj_humanoid/robot/joint_motor/temperatures
```

## robot/joint_motor/set_zero

**Description**

```tex
电机自动标零服务用于对机器人的关节电机进行自动标零，校准关节的零位。标零过程机器人会切换到失能状态，确保吊架安全悬挂。调用该接口后，机器人会自动对所有关节电机进行标零操作，校准关节的零位。标零过程中需要确保机器人处于平稳状态，避免外界干扰。标完零后机器人会关机重启，重启完成后即可正常使用
```

**ROS Type:** `Service`

**Data Type:** `zj_robot/SetZero.srv`

**Version:**

- 1.0.0 : added

**CLI:**

```shell
# 进入标零模式，会杀死当前嵌入式sdk，启动标零sdk
rosservice call /zj_humanoid/robot/joint_motor/set_zero "mode: 0
joint_id: 0"

# 进行标零，输入需要标零的关节joint_id: 关节id
rosservice call /zj_humanoid/robot/joint_motor/set_zero "mode: 1
joint_id: 7"

# 退出标零模式，机器自动重启，等待开机即可
rosservice call /zj_humanoid/robot/joint_motor/set_zero "mode: 2
joint_id: 0"
```

**Example:**

```Python
import rospy
from zj_robot.srv import SetZero, SetZeroRequest

def set_zero_joint(mode, joint_id):
    """
    调用标零服务
    mode: 0=prepare（进入标零模式）, 1=zero_joint（执行关节标零）, 2=exit（退出标零模式）
    joint_id: 关节ID（mode=0/2时填0，mode=1时填具体关节ID）
    
    关节名称到ID的映射：
    leftShoulderPitch: 0, leftShoulderRoll: 1, leftShoulderYaw: 2
    leftElbow: 3, leftWristYaw: 4, leftWristPitch: 5, leftWristRoll: 6
    rightShoulderPitch: 7, rightShoulderRoll: 8, rightShoulderYaw: 9
    rightElbow: 10, rightWristYaw: 11, rightWristPitch: 12, rightWristRoll: 13
    neckYaw: 14, neckPitch: 15, waist: 16
    leftHipYaw: 17, leftHipRoll: 18, leftHipPitch: 19, leftKnee: 20
    leftAnklePitch: 21, leftAnkleRoll: 22
    rightHipYaw: 23, rightHipRoll: 24, rightHipPitch: 25, rightKnee: 26
    rightAnklePitch: 27, rightAnkleRoll: 28
    """
    # 等待标零服务上线（超时10秒，避免无限阻塞）
    service_name = '/zj_humanoid/robot/joint_motor/set_zero'
    if not rospy.wait_for_service(service_name, timeout=10.0):
        rospy.logerr("标零服务%s未上线，超时退出", service_name)
        return False
    
    try:
        set_zero = rospy.ServiceProxy(service_name, SetZero)
        req = SetZeroRequest()
        req.mode = mode
        req.joint_id = joint_id
        resp = set_zero(req)
        
        # 打印服务响应结果
        mode_desc = {0: "进入标零模式", 1: f"关节{joint_id}标零", 2: "退出标零模式"}
        if resp.success:
            rospy.loginfo("[%s] 成功：%s", mode_desc[mode], resp.message)
        else:
            rospy.logerr("[%s] 失败：%s", mode_desc[mode], resp.message)
        return resp.success
    
    except rospy.ServiceException as e:
        rospy.logerr("[%s] 服务调用异常：%s", mode_desc.get(mode, "标零操作"), e)
        return False


if __name__ == '__main__':
    # 1. 初始化ROS节点
    rospy.init_node('set_zero_client', anonymous=True)
    rospy.loginfo("标零客户端节点启动，开始执行标零流程...")
    
    # 2. 配置等待时间（关键！根据机器人实际响应速度调整，单位：秒）
    WAIT_AFTER_ENTER = 25    # 进入标零模式后，等待SDK加载完成的时间
    WAIT_AFTER_CALIB = 10    # 单个关节标零后，等待电机校准完成的时间
    WAIT_BEFORE_EXIT = 5    # 标零完成后，等待结果确认的时间
    
    # 3. 第一步：进入标零模式（杀死原有SDK，启动标零SDK）
    enter_success = set_zero_joint(mode=0, joint_id=0)
    if not enter_success:
        rospy.logfatal("进入标零模式失败，终止标零流程！")
        rospy.signal_shutdown("进入标零模式失败")
    
    # 等待标零SDK加载（必须等SDK就绪才能执行后续标零）
    rospy.loginfo("等待标零SDK加载完成...（剩余%d秒）", WAIT_AFTER_ENTER)
    rospy.sleep(WAIT_AFTER_ENTER)
    
    # 4. 第二步：执行指定关节标零（示例：右肩俯仰关节，ID=7）
    target_joint_id = 5  # 可替换为需要标零的关节ID（如0=左肩俯仰、19=左髋俯仰等）
    calib_success = set_zero_joint(mode=1, joint_id=target_joint_id)
    if not calib_success:
        rospy.logfatal("关节%d标零失败，终止标零流程！", target_joint_id)
        # 失败后仍尝试退出标零模式，避免系统卡在标零状态
        set_zero_joint(mode=2, joint_id=0)
        rospy.signal_shutdown("关节标零失败")
    
    # 等待电机完成物理校准（电机需要运动到零位并记录参数，不能省略）
    rospy.loginfo("关节%d标零中，等待校准完成...（剩余%d秒）", target_joint_id, WAIT_AFTER_CALIB)
    rospy.sleep(WAIT_AFTER_CALIB)
    
    # 5. 第三步：退出标零模式（系统自动重启，恢复正常运行）
    rospy.loginfo("标零完成，准备退出标零模式...（剩余%d秒）", WAIT_BEFORE_EXIT)
    rospy.sleep(WAIT_BEFORE_EXIT)  # 等待标零结果写入，避免数据丢失
    
    exit_success = set_zero_joint(mode=2, joint_id=0)
    if exit_success:
        rospy.loginfo("标零流程全部完成！机器人将自动重启，重启后即可正常使用")
    else:
        rospy.logerr("退出标零模式失败，请手动重启机器人底层系统")
    
    # 关闭节点
    rospy.signal_shutdown("标零流程结束")
```

---

## Appendix - Message Definitions

### zj_robot/RobotState

```python
uint8 state                    # State enums ; state: 5
string state_info              # 打印目前获取的状态 ；state_info: "STATE_ROBOT_RUN"

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
```

### std_srvs/Trigger

```python
---
bool success                   # indicate successful run of triggered service
string message                 # informational, e.g. for error messages
```

### zj_robot/BatteryInfo

```python
# 电池基本信息
uint8 battery_type             # 电池类型 1 - 鼎力2014b, 2 - 鼎力2015型, 3 - 云帆

# LED状态
uint8 led1_status              # 0x01 - 绿灯常亮，0x02 - 红灯常亮，0x03 - 绿灯闪烁，0x04 - 红灯闪烁, 0x05 - 红绿闪烁，其他 - 灭
uint8 led2_status              # 同上

# 电压电流信息
float32 total_voltage          # 电池组总电压，单位V
float32 total_current          # 电池组总电流，单位A

# 容量信息
uint16 soc                     # 剩余容量 0-1000表示0%-100%
float32 remaining_capacity     # 剩余容量，单位Ah
uint32 remaining_time          # 剩余充电时间, 单位分钟

# 单体电池电压信息
float32 max_cell_voltage       # 最高单体电压, 单位V
float32 min_cell_voltage       # 最低单体电压, 单位V
uint8 max_voltage_cell_num     # 最高单体电压电池位置
uint8 min_voltage_cell_num     # 最低单体电压电池位置
float32 voltage_diff           # 单体最高最低电芯压差，单位V

# 温度信息
int16 max_cell_temperature     # 最高电芯温度，单位摄氏度
int16 min_cell_temperature     # 最低电芯温度，单位摄氏度
int16 avg_cell_temperature     # 电芯平均温度，单位摄氏度
uint8 max_temp_cell_num        # 最高温度电池位置
uint8 min_temp_cell_num        # 最低温度电池位置
float32 temperature_diff       # 单体最高最低温度差，单位摄氏度

# 状态信息
uint8 operation_status         # 充放电状态： 0 - 静置， 1 - 充电， 2 - 放电
uint16 cycle_count             # 电池循环次数

# 报警和状态标志
bool is_temperature_high       # 电池温度是否过高
bool is_battery_low            # 电池电量是否过低
uint32[16] warnings            # 报警信息位

# 版本信息
string version                 # 版本号
```

### sensor_msgs/JointState

```python
Header header

string[] name
float64[] position
float64[] velocity
float64[] effort
```

### zj_robot/SetZero

```python
int32 mode                     # 0=prepare, 1=zero_joint, 2=exit
int32 joint_id
---
bool success
string message


#=============================================================================
# 关节名称到算法编号的映射 (注释，供参考)
#=============================================================================
# leftShoulderPitch: 0, leftShoulderRoll: 1, leftShoulderYaw: 2
# leftElbow: 3, leftWristYaw: 4, leftWristPitch: 5, leftWristRoll: 6
# rightShoulderPitch: 7, rightShoulderRoll: 8, rightShoulderYaw: 9
# rightElbow: 10, rightWristYaw: 11, rightWristPitch: 12, rightWristRoll: 13
# neckYaw: 14, neckPitch: 15, waist: 16
# leftHipYaw: 17, leftHipRoll: 18, leftHipPitch: 19, leftKnee: 20
# leftAnklePitch: 21, leftAnkleRoll: 22
# rightHipYaw: 23, rightHipRoll: 24, rightHipPitch: 25, rightKnee: 26
# rightAnklePitch: 27, rightAnkleRoll: 28
```

# 网络与连接
在新的环境中，初次启动机器人，需要确定机器是否已经联网，在没有联网的状态下，部分机器人的功能将无法使用； 建议使用显示器+键鼠登入到orin进行联网设置；

#### 使用显示器和键鼠
使用USB键鼠和DP线连到机器人orin大脑之后，按照Ubuntu系统的方式使机器人连上用户的wifi， 并将大脑orin 设置为固定IP，避免经常更换；

#### 使用机器人AP热点
对于不方便接USB键鼠和HDMI屏幕的场景，也可以通过连接机器人自身的AP热点来配置机器人的网络；
机器人大脑默认的AP名称前缀为nav01ap的Wi-Fi，此Wi-Fi就是机器人大脑的AP热点，密码为88888888。

#### 终端连接
完成机器人的网络配置之后，对于开发者而言，可能还需要使用终端登入大脑系统，支持如下方式登入：
- Linux系统内终端：如果已经使用USB和HDMI登入orin，可以直接使用Linux系统终端登入；
- 外部终端登入Linux：通过标准ssh协议登入orin Linux系统，ssh端口是22；
- 登入到demos容器：
    - 在Linux终端内，支持使用docker exec -it navi_project-demos-1 bash
    - 外部终端，可通过ssh协议登入demos，指令：ssh root@ip -p 2222，密码：naviai@2025

# 运行一个代码示例
首先进入到Orin的demos容器（docker exec -it navi_project-demos-1 bash）。navi_ws工作空间的结构如下

```bash
├── build
├── devel
└── src
    ├── CMakeLists.txt
    ├── demos  # cpp示例代码包
    │   ├── CMakeLists.txt
    │   ├── include
    │   ├── package.xml
    │   └── src
    └── py_demos  # python示例代码包
        ├── CMakeLists.txt
        ├── config
        ├── package.xml
        └── scripts
```
1. 设置环境变量：`source /navi_ws/devel/setup.bash`
2. 运行示例代码：`rosrun py_demos gesture_switch.py`。效果是机器人左手做出拳头的手势，右手做出一的手势。

# 编写代码
为了方便用户开发，Orin`/home/naviai/navi_project/containers/demos/user_code`下的所有文件都映射到demos容器中，用户可以在Orin上编写代码，在容器中执行。

接下来我们将编写一个python程序控制机器人挥手。
#### 新建python代码文件
新建`wave.py`文件
```bash
touch ~/navi_project/containers/demos/user_code/wave.py
chmod +x ~/navi_project/containers/demos/user_code/wave.py
```
写入如下代码
```python
#!/usr/bin/env python3
import rospy
import yaml
import sys
import threading
from upperlimb.srv import MoveJByPath, MoveJByPathRequest
from upperlimb.msg import Joints
from std_srvs.srv import Trigger, TriggerRequest


def load_yaml_data(yaml_file):
    """Load service request data from YAML file."""
    try:
        with open(yaml_file, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)
    except Exception as e:
        rospy.logerr(f"Failed to load YAML file: {e}")
        sys.exit(1)

def main():
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <request_data.yaml>")
        sys.exit(1)
    
    yaml_file = sys.argv[1]
    
    rospy.init_node('wave_demo', anonymous=True)
    
    data = load_yaml_data(yaml_file)
    if 'path' not in data or 'time' not in data or 'arm_type' not in data:
            rospy.logerr("YAML data is missing 'path', 'time', or 'arm_type' keys.")
            sys.exit(1)

    ARM_SERVICE_NAME = '/zj_humanoid/upperlimb/movej_by_path/right_arm'
    GO_DOWN_SERVICE_NAME = '/zj_humanoid/upperlimb/go_down/right_arm'
    rospy.loginfo(f"Waiting for service ...")
    rospy.wait_for_service(ARM_SERVICE_NAME)
    rospy.wait_for_service(GO_DOWN_SERVICE_NAME)
    
    try:
        service_proxy = rospy.ServiceProxy(ARM_SERVICE_NAME, MoveJByPath)
        go_down_proxy = rospy.ServiceProxy(GO_DOWN_SERVICE_NAME, Trigger)

        req = MoveJByPathRequest()
        
        joint_msgs = []
        for joint_data in data['path']:
            if 'joint' in joint_data and isinstance(joint_data['joint'], list):
                joint_msg = Joints()
                joint_msg.joint = [float(j) for j in joint_data['joint']] 
                joint_msgs.append(joint_msg)
            else:
                rospy.logwarn(f"Skipping invalid joint data: {joint_data}")
        
        req.path = joint_msgs
        req.time = float(data['time'])
        req.is_async = bool(data['is_async'])
        req.arm_type = int(data['arm_type'])

        rospy.loginfo(f"Calling service {ARM_SERVICE_NAME}...")
        response = service_proxy(req)
        rospy.loginfo(f"Response: {response}")

        rospy.loginfo(f"Preparing to call {GO_DOWN_SERVICE_NAME} service...")
        go_down_req = TriggerRequest()
        go_down_response = go_down_proxy(go_down_req)
        
        if go_down_response.success:
            rospy.loginfo(f"{GO_DOWN_SERVICE_NAME} service successful. Message: {go_down_response.message}")
        else:
            rospy.logerr(f"{GO_DOWN_SERVICE_NAME} service failed. Message: {go_down_response.message}")
            
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        sys.exit(1)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
```

#### 新建yaml配置文件
新建`wave.yaml`文件用于存放关节轨迹数据
```bash
touch ~/navi_project/containers/demos/user_code/wave.yaml
```

#### 获取关节数据
1. 开启右臂示教模式
```bash
rosservice call /zj_humanoid/upperlimb/teach_mode/enter "arm_type: 2"
```

2. 拖动右臂到目标位置，订阅`/zj_humanoid/upperlimb/joint_states`话题查看实时关节数据
```bash
rostopic echo /zj_humanoid/upperlimb/joint_states
```
数据示例如下，`position`中的数据顺序与`name`中的关节名一一对应：
```bash
name: 
  - Shoulder_Y_L
  - Shoulder_X_L
  - Shoulder_Z_L
  - Elbow_L
  - Wrist_Z_L
  - Wrist_Y_L
  - Wrist_X_L
  - Shoulder_Y_R
  - Shoulder_X_R
  - Shoulder_Z_R
  - Elbow_R
  - Wrist_Z_R
  - Wrist_Y_R
  - Wrist_X_R
  - Neck_Z
  - Neck_Y
  - A_Waist
position: [0.0, 0.174, 0.0, -0.087, 0.0, 0.0, 0.0, 0.0, -0.399, -0.499, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

3. 记录不同姿态的关节角度数据，编辑`wave.yaml`文件，填入轨迹路径点和其他参数
```yaml
path:
  - joint: [0.0, -0.4, -0.5, 0.0, 0.0, 0.0, 0.0]  # 第一个目标位置
  - joint: [0.0, -0.3, -0.6, 0.0, 0.0, 0.0, 0.0]  # 第二个目标位置
  - joint: [0.0, -0.4, -0.5, 0.0, 0.0, 0.0, 0.0]  # 第三个目标位置
time: 8.0       # 轨迹执行总时间（秒）
is_async: false # 是否异步执行
arm_type: 2     # 臂类型：1-左臂，2-右臂，3-双臂
```

4. 关闭示教模式
```bash
rosservice call /zj_humanoid/upperlimb/teach_mode/exit "arm_type: 2"
```

#### 配置CMakeLists.txt
由于`~/navi_project/containers/demos/user_code`中的代码映射到了demos容器中的`py_demos`包的`scripts`目录下，需要在`/navi_ws/src/py_demos/CMakeLists.txt`中添加安装配置。

编辑`/navi_ws/src/py_demos/CMakeLists.txt`，在`catkin_install_python`部分添加：
```cmake
catkin_install_python(PROGRAMS
  scripts/user_code/wave.py
  # 其他已有的脚本...
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

#### 重新编译工作空间
```bash
cd /navi_ws
catkin_make
```

#### 设置环境变量
```bash
source /navi_ws/devel/setup.bash
```

#### 运行程序
```bash
rosrun py_demos wave.py ~/navi_project/containers/demos/user_code/wave.yaml
```
或者直接使用python执行：
```bash
python3 ~/navi_project/containers/demos/user_code/wave.py ~/navi_project/containers/demos/user_code/wave.yaml
```

执行后，机器人右臂会按照yaml文件中定义的轨迹路径运动，完成后自动放下。


<!-- # 其他
### wave_and_say_hello.py程序设计
> <a href="/zj_humanoid_sdk_ros/demos/wave_and_say_hello.py" download="wave_and_say_hello.py">wave_and_say_hello.py</a>的执行效果是机器人抬起右臂挥手，在此期间播放语言，然后右臂放下

1、开启示教模式（`rosservice call /zj_humanoid/upperlimb/teach_mode/enter "arm_type: 2"`，这里只开启右臂示教）

2、拖动右臂，订阅`/zj_humanoid/upperlimb/joint_states`话题查看实时关节数据

`/zj_humanoid/upperlimb/joint_states`数据事例如下所示，`position`、`velocity`、`effort`中数据的顺序与`name`中的关节名一一对应
```bash
root@1424624317905:/navi_ws/src/upperlimb# rostopic echo -n 1 /zj_humanoid/upperlimb/joint_states 
header: 
  seq: 54110
  stamp: 
    secs: 1763087056
    nsecs: 431583463
  frame_id: ''
name: 
  - Shoulder_Y_L
  - Shoulder_X_L
  - Shoulder_Z_L
  - Elbow_L
  - Wrist_Z_L
  - Wrist_Y_L
  - Wrist_X_L
  - Shoulder_Y_R
  - Shoulder_X_R
  - Shoulder_Z_R
  - Elbow_R
  - Wrist_Z_R
  - Wrist_Y_R
  - Wrist_X_R
  - Neck_Z
  - Neck_Y
  - A_Waist
position: [1.1984225238848012e-05, 0.17458619327953784, 1.1984225238848012e-05, -0.08726655158086488, -3.5952675716544036e-05, -5.932784771706937e-08, -1.1865569543413874e-07, 0.0, -0.39993756467083585, -0.4999938611899779, -5.992112619424006e-08, 0.0, 0.0, -1.779835431512081e-07, 1.1865569543413874e-07, 0.0, 3.9947417462826706e-05]
velocity: [0.0003141592741012573, -0.0007330383062362671, 0.00010471975803375244, -9.383648362017994e-05, 0.00041887903213500975, 8.798319816441388e-05, -4.253806681313874e-05, -0.0010471975803375243, 0.002617993950843811, 0.00041887903213500975, 1.2343751996013454e-05, 0.00041887903213500975, 0.00011657922076404131, -5.102194903667966e-05, -3.025720233570538e-06, -2.7172154254417772e-05, 0.019973708731413353]
effort: [-0.5671744998190978, 2.618777306218922, 0.09673953043275081, -0.557714769084589, 0.009540912109074918, -0.20250250389724286, 0.16814606763127612, -0.03338217067486051, -5.839410041480182, -0.012935906014924305, -1.138389479459973, -0.018093309450686476, -0.35837300519526144, -0.3231379588661109, 0.0, -0.09736851808821359, 2.07883569299655e-17]
```


3、把右臂关节数据存放到`./wave_and_say_hello.yaml`，并填写其他`/zj_humanoid/upperlimb/movej_by_path/right_arm`所需参数

4、关闭示教模式`rosservice call /zj_humanoid/upperlimb/teach_mode/exit "arm_type: 2" `

5、执行`python3 wave_and_say_hello.py wave_and_say_hello.yaml`

### right_arm_movej.py程序设计
><a href="/zj_humanoid_sdk_ros/demos/right_arm_movej.py" download="right_arm_movej.py">right_arm_movej.py</a>的执行效果是机器人右臂先回到初始位置，然后运动一小段距离

1、开启示教模式（`rosservice call /zj_humanoid/upperlimb/teach_mode/enter "arm_type: 2"`，这里只开启右臂示教）

2、拖动右臂，订阅`/zj_humanoid/upperlimb/joint_states`话题查看实时关节数据

3、把右臂关节数据存放到`right_arm_movej.yaml`中

4、关闭示教模式`rosservice call /zj_humanoid/upperlimb/teach_mode/exit "arm_type: 2" `

5、执行`python3 right_arm_movej.py right_arm_movej.yaml` -->

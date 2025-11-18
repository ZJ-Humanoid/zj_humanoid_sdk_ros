# 接口调用时序

在机器人双脚离地时，按下A键，此时双腿回零，降低货架，使机器人双脚接触地面，按下Y键，机器人自主站立进入平地模式，此时就可以摇杆行走，在平地模式下，按下X键可以进入斜坡模式（斜坡模式只能前进），行走完毕，按下B键就会双腿缩回（注意挂绳）

# 手柄键位表

| 手柄键位       | 话题名称                     | 触发类型                                |
| -------------- | ---------------------------- | --------------------------------------- |
| **A键**        | `/``set_stand`               | 接收到数据就切换                        |
| **B键**        | `/``emergency_stop`          | 接收到数据就切换                        |
| **Y键**        | `/``load_walkgmp_controller` | 接收到数据就切换                        |
| **X键**        | `/``load_turngmp_controller` | 接收到数据就切换                        |
| **左****摇杆** | `/``cmd_vel`                 | `linear.x  前进后退，linear.y 左右侧移` |
| **右****摇杆** | `/``cmd_vel`                 | `angular.z 左右旋转`                    |

# 接口描述

## `/``set_stand`

**Description:**

站立

手柄**A键**默认绑定该话题

**ROS** **Type:** `Topic/Subscribe`

**Data Type****:** `std_msgs/Float32`

**Version:** 

- 1.0.0: added

**Cli****:**

```Plain
rostopic pub /set_stand std_msgs/Float32 "data: 0.0"
```

**Example:** 

```Python
import time
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from pathlib import Path

PREFIX = ""

rospy.init_node("zj_humanoid"+Path(__file__).stem, anonymous=True)

def test_set_stand(data, *, name=f"{PREFIX}/set_stand"):
    """
    测试站立模式

    Parameters
    ----------
    data : float
        发布的数据值

    Version
    -------
    - 1.0.0: addedi
    """
    pub = rospy.Publisher(name, Float32, queue_size=5, latch=True)  # 设置latch以便接受到一帧数据
    for i in range(1):
        pub.publish(Float32(data=data))
        print(f"Published to {name} with data {data}")
        time.sleep(0.05)
        
        
test_set_stand(0)
```

## `/``emergency_stop`

**Description:**

急停

触发急停,下肢会去使能，机器人会摔倒，请保证吊装正常

手柄**B键**默认绑定该话题

**ROS Type:** `Topic/Subscribe`

**Data Type:** `std_msgs/Float32`

**Version:** 

- 1.0.0: added

**Cli:** 

```Plain
# 触发急停,下肢会去使能，机器人会摔倒，请保证吊装正常
rostopic pub /emergency_stop std_msgs/Float32 "data: 0.0"
```

**Example:** 

```Python
import time
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from pathlib import Path

PREFIX = ""

rospy.init_node("zj_humanoid"+Path(__file__).stem, anonymous=True)


def test_emergency_stop(data, *, name=f"{PREFIX}/emergency_stop"):
    """
    测试急停模式

    Parameters
    ----------
    data : float
        发布的数据值

    Version
    -------
    - 1.0.0: added
    """
    pub = rospy.Publisher(name, Float32, queue_size=5, latch=True)  # 设置latch以便接受到一帧数据
    for i in range(1):
        pub.publish(Float32(data=data))
        print(f"Published to {name} with data {data} (iter {i})")
        time.sleep(0.05)
        
test_emergency_stop(0)
```

## `/``load_walkgmp_controller`

**Description:**

切换到行走模式

手柄**Y键**默认绑定该话题

**ROS Type:** `Topic/Subscribe`

**Data Type:** `std_msgs/Float32`

**Version:** 

- 1.0.0: added

**Cli:**

```Plain
rostopic pub /load_walkgmp_controller std_msgs/Float32 "data: 0.0"
```

**Example:** 

```Python
import time
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from pathlib import Path

PREFIX = ""

rospy.init_node("zj_humanoid"+Path(__file__).stem, anonymous=True)

def test_load_walkgmp_controller(data, *, name=f"{PREFIX}/load_walkgmp_controller"):
    """
    测试切换到行走模式

    Parameters
    ----------
    data : float
        发布的数据值

    Version
    -------
    - 1.0.0: added
    """
    pub = rospy.Publisher(name, Float32, queue_size=5, latch=True)  # 设置latch以便接受到一帧数据
    for i in range(1):
        pub.publish(Float32(data=data))
        print(f"Published to {name} with data {data} (iter {i})")
        time.sleep(0.05)
        
test_load_walkgmp_controller(0)
```

## `/``load_turngmp_controller`

**Description:**

切换到斜坡模式

手柄**X键**默认绑定该话题

**ROS Type:** `Topic/Subscribe`

**Data Type:** `std_msgs/Float32`

**Version:** 

- 1.0.0: added

**Cli:** 

```Plain
rostopic pub /load_turngmp_controller std_msgs/Float32 "data: 0.0"
```

**Example:** 

```Python
import time
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from pathlib import Path

PREFIX = ""

rospy.init_node("zj_humanoid"+Path(__file__).stem, anonymous=True)

def test_load_turngmp_controller(data, *, name=f"{PREFIX}/load_turngmp_controller"):
    """
    测试切换到斜坡模式

    Parameters
    ----------
    data : float
        发布的数据值

    Version
    -------
    - 1.0.0: added
    """
    pub = rospy.Publisher(name, Float32, queue_size=5, latch=True)  # 设置latch以便接受到一帧数据
    for i in range(1):
        pub.publish(Float32(data=data))
        print(f"Published to {name} with data {data} (iter {i})")
        time.sleep(0.05)


test_load_turngmp_controller(0)
```

## `/``cmd_vel`

**Description:**

机器人行走速度

手柄左右摇杆默认绑定该话题

手柄中的左摇杆对应前进后退  `linear.x 前进后退，linear.y 左右侧移`

手柄中的右摇杆对应左右转弯  `angular.z 左右旋转`

**ROS Type:** `Topic/Subscribe`

**Data Type:** `geometry_msgs/Twist`

**Version:** 

- 1.0.0: added

**Cli:** 

```Plain
# 通过rostopic发送数据   速度限制:vx ±0.85,±vy 0.5,±vz 0.5 
# linear.x 前进后退，linear.y 左右侧移
# angular.z 左右转弯

# 请注意该话题为最终机器人行走接受的话题，当同时使用手柄并且向该话题中发送数据时，数据

 rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

**Example:** 

```Python
import time
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from pathlib import Path

PREFIX = ""

rospy.init_node("zj_humanoid"+Path(__file__).stem, anonymous=True)


def test_cmd_vel(linear_x, linear_y, angular_z, *, name="/cmd_vel"):
    """
    测试机器人行走速度话题

    Description
    -----------
    机器人行走速度
    手柄左右摇杆默认绑定该话题
    手柄中的左摇杆对应前进后退  linear.x 前进后退，linear.y 左右侧移
    手柄中的右摇杆对应左右转弯  angular.z 左右旋转

    Version
    -------
    - 1.0.0: added

    Cli
    ---
    # 通过rostopic发送数据   速度限制:vx ±0.85,±vy 0.5,±vz 0.5 
    # linear.x 前进后退，linear.y 左右侧移
    # angular.z 左右转弯

    rostopic pub /cmd_vel geometry_msgs/Twist "linear:
      x: {linear_x}
      y: {linear_y}
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: {angular_z}"

    Example
    -------
    发布速度数据到该话题
    """
    pub = rospy.Publisher(name, Twist, queue_size=1, latch=True, tcp_nodelay=True)  # 设置latch以便接受到一帧数据
    rospy.sleep(0.02)
    twist = Twist()
    twist.linear.x = linear_x
    twist.linear.y = linear_y
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = angular_z
    for i in range(1):
        print(f"Published to {name} with linear: ({linear_x}, {linear_y}, 0.0) and angular: (0.0, 0.0, {angular_z})")
        pub.publish(twist)
        time.sleep(0.05)


test_cmd_vel(0.1, 0, 0)
```
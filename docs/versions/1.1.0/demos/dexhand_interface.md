## `gesture_switch/left`

**Description:**
```tex
左手手势切换服务
```

**ROS Type:** `Service`

**Data Type:** `hand/Gesture`

**Version:** 
- 1.0.0: added

**Cli:** ` rosservice call zj_humanoid/hand/gesture_switch/left "gesture_name: ['RESET']"`

**Example:** 
```python
def test_gesture_switch_left(gesture_name, *, name=f"{PREFIX}/gesture_switch/left"):
    """
    左手手势切换服务

    Parameters
    ----------
    gesture_name : list[str]
        手势名称列表

    Notes
    -----
    控制左手执行指定手势。支持的手势包括:
    RESET、ROCK、ONE、TWO、THREE、FOUR等
    手势名称大小写不敏感

    Version
    -------
    - 1.0.0: added

    Cli Examples
    ------------
    $ rosservice call zj_humanoid/hand/gesture_switch/left "gesture_name: ['RESET']"
    """
    rospy.wait_for_service(name)
    try:
        gesture_service = rospy.ServiceProxy(name, Gesture)
        req = GestureRequest()
        req.gesture_name = gesture_name
        resp = gesture_service(req)
        print(f"Service call to {name}")
        print(f"Request: gesture_name={gesture_name}")
        print(f"Response: success={resp.success}, message='{resp.message}'")
        return resp.success
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
        return False

```

## `gesture_switch/right`

**Description:**
```tex
调用右手手势切换服务
```

**ROS Type:** `Service`

**Data Type:** `hand/Gesture`

**Version:** 
- 1.0.0: added

**Cli:** ` rosservice call zj_humanoid/hand/gesture_switch/right "gesture_name: ['RESET']"`

**Example:** 
```python
def test_gesture_switch_right(gesture_name, *, name=f"{PREFIX}/gesture_switch/right"):
    """
    调用右手手势切换服务

    Parameters
    ----------
    gesture_name : list[str]
        手势名称列表

    Notes
    -----
    控制右手执行指定手势。支持的手势包括:
    RESET、ROCK、ONE、TWO、THREE、FOUR等
    手势名称大小写不敏感

    Version
    -------
    - 1.0.0: added

    Cli Examples
    ------------
    $ rosservice call zj_humanoid/hand/gesture_switch/right "gesture_name: ['RESET']"
    """
    rospy.wait_for_service(name)
    try:
        gesture_service = rospy.ServiceProxy(name, Gesture)
        req = GestureRequest()
        req.gesture_name = gesture_name
        resp = gesture_service(req)
        print(f"Service call to {name}")
        print(f"Request: gesture_name={gesture_name}")
        print(f"Response: success={resp.success}, message='{resp.message}'")
        return resp.success
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
        return False

```

## `gesture_switch/dual`

**Description:**
```tex
调用双手手势切换服务
```

**ROS Type:** `Service`

**Data Type:** `hand/Gesture`

**Version:** 
- 1.0.0: added

**Cli:** ` rosservice call zj_humanoid/hand/gesture_switch/dual "gesture_name: ['RESET', 'ROCK']"`

**Example:** 
```python
def test_gesture_switch_dual(gesture_name, *, name=f"{PREFIX}/gesture_switch/dual"):
    """
    调用双手手势切换服务

    Parameters
    ----------
    gesture_name : list[str]
        手势名称列表,索引0为左手,索引1为右手

    Notes
    -----
    同时控制左右手执行指定手势。
    gesture_name数组中索引0为左手,索引1为右手
    支持的手势包括:RESET、ROCK、ONE、TWO、THREE、FOUR等

    Version
    -------
    - 1.0.0: added

    Cli Examples
    ------------
    $ rosservice call zj_humanoid/hand/gesture_switch/dual "gesture_name: ['RESET', 'ROCK']"
    """
    rospy.wait_for_service(name)
    try:
        gesture_service = rospy.ServiceProxy(name, Gesture)
        req = GestureRequest()
        req.gesture_name = gesture_name
        resp = gesture_service(req)
        print(f"Service call to {name}")
        print(f"Request: gesture_name={gesture_name}")
        print(f"Response: success={resp.success}, message='{resp.message}'")
        return resp.success
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
        return False

```

## `joint_switch/left`

**Description:**
```tex
调用左手关节控制服务
```

**ROS Type:** `Service`

**Data Type:** `hand/HandJoint`

**Version:** 
- 1.0.0: added

**Cli:** ` rosservice call zj_humanoid/hand/joint_switch/left "q: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"`

**Example:** 
```python
def test_joint_switch_left(q, *, name=f"{PREFIX}/joint_switch/left"):
    """
    调用左手关节控制服务

    Parameters
    ----------
    q : list[float]
        关节角度数组，顺序为[拇指弯曲,拇指摆动,食指弯曲,中指弯曲,无名指弯曲,小指弯曲]，单位:弧度

    Notes
    -----
    控制左手各关节运动到指定角度。
    关节角度会被限制在安全范围内，超出限位的数据会被自动限制并提示

    Version
    -------
    - 1.0.0: added

    Cli Examples
    ------------
    $ rosservice call zj_humanoid/hand/joint_switch/left "q: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
    """
    rospy.wait_for_service(name)
    try:
        joint_service = rospy.ServiceProxy(name, HandJoint)
        req = HandJointRequest()
        req.q = q
        resp = joint_service(req)
        print(f"Service call to {name}")
        print(f"Request: q={q}")
        print(f"Response: success={resp.success}, message='{resp.message}'")
        return resp.success
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
        return False

```

## `joint_switch/right`

**Description:**
```tex
调用右手关节控制服务
```

**ROS Type:** `Service`

**Data Type:** `hand/HandJoint`

**Version:** 
- 1.0.0: added

**Cli:** ` rosservice call zj_humanoid/hand/joint_switch/right "q: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"`

**Example:** 
```python
def test_joint_switch_right(q, *, name=f"{PREFIX}/joint_switch/right"):
    """
    调用右手关节控制服务

    Parameters
    ----------
    q : list[float]
        关节角度数组，顺序为[拇指弯曲,拇指摆动,食指弯曲,中指弯曲,无名指弯曲,小指弯曲]，单位:弧度

    Notes
    -----
    控制右手各关节运动到指定角度。
    关节角度会被限制在安全范围内，超出限位的数据会被自动限制并提示

    Version
    -------
    - 1.0.0: added

    Cli Examples
    ------------
    $ rosservice call zj_humanoid/hand/joint_switch/right "q: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
    """
    rospy.wait_for_service(name)
    try:
        joint_service = rospy.ServiceProxy(name, HandJoint)
        req = HandJointRequest()
        req.q = q
        resp = joint_service(req)
        print(f"Service call to {name}")
        print(f"Request: q={q}")
        print(f"Response: success={resp.success}, message='{resp.message}'")
        return resp.success
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
        return False

```

## `joint_switch/dual`

**Description:**
```tex
调用双手关节控制服务
```

**ROS Type:** `Service`

**Data Type:** `hand/HandJoint`

**Version:** 
- 1.0.0: added

**Cli:** ` rosservice call zj_humanoid/hand/joint_switch/dual "q: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]"`

**Example:** 
```python
def test_joint_switch_dual(left_q, right_q, *, name=f"{PREFIX}/joint_switch/dual"):
    """
    调用双手关节控制服务

    Parameters
    ----------
    left_q : list[float]
        左手关节角度数组，顺序为[拇指弯曲,拇指摆动,食指弯曲,中指弯曲,无名指弯曲,小指弯曲]
    right_q : list[float]
        右手关节角度数组，顺序为[拇指弯曲,拇指摆动,食指弯曲,中指弯曲,无名指弯曲,小指弯曲]

    Notes
    -----
    同时控制双手各关节运动到指定角度。
    双手关节数组会被合并为12个元素的数组发送给服务

    Version
    -------
    - 1.0.0: added

    Cli Examples
    ------------
    $ rosservice call zj_humanoid/hand/joint_switch/dual "q: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]"
    """
    rospy.wait_for_service(name)
    try:
        joint_service = rospy.ServiceProxy(name, HandJoint)
        req = HandJointRequest()
        # 合并左右手关节角度为一个12元素数组
        req.q = left_q + right_q
        resp = joint_service(req)
        print(f"Service call to {name}")
        print(f"Request: left_q={left_q}, right_q={right_q}")
        print(f"Combined q={req.q}")
        print(f"Response: success={resp.success}, message='{resp.message}'")
        return resp.success
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
        return False

```

## `joint_states`

**Description:**
```tex
订阅手部关节状态数据
```

**ROS Type:** `Topic/Publish`

**Data Type:** `sensor_msgs/JointState`

**Version:** 
- 1.0.0: added

**Cli:** ` rostopic echo /zj_humanoid/hand/joint_states`

**Example:** 
```python
def test_sub_joint_states(*, name=f"{PREFIX}/joint_states"):
    """
    订阅手部关节状态数据

    Notes
    -----
    订阅手部所有关节的位置状态，包括左右手各6个关节：
    拇指弯曲、拇指摆动、食指弯曲、中指弯曲、无名指弯曲、小指弯曲

    Version
    -------
    - 1.0.0: added

    Cli Examples
    ------------
    $ rostopic echo /zj_humanoid/hand/joint_states
    """
    def callback(msg):
        print(f"hand joint states | {msg}")
    
    rospy.Subscriber(name, JointState, callback)
    print(f"Subscribing to {name}")
    rospy.sleep(2)

```

## `finger_pressures/left`

**Description:**
```tex
订阅左手指尖压力传感器数据
```

**ROS Type:** `Topic/Publish`

**Data Type:** `hand/PressureSensor`

**Version:** 
- 1.0.0: added

**Cli:** ` rostopic echo /zj_humanoid/hand/finger_pressures/left`

**Example:** 
```python
def test_sub_finger_pressures_left(*, name=f"{PREFIX}/finger_pressures/left"):
    """
    订阅左手指尖压力传感器数据

    Notes
    -----
    接收左手指尖压力传感器数据，压力值顺序为：
    [大拇指, 食指, 中指, 无名指, 小拇指] 单位为0.1N

    Version
    -------
    - 1.0.0: added

    Cli Examples
    ------------
    $ rostopic echo /zj_humanoid/hand/finger_pressures/left
    """
    def callback(msg):
        # print(msg)
        print(f"hand finger pressures left | {msg}")

    rospy.Subscriber(name, PressureSensor, callback)
    print(f"Subscribing to {name}")
    rospy.sleep(2)

```

## `finger_pressures/right`

**Description:**
```tex
订阅右手指尖压力传感器数据
```

**ROS Type:** `Topic/Publish`

**Data Type:** `hand/PressureSensor`

**Version:** 
- 1.0.0: added

**Cli:** ` rostopic echo /zj_humanoid/hand/finger_pressures/right`

**Example:** 
```python
def test_sub_finger_pressures_right(*, name=f"{PREFIX}/finger_pressures/right"):
    """
    订阅右手指尖压力传感器数据

    Notes
    -----
    接收右手指尖压力传感器数据，压力值顺序为：
    [大拇指, 食指, 中指, 无名指, 小拇指] 单位为0.1N

    Version
    -------
    - 1.0.0: added

    Cli Examples
    ------------
    $ rostopic echo /zj_humanoid/hand/finger_pressures/right
    """
    def callback(msg):
        print(f"hand finger pressures right | {msg}")
    
    rospy.Subscriber(name, PressureSensor, callback)
    print(f"Subscribing to {name}")
    rospy.sleep(2)

```

## `wrist_force_sensor/left`

**Description:**
```tex
订阅左手腕力传感器数据
```

**ROS Type:** `Topic/Publish`

**Data Type:** `geometry_msgs/WrenchStamped`

**Version:** 
- 1.0.0: added

**Cli:** ` rostopic echo /zj_humanoid/hand/wrist_force_sensor/left`

**Example:** 
```python
def test_sub_wrist_force_sensor_left(*, name=f"{PREFIX}/wrist_force_sensor/left"):
    """
    订阅左手腕力传感器数据

    Notes
    -----
    接收左手腕力传感器数据，包括力和力矩的三轴分量

    Version
    -------
    - 1.0.0: added

    Cli Examples
    ------------
    $ rostopic echo /zj_humanoid/hand/wrist_force_sensor/left
    """
    def callback(msg):
        print(f"hand wrist force sensor left | {msg}")
    
    rospy.Subscriber(name, WrenchStamped, callback)
    print(f"Subscribing to {name}")
    rospy.sleep(2)

```

## `wrist_force_sensor/right`

**Description:**
```tex
订阅右手腕力传感器数据
```

**ROS Type:** `Topic/Publish`

**Data Type:** `geometry_msgs/WrenchStamped`

**Version:** 
- 1.0.0: added

**Cli:** ` rostopic echo /zj_humanoid/hand/wrist_force_sensor/right`

**Example:** 
```python
def test_sub_wrist_force_sensor_right(*, name=f"{PREFIX}/wrist_force_sensor/right"):
    """
    订阅右手腕力传感器数据

    Notes
    -----
    接收右手腕力传感器数据，包括力和力矩的三轴分量

    Version
    -------
    - 1.0.0: added

    Cli Examples
    ------------
    $ rostopic echo /zj_humanoid/hand/wrist_force_sensor/right
    """
    def callback(msg):
        print(f"hand wrist force sensor right | {msg}")
    
    rospy.Subscriber(name, WrenchStamped, callback)
    print(f"Subscribing to {name}")
    rospy.sleep(2)

```

## `wrist_force_sensor/left/zero`

**Description:**
```tex
调用左手腕力传感器零位校准服务
```

**ROS Type:** `Service`

**Data Type:** `std_srvs/Trigger`

**Version:** 
- 1.0.0: added

**Cli:** ` rosservice call /hand/wrist_force_sensor/left/zero "{}"`

**Example:** 
```python
def test_zero_wrist_force_sensor_left(*, name=f"{PREFIX}/wrist_force_sensor/left/zero"):
    """
    调用左手腕力传感器零位校准服务

    Notes
    -----
    对左手腕力传感器进行零位校准，
    清除当前传感器偏置，将当前力和力矩读数设为零点

    Version
    -------
    - 1.0.0: added

    Cli Examples
    ------------
    $ rosservice call /hand/wrist_force_sensor/left/zero "{}"
    """
    rospy.wait_for_service(name)
    try:
        zero_service = rospy.ServiceProxy(name, Trigger)
        req = TriggerRequest()
        resp = zero_service(req)
        print(f"Service call to {name}")
        print(f"Response: success={resp.success}, message='{resp.message}'")
        return resp.success
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
        return False

```

## `wrist_force_sensor/right/zero`

**Description:**
```tex
调用右手腕力传感器零位校准服务
```

**ROS Type:** `Service`

**Data Type:** `std_srvs/Trigger`

**Version:** 
- 1.0.0: added

**Cli:** ` rosservice call /hand/wrist_force_sensor/right/zero "{}"`

**Example:** 
```python
def test_zero_wrist_force_sensor_right(*, name=f"{PREFIX}/wrist_force_sensor/right/zero"):
    """
    调用右手腕力传感器零位校准服务

    Notes
    -----
    对右手腕力传感器进行零位校准，
    清除当前传感器偏置，将当前力和力矩读数设为零点

    Version
    -------
    - 1.0.0: added

    Cli Examples
    ------------
    $ rosservice call /hand/wrist_force_sensor/right/zero "{}"
    """
    rospy.wait_for_service(name)
    try:
        zero_service = rospy.ServiceProxy(name, Trigger)
        req = TriggerRequest()
        resp = zero_service(req)
        print(f"Service call to {name}")
        print(f"Response: success={resp.success}, message='{resp.message}'")
        return resp.success
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
        return False

```

## `finger_pressures/left/zero`

**Description:**
```tex
调用左手指尖压力传感器零位校准服务
```

**ROS Type:** `Service`

**Data Type:** `std_srvs/Trigger`

**Version:** 
- 1.0.0: added

**Cli:** ` rosservice call /hand/finger_pressures/left/zero "{}"`

**Example:** 
```python
def test_zero_finger_pressures_left(*, name=f"{PREFIX}/finger_pressures/left/zero"):
    """
    调用左手指尖压力传感器零位校准服务

    Notes
    -----
    对左手指尖压力传感器进行零位校准，
    清除当前传感器偏置，将当前读数设为零点

    Version
    -------
    - 1.0.0: added

    Cli Examples
    ------------
    $ rosservice call /hand/finger_pressures/left/zero "{}"
    """
    rospy.wait_for_service(name)
    try:
        zero_service = rospy.ServiceProxy(name, Trigger)
        req = TriggerRequest()
        resp = zero_service(req)
        print(f"Service call to {name}")
        print(f"Response: success={resp.success}, message='{resp.message}'")
        return resp.success
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
        return False

```

## `finger_pressures/right/zero`

**Description:**
```tex
调用右手指尖压力传感器零位校准服务
```

**ROS Type:** `Service`

**Data Type:** `std_srvs/Trigger`

**Version:** 
- 1.0.0: added

**Cli:** ` rosservice call /hand/finger_pressures/right/zero "{}"`

**Example:** 
```python
def test_zero_finger_pressures_right(*, name=f"{PREFIX}/finger_pressures/right/zero"):
    """
    调用右手指尖压力传感器零位校准服务

    Notes
    -----
    对右手指尖压力传感器进行零位校准，
    清除当前传感器偏置，将当前读数设为零点

    Version
    -------
    - 1.0.0: added

    Cli Examples
    ------------
    $ rosservice call /hand/finger_pressures/right/zero "{}"
    """
    rospy.wait_for_service(name)
    try:
        zero_service = rospy.ServiceProxy(name, Trigger)
        req = TriggerRequest()
        resp = zero_service(req)
        print(f"Service call to {name}")
        print(f"Response: success={resp.success}, message='{resp.message}'")
        return resp.success
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
        return False

```


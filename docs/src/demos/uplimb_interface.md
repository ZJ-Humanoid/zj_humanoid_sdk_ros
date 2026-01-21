## `FK/left_arm`

**Description:**
```tex
左臂正解
```

**ROS Type:** `Service`

**Data Type:** `upperlimb/FK`

**Version:** 
- 1.0.0 : added

**Cli:** ` `

**Example:** 
```python
def test_left_arm_FK(joints, *,name=f"{PREFIX}/FK/left_arm"):
    """
    左臂正解

    Parameters
    ----------
    joints : List[float]
        关节角度列表

    Version
    -------
    - 1.0.0 : added
    """
    client = rospy.ServiceProxy(name, FK)

    data = FKRequest()
    data.joints = joints

    resp:FKResponse = client.call(data)
    print(f"Service Name:{name} | Resp Pose:{resp.pose}")

```

## `FK/right_arm`

**Description:**
```tex
右臂正解
```

**ROS Type:** `Service`

**Data Type:** `upperlimb/FK`

**Version:** 
- 1.0.0: added

**Cli:** ` `

**Example:** 
```python
def test_right_arm_FK(joints, *,name=f"{PREFIX}/FK/right_arm"):
    """
    右臂正解

    Parameters
    ----------
    joints : List[float]
        关节角度列表

    Version
    -------
    - 1.0.0: added
    """
    client = rospy.ServiceProxy(name, FK)

    data = FKRequest()
    data.joints = joints
    
    resp:FKResponse = client.call(data)
    print(f"Service Name:{name} | Resp Pose:{resp.pose}")

```

## `IK/left_arm`

**Description:**
```tex
左臂逆解
```

**ROS Type:** `Service`

**Data Type:** `upperlimb/IK`

**Version:** 
- 1.0.0: added

**Cli:** ` `

**Example:** 
```python
def test_left_arm_IK(pose, q7, *, name=f"{PREFIX}/IK/left_arm"):
    """
    左臂逆解

    Parameters
    ----------
    pose : List[float]
        [x, y, z, qx, qy, qz, qw] 笛卡尔空间坐标和四元数
    q7 : float
        第7关节角度

    Version
    -------
    - 1.0.0: added
    """
    client = rospy.ServiceProxy(name, IK)

    data = IKRequest()
    data.pose.position.x = pose[0]
    data.pose.position.y = pose[1]
    data.pose.position.z = pose[2]
    data.pose.orientation.x = pose[3]
    data.pose.orientation.y = pose[4]
    data.pose.orientation.z = pose[5]
    data.pose.orientation.w = pose[6]
    data.q7   = q7

    resp:IKResponse = client.call(data)
    print(f"Service Name:{name} | Success:{resp.success} | Resp Joints:{resp.joints} | Resp Nums:{resp.nums} | Resp phi:{resp.phi}")

```

## `IK/right_arm`

**Description:**
```tex
右臂逆解
```

**ROS Type:** `Service`

**Data Type:** `upperlimb/IK`

**Version:** 
- 1.0.0: added

**Cli:** ` `

**Example:** 
```python
def test_right_arm_IK(pose, q7, *, name=f"{PREFIX}/IK/right_arm"):
    """
    右臂逆解

    Parameters
    ----------
    pose : List[float]
        [x, y, z, qx, qy, qz, qw] 笛卡尔空间坐标和四元数
    q7 : float
        第7关节角度

    Version
    -------
    - 1.0.0: added
    """
    client = rospy.ServiceProxy(name, IK)

    data = IKRequest()
    data.pose.position.x = pose[0]
    data.pose.position.y = pose[1]
    data.pose.position.z = pose[2]
    data.pose.orientation.x = pose[3]
    data.pose.orientation.y = pose[4]
    data.pose.orientation.z = pose[5]
    data.pose.orientation.w = pose[6]
    data.q7   = q7

    resp:IKResponse = client.call(data)
    print(f"Service Name:{name} | Success:{resp.success} | Resp Joints:{resp.joints} | Resp Nums:{resp.nums} | Resp phi:{resp.phi}")

```

## `go_down/left_arm`

**Description:**
```tex
左臂放下
```

**ROS Type:** `Service`

**Data Type:** `std_srvs/Trigger`

**Version:** 
- 1.0.0: added

**Cli:** ` `

**Example:** 
```python
def test_left_arm_go_down(*, name=f"{PREFIX}/go_down/left_arm"):
    """
    左臂放下
    
    Version
    -------
    - 1.0.0: added
    """
    client = rospy.ServiceProxy(name, Trigger)
    resp: TriggerResponse = client.call(TriggerRequest())
    print(
        f"Service Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}"
    )

```

## `go_down/right_arm`

**Description:**
```tex
右臂放下
```

**ROS Type:** `Service`

**Data Type:** `std_srvs/Trigger`

**Version:** 
- 1.0.0: added

**Cli:** ` `

**Example:** 
```python
def test_right_arm_go_down(*, name=f"{PREFIX}/go_down/right_arm"):
    """
    右臂放下
    
    Version
    -------
    - 1.0.0: added
    """
    client = rospy.ServiceProxy(name, Trigger)
    resp: TriggerResponse = client.call(TriggerRequest())
    print(
        f"Service Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}"
    )

```

## `go_down/dual_arm`

**Description:**
```tex
双臂放下
```

**ROS Type:** `Service`

**Data Type:** `std_srvs/Trigger`

**Version:** 
 

**Cli:** ` `

**Example:** 
```python
def test_dual_arm_go_down(*, name=f"{PREFIX}/go_down/dual_arm"):
    """
    双臂放下
    """
    client = rospy.ServiceProxy(name, Trigger)
    resp: TriggerResponse = client.call(TriggerRequest())
    print(
        f"Service Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}"
    )

```

## `go_home/left_arm`

**Description:**
```tex
左臂回到内置设置的home点
```

**ROS Type:** `Service`

**Data Type:** `std_srvs/Trigger`

**Version:** 
- 1.0.0: added

**Cli:** ` `

**Example:** 
```python
def test_left_arm_go_home(*,name=f"{PREFIX}/go_home/left_arm"):
    """
    左臂回到内置设置的home点
    
    Version
    -------
    - 1.0.0: added
    """
    client = rospy.ServiceProxy(name, Trigger)
    resp: TriggerResponse = client.call(TriggerRequest())
    print(f"Service Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}"
    )

```

## `go_home/right_arm`

**Description:**
```tex
右臂回到内置设置的home点
```

**ROS Type:** `Service`

**Data Type:** `std_srvs/Trigger`

**Version:** 
- 1.0.0: added

**Cli:** ` `

**Example:** 
```python
def test_right_arm_go_home(*,name=f"{PREFIX}/go_home/right_arm"):
    """
    右臂回到内置设置的home点
    
    Version
    -------
    - 1.0.0: added
    """
    client = rospy.ServiceProxy(name, Trigger)
    resp: TriggerResponse = client.call(TriggerRequest())
    print(
        f"Service Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}"
    )

```

## `go_home/dual_arm`

**Description:**
```tex
双臂回到内置设置的home点
```

**ROS Type:** `Service`

**Data Type:** `std_srvs/Trigger`

**Version:** 
 

**Cli:** ` `

**Example:** 
```python
def test_dual_arm_go_home(*,name=f"{PREFIX}/go_home/dual_arm"):
    """
    双臂回到内置设置的home点
    """
    client = rospy.ServiceProxy(name, Trigger)
    resp: TriggerResponse = client.call(TriggerRequest())
    print(
        f"Service Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}"
    )

```

## `go_home/neck`

**Description:**
```tex
脖子回到内置设置的home点
```

**ROS Type:** `Service`

**Data Type:** `std_srvs/Trigger`

**Version:** 
 

**Cli:** ` `

**Example:** 
```python
def test_neck_go_home(*,name=f"{PREFIX}/go_home/neck"):
    """
    脖子回到内置设置的home点
    """
    client = rospy.ServiceProxy(name, Trigger)
    resp: TriggerResponse = client.call(TriggerRequest())
    print(
        f"Service Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}"
    )

```

## `go_home/waist`

**Description:**
```tex
腰部回到内置设置的home点
```

**ROS Type:** `Service`

**Data Type:** `std_srvs/Trigger`

**Version:** 
 

**Cli:** ` `

**Example:** 
```python
def test_waist_go_home(*,name=f"{PREFIX}/go_home/waist"):
    """
    腰部回到内置设置的home点
    """
    client = rospy.ServiceProxy(name, Trigger)
    resp: TriggerResponse = client.call(TriggerRequest())
    print(
        f"Service Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}"
    )

```

## `go_home/lifting`

**Description:**
```tex
升降回到内置设置的home点
```

**ROS Type:** `Service`

**Data Type:** `std_srvs/Trigger`

**Version:** 
 

**Cli:** ` `

**Example:** 
```python
def test_lifting_go_home(*,name=f"{PREFIX}/go_home/lifting"):
    """
    升降回到内置设置的home点
    """
    client = rospy.ServiceProxy(name, Trigger)
    resp: TriggerResponse = client.call(TriggerRequest())
    print(
        f"Service Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}"
    )

```

## `go_home/whole_body`

**Description:**
```tex
全身指定部位回到内置设置的home点
```

**ROS Type:** `Service`

**Data Type:** `upperlimb/ArmType`

**Version:** 
 

**Cli:** ` `

**Example:** 
```python
def test_whole_body_go_home(arm_type, *, name=f"{PREFIX}/go_home/whole_body"):
    """
    全身指定部位回到内置设置的home点

    Parameters
    ----------
    arm_type : int
        机械臂类型标识
    """
    client = rospy.ServiceProxy(name, ArmType)
    resp: ArmTypeResponse = client.call(ArmTypeRequest(arm_type))
    print(
        f"Service Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}"
    )

```

## `is_singular/left_arm`

**Description:**
```tex
左臂奇异性判断
```

**ROS Type:** `Service`

**Data Type:** `upperlimb/IsSingular`

**Version:** 
 

**Cli:** ` `

**Example:** 
```python
def test_left_arm_is_singular(joints,*, name: str = f"{PREFIX}/is_singular/left_arm"):
    """
    左臂奇异性判断

    Parameters
    ----------
    joints : List[float]
        关节角度列表

    Examples
    --------
    CLI 命令::
    
        rosservice call /zj_humanoid/upperlimb/is_singular/left_arm \
            "joints: [0.0, 0.4, 0.5, 0.0, 0.0, 0.0, 0.0]"

    Notes
    -----
    .. versionadded:: 1.0.0
        添加基本功能
    """
    # 创建 service client
    client = rospy.ServiceProxy(name, IsSingular)

    # 等待服务可用，超时 5s
    rospy.wait_for_service(name, timeout=5.0)

    req = IsSingular._request_class()
    req.joints = joints

    # 调用服务
    resp = client.call(req)

    print(f"Service Name:{name} | Request Joints:{joints} | Resp is_singular:{resp.is_singular} | message:{resp.message}")

```

## `is_singular/right_arm`

**Description:**
```tex
右臂奇异性判断
```

**ROS Type:** `Service`

**Data Type:** `upperlimb/IsSingular`

**Version:** 
- 1.0.0: added

**Cli:** ` `

**Example:** 
```python
def test_right_arm_is_singular(joints, *, name: str = f"{PREFIX}/is_singular/right_arm"):
    """
    右臂奇异性判断

    Parameters
    ----------
    joints : List[float]
        关节角度列表

    Version
    -------
    - 1.0.0: added
    """
    # 创建 service client
    client = rospy.ServiceProxy(name, IsSingular)

    # 等待服务可用，超时 5s
    rospy.wait_for_service(name, timeout=5.0)

    req = IsSingular._request_class()
    req.joints = joints

    # 调用服务
    resp = client.call(req)

    print(f"Service Name:{name} | Request Joints:{joints} | Resp is_singular:{resp.is_singular} | message:{resp.message}")

```

## `movej/lift/`

**Description:**
```tex
关节空间下,升降点到点运动
```

**ROS Type:** `Service`

**Data Type:** `upperlimb/MoveJ`

**Version:** 
- 1.0.0: added

**Cli:** ` `

**Example:** 
```python
def test_lift_movej_v_acc(joints: List[float], v: float, acc: float, is_async: bool, *, name: str = f"{PREFIX}/movej/lift/"):
    """
    关节空间下,升降点到点运动

    Parameters
    ----------
    joints : List[float]
        目标关节位置列表
    v : float
        运动速度
    acc : float
        加速度
    is_async : bool
        是否异步执行
        
    Version
    -------
    - 1.0.0: added
    """
    client = rospy.ServiceProxy(name, MoveJ)
    data = MoveJRequest()
    data.joints = joints
    data.v = v
    data.acc = acc
    data.is_async = is_async
    resp: MoveJResponse = client.call(data)
    print(f"Service Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}")

```

## `movej/waist/`

**Description:**
```tex
关节空间下,腰部点到点运动
```

**ROS Type:** `Service`

**Data Type:** `upperlimb/MoveJ`

**Version:** 
- 1.0.0: added

**Cli:** ` `

**Example:** 
```python
def test_waist_movej_v_acc(joints: List[float], v: float, acc: float, is_async: bool, *, name: str = f"{PREFIX}/movej/waist/"):
    """
    关节空间下,腰部点到点运动

    Parameters
    ----------
    joints : List[float]
        目标关节位置列表
    v : float
        运动速度
    acc : float
        加速度
    is_async : bool
        是否异步执行
        
    Version
    -------
    - 1.0.0: added
    """
    client = rospy.ServiceProxy(name, MoveJ)
    data = MoveJRequest()
    data.joints = joints
    data.v = v
    data.acc = acc
    data.is_async = is_async
    resp: MoveJResponse = client.call(data)
    print(f"Service Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}")

```

## `movej/neck/`

**Description:**
```tex
关节空间下,脖子点到点运动
```

**ROS Type:** `Service`

**Data Type:** `upperlimb/MoveJ`

**Version:** 
- 1.0.0: added

**Cli:** ` `

**Example:** 
```python
def test_neck_movej_v_acc(joints: List[float], v: float, acc: float, is_async: bool, *, name: str = f"{PREFIX}/movej/neck/"):
    """
    关节空间下,脖子点到点运动

    Parameters
    ----------
    joints : List[float]
        目标关节位置列表
    v : float
        运动速度
    acc : float
        加速度
    is_async : bool
        是否异步执行
        
    Version
    -------
    - 1.0.0: added
    """
    client = rospy.ServiceProxy(name, MoveJ)
    data = MoveJRequest()
    data.joints = joints
    data.v = v
    data.acc = acc
    data.is_async = is_async
    resp: MoveJResponse = client.call(data)
    print(f"Service Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}")

```

## `movej/left_arm/`

**Description:**
```tex
关节空间下,左臂点到点运动
```

**ROS Type:** `Service`

**Data Type:** `upperlimb/MoveJ`

**Version:** 
 

**Cli:** ` `

**Example:** 
```python
def test_left_arm_movej_v_acc(joints: List[float], v: float, acc: float, is_async: bool, *, name: str = f"{PREFIX}/movej/left_arm/"):
    """
    关节空间下,左臂点到点运动

    Parameters
    ----------
    joints : List[float]
        目标关节位置列表
    v : float
        运动速度
    acc : float
        加速度
    is_async : bool
        是否异步执行
    """
    client = rospy.ServiceProxy(name, MoveJ)
    data = MoveJRequest()
    data.joints = joints
    data.v = v
    data.acc = acc
    data.is_async = is_async
    resp: MoveJResponse = client.call(data)
    print(f"Service Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}")

```

## `movej/right_arm/`

**Description:**
```tex
关节空间下,右臂点到点运动
```

**ROS Type:** `Service`

**Data Type:** `upperlimb/MoveJ`

**Version:** 
 

**Cli:** ` `

**Example:** 
```python
def test_right_arm_movej_v_acc(joints: List[float], v: float, acc: float, is_async: bool, *, name: str = f"{PREFIX}/movej/right_arm/"):
    """
    关节空间下,右臂点到点运动

    Parameters
    ----------
    joints : List[float]
        目标关节位置列表
    v : float
        运动速度
    acc : float
        加速度
    is_async : bool
        是否异步执行
    """
    client = rospy.ServiceProxy(name, MoveJ)
    data = MoveJRequest()
    data.joints = joints
    data.v = v
    data.acc = acc
    data.is_async = is_async
    resp: MoveJResponse = client.call(data)
    print(f"Service Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}")

```

## `movej/dual_arm/`

**Description:**
```tex
关节空间下,双臂点到点运动
```

**ROS Type:** `Service`

**Data Type:** `upperlimb/MoveJ`

**Version:** 
 

**Cli:** ` `

**Example:** 
```python
def test_dual_arm_movej_v_acc(joints: List[float], v: float, acc: float, is_async: bool, *, name: str = f"{PREFIX}/movej/dual_arm/"):
    """
    关节空间下,双臂点到点运动

    Parameters
    ----------
    joints : List[float]
        目标关节位置列表
    v : float
        运动速度
    acc : float
        加速度
    is_async : bool
        是否异步执行
    """
    client = rospy.ServiceProxy(name, MoveJ)
    data = MoveJRequest()
    data.joints = joints
    data.v = v
    data.acc = acc
    data.is_async = is_async
    resp: MoveJResponse = client.call(data)
    print(f"Service Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}")

```

## `movej/whole_body/`

**Description:**
```tex
关节空间下,全身各部位点到点运动
```

**ROS Type:** `Service`

**Data Type:** `upperlimb/MoveJ`

**Version:** 
 

**Cli:** ` `

**Example:** 
```python
def test_whole_body_movej_v_acc(joints: List[float], v: float, acc: float, arm_type: int, is_async: bool, *, name: str = f"{PREFIX}/movej/whole_body/"):
    """
    关节空间下,全身各部位点到点运动

    Parameters
    ----------
    joints : List[float]
        目标关节位置列表
    v : float
        运动速度
    acc : float
        加速度
    arm_type : int
        机械臂类型标识
    is_async : bool
        是否异步执行
    """
    client = rospy.ServiceProxy(name, MoveJ)
    data = MoveJRequest()
    data.joints = joints
    data.v = v
    data.acc = acc
    data.arm_type = arm_type
    data.is_async = is_async
    resp: MoveJResponse = client.call(data)
    print(f"Service Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}")

```

## `movej_by_path/right_arm`

**Description:**
```tex
关节空间下,右臂轨迹点路径运动
```

**ROS Type:** `Service`

**Data Type:** `upperlimb/MoveJByPath`

**Version:** 
 

**Cli:** ` `

**Example:** 
```python
def test_service_right_movej_by_path1(path, total_time, is_async, movej_t=5,  *,name=f"{PREFIX}/movej_by_path/right_arm",):
    """
    关节空间下,右臂轨迹点路径运动

    Parameters
    ----------
    path : List[List[float]]
        轨迹点路径
    total_time : float
        总时间
    is_async : bool
        是否异步执行
    movej_t : float, optional
        移动到初始位置的时间,默认5秒
    """
    client = rospy.ServiceProxy(name, MoveJByPath)

    data = MoveJByPathRequest()

    [data.path.append(Joints(joint)) for joint in path]
    data.time = total_time
    data.is_async = is_async 

    movej = rospy.ServiceProxy(f"{PREFIX}/movej/right_arm/", MoveJ)
    movej_data = MoveJRequest()
    movej_data.joints = path[0]
    movej_data.t = movej_t
    movej_data.is_async = False

    movej.call(movej_data)


    resp:MoveJByPathResponse = client.call(data)
    print(f"Service Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}")

```

## `movej_by_path/left_arm`

**Description:**
```tex
关节空间下,左臂轨迹点路径运动
```

**ROS Type:** `Service`

**Data Type:** `upperlimb/MoveJByPath`

**Version:** 
 

**Cli:** ` `

**Example:** 
```python
def test_service_left_movej_by_path1(path, total_time, is_async,  movej_t=5, *,name=f"{PREFIX}/movej_by_path/left_arm",):
    """
    关节空间下,左臂轨迹点路径运动

    Parameters
    ----------
    path : List[List[float]]
        轨迹点路径
    total_time : float
        总时间
    is_async : bool
        是否异步执行
    movej_t : float, optional
        移动到初始位置的时间,默认5秒
    """
    client = rospy.ServiceProxy(name, MoveJByPath)

    data = MoveJByPathRequest()

    [data.path.append(Joints(joint)) for joint in path]
    data.time = total_time
    data.is_async = is_async 

    movej = rospy.ServiceProxy(f"{PREFIX}/movej/left_arm/", MoveJ)
    movej_data = MoveJRequest()
    movej_data.joints = path[0]
    movej_data.t = movej_t
    movej_data.is_async = False

    movej.call(movej_data)

    resp:MoveJByPathResponse = client.call(data)
    print(f"Service Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}")

```

## `movej_by_path/dual_arm`

**Description:**
```tex
关节空间下,双臂轨迹点路径运动
```

**ROS Type:** `Service`

**Data Type:** `upperlimb/MoveJByPath`

**Version:** 
 

**Cli:** ` `

**Example:** 
```python
def test_service_dual_movej_by_path1(path, total_time, is_async, movej_t=5, *,name=f"{PREFIX}/movej_by_path/dual_arm",):
    """
    关节空间下,双臂轨迹点路径运动

    Parameters
    ----------
    path : List[List[float]]
        轨迹点路径
    total_time : float
        总时间
    is_async : bool
        是否异步执行
    movej_t : float, optional
        移动到初始位置的时间,默认5秒
    """
    client = rospy.ServiceProxy(name, MoveJByPath)

    data = MoveJByPathRequest()


    [data.path.append(Joints(joint)) for joint in path]
    data.time = total_time
    data.is_async = is_async 

    movej = rospy.ServiceProxy(f"{PREFIX}/movej/dual_arm/", MoveJ)
    movej_data = MoveJRequest()
    movej_data.joints = path[0]
    movej_data.t = movej_t
    movej_data.is_async = False

    movej.call(movej_data)

    resp:MoveJByPathResponse = client.call(data)
    print(f"Service Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}")

```

## `movej_by_path/dual_arm/`

**Description:**
```tex
关节空间下,双臂轨迹点路径运动

Args:
    joints (_type_): _description_
    timestamp (_type_): _description_
    movej_t (_type_): _description_
    is_async (bool, optional): _description_. Defaults to False.
    name (_type_, optional): _description_. Defaults to f"{PREFIX}/movej_by_path/dual_arm/".
```

**ROS Type:** `Service`

**Data Type:** `upperlimb/MoveJByPath`

**Version:** 
 

**Cli:** ` `

**Example:** 
```python
def test_service_neck_movej_by_path1(joints, timestamp, is_async,movej_t=5, *,name=f"{PREFIX}/movej_by_path/dual_arm/",):
    """
    关节空间下,双臂轨迹点路径运动

    Args:
        joints (_type_): _description_
        timestamp (_type_): _description_
        movej_t (_type_): _description_
        is_async (bool, optional): _description_. Defaults to False.
        name (_type_, optional): _description_. Defaults to f"{PREFIX}/movej_by_path/dual_arm/".
    """
    client = rospy.ServiceProxy(name, MoveJByPath)

    data = MoveJByPathRequest()


    [data.timestamp.append(t) for t in timestamp]
    [data.path.append(Joints(joint)) for joint in joints]
    data.is_async = is_async

    movej = rospy.ServiceProxy(f"{PREFIX}/movej/dual_arm/", MoveJ)
    movej_data = MoveJRequest()
    movej_data.joints = joints[0]
    movej_data.t = movej_t
    movej_data.is_async = True
    movej.call(movej_data)

    resp:MoveJByPathResponse = client.call(data)
    print(f"Service Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}")

```

## `movej_by_path/neck`

**Description:**
```tex
关节空间下,脖子轨迹点路径运动
```

**ROS Type:** `Service`

**Data Type:** `upperlimb/MoveJByPath`

**Version:** 
 

**Cli:** ` `

**Example:** 
```python
def test_service_neck_movej_by_path2(path, total_time, is_async, movej_t=5, *,name=f"{PREFIX}/movej_by_path/neck",):
    """
    关节空间下,脖子轨迹点路径运动

    Parameters
    ----------
    path : List[List[float]]
        轨迹点路径
    total_time : float
        总时间
    is_async : bool
        是否异步执行
    movej_t : float, optional
        移动到初始位置的时间,默认5秒
    """
    client = rospy.ServiceProxy(name, MoveJByPath)

    data = MoveJByPathRequest()

    [data.path.append(Joints(joint)) for joint in path]
    data.time = total_time
    data.is_async = is_async 

    movej = rospy.ServiceProxy(f"{PREFIX}/movej/neck/", MoveJ)
    movej_data = MoveJRequest()
    movej_data.joints = path[0]
    movej_data.t = movej_t
    movej_data.is_async = False

    movej.call(movej_data)

    resp:MoveJByPathResponse = client.call(data)
    print(f"Service Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}")

```

## `movej_by_path/waist`

**Description:**
```tex
关节空间下,腰部轨迹点路径运动
```

**ROS Type:** `Service`

**Data Type:** `upperlimb/MoveJByPath`

**Version:** 
 

**Cli:** ` `

**Example:** 
```python
def test_service_waist_movej_by_path1(path, total_time, is_async, movej_t=5, *,name=f"{PREFIX}/movej_by_path/waist",):
    """
    关节空间下,腰部轨迹点路径运动

    Parameters
    ----------
    path : List[List[float]]
        轨迹点路径
    total_time : float
        总时间
    is_async : bool
        是否异步执行
    movej_t : float, optional
        移动到初始位置的时间,默认5秒
    """
    client = rospy.ServiceProxy(name, MoveJByPath)

    data = MoveJByPathRequest()

    [data.path.append(Joints(joint)) for joint in path]
    data.time = total_time
    data.is_async = is_async 

    movej = rospy.ServiceProxy(f"{PREFIX}/movej/waist/", MoveJ)
    movej_data = MoveJRequest()
    movej_data.joints = path[0]
    movej_data.t = movej_t
    movej_data.is_async = False

    movej.call(movej_data)

    resp:MoveJByPathResponse = client.call(data)
    print(f"Service Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}")

```

## `movej_by_path/whole_body`

**Description:**
```tex
关节空间下,全身轨迹点路径运动
```

**ROS Type:** `Service`

**Data Type:** `upperlimb/MoveJByPath`

**Version:** 
 

**Cli:** ` `

**Example:** 
```python
def test_service_waist_movej_by_path2(path, total_time, is_async, movej_t=5, *,name=f"{PREFIX}/movej_by_path/whole_body",):
    """
    关节空间下,全身轨迹点路径运动

    Parameters
    ----------
    path : List[List[float]]
        轨迹点路径
    total_time : float
        总时间
    is_async : bool
        是否异步执行
    movej_t : float, optional
        移动到初始位置的时间,默认5秒
    """
    client = rospy.ServiceProxy(name, MoveJByPath)

    data = MoveJByPathRequest()

    [data.path.append(Joints(joint)) for joint in path]
    data.time = total_time
    data.is_async = is_async 
    data.arm_type = 4

    movej = rospy.ServiceProxy(f"{PREFIX}/movej/whole_body/", MoveJ)
    movej_data = MoveJRequest()
    movej_data.joints = path[0]
    movej_data.t = movej_t
    movej_data.is_async = False
    movej_data.arm_type = 4

    movej.call(movej_data)

    resp:MoveJByPathResponse = client.call(data)
    print(f"Service Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}")

```

## `movej_by_pose/left_arm/`

**Description:**
```tex
 
```

**ROS Type:** `Service`

**Data Type:** `upperlimb/MoveJByPose`

**Version:** 
 

**Cli:** ` `

**Example:** 
```python
def test_left_movej_by_pose(pose_data: List[List[float]], q7_data: float, v: float, acc: float, *, name: str=f"{PREFIX}/movej_by_pose/left_arm/"):
    client = rospy.ServiceProxy(name, MoveJByPose)
    data = MoveJByPoseRequest()
    data.v = v
    data.acc = acc
    data.q7 = q7_data
    data.pose[0].position.x = pose_data[0][0]
    data.pose[0].position.y = pose_data[0][1]
    data.pose[0].position.z = pose_data[0][2]
    data.pose[0].orientation.x = pose_data[0][3]
    data.pose[0].orientation.y = pose_data[0][4]
    data.pose[0].orientation.z = pose_data[0][5]
    data.pose[0].orientation.w = pose_data[0][6]

    resp: MoveLResponse = client.call(data)
    print(f"Service Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}")

```

## `movej_by_pose/right_arm/`

**Description:**
```tex
 
```

**ROS Type:** `Service`

**Data Type:** `upperlimb/MoveJByPose`

**Version:** 
 

**Cli:** ` `

**Example:** 
```python
def test_right_movej_by_pose(pose_data: List[List[float]], q7_data: float, v: float, acc: float, *, name: str=f"{PREFIX}/movej_by_pose/right_arm/"):
    client = rospy.ServiceProxy(name, MoveJByPose)
    data = MoveJByPoseRequest()
    data.v = v
    data.acc = acc
    data.q7 = q7_data

    data.pose[0].position.x = pose_data[0][0]
    data.pose[0].position.y = pose_data[0][1]
    data.pose[0].position.z = pose_data[0][2]
    data.pose[0].orientation.x = pose_data[0][3]
    data.pose[0].orientation.y = pose_data[0][4]
    data.pose[0].orientation.z = pose_data[0][5]
    data.pose[0].orientation.w = pose_data[0][6]

    resp: MoveLResponse = client.call(data)
    print(f"Service Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}")

```

## `movej_by_pose/dual_arm/`

**Description:**
```tex
 
```

**ROS Type:** `Service`

**Data Type:** `upperlimb/MoveJByPose`

**Version:** 
 

**Cli:** ` `

**Example:** 
```python
def test_dual_movej_by_pose(pose_data: List[List[float]], q7_data: float, v: float, acc: float, *, name: str=f"{PREFIX}/movej_by_pose/dual_arm/"):
    client = rospy.ServiceProxy(name, MoveJByPose)
    data = MoveJByPoseRequest()
    data.v = v
    data.acc = acc
    data.q7 = q7_data
    data.pose[0].position.x = pose_data[0][0]
    data.pose[0].position.y = pose_data[0][1]
    data.pose[0].position.z = pose_data[0][2]
    data.pose[0].orientation.x = pose_data[0][3]
    data.pose[0].orientation.y = pose_data[0][4]
    data.pose[0].orientation.z = pose_data[0][5]
    data.pose[0].orientation.w = pose_data[0][6]
    data.pose[1].position.x = pose_data[1][0]
    data.pose[1].position.y = pose_data[1][1]
    data.pose[1].position.z = pose_data[1][2]
    data.pose[1].orientation.x = pose_data[1][3]
    data.pose[1].orientation.y = pose_data[1][4]
    data.pose[1].orientation.z = pose_data[1][5]
    data.pose[1].orientation.w = pose_data[1][6]


    resp: MoveLResponse = client.call(data)
    print(f"Service Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}")

```

## `movel/left_arm/`

**Description:**
```tex
直线轨迹运动 - 左臂

从当前位置沿直线运动到目标笛卡尔空间坐标,指定运动速度和加速度。
```

**ROS Type:** `Service`

**Data Type:** `upperlimb/MoveL`

**Version:** 
- 1.0.0: added

**Cli:** ` `

**Example:** 
```python
def test_service_left_movel(pose: List[float], v: float, acc: float, *, name: str=f"{PREFIX}/movel/left_arm/"):
    """
    直线轨迹运动 - 左臂
    
    从当前位置沿直线运动到目标笛卡尔空间坐标,指定运动速度和加速度。
    
    Parameters
    ----------
    pose : List[float]
        [x, y, z, qx, qy, qz, qw] 笛卡尔空间坐标和四元数
    v : float
        运动速度 (m/s)
    acc : float
        加速度 (m/s²)
        
    Version
    -------
    - 1.0.0: added
    """
    client = rospy.ServiceProxy(name, MoveL)

    data = MoveLRequest()
    
    data.pose[0].position.x = pose[0]
    data.pose[0].position.y = pose[1]
    data.pose[0].position.z = pose[2]

    data.pose[0].orientation.x = pose[3]
    data.pose[0].orientation.y = pose[4]
    data.pose[0].orientation.z = pose[5]
    data.pose[0].orientation.w = pose[6]

    data.v = v
    data.acc = acc

    resp:MoveLResponse = client.call(data)
    print(f"Service Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}")

```

## `movel/right_arm/`

**Description:**
```tex
直线轨迹运动 - 右臂

从当前位置沿直线运动到目标笛卡尔空间坐标,指定运动速度和加速度。
```

**ROS Type:** `Service`

**Data Type:** `upperlimb/MoveL`

**Version:** 
- 1.0.0: added

**Cli:** ` `

**Example:** 
```python
def test_service_right_movel(pose: List[float], v: float, acc: float, *, name: str=f"{PREFIX}/movel/right_arm/",):
    """
    直线轨迹运动 - 右臂
    
    从当前位置沿直线运动到目标笛卡尔空间坐标,指定运动速度和加速度。
    
    Parameters
    ----------
    pose : List[float]
        [x, y, z, qx, qy, qz, qw] 笛卡尔空间坐标和四元数
    v : float
        运动速度 (m/s)
    acc : float
        加速度 (m/s²)
        
    Version
    -------
    - 1.0.0: added
    """
    client = rospy.ServiceProxy(name, MoveL)

    data = MoveLRequest()
    
    data.pose[0].position.x = pose[0]
    data.pose[0].position.y = pose[1]
    data.pose[0].position.z = pose[2]

    data.pose[0].orientation.x = pose[3]
    data.pose[0].orientation.y = pose[4]
    data.pose[0].orientation.z = pose[5]
    data.pose[0].orientation.w = pose[6]

    data.v = v
    data.acc = acc


    resp:MoveLResponse = client.call(data)
    print(f"Service Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}")

```

## `movel/dual_arm/`

**Description:**
```tex
直线轨迹运动 - 双臂

从当前位置沿直线运动到目标笛卡尔空间坐标,指定运动速度和加速度。
```

**ROS Type:** `Service`

**Data Type:** `upperlimb/MoveL`

**Version:** 
 

**Cli:** ` `

**Example:** 
```python
def test_service_dual_movel(pose: List[float], v: float, acc: float, *, name: str=f"{PREFIX}/movel/dual_arm/",):
    """
    直线轨迹运动 - 双臂
    
    从当前位置沿直线运动到目标笛卡尔空间坐标,指定运动速度和加速度。
    
    Parameters
    ----------
    pose : List[float]
        [左臂x, y, z, qx, qy, qz, qw, 右臂x, y, z, qx, qy, qz, qw] 双臂笛卡尔空间坐标和四元数
    v : float
        运动速度 (m/s)
    acc : float
        加速度 (m/s²)
    """
    client = rospy.ServiceProxy(name, MoveL)

    data = MoveLRequest()
    
    data.pose[0].position.x = pose[0]
    data.pose[0].position.y = pose[1]
    data.pose[0].position.z = pose[2]

    data.pose[0].orientation.x = pose[3]
    data.pose[0].orientation.y = pose[4]
    data.pose[0].orientation.z = pose[5]
    data.pose[0].orientation.w = pose[6]

    data.pose[1].position.x = pose[0+7]
    data.pose[1].position.y = pose[1+7]
    data.pose[1].position.z = pose[2+7]

    data.pose[1].orientation.x = pose[3+7]
    data.pose[1].orientation.y = pose[4+7]
    data.pose[1].orientation.z = pose[5+7]
    data.pose[1].orientation.w = pose[6+7]

    data.v = v
    data.acc = acc

    resp:MoveLResponse = client.call(data)
    print(f"Service Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}")

```

## `movel_by_path/right_arm`

**Description:**
```tex
笛卡尔空间下,右臂轨迹点路径运动 (按总时间)
```

**ROS Type:** `Service`

**Data Type:** `upperlimb/MoveLByPath`

**Version:** 
- 1.0.0: added

**Cli:** ` `

**Example:** 
```python
def test_service_right_movel_by_path1(pose_path, time, *, name=f"{PREFIX}/movel_by_path/right_arm"):
    """
    笛卡尔空间下,右臂轨迹点路径运动 (按总时间)

    Parameters
    ----------
    pose_path : List[List[float]]
        位姿轨迹点路径
    time : float
        总时间

    Version
    -------
    - 1.0.0: added
    """
    client = rospy.ServiceProxy(name, MoveLByPath)

    data = MoveLByPathRequest()

    for index, tcp in enumerate(pose_path):
        _pose = GeoPose()
        _pose.position = Point(tcp[0], tcp[1], tcp[2])
        _pose.orientation = Quaternion(tcp[3], tcp[4], tcp[5], tcp[6])
        data.right_arm_path.append(_pose)
        if index > 600:
            break

    data.time = time

    movej = rospy.ServiceProxy(f"{PREFIX}/movej/right_arm/", MoveJ)
    movej_data = MoveJRequest()
    movej_data.joints = right_joints[0]
    movej_data.t = 5

    movej.call(movej_data)

    resp:MoveLByPathResponse = client.call(data)
    print(f"Service Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}")

```

## `movel_by_path/left_arm`

**Description:**
```tex
笛卡尔空间下,左臂轨迹点路径运动 (按总时间)
```

**ROS Type:** `Service`

**Data Type:** `upperlimb/MoveLByPath`

**Version:** 
- 1.0.0: added

**Cli:** ` `

**Example:** 
```python
def test_service_left_movel_by_path1(pose_path, time, *, name=f"{PREFIX}/movel_by_path/left_arm"):
    """
    笛卡尔空间下,左臂轨迹点路径运动 (按总时间)

    Parameters
    ----------
    pose_path : List[List[float]]
        位姿轨迹点路径
    time : float
        总时间

    Version
    -------
    - 1.0.0: added
    """
    client = rospy.ServiceProxy(name, MoveLByPath)

    data = MoveLByPathRequest()


    for index, tcp in enumerate(pose_path):
        _pose = GeoPose()
        _pose.position = Point(tcp[0], tcp[1], tcp[2])
        _pose.orientation = Quaternion(tcp[3], tcp[4], tcp[5], tcp[6])
        data.left_arm_path.append(_pose)
        if index > 600:
            break

    data.time = time

    movej = rospy.ServiceProxy(f"{PREFIX}/movej/left_arm/", MoveJ)
    movej_data = MoveJRequest()
    movej_data.joints = left_joints[0]
    movej_data.t = 5

    movej.call(movej_data)

    resp:MoveLByPathResponse = client.call(data)
    print(f"Service Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}")

```

## `movel_by_path/dual_arm`

**Description:**
```tex
笛卡尔空间下,双臂轨迹点路径运动 (按总时间)
```

**ROS Type:** `Service`

**Data Type:** `upperlimb/MoveLByPath`

**Version:** 
- 1.0.0: added

**Cli:** ` `

**Example:** 
```python
def test_service_dual_movel_by_path1(pose_path, time, *, name=f"{PREFIX}/movel_by_path/dual_arm"):
    """
    笛卡尔空间下,双臂轨迹点路径运动 (按总时间)

    Parameters
    ----------
    pose_path : List[List[float]]
        位姿轨迹点路径
    time : float
        总时间

    Version
    -------
    - 1.0.0: added
    """
    client = rospy.ServiceProxy(name, MoveLByPath)

    data = MoveLByPathRequest()


    for index, tcp in enumerate(pose_path):
        _pose = GeoPose()
        _pose.position = Point(tcp[0], tcp[1], tcp[2])
        _pose.orientation = Quaternion(tcp[3], tcp[4], tcp[5], tcp[6])
        data.right_arm_path.append(_pose)
        _pose = GeoPose()       # 防止出现浅拷贝
        _pose.position = Point(tcp[7], tcp[8], tcp[9])
        _pose.orientation = Quaternion(tcp[10], tcp[11], tcp[12], tcp[13])
        data.left_arm_path.append(_pose)
        if index >= 599:
            break
    data.time = time

    movej = rospy.ServiceProxy(f"{PREFIX}/movej/dual_arm/", MoveJ)
    movej_data = MoveJRequest()
    movej_data.joints = left_joints[0] + right_joints[0]
    movej_data.t = 5

    movej.call(movej_data)

    resp:MoveLByPathResponse = client.call(data)
    print(f"Service Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}")

```

## `servoj/left_arm`

**Description:**
```tex
关节空间下,左臂高频位置控制接口
```

**ROS Type:** `Topic/Subscribe`

**Data Type:** `upperlimb/Joints`

**Version:** 
- 1.0.0: added

**Cli:** ` `

**Example:** 
```python
def test_pub_servoj_left_arm(*, name=f"{PREFIX}/servoj/left_arm"):
    """
    关节空间下,左臂高频位置控制接口

    Note
    ----
    不要使用定时sleep,因为前后逻辑的执行本身也需要时间,该接口执行需要准确的时间戳会达到更好的效果

    Version
    -------
    - 1.0.0: added
    """
    servoj = rospy.Publisher(name, Joints, queue_size=1)

    csv = CSVReader("tests/dual_arm.csv")
    joints = csv.getColumnData([i for i in range(8,15)])


    movej = rospy.ServiceProxy(f"{PREFIX}/movej/left_arm/", MoveJ)
    movej_data = MoveJRequest()
    movej_data.joints = joints[0] 
    movej_data.t = 5
    movej.call(movej_data)

    tt = 0.02
    rospy.ServiceProxy(f"{PREFIX}/set_servo_params", Servo).call(ServoRequest(time=tt, gain=800)) 

    for index, joint in enumerate(joints):
        print(time.time(), index, joint)
        servoj.publish(Joints(joint))
        if index > 1000:
            break
        time.sleep(tt)

    resp = rospy.ServiceProxy(f"{PREFIX}/clear_servo_params", Servo).call(ServoRequest())
    print(f"Topic Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}")

```

## `servoj/right_arm`

**Description:**
```tex
关节空间下,右臂高频位置控制接口
```

**ROS Type:** `Topic/Subscribe`

**Data Type:** `upperlimb/Joints`

**Version:** 
- 1.0.0: added

**Cli:** ` `

**Example:** 
```python
def test_pub_servoj_right_arm(*, name=f"{PREFIX}/servoj/right_arm"):
    """
    关节空间下,右臂高频位置控制接口

    Note
    ----
    不要使用定时sleep,因为前后逻辑的执行本身也需要时间,该接口执行需要准确的时间戳会达到更好的效果

    Version
    -------
    - 1.0.0: added
    """
    servoj = rospy.Publisher(name, Joints, queue_size=1)

    csv = CSVReader("tests/dual_arm.csv")
    joints = csv.getColumnData([i for i in range(1,8)])


    movej = rospy.ServiceProxy(f"{PREFIX}/movej/right_arm/", MoveJ)
    movej_data = MoveJRequest()
    movej_data.joints = joints[0] 
    movej_data.t = 5
    movej.call(movej_data)

    tt = 0.02
    rospy.ServiceProxy(f"{PREFIX}/set_servo_params", Servo).call(ServoRequest(time=tt, gain=800)) 

    for index, joint in enumerate(joints):
        print(time.time(), index, joint)
        servoj.publish(Joints(joint))
        if index > 1000:
            break
        time.sleep(tt)

    resp = rospy.ServiceProxy(f"{PREFIX}/clear_servo_params", Servo).call(ServoRequest())
    print(f"Topic Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}")

```

## `servoj/dual_arm`

**Description:**
```tex
关节空间下,双臂高频位置控制接口
```

**ROS Type:** `Topic/Subscribe`

**Data Type:** `upperlimb/Joints`

**Version:** 
- 1.0.0: added

**Cli:** ` `

**Example:** 
```python
def test_pub_servoj_dual_arm(*, name=f"{PREFIX}/servoj/dual_arm"):
    """
    关节空间下,双臂高频位置控制接口

    Note
    ----
    不要使用定时sleep,因为前后逻辑的执行本身也需要时间,该接口执行需要准确的时间戳会达到更好的效果

    Version
    -------
    - 1.0.0: added
    """
    servoj = rospy.Publisher(name, Joints, queue_size=1)

    csv = CSVReader("tests/dual_arm.csv")
    right_joints = csv.getColumnData([i for i in range(1,8)])
    left_joints = csv.getColumnData([i for i in range(8,15)])

    joints = np.hstack((np.asarray(left_joints),np.asarray(right_joints))).tolist()

    movej = rospy.ServiceProxy(f"{PREFIX}/movej/dual_arm/", MoveJ)
    movej_data = MoveJRequest()
    movej_data.joints = joints[0]
    movej_data.t = 5
    movej.call(movej_data)

    tt = 0.02
    rospy.ServiceProxy(f"{PREFIX}/set_servo_params", Servo).call(ServoRequest(time=tt, gain=800)) 


    for index, joint in enumerate(joints):
        print(time.time(), index, joint)
        servoj.publish(Joints(joint))
        if index > 600:
            break
        time.sleep(tt)


    resp = rospy.ServiceProxy(f"{PREFIX}/clear_servo_params", Servo).call(ServoRequest())
    print(f"Topic Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}")

```

## `servoj/neck`

**Description:**
```tex
关节空间下,颈部高频位置控制接口
```

**ROS Type:** `Topic/Subscribe`

**Data Type:** `upperlimb/Joints`

**Version:** 
 

**Cli:** ` `

**Example:** 
```python
def test_pub_servoj_neck(*, name=f"{PREFIX}/servoj/neck"):
    """
    关节空间下,颈部高频位置控制接口

    Note
    ----
    不要使用定时sleep,因为前后逻辑的执行本身也需要时间,该接口执行需要准确的时间戳会达到更好的效果
    """
    servoj = rospy.Publisher(name, Joints, queue_size=1)


    movej = rospy.ServiceProxy(f"{PREFIX}/movej/neck/", MoveJ)
    movej_data = MoveJRequest()
    movej_data.joints = [0, 0]
    movej_data.t = 5
    movej.call(movej_data)


    tt = 0.02
    rospy.ServiceProxy(f"{PREFIX}/set_servo_params", Servo).call(ServoRequest(time=tt, gain=800)) 


    
    for i in range(600):
        angle = np.sin(i/100)*0.1
        print(time.time(), i, [angle])
        servoj.publish(Joints([angle]))
        time.sleep(tt)
    
    resp = rospy.ServiceProxy(f"{PREFIX}/clear_servo_params", Servo).call(ServoRequest())
    print(f"Topic Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}")

```

## `set_servo_params`

**Description:**
```tex
设置伺服参数
```

**ROS Type:** `Topic/Subscribe`

**Data Type:** `upperlimb/Servo`

**Version:** 
 

**Cli:** ` `

**Example:** 
```python
def test_set_servo_params(*, name=f"{PREFIX}/set_servo_params"):
    """
    设置伺服参数
    """
    t = 0.02
    gain = 800
    resp = rospy.ServiceProxy(f"{PREFIX}/set_servo_params", Servo).call(ServoRequest(time=t, gain=gain))
    print(f"Service Name:{name} | Set time:{t}, gain:{gain} | Resp Success:{resp.success}, Resp Message:{resp.message}")

```

## `clear_servo_params`

**Description:**
```tex
清除伺服参数
```

**ROS Type:** `Topic/Subscribe`

**Data Type:** `upperlimb/Servo`

**Version:** 
 

**Cli:** ` `

**Example:** 
```python
def test_clear_servo_params(*, name=f"{PREFIX}/clear_servo_params"):
    """
    清除伺服参数
    """
    resp = rospy.ServiceProxy(f"{PREFIX}/clear_servo_params", Servo).call(ServoRequest())
    print(f"Service Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}")

```

## `servoj/waist`

**Description:**
```tex
关节空间下,腰部高频位置控制接口
```

**ROS Type:** `Topic/Publish`

**Data Type:** `upperlimb/Joints`

**Version:** 
 

**Cli:** ` `

**Example:** 
```python
def test_pub_servoj_waist(*, name=f"{PREFIX}/servoj/waist"):
    """
    关节空间下,腰部高频位置控制接口


    Note
    ----
    不要使用定时sleep,因为前后逻辑的执行本身也需要时间,该接口执行需要准确的时间戳会达到更好的效果

    """
    servoj = rospy.Publisher(name, Joints, queue_size=1)


    movej = rospy.ServiceProxy(f"{PREFIX}/movej/waist/", MoveJ)
    movej_data = MoveJRequest()
    movej_data.joints = [0]
    movej_data.t = 5
    movej.call(movej_data)


    tt = 0.02
    rospy.ServiceProxy(f"{PREFIX}/set_servo_params", Servo).call(ServoRequest(time=tt, gain=800)) 


    
    for i in range(600):
        angle = np.sin(i/100)*0.1
        print(time.time(), i, [angle])
        servoj.publish(Joints([angle]))
        time.sleep(tt)
    
    resp = rospy.ServiceProxy(f"{PREFIX}/clear_servo_params", Servo).call(ServoRequest())

```

## `servol/left_arm`

**Description:**
```tex
笛卡尔空间下,左臂高频位置控制
```

**ROS Type:** `Topic/Subscribe`

**Data Type:** `upperlimb/DualPose`

**Version:** 
- 1.0.0: added

**Cli:** ` `

**Example:** 
```python
def test_pub_servol_left_arm(*, name=f"{PREFIX}/servol/left_arm"):
    """
    笛卡尔空间下,左臂高频位置控制

    Note
    ----
    不要使用定时sleep,因为前后逻辑的执行本身也需要时间,该接口执行需要准确的时间戳会达到更好的效果

    Version
    -------
    - 1.0.0: added
    """
    servol = rospy.Publisher(name, DualPose, queue_size=1)

    csv = CSVReader("tests/dual_arm.csv")
    joints = csv.getColumnData([i for i in range(8,15)])
    csv_tcp = CSVReader("tests/dual_tcp.csv")
    tcps = csv_tcp.getColumnData([i for i in range(8,15)])
   
    movej = rospy.ServiceProxy(f"{PREFIX}/movej/left_arm/", MoveJ)
    movej_data = MoveJRequest()
    movej_data.joints = joints[0]
    movej_data.t = 5
    movej.call(movej_data)

    tt = 0.02
    rospy.ServiceProxy(f"{PREFIX}/set_servo_params", Servo).call(ServoRequest(time=tt, gain=800)) 


    for index, tcp in enumerate(tcps):
        print(time.time(), index, tcp)
        _pose = DualPose()
        _pose.left_arm_pose.position = Point(tcp[0], tcp[1], tcp[2])
        _pose.left_arm_pose.orientation = Quaternion(tcp[3], tcp[4], tcp[5], tcp[6])
        servol.publish(_pose)
        if index > 600:
            break
        time.sleep(tt)

    resp = rospy.ServiceProxy(f"{PREFIX}/clear_servo_params", Servo).call(ServoRequest())
    print(f"Topic Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}")

```

## `servol/right_arm`

**Description:**
```tex
笛卡尔空间下,右臂高频位置控制
```

**ROS Type:** `Topic/Subscribe`

**Data Type:** `upperlimb/DualPose`

**Version:** 
- 1.0.0: added

**Cli:** ` `

**Example:** 
```python
def test_pub_servol_right_arm(*, name=f"{PREFIX}/servol/right_arm"):
    """
    笛卡尔空间下,右臂高频位置控制

    Note
    ----
    不要使用定时sleep,因为前后逻辑的执行本身也需要时间,该接口执行需要准确的时间戳会达到更好的效果

    Version
    -------
    - 1.0.0: added
    """
    servol = rospy.Publisher(name, DualPose, queue_size=1)

    csv = CSVReader("tests/dual_arm.csv")
    joints = csv.getColumnData([i for i in range(1,8)])
    csv_tcp = CSVReader("tests/dual_tcp.csv")
    tcps = csv_tcp.getColumnData([i for i in range(1,8)])
   
    movej = rospy.ServiceProxy(f"{PREFIX}/movej/right_arm/", MoveJ)
    movej_data = MoveJRequest()
    movej_data.joints = joints[0]
    movej_data.t = 5
    movej.call(movej_data)

    tt = 0.02
    rospy.ServiceProxy(f"{PREFIX}/set_servo_params", Servo).call(ServoRequest(time=tt, gain=800)) 

    for index, tcp in enumerate(tcps):
        print(time.time(), index, tcp)
        _pose = DualPose()
        _pose.right_arm_pose.position = Point(tcp[0], tcp[1], tcp[2])
        _pose.right_arm_pose.orientation = Quaternion(tcp[3], tcp[4], tcp[5], tcp[6])
        servol.publish(_pose)
        if index > 600:
            break
        time.sleep(tt)

    resp = rospy.ServiceProxy(f"{PREFIX}/clear_servo_params", Servo).call(ServoRequest())
    print(f"Topic Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}")

```

## `servol/dual_arm`

**Description:**
```tex
笛卡尔空间下,双臂高频位置控制
```

**ROS Type:** `Topic/Subscribe`

**Data Type:** `upperlimb/DualPose`

**Version:** 
- 1.0.0: added

**Cli:** ` `

**Example:** 
```python
def test_pub_servol_dual_arm(*, name=f"{PREFIX}/servol/dual_arm"):
    """
    笛卡尔空间下,双臂高频位置控制

    Note
    ----
    不要使用定时sleep,因为前后逻辑的执行本身也需要时间,该接口执行需要准确的时间戳会达到更好的效果

    Version
    -------
    - 1.0.0: added
    """
    servol = rospy.Publisher(name, DualPose, queue_size=1)

    csv = CSVReader("tests/dual_arm.csv")
    right_joints = csv.getColumnData([i for i in range(1,8)])
    left_joints = csv.getColumnData([i for i in range(8,15)])
    csv_tcp = CSVReader("tests/dual_tcp.csv")
    right_tcps = csv_tcp.getColumnData([i for i in range(1,8)])
    left_tcps = csv_tcp.getColumnData([i for i in range(8,15)])
   

    movej = rospy.ServiceProxy(f"{PREFIX}/movej/dual_arm/", MoveJ)
    movej_data = MoveJRequest()
    movej_data.joints = left_joints[0] + right_joints[0]
    movej_data.t = 5
    movej.call(movej_data)

    tt = 0.02
    rospy.ServiceProxy(f"{PREFIX}/set_servo_params", Servo).call(ServoRequest(time=tt, gain=800)) 

    for index, tcp in enumerate(left_tcps):
        print(time.time(), index, tcp)
        _pose = DualPose()
        _pose.left_arm_pose.position = Point(left_tcps[index][0], left_tcps[index][1], left_tcps[index][2])
        _pose.left_arm_pose.orientation = Quaternion(left_tcps[index][3], left_tcps[index][4], left_tcps[index][5], left_tcps[index][6])
        _pose.right_arm_pose.position = Point(right_tcps[index][0], right_tcps[index][1], right_tcps[index][2])
        _pose.right_arm_pose.orientation = Quaternion(right_tcps[index][3], right_tcps[index][4], right_tcps[index][5], right_tcps[index][6])
        servol.publish(_pose)
        if index > 600:
            break
        time.sleep(tt)


    resp = rospy.ServiceProxy(f"{PREFIX}/clear_servo_params", Servo).call(ServoRequest())
    print(f"Topic Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}")

```

## `enable_speedj`

**Description:**
```tex
启用/禁用关节速度控制模式
```

**ROS Type:** `Topic/Subscribe`

**Data Type:** `std_srvs/SetBool`

**Version:** 
- 1.0.0: added

**Cli:** ` `

**Example:** 
```python
def test_enable_speedj(*, name=f"{PREFIX}/enable_speedj"):
    """
    启用/禁用关节速度控制模式

    Version
    -------
    - 1.0.0: added
    """
    client = rospy.ServiceProxy(name, SetBool)

    # 开启speedj模式
    resp:SetBoolResponse = client.call(SetBoolRequest(True))
    print(f"Service Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}")

    count = 0
    while 1:
        pass


    # 关闭speedj模式
    resp:SetBoolResponse = client.call(SetBoolRequest(False))
    print(f"Service Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}")

```

## `speedj/left_arm`

**Description:**
```tex
左臂关节速度控制
```

**ROS Type:** `Topic/Subscribe`

**Data Type:** `upperlimb/SpeedJ`

**Version:** 
- 1.0.0: added

**Cli:** ` `

**Example:** 
```python
def test_pub_speedj_left_arm(*, name=f"{PREFIX}/speedj/left_arm"):
    """
    左臂关节速度控制

    Version
    -------
    - 1.0.0: added
    """
    enable_client = rospy.ServiceProxy(f"{PREFIX}/enable_speedj", SetBool)
    # 开启speedj
    resp = enable_client.call(SetBoolRequest(True))

    pub = rospy.Publisher(name, SpeedJ, queue_size=1)
    rate = rospy.Rate(0.5)  # 设置发布频率为1Hz

    target_speed = 0.01
    counter = 0
    while 1:
        data = SpeedJ()
        data.joint_speed = [0, 0, target_speed, 0, 0, 0, 0]
        data.acc = 0.5
        target_speed += 0.02
        print(target_speed)

        counter += 1
        if counter > 10:
            break

        pub.publish(data)
        rate.sleep()

    # 关闭speedj
    resp = enable_client.call(SetBoolRequest(False))

```

## `speedj/right_arm`

**Description:**
```tex
右臂关节速度控制
```

**ROS Type:** `Topic/Subscribe`

**Data Type:** `upperlimb/SpeedJ`

**Version:** 
- 1.0.0: added

**Cli:** ` `

**Example:** 
```python
def test_pub_speedj_right_arm(*, name=f"{PREFIX}/speedj/right_arm"):
    """
    右臂关节速度控制

    Version
    -------
    - 1.0.0: added
    """
    enable_client = rospy.ServiceProxy(f"{PREFIX}/enable_speedj", SetBool)
    # 开启speedj
    resp = enable_client.call(SetBoolRequest(True))


    pub = rospy.Publisher(name, SpeedJ, queue_size=1)
    rate = rospy.Rate(5)  # 设置发布频率为1Hz

    target_speed = 0.01
    counter = 0
    while 1:
        data = SpeedJ()
        data.joint_speed = [0, 0, target_speed, 0, 0, 0, 0]
        data.acc = 0.5
        target_speed += 0.02
        print(target_speed)

        counter += 1
        if counter > 10:
            break

        pub.publish(data)
        rate.sleep()

    # 关闭speedj
    resp = enable_client.call(SetBoolRequest(False))

```

## `speedj/dual_arm`

**Description:**
```tex
双臂关节速度控制
```

**ROS Type:** `Topic/Subscribe`

**Data Type:** `upperlimb/SpeedJ`

**Version:** 
- 1.0.0: added

**Cli:** ` `

**Example:** 
```python
def test_pub_speedj_dual_arm(*, name=f"{PREFIX}/speedj/dual_arm"):
    """
    双臂关节速度控制

    Version
    -------
    - 1.0.0: added
    """
    enable_client = rospy.ServiceProxy(f"{PREFIX}/enable_speedj", SetBool)
    # 开启speedj
    resp = enable_client.call(SetBoolRequest(True))


    pub = rospy.Publisher(name, SpeedJ, queue_size=1)
    rate = rospy.Rate(5)  # 设置发布频率为1Hz

    target_speed = 0.01
    counter = 0
    while 1:
        data = SpeedJ()
        data.joint_speed = [0, 0, target_speed, 0, 0, 0, 0,
                            0, 0, target_speed, 0, 0, 0, 0]
        data.acc = 0.5
        target_speed += 0.02
        print(target_speed)

        counter += 1
        if counter > 10:
            break

        pub.publish(data)
        rate.sleep()

    # 关闭speedj
    resp = enable_client.call(SetBoolRequest(False))

```

## `speedl/left_arm`

**Description:**
```tex
左臂笛卡尔空间速度控制
```

**ROS Type:** `Topic/Subscribe`

**Data Type:** `upperlimb/SpeedL`

**Version:** 
- 1.0.0: added

**Cli:** ` `

**Example:** 
```python
def test_pub_speedl_left_arm(*, name=f"{PREFIX}/speedl/left_arm"):
    """
    左臂笛卡尔空间速度控制
    
    Version
    -------
    - 1.0.0: added
    """
    enable_client = rospy.ServiceProxy(f"{PREFIX}/enable_speedl", SetBool)
    # 开启speedl
    resp = enable_client.call(SetBoolRequest(True))


    pub = rospy.Publisher(name, SpeedL, queue_size=1)
    rate = rospy.Rate(5)

    target_speed = 0.01
    counter = 0
    while 1:
        data = SpeedL()
        data.tcp_speed = [0, 0, target_speed, 0, 0, 0]
        data.acc = 0.5
        target_speed += 0.002
        print(target_speed)

        counter += 1
        if counter > 20:
            break

        pub.publish(data)
        print(time.time())
        rate.sleep()

    # 关闭speedl
    resp = enable_client.call(SetBoolRequest(False))

```

## `speedl/right_arm`

**Description:**
```tex
右臂笛卡尔空间速度控制
```

**ROS Type:** `Topic/Subscribe`

**Data Type:** `upperlimb/SpeedL`

**Version:** 
- 1.0.0: added

**Cli:** ` `

**Example:** 
```python
def test_pub_speedl_right_arm(*, name=f"{PREFIX}/speedl/right_arm"):
    """
    右臂笛卡尔空间速度控制
    
    Version
    -------
    - 1.0.0: added
    """
    enable_client = rospy.ServiceProxy(f"{PREFIX}/enable_speedl", SetBool)
    # 开启speedl
    resp = enable_client.call(SetBoolRequest(True))

    pub = rospy.Publisher(name, SpeedL, queue_size=1)
    rate = rospy.Rate(5)

    target_speed = 0.01
    counter = 0
    while 1:
        data = SpeedL()
        data.tcp_speed = [0, 0, target_speed, 0, 0, 0]
        data.acc = 0.5
        target_speed += 0.002
        print(target_speed)

        counter += 1
        if counter > 20:
            break

        pub.publish(data)
        rate.sleep()

    # 关闭speedl
    resp = enable_client.call(SetBoolRequest(False))

```

## `speedl/dual_arm`

**Description:**
```tex
双臂笛卡尔空间速度控制
```

**ROS Type:** `Topic/Subscribe`

**Data Type:** `upperlimb/SpeedL`

**Version:** 
- 1.0.0: added

**Cli:** ` `

**Example:** 
```python
def test_pub_speedl_dual_arm(*, name=f"{PREFIX}/speedl/dual_arm"):
    """
    双臂笛卡尔空间速度控制
    
    Version
    -------
    - 1.0.0: added
    """
    enable_client = rospy.ServiceProxy(f"{PREFIX}/enable_speedl", SetBool)
    # 开启speedl
    resp = enable_client.call(SetBoolRequest(True))


    pub = rospy.Publisher(name, SpeedL, queue_size=1)
    rate = rospy.Rate(5)

    target_speed = 0.01
    counter = 0
    while 1:
        data = SpeedL()
        data.tcp_speed = [0, 0, target_speed, 0, 0, 0] * 2
        data.acc = 0.5
        target_speed += 0.002
        print(target_speed)

        counter += 1
        if counter > 20:
            break

        pub.publish(data)
        rate.sleep()

    # 关闭speedl
    resp = enable_client.call(SetBoolRequest(False))

```

## `enable_speedl`

**Description:**
```tex
启用/禁用笛卡尔速度控制模式
```

**ROS Type:** `Service`

**Data Type:** `std_srvs/SetBool`

**Version:** 
- 1.0.0: added

**Cli:** ` `

**Example:** 
```python
def test_enable_speedl(*, name=f"{PREFIX}/enable_speedl"):
    """
    启用/禁用笛卡尔速度控制模式
    
    Version
    -------
    - 1.0.0: added
    """
    enable_client = rospy.ServiceProxy(name, SetBool)
    # 开启speedl
    resp = enable_client.call(SetBoolRequest(True))
    print(f"Service Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}")

    time.sleep(2)

    resp = enable_client.call(SetBoolRequest(False))
    print(f"Service Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}")

```

## `stop`

**Description:**
```tex
停止机器人运动
```

**ROS Type:** `Service`

**Data Type:** `std_srvs/Trigger`

**Version:** 
- 1.0.0: added

**Cli:** ` `

**Example:** 
```python
def test_stop(*, name=f"{PREFIX}/stop"):
    """
    停止机器人运动
    
    Version
    -------
    - 1.0.0: added
    """
    client = rospy.ServiceProxy(name, Trigger)
    resp:TriggerResponse = client.call(TriggerRequest())
    print(f"Service Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}")

```

## `joint_states`

**Description:**
```tex
订阅关节状态(位置、速度、力矩)
```

**ROS Type:** `Topic/Publish`

**Data Type:** `sensor_msgs/JointState`

**Version:** 
- 1.0.0: added

**Cli:** ` rostopic echo /zj_humanoid/upperlimb/joint_states`

**Example:** 
```python
def test_sub_joint_states(*, name=f"{PREFIX}/joint_states"):
    """
    订阅关节状态(位置、速度、力矩)

    Notes
    -----
    该话题发布机器人当前的关节状态信息，包括关节名称、位置、速度和力矩。

    Cli Examples
    ------------
    $ rostopic echo /zj_humanoid/upperlimb/joint_states
    
    Version
    -------
    - 1.0.0: added
    """
    print(f"\n>>> CLI命令: rostopic echo {name}")
    
    def callback(msg):
        print(f"\n[{name}] Received JointState:")
        print(f"  Names: {msg.name}")
        print(f"  Position: {msg.position}")
        print(f"  Velocity: {msg.velocity}")
        print(f"  Effort: {msg.effort}")
    
    sub = rospy.Subscriber(name, JointState, callback)
    rospy.sleep(2)  # 等待接收消息

```

## `uplimb_state`

**Description:**
```tex
订阅上肢机器人命令状态
```

**ROS Type:** `Topic/Publish`

**Data Type:** `upperlimb/UplimbState`

**Version:** 
- 1.0.0: added

**Cli:** ` rostopic echo /zj_humanoid/upperlimb/uplimb_state`

**Example:** 
```python
def test_sub_uplimb_state(*, name=f"{PREFIX}/uplimb_state"):
    """
    订阅上肢机器人命令状态

    Notes
    -----
    该话题发布上肢机器人的当前命令状态信息。

    Cli Examples
    ------------
    $ rostopic echo /zj_humanoid/upperlimb/uplimb_state
    
    Version
    -------
    - 1.0.0: added
    """
    print(f"\n>>> CLI命令: rostopic echo {name}")
    
    def callback(msg):
        print(f"\n[{name}] Received UplimbState:")
        print(f"  State: {msg}")
    
    sub = rospy.Subscriber(name, UplimbState, callback)
    rospy.sleep(2)  # 等待接收消息

```

## `tcp_speed/dual_arm`

**Description:**
```tex
订阅双臂TCP速度
```

**ROS Type:** `Topic/Publish`

**Data Type:** `upperlimb/TcpSpeed`

**Version:** 
- 1.0.0: added

**Cli:** ` rostopic echo /zj_humanoid/upperlimb/tcp_speed/dual_arm`

**Example:** 
```python
def test_sub_tcp_speed_dual_arm(*, name=f"{PREFIX}/tcp_speed/dual_arm"):
    """
    订阅双臂TCP速度

    Notes
    -----
    该话题发布双臂末端执行器(TCP)的实时速度信息。

    Cli Examples
    ------------
    $ rostopic echo /zj_humanoid/upperlimb/tcp_speed/dual_arm
    
    Version
    -------
    - 1.0.0: added
    """
    print(f"\n>>> CLI命令: rostopic echo {name}")
    
    def callback(msg):
        print(f"\n[{name}] Received TcpSpeed:")
        print(f"  Speed: {msg}")
    
    sub = rospy.Subscriber(name, TcpSpeed, callback)
    rospy.sleep(2)  # 等待接收消息

```

## `tcp_pose/left_arm`

**Description:**
```tex
订阅左臂TCP位姿
```

**ROS Type:** `Topic/Publish`

**Data Type:** `upperlimb/Pose`

**Version:** 
 

**Cli:** ` rostopic echo /zj_humanoid/upperlimb/tcp_pose/left_arm`

**Example:** 
```python
def test_sub_tcp_pose_left_arm(*, name=f"{PREFIX}/tcp_pose/left_arm"):
    """
    订阅左臂TCP位姿

    Notes
    -----
    该话题发布左臂末端执行器(TCP)的实时位姿信息，包括位置(x,y,z)和姿态(四元数)。

    Cli Examples
    ------------
    $ rostopic echo /zj_humanoid/upperlimb/tcp_pose/left_arm
    """
    print(f"\n>>> CLI命令: rostopic echo {name}")
    
    def callback(msg):
        print(f"\n[{name}] Received Pose:")
        print(f"  Position: [{msg.position.x}, {msg.position.y}, {msg.position.z}]")
        print(f"  Orientation: [{msg.orientation.x}, {msg.orientation.y}, {msg.orientation.z}, {msg.orientation.w}]")
    
    sub = rospy.Subscriber(name, Pose, callback)
    rospy.sleep(2)  # 等待接收消息

```

## `tcp_pose/right_arm`

**Description:**
```tex
订阅右臂TCP位姿
```

**ROS Type:** `Topic/Publish`

**Data Type:** `upperlimb/Pose`

**Version:** 
 

**Cli:** ` rostopic echo /zj_humanoid/upperlimb/tcp_pose/right_arm`

**Example:** 
```python
def test_sub_tcp_pose_right_arm(*, name=f"{PREFIX}/tcp_pose/right_arm"):
    """
    订阅右臂TCP位姿

    Notes
    -----
    该话题发布右臂末端执行器(TCP)的实时位姿信息，包括位置(x,y,z)和姿态(四元数)。

    Cli Examples
    ------------
    $ rostopic echo /zj_humanoid/upperlimb/tcp_pose/right_arm
    """
    print(f"\n>>> CLI命令: rostopic echo {name}")
    
    def callback(msg):
        print(f"\n[{name}] Received Pose:")
        print(f"  Position: [{msg.position.x}, {msg.position.y}, {msg.position.z}]")
        print(f"  Orientation: [{msg.orientation.x}, {msg.orientation.y}, {msg.orientation.z}, {msg.orientation.w}]")
    
    sub = rospy.Subscriber(name, Pose, callback)
    rospy.sleep(2)  # 等待接收消息

```

## `occupancy_state`

**Description:**
```tex
订阅上肢占用状态
```

**ROS Type:** `Topic/Publish`

**Data Type:** `std_msgs/Int8`

**Version:** 
 

**Cli:** ` rostopic echo /zj_humanoid/upperlimb/occupancy_state`

**Example:** 
```python
def test_sub_occupancy_state(*, name=f"{PREFIX}/occupancy_state"):
    """
    订阅上肢占用状态

    Notes
    -----
    该话题发布上肢的当前占用状态，用于防止多个控制源同时控制机器人。

    Cli Examples
    ------------
    $ rostopic echo /zj_humanoid/upperlimb/occupancy_state
    """
    print(f"\n>>> CLI命令: rostopic echo {name}")
    
    def callback(msg):
        print(f"\n[{name}] Received Int8:")
        print(f"  Data: {msg.data}")
    
    sub = rospy.Subscriber(name, Int8, callback)
    rospy.sleep(2)  # 等待接收消息

```

## `teach_mode/enter`

**Description:**
```tex
进入示教模式
```

**ROS Type:** `Service`

**Data Type:** `upperlimb/ArmType`

**Version:** 
- 1.0.0: added

**Cli:** ` `

**Example:** 
```python
def test_teach_mode(arm_type:int, *,name=f"{PREFIX}/teach_mode/enter"):
    """
    进入示教模式

    Parameters
    ----------
    arm_type : int
        机械臂类型标识
        
    Version
    -------
    - 1.0.0: added
    """
    client = rospy.ServiceProxy(name, ArmType)
    # 左臂进入示教模式
    resp:TriggerResponse = client.call(ArmTypeRequest(arm_type=arm_type))
    print(f"Service Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}")

```

## `teach_mode/exit`

**Description:**
```tex
退出示教模式
```

**ROS Type:** `Service`

**Data Type:** `upperlimb/ArmType`

**Version:** 
- 1.0.0: added

**Cli:** ` `

**Example:** 
```python
def test_end_teach_mode(arm_type:int, *,name=f"{PREFIX}/teach_mode/exit"):
    """
    退出示教模式

    Parameters
    ----------
    arm_type : int
        机械臂类型标识
        
    Version
    -------
    - 1.0.0: added
    """
    client = rospy.ServiceProxy(name, ArmType)
    # 左臂推出示教模式
    resp:TriggerResponse = client.call(ArmTypeRequest(arm_type=arm_type))
    print(f"Service Name:{name} | Resp Success:{resp.success}, Resp Message:{resp.message}")

```

## `uplimb_occupation`

**Description:**
```tex
发布上肢占用状态
```

**ROS Type:** `Topic/Publish`

**Data Type:** `std_msgs/Int8`

**Version:** 
- 1.0.0: added

**Cli:** ` `

**Example:** 
```python
def test_pub_uplimb_occupation(*, name=f"{PREFIX}/uplimb_occupation"):
    """
    发布上肢占用状态
    
    Version
    -------
    - 1.0.0: added
    """
    # pass
    print()

```


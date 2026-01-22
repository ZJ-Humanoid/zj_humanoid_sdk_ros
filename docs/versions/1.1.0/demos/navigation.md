## **task_info**

**Description**

```tex
通过 /zj_humanoid/navigation/task_info 话题向导航服务端发布异步控制指令，包括目标位置和姿态信息，适用于自主规划导航的场景。机器人开始规划并移动，状态从 STANDBY 切换成 PLANNING
```

**ROS Type:** `Topic`

**Data Type:** `navigation/TaskInfo.msg`

**Version:**

- 1.0.0 : added

**CLI：**

```Go
timeout 3 rostopic pub -r 10 /zj_humanoid/navigation/task_info navigation/TaskInfo "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
waypoints:
- id: 1
  pose:
    position:
      x: 2.71
      y: 11.42
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: -0.82
      w: 0.568
  action: 0
  audio: 0"
```

**Example:**

```Python
#!/usr/bin/env python3
import rospy
import yaml
import os
import sys
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion
from navigation.msg import TaskInfo

def load_task_from_yaml(yaml_path):
    """
    从 YAML 文件加载任务配置
    :param yaml_path: 命令行传入的 YAML 文件路径
    :return: 解析后的字典数据
    """
    if not os.path.exists(yaml_path):
        rospy.logerr(f"错误：YAML 文件不存在 -> {yaml_path}")
        return None
        
    try:
        with open(yaml_path, 'r', encoding='utf-8') as f:
            yaml_data = yaml.safe_load(f)
        rospy.loginfo(f"成功加载 YAML 配置文件：{yaml_path}")
        return yaml_data
    except yaml.YAMLError as e:
        rospy.logerr(f"YAML 文件解析失败：{e}")
        return None
    except Exception as e:
        rospy.logerr(f"读取 YAML 文件失败：{e}")
        return None

def print_usage():
    """打印脚本使用说明（参数错误时调用）"""
    usage = """
    用法：rosrun <你的功能包名> publish_task_info_from_args.py <YAML文件路径>
    示例：
      1. 相对路径（YAML与脚本同目录）：
         rosrun navigation publish_task_info_from_args.py task_config.yaml
      2. 绝对路径（指定完整目录）：
         rosrun navigation publish_task_info_from_args.py /home/naviai/config/task_config.yaml
    说明：
      - 传入的 YAML 文件需包含 header 和 waypoints 字段（格式见README）
      - 脚本会读取 YAML 中的路径点信息，以 10Hz 频率发布 3 次 TaskInfo 消息
    """
    print(usage)

def publish_task_info():
    if len(sys.argv) != 2:
        rospy.logerr("参数错误：缺少 YAML 文件路径！")
        print_usage()
        return
        
    yaml_file_path = sys.argv[1]
    rospy.init_node('task_info_publisher_from_args', anonymous=True)
    yaml_data = load_task_from_yaml(yaml_file_path)
    if not yaml_data:
        rospy.logerr("任务配置加载失败，节点退出")
        return

    pub = rospy.Publisher('/zj_humanoid/navigation/task_info', TaskInfo, queue_size=10)
    rate = rospy.Rate(10) 
    seq = 1
    publish_count = 0
    max_count = 3 
    
    rospy.loginfo(f"开始发布 /task_info 话题（10Hz，共发布 {max_count} 次）...")
    
    while publish_count < max_count and not rospy.is_shutdown():
        task_msg = TaskInfo()

        yaml_header = yaml_data.get('header', {})
        task_msg.header = Header()
        task_msg.header.seq = seq
        task_msg.header.stamp = rospy.Time.now()
        task_msg.header.frame_id = yaml_header.get('frame_id', '')

        yaml_waypoints = yaml_data.get('waypoints', [])
        task_waypoints = []
        
        for yaml_wp in yaml_waypoints:
            waypoint = TaskInfo.Waypoint()
            waypoint.id = yaml_wp.get('id', 0)
            yaml_pose = yaml_wp.get('pose', {})
            yaml_position = yaml_pose.get('position', {})
            waypoint.pose.position = Point(
                x=yaml_position.get('x', 0.0),
                y=yaml_position.get('y', 0.0),
                z=yaml_position.get('z', 0.0)
            )
            
            yaml_orientation = yaml_pose.get('orientation', {})
            waypoint.pose.orientation = Quaternion(
                x=yaml_orientation.get('x', 0.0),
                y=yaml_orientation.get('y', 0.0),
                z=yaml_orientation.get('z', 0.0),
                w=yaml_orientation.get('w', 1.0)
            )
            
            waypoint.action = yaml_wp.get('action', 0)
            waypoint.audio = yaml_wp.get('audio', 0)
            
            task_waypoints.append(waypoint)
        
        task_msg.waypoints = task_waypoints   
        pub.publish(task_msg)
        wp_ids = [wp.id for wp in task_waypoints]
        first_wp_pos = task_waypoints[0].pose.position if task_waypoints else Point()
        rospy.loginfo(f"第 {publish_count+1}/{max_count} 次发布：seq={seq}，路径点ID={wp_ids}，位置=({first_wp_pos.x:.2f}, {first_wp_pos.y:.2f})")
        seq += 1
        publish_count += 1
        rate.sleep()
    
    rospy.loginfo(f"已完成 {max_count} 次发布，节点退出！")

if __name__ == '__main__':
    try:
        publish_task_info()
    except rospy.ROSInterruptException:
        rospy.logwarn("TaskInfo 发布节点已被中断")
    except ImportError as e:
        rospy.logerr(f"导入消息类型失败：{e}")
        rospy.logerr("请确认 navigation 功能包已编译，且包含 TaskInfo.msg 定义")
```

## **odom_info**

**Description**

```tex
通过 /zj_humanoid/navigation/odom_info 话题向定位服务端订阅机器人在世界坐标系下的位姿，用于实时跟踪机器人的运动。包括机器人当前的三轴位置姿态、三轴速度角速度信息。
```

**ROS Type:** `Topic`

**Data Type:** `/zj_humanoid/navigation/odom_info.msg`

**Version:**

- 1.0.0 : added

**CLI:** 

```Go
rostopic echo /zj_humanoid/navigation/odom_info 
```

**Example:**

```Python
#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist

def odom_callback(msg):
    """
    话题回调函数：接收 Odometry 消息并解析
    :param msg: nav_msgs/Odometry 消息对象
    """
    header = msg.header
    rospy.loginfo(f"\n=== 里程计数据更新 ===")
    rospy.loginfo(f"时间戳：{header.stamp.secs}.{header.stamp.nsecs:09d}")
    rospy.loginfo(f"坐标帧：{header.frame_id} -> 子坐标帧：{msg.child_frame_id}")
    rospy.loginfo(f"消息序列号：{header.seq}")

    position: Point = msg.pose.pose.position
    rospy.loginfo(f"\n位置信息：")
    rospy.loginfo(f"  X 坐标：{position.x:.3f} m")
    rospy.loginfo(f"  Y 坐标：{position.y:.3f} m")
    rospy.loginfo(f"  Z 坐标：{position.z:.3f} m")

    orientation: Quaternion = msg.pose.pose.orientation
    rospy.loginfo(f"\n姿态信息（四元数）：")
    rospy.loginfo(f"  x: {orientation.x:.4f}")
    rospy.loginfo(f"  y: {orientation.y:.4f}")
    rospy.loginfo(f"  z: {orientation.z:.4f}")
    rospy.loginfo(f"  w: {orientation.w:.4f}")

    linear: Twist = msg.twist.twist.linear
    angular: Twist = msg.twist.twist.angular
    rospy.loginfo(f"\n速度信息：")
    rospy.loginfo(f"  线速度（x/y/z）：{linear.x:.3f} / {linear.y:.3f} / {linear.z:.3f} m/s")
    rospy.loginfo(f"  角速度（x/y/z）：{angular.x:.3f} / {angular.y:.3f} / {angular.z:.3f} rad/s")

    rospy.loginfo(f"\n定位协方差（前3项为 x/y/z 轴定位精度）：")
    rospy.loginfo(f"  x 轴方差：{msg.pose.covariance[0]:.6f}")
    rospy.loginfo(f"  y 轴方差：{msg.pose.covariance[7]:.6f}")
    rospy.loginfo(f"  z 轴方差：{msg.pose.covariance[14]:.6f}")
    rospy.loginfo(f"=====================")

def subscribe_odom_info():
    rospy.init_node('odom_info_subscriber', anonymous=True)
    rospy.Subscriber(
        topic='/zj_humanoid/navigation/odom_info',
        data_class=Odometry,
        callback=odom_callback,
        queue_size=10
    ) 
    rospy.loginfo("已启动里程计订阅节点，正在监听 /zj_humanoid/navigation/odom_info ...")
    rospy.spin()

if __name__ == '__main__':
    try:
        subscribe_odom_info()
    except rospy.ROSInterruptException:
        rospy.logwarn("里程计订阅节点已停止")
```

## **local_map**

**Description**

```tex
通过 /zj_humanoid/navigation/local_map 话题向定位服务端订阅机器人周围局部障碍物信息，包括障碍物的位置、类型和速度等，用于规划安全的轨迹。
```

**ROS Type:** `Topic`

**Data Type:** `/zj_humanoid/navigation/local_map.msg`

**Version:**

- 1.0.0 : added

**CLI:** 

```Go
rostopic echo /zj_humanoid/navigation/local_map
```

**Example:**

```Python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import Header
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
from navigation.msg import LocalMap, LocalMapData

def local_map_callback(msg):
    """
    话题回调函数：接收 LocalMap 消息并解析
    :param msg: navigation/LocalMap 消息对象
    """
    header = msg.header
    rospy.loginfo(f"\n=== 局部地图数据更新 ===")
    rospy.loginfo(f"时间戳：{header.stamp.secs}.{header.stamp.nsecs:09d}")
    rospy.loginfo(f"坐标帧：{header.frame_id}")
    rospy.loginfo(f"消息序列号：{header.seq}")

    map_info: MapMetaData = msg.info
    rospy.loginfo(f"\n地图元数据：")
    rospy.loginfo(f"  地图加载时间：{map_info.map_load_time.secs}.{map_info.map_load_time.nsecs:09d}")
    rospy.loginfo(f"  地图分辨率：{map_info.resolution:.3f} m/像素")
    rospy.loginfo(f"  地图尺寸：宽度={map_info.width} 像素，高度={map_info.height} 像素")
    rospy.loginfo(f"  地图实际尺寸：宽度={map_info.width * map_info.resolution:.3f} m，高度={map_info.height * map_info.resolution:.3f} m")

    origin: Pose = map_info.origin
    origin_pos: Point = origin.position
    origin_orient: Quaternion = origin.orientation
    rospy.loginfo(f"  地图原点（左下角）坐标：")
    rospy.loginfo(f"    x: {origin_pos.x:.3f} m, y: {origin_pos.y:.3f} m, z: {origin_pos.z:.3f} m")
    rospy.loginfo(f"  地图旋转角度（四元数）：")
    rospy.loginfo(f"    x: {origin_orient.x:.4f}, y: {origin_orient.y:.4f}, z: {origin_orient.z:.4f}, w: {origin_orient.w:.4f}")

    map_data: list[LocalMapData] = msg.data
    rospy.loginfo(f"\n局部地图数据：")
    rospy.loginfo(f"  数据点总数：{len(map_data)} 个")
    
    print_limit = min(5, len(map_data))
    for i in range(print_limit):
        data_point = map_data[i]
        rospy.loginfo(f"  数据点 {i+1}：")
        rospy.loginfo(f"    占用状态（occupancy）：{'占用' if data_point.occupancy else '空闲'}")
        rospy.loginfo(f"    语义类型（semantic）：{data_point.semantic}（0=未知，1=障碍物，2=可通行，3=动态物体...）")
        rospy.loginfo(f"    动态标记（dynamic）：{'动态' if data_point.dynamic else '静态'}")
        rospy.loginfo(f"    速度（speed）：{data_point.speed:.3f} m/s")
        rospy.loginfo(f"    方向（direction）：{data_point.direction:.3f} rad（{math.degrees(data_point.direction):.2f}°）")
    
    if len(map_data) > print_limit:
        rospy.loginfo(f"  ... 省略剩余 {len(map_data) - print_limit} 个数据点")

    rospy.loginfo(f"=====================")

def subscribe_local_map():
    rospy.init_node('local_map_subscriber', anonymous=True)
    
    rospy.Subscriber(
        topic='/zj_humanoid/navigation/local_map',  
        data_class=LocalMap,                        
        callback=local_map_callback,           
        queue_size=5                                
    )
    
    rospy.loginfo("已启动局部地图订阅节点，正在监听 /zj_humanoid/navigation/local_map ...")
    rospy.spin()

if __name__ == '__main__':
    import math 
    try:
        subscribe_local_map()
    except rospy.ROSInterruptException:
        rospy.logwarn("局部地图订阅节点已停止")
    except ImportError as e:
        rospy.logerr(f"导入消息类型失败：{e}")
        rospy.logerr("请确认 navigation 功能包已编译，且包含 LocalMap.msg 和 LocalMapData.msg 定义")
```

## **map**

**Description**

```tex
通过 /map 话题向导航服务端订阅全局地图信息，表示机器人周围的静态环境信息，信息来源于事先扫描建立的地图图片，使用时由map server发布。
```

**ROS Type:** `Topic`

**Data Type:** `sensor_msgs/Image.msg`

**Version:**

- 1.0.0 : added

**CLI:** 

```Go
rostopic echo /zj_humanoid/navigation/map
```

**Example:**

```Python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion

def map_callback(msg):
    """
    话题回调函数：接收 OccupancyGrid 地图消息并解析
    :param msg: nav_msgs/OccupancyGrid 消息对象
    """
    header = msg.header
    rospy.loginfo(f"\n=== 全局地图数据更新 ===")
    rospy.loginfo(f"时间戳：{header.stamp.secs}.{header.stamp.nsecs:09d}")
    rospy.loginfo(f"坐标帧：{header.frame_id}")
    rospy.loginfo(f"消息序列号：{header.seq}")

    map_info: MapMetaData = msg.info
    rospy.loginfo(f"\n地图元数据：")
    rospy.loginfo(f"  地图加载时间：{map_info.map_load_time.secs}.{map_info.map_load_time.nsecs:09d}")
    rospy.loginfo(f"  地图分辨率：{map_info.resolution:.3f} m/像素")
    rospy.loginfo(f"  地图像素尺寸：宽度={map_info.width} 像素，高度={map_info.height} 像素")
    rospy.loginfo(f"  地图实际尺寸：宽度={map_info.width * map_info.resolution:.3f} m，高度={map_info.height * map_info.resolution:.3f} m")

    origin: Pose = map_info.origin
    origin_pos: Point = origin.position
    origin_orient: Quaternion = origin.orientation
    rospy.loginfo(f"  地图原点（左下角）坐标：")
    rospy.loginfo(f"    x: {origin_pos.x:.3f} m, y: {origin_pos.y:.3f} m, z: {origin_pos.z:.3f} m")
    rospy.loginfo(f"  地图旋转角度（四元数）：")
    rospy.loginfo(f"    x: {origin_orient.x:.4f}, y: {origin_orient.y:.4f}, z: {origin_orient.z:.4f}, w: {origin_orient.w:.4f}")

    grid_data: list[int] = msg.data
    total_cells = len(grid_data)
    rospy.loginfo(f"\n栅格地图数据：")
    rospy.loginfo(f"  总栅格数：{total_cells} 个（与 {map_info.width}×{map_info.height} 一致）")
    
    unknown_count = grid_data.count(-1)
    free_count = sum(1 for cell in grid_data if cell == 0)
    occupied_count = sum(1 for cell in grid_data if 1 <= cell <= 100)
    rospy.loginfo(f"  栅格状态统计：")
    rospy.loginfo(f"    未知栅格：{unknown_count} 个（{unknown_count/total_cells*100:.1f}%）")
    rospy.loginfo(f"    空闲栅格：{free_count} 个（{free_count/total_cells*100:.1f}%）")
    rospy.loginfo(f"    占用栅格：{occupied_count} 个（{occupied_count/total_cells*100:.1f}%）")

    key_positions = [
        ("左上角", 0),
        ("右上角", map_info.width - 1),
        ("左下角", (map_info.height - 1) * map_info.width),
        ("右下角", map_info.width * map_info.height - 1),
        ("中心", (map_info.height // 2) * map_info.width + (map_info.width // 2))
    ]
    rospy.loginfo(f"\n关键位置栅格状态：")
    for pos_name, idx in key_positions:
        if 0 <= idx < total_cells:
            cell_val = grid_data[idx]
            if cell_val == -1:
                status = "未知"
            elif cell_val == 0:
                status = "空闲"
            else:
                status = f"占用（概率：{cell_val}%）"
            rospy.loginfo(f"  {pos_name}（索引 {idx}）：{status}")

    rospy.loginfo(f"=====================")

def subscribe_global_map():
    rospy.init_node('global_map_subscriber', anonymous=True)
    
    rospy.Subscriber(
        topic='/zj_humanoid/navigation/map',  
        data_class=OccupancyGrid,             
        callback=map_callback,                
        queue_size=2                          
    )
    
    rospy.loginfo("已启动全局地图订阅节点，正在监听 /zj_humanoid/navigation/map ...")
    rospy.spin()

if __name__ == '__main__':
    try:
        subscribe_global_map()
    except rospy.ROSInterruptException:
        rospy.logwarn("全局地图订阅节点已停止")
    except ImportError as e:
        rospy.logerr(f"导入消息类型失败：{e}")
        rospy.logerr("请确认已安装 nav_msgs 功能包：sudo apt-get install ros-noetic-nav-msgs")
```

## **navigation_status**

**Description**

```tex
通过 /zj_humanoid/navigation/navigation_status 话题向导航服务端订阅当前导航状态，适用于需要通过导航状态进行对应的业务逻辑开发等场景。
```

**ROS Type:** `Topic`

**Data Type:** `/zj_humanoid/navigation/navigation_status.msg`

**Version:**

- 1.0.0 : added

**CLI:** 

```Go
rostopic echo /zj_humanoid/navigation/navigation_status
```

**Example:**

```Python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import Header
from navigation.msg import navigation_status, PlanState

def navigation_status_callback(msg):
    """
    话题回调函数：接收 navigation_status 消息并解析
    :param msg: navigation/navigation_status 消息对象
    """
    header = msg.header
    rospy.loginfo(f"\n=== 导航状态更新 ===")
    rospy.loginfo(f"时间戳：{header.stamp.secs}.{header.stamp.nsecs:09d}")
    rospy.loginfo(f"坐标帧：{header.frame_id}")
    rospy.loginfo(f"消息序列号：{header.seq}")

    plan_state: PlanState = msg.state
    state_value = plan_state.value
    
    state_map = {
        PlanState.NONE: "NONE（没有定位）",
        PlanState.STANDBY: "STANDBY（任务空闲）",
        PlanState.PLANNING: "PLANNING（规划中）",
        PlanState.RUNNING: "RUNNING（运行中）",
        PlanState.STOPPING: "STOPPING（规划停止）",
        PlanState.FINISHED: "FINISHED（规划完成）",
        PlanState.FAILURE: "FAILURE（规划失败）"
    }
    
    state_desc = state_map.get(state_value, f"未知状态（值：{state_value}）")
    
    rospy.loginfo(f"\n导航状态信息：")
    rospy.loginfo(f"  状态值：{state_value}")
    rospy.loginfo(f"  状态描述：{state_desc}")

    rospy.loginfo(f"=====================")

def subscribe_navigation_status():
    rospy.init_node('navigation_status_subscriber', anonymous=True)
    
    rospy.Subscriber(
        topic='/zj_humanoid/navigation/navigation_status',  
        data_class=navigation_status,                       
        callback=navigation_status_callback,               
        queue_size=10                                      
    )
    
    rospy.loginfo("已启动导航状态订阅节点，正在监听 /zj_humanoid/navigation/navigation_status ...")
    rospy.spin()

if __name__ == '__main__':
    try:
        subscribe_navigation_status()
    except rospy.ROSInterruptException:
        rospy.logwarn("导航状态订阅节点已停止")
    except ImportError as e:
        rospy.logerr(f"导入消息类型失败：{e}")
        rospy.logerr("请确认 navigation 功能包已编译，且包含 navigation_status.msg 和 PlanState.msg 定义")
```

## **version**

**Description**

```tex
通过 /zj_humanoid/navigation/version 话题向导航服务端获取算法版本信息。
```

**ROS Type:** `Service`

**Data Type:** `std_srvs/Trigger.srv`

**Version:**

- 1.0.0 : added

**CLI:** 

```Go
rosservice call /zj_humanoid/navigation/version
```

**Example:**

```Python
#!/usr/bin/env python3
import rospy
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

def call_version_service():
    rospy.init_node('call_version_service_node', anonymous=True)
    service_name = '/zj_humanoid/navigation/version'
    rospy.loginfo(f"等待服务 {service_name} 启动...")
    try:
        rospy.wait_for_service(service_name, timeout=10)
    except rospy.ROSException:
        rospy.logerr(f"超时错误：服务 {service_name} 未启动，10 秒内未连接成功")
        return
    
    rospy.loginfo(f"服务 {service_name} 已就绪，开始发送请求...")
    
    try:
        version_client = rospy.ServiceProxy(service_name, Trigger)
        request = TriggerRequest()
        response: TriggerResponse = version_client.call(request)

        rospy.loginfo(f"\n=== 导航版本服务响应 ===")
        rospy.loginfo(f"服务调用结果：{'成功' if response.success else '失败'}")
        rospy.loginfo(f"版本信息/描述：{response.message}")
        rospy.loginfo(f"=====================")
        
    except rospy.ServiceException as e:
        rospy.logerr(f"服务调用失败：{e}")
    except Exception as e:
        rospy.logerr(f"未知错误：{e}")

if __name__ == '__main__':
    try:
        call_version_service()
    except rospy.ROSInterruptException:
        rospy.logwarn("服务调用节点已被中断")
    except ImportError as e:
        rospy.logerr(f"导入消息类型失败：{e}")
        rospy.logerr("请确认已安装 std_srvs 功能包：sudo apt-get install ros-noetic-std-srvs")
```
## realsense_up/color/image_raw

**Description**

```tex
发布机器人胸口处深度相机的原始彩色图像
```
**ROS Type:** `Topic`

**Data Type:** `sensor_msgs/Image.msg`

**Version:**

- 1.0.0 : added

**CLI:** 

```shell
rostopic echo -n 1 /zj_humanoid/sensor/realsense_up/color/image_raw
```

**Example:**

```Python
import rospy
from sensor_msgs.msg import Image

def color_image_callback(msg):
    # 处理图像数据的逻辑，例如：
    # 1. 使用 cv_bridge 转换为 OpenCV 格式
    # 2. 打印图像头信息：msg.header.stamp
    rospy.loginfo("Received color image with sequence ID: %d", msg.header.seq)

if __name__ == '__main__':
    rospy.init_node('color_image_subscriber', anonymous=True)
    rospy.Subscriber('/zj_humanoid/sensor/realsense_up/color/image_raw', Image, color_image_callback)
    rospy.spin()
```

## realsense_up/color/camera_info

**Description**

```tex
发布机器人胸口处深度相机彩色图像的内参信息，如图像长宽、畸变模型、内参矩阵K等
```

**ROS Type:** `Topic`

**Data Type:** `sensor_msgs/CameraInfo.msg`

**Version:**

- 1.0.0 : added

**CLI：**

```Plain
rostopic echo -n 1 /zj_humanoid/sensor/realsense_up/color/camera_info
```

## realsense_up/depth/image_rect_raw

**Description**

```tex
发布机器人胸口处深度相机的原始或经过校正的深度图像
```

**ROS Type:** `Topic`

**Data Type:** `sensor_msgs/Image.msg`

**Version:**

- 1.0.0 : added

**CLI：**

```Bash
rostopic echo -n 1 /zj_humanoid/sensor/realsense_up/depth/image_rect_raw
```

**Example：**

```Python
import rospy
from sensor_msgs.msg import Image

def depth_image_callback(msg):
    # 深度图处理逻辑，例如：
    # 1. 检查编码格式（encoding）以确定数据类型，如 '16UC1'
    # 2. 从图像中心点读取深度值
    rospy.loginfo("Received depth image with encoding: %s", msg.encoding)

if __name__ == '__main__':
    rospy.init_node('depth_image_subscriber', anonymous=True)
    rospy.Subscriber('/zj_humanoid/sensor/realsense_up/depth/image_rect_raw', Image, depth_image_callback)
    rospy.spin()
```

## realsense_up/depth/camera_info

**Description**

```tex
发布机器人胸口处深度相机深度图像的内参信息
```

**ROS Type:** `Topic`

**Data Type:** `sensor_msgs/CameraInfo.msg`

**Version:**

- 1.0.0 : added

**CLI：**

```Bash
rostopic echo -n 1 /zj_humanoid/sensor/realsense_up/depth/camera_info
```

## realsense_up/aligned_depth_to_color/image_raw

**Description**

```tex
发布已对齐（Registered）的深度图像，该图像被重投影到彩色图像的坐标系和分辨率上
```

**ROS Type:** `Topic`

**Data Type:** `sensor_msgs/Image.msg`

**Version:**

- 1.0.0 : added

**CLI：**

```Bash
rostopic echo -n 1 /zj_humanoid/sensor/realsense_up/aligned_depth_to_color/image_raw
```

## realsense_up/aligned_depth_to_color/camera_info

**Description**

```tex
发布已对齐深度图所使用的相机内参信息
```

**ROS Type:** `Topic`

**Data Type:** `sensor_msgs/CameraInfo.msg`

**Version:**

- 1.0.0 : added

**CLI：**

```Bash
rostopic echo -n 1 /zj_humanoid/sensor/realsense_up/aligned_depth_to_color/camera_info
```

## CAM_A/camera_info

**Description**

```tex

```

**ROS Type:** `Service`

**Data Type:** `std_srvs/Trigger.srv`

**Version:**

- 1.0.0 : added

**CLI：**

```Plain
rosservice call /zj_humanoid/sensor/CAM_A/camera_info "{}" 
```

## CAM_A/image_raw

**Description**

```tex
发布来自OAK相机A的原始（未压缩）**图像数据。这是进行主要计算机视觉处理（如物体检测、SLAM、特征提取）的数据源
```

**ROS Type:** `Topic`

**Data Type:** `sensor_msgs/Image.msg`

**Version:**

- 1.0.0 : added

**CLI：**

```Plain
rostopic echo -n 1 /zj_humanoid/sensor/CAM_A/image_raw
```

## CAM_A/compressed

**Description**

```tex
发布来自OAK相机A的压缩后的图像数据（通常为JPEG或PNG格式）。此话题用于低带宽场景，如网络传输（例如到Rviz或Web界面）或高效的rosbag日志记录
```

**ROS Type:** `Topic`

**Data Type:** `sensor_msgs/CompressedImage.msg`

**Version:**

- 1.0.0 : added

**CLI：**

```Plain
rostopic echo -n 1 /zj_humanoid/sensor/CAM_A/compressed
```

## CAM_B/camera_info

**Description**

```tex
发布OAK相机B的标定参数。此话题包含相机的内参（K矩阵）、畸变系数（D矩阵）、旋转（R）和投影（P）矩阵。这些信息对于图像去畸变和3D重建至关重要
```

**ROS Type:** `Topic`

**Data Type:** `std_srvs/Trigger.srv`

**Version:**

- 1.0.0 : added

## CAM_B/image_raw

**Description**

```tex
发布来自OAK相机B的原始（未压缩）图像数据
```

**ROS Type:** `Topic`

**Data Type:** `sensor_msgs/Image.msg`

**Version:**

- 1.0.0 : added

**CLI：**

```Plain
rostopic echo -n 1 /zj_humanoid/sensor/CAM_B/image_raw
```

## CAM_B/compressed

**Description**

```tex
发布来自OAK相机B的压缩后的图像数据
```

**ROS Type:** `Topic`

**Data Type:** `sensor_msgs/CompressedImage.msg`

**Version:**

- 1.0.0 : added

**CLI：**

```Plain
rostopic echo -n 1 /zj_humanoid/sensor/CAM_B/compressed
```

## CAM_C/camera_info

**Description**

```tex
发布OAK相机C的标定参数。此话题包含相机的内参（K矩阵）、畸变系数（D矩阵）、旋转（R）和投影（P）矩阵。这些信息对于图像去畸变和3D重建至关重要
```

**ROS Type:** `Service`

**Data Type:** `std_srvs/Trigger.srv`

**Version:**

- 1.0.0 : added

## CAM_C/image_raw

**Description**

```tex
发布来自OAK相机C的原始（未压缩）图像数据
```

**ROS Type:** `Service`

**Data Type:** `sensor_msgs/Image.msg`

**Version:**

- 1.0.0 : added

**CLI：**

```Plain
rostopic echo -n 1 /zj_humanoid/sensor/CAM_C/image_raw
```

## CAM_C/compressed

**Description**

```tex
发布来自OAK相机C的压缩后的图像数据
```

**ROS Type:** `Topic`

**Data Type:** `sensor_msgs/CompressedImage.msg`

**Version:**

- 1.0.0 : added

## CAM_D/camera_info

**Description**

```tex
发布OAK相机D的标定参数。此话题包含相机的内参（K矩阵）、畸变系数（D矩阵）、旋转（R）和投影（P）矩阵。这些信息对于图像去畸变和3D重建至关重要
```

**ROS Type:** `Service`

**Data Type:** `std_srvs/Trigger.srv`

**Version:**

- 1.0.0 : added

## CAM_D/image_raw

**Description**

```tex
发布来自OAK相机D的原始（未压缩）图像数据
```

**ROS Type:** `Topic`

**Data Type:** `sensor_msgs/Image.msg`

**Version:**

- 1.0.0 : added

**CLI：**

```Plain
rostopic echo -n 1 /zj_humanoid/sensor/CAM_D/image_raw
```

## CAM_D/compressed

**Description**

```tex
发布来自OAK相机D的压缩后的图像数据
```

**ROS Type:** `Topic`

**Data Type:** `sensor_msgs/CompressedImage.msg`

**Version:**

- 1.0.0 : added

**CLI：**

```Plain
rostopic echo -n 1 /zj_humanoid/sensor/CAM_D/compressed
```

## head_imu

**Description**

```tex
发布来自机器人头部IMU（惯性测量单元）的传感器数据。这通常包括三轴的角速度、三轴的线加速度
```

**ROS Type:** `Topic`

**Data Type:** `sensor_msgs/Imu.msg`

**Version:**

- 1.0.0 : added

- 数据类型：

**CLI：**

```Plain
rostopic echo -n 1 /zj_humanoid/sensor/head_imu
```


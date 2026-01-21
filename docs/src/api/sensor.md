---
title: SENSOR 子系统
description: SENSOR 子系统的所有ROS接口
---

## 📊 接口概览

<Markmap :content="markmapContent" />

<script setup>
const markmapContent = `---
markmap:
  initialExpandLevel: 3
  colorFreezeLevel: 3
  maxWidth: 200
---

# 📷 SENSOR 子系统
## 📦 Services (4)
- CAM_A
  - camera_info
- CAM_B
  - camera_info
- CAM_C
  - camera_info
- CAM_D
  - camera_info
## 📡 Topics (36)
- head_imu
- CAM_A
  - compressed
  - image_raw
- CAM_B
  - compressed
  - image_raw
- CAM_C
  - compressed
  - image_raw
- CAM_D
  - compressed
  - image_raw
- realsense_down
  - camera_info
  - image_raw
  - compressed
  - camera_info
  - image_raw
  - compressed
  - camera_info
  - image_rect_raw
  - compressed
- realsense_head
  - camera_info
  - image_raw
  - compressed
  - camera_info
  - image_raw
  - compressed
  - camera_info
  - image_rect_raw
  - compressed
- realsense_up
  - camera_info
  - image_raw
  - compressed
  - camera_info
  - image_raw
  - compressed
  - camera_info
  - image_rect_raw
  - compressed`
</script>

---

## 📦 Services (4)

### 1. `CAM_A/camera_info`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/sensor/CAM_A/camera_info |
| **Type** | [sensor/CameraInfo](../zj_humanoid_types#camerainfo) |
| **Description** | 左眼参数信息 |
| **Note** | 相机A的参数信息,相机A安装在机器人左眼的位置上，相机A的分辨率是多少 回复应包含1280和720 |

### 2. `CAM_B/camera_info`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/sensor/CAM_B/camera_info |
| **Type** | [sensor/CameraInfo](../zj_humanoid_types#camerainfo) |
| **Description** | 右眼参数信息 |
| **Note** | 相机B的参数信息，相机B安装在机器人右眼的位置上，相机B的分辨率是多少 回复应包含1280和720 |

### 3. `CAM_C/camera_info`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/sensor/CAM_C/camera_info |
| **Type** | [sensor/CameraInfo](../zj_humanoid_types#camerainfo) |
| **Description** | 相机C参数信息 |
| **Note** | 相机C的参数信息，相机C大致安装在机器人右侧太阳穴的位置上，相机C的分辨率是多少 回复应包含1280和720 |

### 4. `CAM_D/camera_info`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/sensor/CAM_D/camera_info |
| **Type** | [sensor/CameraInfo](../zj_humanoid_types#camerainfo) |
| **Description** | 相机D参数信息 |
| **Note** | 相机D的参数信息，相机D大致安装在机器人左侧太阳穴的位置上，相机D的分辨率是多少 回复应包含1280和720 |

## 📡 Topics (36)

### 1. `CAM_A/compressed`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/sensor/CAM_A/compressed |
| **Type** | sensor_msgs/CompressedImage |
| **Direction** | 📤 Publish |
| **Description** | 左眼相机JPG |
| **Note** | 左眼相机的JPG图像数据 |

### 2. `CAM_A/image_raw`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/sensor/CAM_A/image_raw |
| **Type** | sensor_msgs/Image |
| **Direction** | 📤 Publish |
| **Description** | 左眼相机RGB |
| **Note** | 左眼相机的RGB图像源数据 |

### 3. `CAM_B/compressed`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/sensor/CAM_B/compressed |
| **Type** | sensor_msgs/CompressedImage |
| **Direction** | 📤 Publish |
| **Description** | 右眼相机JPG |
| **Note** | 右眼相机的JPG图像数据 |

### 4. `CAM_B/image_raw`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/sensor/CAM_B/image_raw |
| **Type** | sensor_msgs/Image |
| **Direction** | 📤 Publish |
| **Description** | 右眼相机RGB |
| **Note** | 右眼相机的RGB图像源数据 |

### 5. `CAM_C/compressed`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/sensor/CAM_C/compressed |
| **Type** | sensor_msgs/CompressedImage |
| **Direction** | 📤 Publish |
| **Description** | 相机C的JPG |
| **Note** | 右侧太阳穴相机C的JPG图像数据 |

### 6. `CAM_C/image_raw`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/sensor/CAM_C/image_raw |
| **Type** | sensor_msgs/Image |
| **Direction** | 📤 Publish |
| **Description** | 相机C的RGB |
| **Note** | 右侧太阳穴相机C的RGB图像源数据 |

### 7. `CAM_D/compressed`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/sensor/CAM_D/compressed |
| **Type** | sensor_msgs/CompressedImage |
| **Direction** | 📤 Publish |
| **Description** | 相机D的JPG |
| **Note** | 左侧太阳穴相机D的JPG图像数据 |

### 8. `CAM_D/image_raw`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/sensor/CAM_D/image_raw |
| **Type** | sensor_msgs/Image |
| **Direction** | 📤 Publish |
| **Description** | 相机D的RGB |
| **Note** | 左侧太阳穴相机D的RGB图像源数据 |

### 9. `head_imu`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/sensor/head_imu |
| **Type** | sensor_msgs/Imu |
| **Direction** | 📤 Publish |
| **Description** | 头部IMU数据 |
| **Note** | 头部IMU的目前帧率是多少 回复应接近100 |

### 10. `realsense_down/aligned_depth_to_color/camera_info`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/sensor/realsense_down/aligned_depth_to_color/camera_info |
| **Type** | sensor_msgs/CameraInfo |
| **Direction** | 📤 Publish |
| **Description** | 腹部深度aligned参数 |
| **Note** | 腹部深度相机的aligned_depth_to_color参数信息 |

### 11. `realsense_down/aligned_depth_to_color/image_raw`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/sensor/realsense_down/aligned_depth_to_color/image_raw |
| **Type** | sensor_msgs/Image |
| **Direction** | 📤 Publish |
| **Description** | 腹部深度aligned图像 |
| **Note** | 腹部深度相机的aligned_depth_to_color RGB图像源数据 |

### 12. `realsense_down/aligned_depth_to_color/image_raw/compressed`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/sensor/realsense_down/aligned_depth_to_color/image_raw/compressed |
| **Type** | sensor_msgs/Image |
| **Direction** | 📤 Publish |
| **Description** | 腹部深度aligned压缩图 |
| **Note** | 腹部深度相机的aligned_depth_to_color压缩格式 |

### 13. `realsense_down/color/camera_info`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/sensor/realsense_down/color/camera_info |
| **Type** | sensor_msgs/CameraInfo |
| **Direction** | 📤 Publish |
| **Description** | 腹部深度RGB参数 |
| **Note** | 腹部深度相机的参数信息 |

### 14. `realsense_down/color/image_raw`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/sensor/realsense_down/color/image_raw |
| **Type** | sensor_msgs/Image |
| **Direction** | 📤 Publish |
| **Description** | 腹部深度RGB图像 |
| **Note** | 腹部深度相机的RGB图像源数据 |

### 15. `realsense_down/color/image_raw/compressed`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/sensor/realsense_down/color/image_raw/compressed |
| **Type** | sensor_msgs/Image |
| **Direction** | 📤 Publish |
| **Description** | 腹部深度压缩图 |
| **Note** | 腹部深度相机的RGB图像JPG格式 |

### 16. `realsense_down/depth/camera_info`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/sensor/realsense_down/depth/camera_info |
| **Type** | sensor_msgs/CameraInfo |
| **Direction** | 📤 Publish |
| **Description** | 腹部相机深度参数 |
| **Note** | 腹部深度相机的参数信息 |

### 17. `realsense_down/depth/image_rect_raw`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/sensor/realsense_down/depth/image_rect_raw |
| **Type** | sensor_msgs/Image |
| **Direction** | 📤 Publish |
| **Description** | 腹部深度RGB图像 |
| **Note** | 腹部深度相机的RGB图像源数据 |

### 18. `realsense_down/depth/image_rect_raw/compressed`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/sensor/realsense_down/depth/image_rect_raw/compressed |
| **Type** | sensor_msgs/Image |
| **Direction** | 📤 Publish |
| **Description** | 腹部深度压缩图 |
| **Note** | 腹部深度相机的RGB图像JPG格式 |

### 19. `realsense_head/aligned_depth_to_color/camera_info`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/sensor/realsense_head/aligned_depth_to_color/camera_info |
| **Type** | sensor_msgs/CameraInfo |
| **Direction** | 📤 Publish |
| **Description** | 头部深度aligned参数 |
| **Note** | 头部深度相机的aligned_depth_to_color参数信息 |

### 20. `realsense_head/aligned_depth_to_color/image_raw`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/sensor/realsense_head/aligned_depth_to_color/image_raw |
| **Type** | sensor_msgs/Image |
| **Direction** | 📤 Publish |
| **Description** | 头部深度aligned图像 |
| **Note** | 头部深度相机的aligned_depth_to_color RGB图像源数据 |

### 21. `realsense_head/aligned_depth_to_color/image_raw/compressed`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/sensor/realsense_head/aligned_depth_to_color/image_raw/compressed |
| **Type** | sensor_msgs/Image |
| **Direction** | 📤 Publish |
| **Description** | 头部深度aligned压缩图 |
| **Note** | 头部深度相机的aligned_depth_to_color压缩格式 |

### 22. `realsense_head/color/camera_info`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/sensor/realsense_head/color/camera_info |
| **Type** | sensor_msgs/CameraInfo |
| **Direction** | 📤 Publish |
| **Description** | 头部深度RGB参数 |
| **Note** | 头部深度相机的参数信息 |

### 23. `realsense_head/color/image_raw`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/sensor/realsense_head/color/image_raw |
| **Type** | sensor_msgs/Image |
| **Direction** | 📤 Publish |
| **Description** | 头部深度RGB图像 |
| **Note** | 头部深度相机的RGB图像源数据 |

### 24. `realsense_head/color/image_raw/compressed`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/sensor/realsense_head/color/image_raw/compressed |
| **Type** | sensor_msgs/Image |
| **Direction** | 📤 Publish |
| **Description** | 头部深度压缩图 |
| **Note** | 头部深度相机的RGB图像JPG格式 |

### 25. `realsense_head/depth/camera_info`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/sensor/realsense_head/depth/camera_info |
| **Type** | sensor_msgs/CameraInfo |
| **Direction** | 📤 Publish |
| **Description** | 头部相机深度参数 |
| **Note** | 头部深度相机的参数信息 |

### 26. `realsense_head/depth/image_rect_raw`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/sensor/realsense_head/depth/image_rect_raw |
| **Type** | sensor_msgs/Image |
| **Direction** | 📤 Publish |
| **Description** | 头部深度RGB图像 |
| **Note** | 头部深度相机的深度图像源数据 |

### 27. `realsense_head/depth/image_rect_raw/compressed`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/sensor/realsense_head/depth/image_rect_raw/compressed |
| **Type** | sensor_msgs/Image |
| **Direction** | 📤 Publish |
| **Description** | 头部深度压缩图 |
| **Note** | 头部深度相机的深度图像JPG格式 |

### 28. `realsense_up/aligned_depth_to_color/camera_info`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/sensor/realsense_up/aligned_depth_to_color/camera_info |
| **Type** | sensor_msgs/CameraInfo |
| **Direction** | 📤 Publish |
| **Description** | 胸部深度aligned参数 |
| **Note** | 胸部深度相机的aligned_depth_to_color参数信息 |

### 29. `realsense_up/aligned_depth_to_color/image_raw`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/sensor/realsense_up/aligned_depth_to_color/image_raw |
| **Type** | sensor_msgs/Image |
| **Direction** | 📤 Publish |
| **Description** | 胸部深度aligned图像 |
| **Note** | 胸部深度相机的aligned_depth_to_color RGB图像源数据 |

### 30. `realsense_up/aligned_depth_to_color/image_raw/compressed`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/sensor/realsense_up/aligned_depth_to_color/image_raw/compressed |
| **Type** | sensor_msgs/Image |
| **Direction** | 📤 Publish |
| **Description** | 胸部深度aligned压缩图 |
| **Note** | 胸部深度相机的aligned_depth_to_color压缩格式 |

### 31. `realsense_up/color/camera_info`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/sensor/realsense_up/color/camera_info |
| **Type** | sensor_msgs/CameraInfo |
| **Direction** | 📤 Publish |
| **Description** | 胸部深度RGB参数 |
| **Note** | 胸部深度相机的参数信息 |

### 32. `realsense_up/color/image_raw`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/sensor/realsense_up/color/image_raw |
| **Type** | sensor_msgs/Image |
| **Direction** | 📤 Publish |
| **Description** | 胸部深度RGB图像 |
| **Note** | 胸部深度相机的RGB图像源数据 |

### 33. `realsense_up/color/image_raw/compressed`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/sensor/realsense_up/color/image_raw/compressed |
| **Type** | sensor_msgs/Image |
| **Direction** | 📤 Publish |
| **Description** | 胸部深度压缩图 |
| **Note** | 胸部深度相机的RGB图像JPG格式 |

### 34. `realsense_up/depth/camera_info`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/sensor/realsense_up/depth/camera_info |
| **Type** | sensor_msgs/CameraInfo |
| **Direction** | 📤 Publish |
| **Description** | 胸部相机深度参数 |
| **Note** | 胸部深度相机的参数信息 |

### 35. `realsense_up/depth/image_rect_raw`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/sensor/realsense_up/depth/image_rect_raw |
| **Type** | sensor_msgs/Image |
| **Direction** | 📤 Publish |
| **Description** | 胸部深度RGB图像 |
| **Note** | 胸部深度相机的RGB图像源数据 |

### 36. `realsense_up/depth/image_rect_raw/compressed`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/sensor/realsense_up/depth/image_rect_raw/compressed |
| **Type** | sensor_msgs/Image |
| **Direction** | 📤 Publish |
| **Description** | 胸部深度压缩图 |
| **Note** | 胸部深度相机的RGB图像JPG格式 |


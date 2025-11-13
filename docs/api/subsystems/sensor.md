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
### CAM_A
- camera_info
### CAM_B
- camera_info
### CAM_C
- camera_info
### CAM_D
- camera_info
## 📡 Topics (27)
- compressed
- image_raw
- compressed
- image_raw
- compressed
- image_raw
- compressed
- image_raw
- head_imu
- camera_info
- image_raw
- compressed
- camera_info
- image_raw
- compressed
- camera_info
- image_rect_raw
- compressed
- camera_info
- image_raw
- ... 还有 7 个话题`
</script>

---

## 📦 Services (4)

### 1. `/zj_humanoid/sensor/CAM_A/camera_info`

| 字段 | 值 |
|------|-----|
| **Service Name** | `/zj_humanoid/sensor/CAM_A/camera_info` |
| **Type** | `sensor/CameraInfo` |
| **Description** | 左眼参数信息 |
| **Note** | 相机A的参数信息,相机A安装在机器人左眼的位置上，相机A的分辨率是多少 回复应包含1280和720 |

### 2. `/zj_humanoid/sensor/CAM_B/camera_info`

| 字段 | 值 |
|------|-----|
| **Service Name** | `/zj_humanoid/sensor/CAM_B/camera_info` |
| **Type** | `sensor/CameraInfo` |
| **Description** | 右眼参数信息 |
| **Note** | 相机B的参数信息，相机B安装在机器人右眼的位置上，相机B的分辨率是多少 回复应包含1280和720 |

### 3. `/zj_humanoid/sensor/CAM_C/camera_info`

| 字段 | 值 |
|------|-----|
| **Service Name** | `/zj_humanoid/sensor/CAM_C/camera_info` |
| **Type** | `sensor/CameraInfo` |
| **Description** | 相机C参数信息 |
| **Note** | 相机C的参数信息，相机C大致安装在机器人右侧太阳穴的位置上，相机C的分辨率是多少 回复应包含1280和720 |

### 4. `/zj_humanoid/sensor/CAM_D/camera_info`

| 字段 | 值 |
|------|-----|
| **Service Name** | `/zj_humanoid/sensor/CAM_D/camera_info` |
| **Type** | `sensor/CameraInfo` |
| **Description** | 相机D参数信息 |
| **Note** | 相机D的参数信息，相机D大致安装在机器人左侧太阳穴的位置上，相机D的分辨率是多少 回复应包含1280和720 |

## 📡 Topics (27)

### 1. `/zj_humanoid/sensor/CAM_A/compressed`

| 字段 | 值 |
|------|-----|
| **Topic Name** | `/zj_humanoid/sensor/CAM_A/compressed` |
| **Type** | `sensor_msgs/CompressedImage` |
| **Direction** | 📤 Publish |
| **Description** | 左眼相机JPG |
| **Note** | 左眼相机的JPG图像数据 |

### 2. `/zj_humanoid/sensor/CAM_A/image_raw`

| 字段 | 值 |
|------|-----|
| **Topic Name** | `/zj_humanoid/sensor/CAM_A/image_raw` |
| **Type** | `sensor_msgs/Image` |
| **Direction** | 📤 Publish |
| **Description** | 左眼相机RGB |
| **Note** | 左眼相机的RGB图像源数据 |

### 3. `/zj_humanoid/sensor/CAM_B/compressed`

| 字段 | 值 |
|------|-----|
| **Topic Name** | `/zj_humanoid/sensor/CAM_B/compressed` |
| **Type** | `sensor_msgs/CompressedImage` |
| **Direction** | 📤 Publish |
| **Description** | 右眼相机JPG |
| **Note** | 右眼相机的JPG图像数据 |

### 4. `/zj_humanoid/sensor/CAM_B/image_raw`

| 字段 | 值 |
|------|-----|
| **Topic Name** | `/zj_humanoid/sensor/CAM_B/image_raw` |
| **Type** | `sensor_msgs/Image` |
| **Direction** | 📤 Publish |
| **Description** | 右眼相机RGB |
| **Note** | 右眼相机的RGB图像源数据 |

### 5. `/zj_humanoid/sensor/CAM_C/compressed`

| 字段 | 值 |
|------|-----|
| **Topic Name** | `/zj_humanoid/sensor/CAM_C/compressed` |
| **Type** | `sensor_msgs/CompressedImage` |
| **Direction** | 📤 Publish |
| **Description** | 相机C的JPG |
| **Note** | 右侧太阳穴相机C的JPG图像数据 |

### 6. `/zj_humanoid/sensor/CAM_C/image_raw`

| 字段 | 值 |
|------|-----|
| **Topic Name** | `/zj_humanoid/sensor/CAM_C/image_raw` |
| **Type** | `sensor_msgs/Image` |
| **Direction** | 📤 Publish |
| **Description** | 相机C的RGB |
| **Note** | 右侧太阳穴相机C的RGB图像源数据 |

### 7. `/zj_humanoid/sensor/CAM_D/compressed`

| 字段 | 值 |
|------|-----|
| **Topic Name** | `/zj_humanoid/sensor/CAM_D/compressed` |
| **Type** | `sensor_msgs/CompressedImage` |
| **Direction** | 📤 Publish |
| **Description** | 相机D的JPG |
| **Note** | 左侧太阳穴相机D的JPG图像数据 |

### 8. `/zj_humanoid/sensor/CAM_D/image_raw`

| 字段 | 值 |
|------|-----|
| **Topic Name** | `/zj_humanoid/sensor/CAM_D/image_raw` |
| **Type** | `sensor_msgs/Image` |
| **Direction** | 📤 Publish |
| **Description** | 相机D的RGB |
| **Note** | 左侧太阳穴相机D的RGB图像源数据 |

### 9. `/zj_humanoid/sensor/head_imu`

| 字段 | 值 |
|------|-----|
| **Topic Name** | `/zj_humanoid/sensor/head_imu` |
| **Type** | `sensor_msgs/Imu` |
| **Direction** | 📤 Publish |
| **Description** | 头部IMU数据 |
| **Note** | 头部IMU的目前帧率是多少 回复应接近100 |

### 10. `/zj_humanoid/sensor/realsense_down/aligned_depth_to_color/camera_info`

| 字段 | 值 |
|------|-----|
| **Topic Name** | `/zj_humanoid/sensor/realsense_down/aligned_depth_to_color/camera_info` |
| **Type** | `sensor_msgs/CameraInfo` |
| **Direction** | 📤 Publish |
| **Description** | 腹部深度aligned参数 |
| **Note** | 腹部深度相机的aligned_depth_to_color参数信息 |

### 11. `/zj_humanoid/sensor/realsense_down/aligned_depth_to_color/image_raw`

| 字段 | 值 |
|------|-----|
| **Topic Name** | `/zj_humanoid/sensor/realsense_down/aligned_depth_to_color/image_raw` |
| **Type** | `sensor_msgs/Image` |
| **Direction** | 📤 Publish |
| **Description** | 腹部深度aligned图像 |
| **Note** | 腹部深度相机的aligned_depth_to_color RGB图像源数据 |

### 12. `/zj_humanoid/sensor/realsense_down/aligned_depth_to_color/image_raw/compressed`

| 字段 | 值 |
|------|-----|
| **Topic Name** | `/zj_humanoid/sensor/realsense_down/aligned_depth_to_color/image_raw/compressed` |
| **Type** | `sensor_msgs/Image` |
| **Direction** | 📤 Publish |
| **Description** | 腹部深度aligned压缩图 |
| **Note** | 腹部深度相机的aligned_depth_to_color压缩格式 |

### 13. `/zj_humanoid/sensor/realsense_down/color/camera_info`

| 字段 | 值 |
|------|-----|
| **Topic Name** | `/zj_humanoid/sensor/realsense_down/color/camera_info` |
| **Type** | `sensor_msgs/CameraInfo` |
| **Direction** | 📤 Publish |
| **Description** | 腹部深度RGB参数 |
| **Note** | 腹部深度相机的参数信息 |

### 14. `/zj_humanoid/sensor/realsense_down/color/image_raw`

| 字段 | 值 |
|------|-----|
| **Topic Name** | `/zj_humanoid/sensor/realsense_down/color/image_raw` |
| **Type** | `sensor_msgs/Image` |
| **Direction** | 📤 Publish |
| **Description** | 腹部深度RGB图像 |
| **Note** | 腹部深度相机的RGB图像源数据 |

### 15. `/zj_humanoid/sensor/realsense_down/color/image_raw/compressed`

| 字段 | 值 |
|------|-----|
| **Topic Name** | `/zj_humanoid/sensor/realsense_down/color/image_raw/compressed` |
| **Type** | `sensor_msgs/Image` |
| **Direction** | 📤 Publish |
| **Description** | 腹部深度压缩图 |
| **Note** | 腹部深度相机的RGB图像JPG格式 |

### 16. `/zj_humanoid/sensor/realsense_down/depth/camera_info`

| 字段 | 值 |
|------|-----|
| **Topic Name** | `/zj_humanoid/sensor/realsense_down/depth/camera_info` |
| **Type** | `sensor_msgs/CameraInfo` |
| **Direction** | 📤 Publish |
| **Description** | 腹部相机深度参数 |
| **Note** | 腹部深度相机的参数信息 |

### 17. `/zj_humanoid/sensor/realsense_down/depth/image_rect_raw`

| 字段 | 值 |
|------|-----|
| **Topic Name** | `/zj_humanoid/sensor/realsense_down/depth/image_rect_raw` |
| **Type** | `sensor_msgs/Image` |
| **Direction** | 📤 Publish |
| **Description** | 腹部深度RGB图像 |
| **Note** | 腹部深度相机的RGB图像源数据 |

### 18. `/zj_humanoid/sensor/realsense_down/depth/image_rect_raw/compressed`

| 字段 | 值 |
|------|-----|
| **Topic Name** | `/zj_humanoid/sensor/realsense_down/depth/image_rect_raw/compressed` |
| **Type** | `sensor_msgs/Image` |
| **Direction** | 📤 Publish |
| **Description** | 腹部深度压缩图 |
| **Note** | 腹部深度相机的RGB图像JPG格式 |

### 19. `/zj_humanoid/sensor/realsense_up/aligned_depth_to_color/camera_info`

| 字段 | 值 |
|------|-----|
| **Topic Name** | `/zj_humanoid/sensor/realsense_up/aligned_depth_to_color/camera_info` |
| **Type** | `sensor_msgs/CameraInfo` |
| **Direction** | 📤 Publish |
| **Description** | 胸部深度aligned参数 |
| **Note** | 胸部深度相机的aligned_depth_to_color参数信息 |

### 20. `/zj_humanoid/sensor/realsense_up/aligned_depth_to_color/image_raw`

| 字段 | 值 |
|------|-----|
| **Topic Name** | `/zj_humanoid/sensor/realsense_up/aligned_depth_to_color/image_raw` |
| **Type** | `sensor_msgs/Image` |
| **Direction** | 📤 Publish |
| **Description** | 胸部深度aligned图像 |
| **Note** | 胸部深度相机的aligned_depth_to_color RGB图像源数据 |

### 21. `/zj_humanoid/sensor/realsense_up/aligned_depth_to_color/image_raw/compressed`

| 字段 | 值 |
|------|-----|
| **Topic Name** | `/zj_humanoid/sensor/realsense_up/aligned_depth_to_color/image_raw/compressed` |
| **Type** | `sensor_msgs/Image` |
| **Direction** | 📤 Publish |
| **Description** | 胸部深度aligned压缩图 |
| **Note** | 胸部深度相机的aligned_depth_to_color压缩格式 |

### 22. `/zj_humanoid/sensor/realsense_up/color/camera_info`

| 字段 | 值 |
|------|-----|
| **Topic Name** | `/zj_humanoid/sensor/realsense_up/color/camera_info` |
| **Type** | `sensor_msgs/CameraInfo` |
| **Direction** | 📤 Publish |
| **Description** | 胸部深度RGB参数 |
| **Note** | 胸部深度相机的参数信息 |

### 23. `/zj_humanoid/sensor/realsense_up/color/image_raw`

| 字段 | 值 |
|------|-----|
| **Topic Name** | `/zj_humanoid/sensor/realsense_up/color/image_raw` |
| **Type** | `sensor_msgs/Image` |
| **Direction** | 📤 Publish |
| **Description** | 胸部深度RGB图像 |
| **Note** | 胸部深度相机的RGB图像源数据 |

### 24. `/zj_humanoid/sensor/realsense_up/color/image_raw/compressed`

| 字段 | 值 |
|------|-----|
| **Topic Name** | `/zj_humanoid/sensor/realsense_up/color/image_raw/compressed` |
| **Type** | `sensor_msgs/Image` |
| **Direction** | 📤 Publish |
| **Description** | 胸部深度压缩图 |
| **Note** | 胸部深度相机的RGB图像JPG格式 |

### 25. `/zj_humanoid/sensor/realsense_up/depth/camera_info`

| 字段 | 值 |
|------|-----|
| **Topic Name** | `/zj_humanoid/sensor/realsense_up/depth/camera_info` |
| **Type** | `sensor_msgs/CameraInfo` |
| **Direction** | 📤 Publish |
| **Description** | 胸部相机深度参数 |
| **Note** | 胸部深度相机的参数信息 |

### 26. `/zj_humanoid/sensor/realsense_up/depth/image_rect_raw`

| 字段 | 值 |
|------|-----|
| **Topic Name** | `/zj_humanoid/sensor/realsense_up/depth/image_rect_raw` |
| **Type** | `sensor_msgs/Image` |
| **Direction** | publishs |
| **Description** | 胸部深度RGB图像 |
| **Note** | 胸部深度相机的RGB图像源数据 |

### 27. `/zj_humanoid/sensor/realsense_up/depth/image_rect_raw/compressed`

| 字段 | 值 |
|------|-----|
| **Topic Name** | `/zj_humanoid/sensor/realsense_up/depth/image_rect_raw/compressed` |
| **Type** | `sensor_msgs/Image` |
| **Direction** | 📤 Publish |
| **Description** | 胸部深度压缩图 |
| **Note** | 胸部深度相机的RGB图像JPG格式 |


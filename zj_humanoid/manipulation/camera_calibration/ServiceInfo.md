# /zj_humanoid/manipulation/camera_calibration

## description
- 相机内外参标定

## type
- Service

## msg_type
- [CameraCalibration](../../../../zj_humanoid_types.md#CameraCalibration)

## demos
- rosservice call /zj_humanoid/manipulation/camera_calibration "{camera_name: 'sacrum_to_hand', purpose: 'intrinsic', mode: 'from_folder'}"

## agent
- 自动相机内外参标定，外参标定时机器人会执行一段轨迹，拍摄不同角度的照片，从而计算外参


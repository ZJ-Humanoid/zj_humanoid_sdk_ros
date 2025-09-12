# /zj_humanoid/manipulation/instance_segmentation_action/goal

## description
- 实例分割Action

## type
- Topic/Publish

## msg_type
- [InstSeg](../../../../zj_humanoid_types.md#InstSeg)

## demos
- rostopic pub /zj_humanoid/manipulation/instance_segmentation_action/goal manipulation/InstSegActionGoal "header:
    seq: 0
    stamp:
        secs: 0
        nsecs: 0
    frame_id: ''
    goal_id:
    stamp:
        secs: 0
        nsecs: 0
    id: ''
    goal:
    labels:
      - 'melon_seeds'"

## agent
- 输入图像获取指定物品的实例分割信息


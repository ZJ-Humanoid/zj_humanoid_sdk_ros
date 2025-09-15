# /zj_humanoid/robot/orin_states/resource

## description
- 机器人大脑orin资源统计，包括cpu,temperature,memory,disk等信息

## type
- Topic/Publish

## msg_type
- [Resource](../../../../../zj_humanoid_types.md#Resource)

## hz
- 1

## agent
- 机器人大脑的资源状态

## demos
- get_orin_resouce
rostopic echo /zj_humanoid/robot/orin_states/resource

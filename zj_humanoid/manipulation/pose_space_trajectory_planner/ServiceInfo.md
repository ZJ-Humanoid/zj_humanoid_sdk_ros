# /zj_humanoid/manipulation/pose_space_trajectory_planner

## description
- 末端空间轨迹规划

## type
- Service

## msg_type
- [MotionPlan](../../../../zj_humanoid_types.md#MotionPlan)

## demos
- rosservice call /zj_humanoid/manipulation/pose_space_trajectory_planner "which_arm: 'right'
    waypoints:
      - position:
          x: 0.2801810664964644
          y: -0.4320977449761974
          z: 0.04364840767672533
      orientation:
          x: -0.3160102350523244
          y: -0.2673918536851587
          z: 0.6511814324582815
          w: 0.6360832256447027"

## agent
- 示教模式下记录各个末端执行器数据，据此生成完整的执行轨迹


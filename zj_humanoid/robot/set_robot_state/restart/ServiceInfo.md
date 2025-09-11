# /zj_humanoid/robot/set_robot_state/restart

## description
- 机器人先进stop软急停状态，再自动变为RUN启动运行，在某些故障状态可以执行，但如果有异常的存在，也可能会失败

## type
- Service

## msg_type
- std_srvs/Trigger

## demos
- restart_robot

## agent
- 将机器人状态机重启


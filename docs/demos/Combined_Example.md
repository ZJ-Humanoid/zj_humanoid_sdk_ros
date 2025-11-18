# wave_and_say_hello.py程序设计
> <a href="/navi_sdk_documents/demos/wave_and_say_hello.py" download="wave_and_say_hello.py">wave_and_say_hello.py</a>的执行效果是机器人抬起右臂挥手，在此期间播放语言，然后右臂放下

1、开启示教模式（`rosservice call /zj_humanoid/upperlimb/teach_mode/enter "arm_type: 2"`，这里只开启右臂示教）

2、拖动右臂，订阅`/zj_humanoid/upperlimb/joint_states`话题查看实时关节数据

`/zj_humanoid/upperlimb/joint_states`数据事例如下所示，`position`、`velocity`、`effort`中数据的顺序与`name`中的关节名一一对应
```bash
root@1424624317905:/navi_ws/src/upperlimb# rostopic echo -n 1 /zj_humanoid/upperlimb/joint_states 
header: 
  seq: 54110
  stamp: 
    secs: 1763087056
    nsecs: 431583463
  frame_id: ''
name: 
  - Shoulder_Y_L
  - Shoulder_X_L
  - Shoulder_Z_L
  - Elbow_L
  - Wrist_Z_L
  - Wrist_Y_L
  - Wrist_X_L
  - Shoulder_Y_R
  - Shoulder_X_R
  - Shoulder_Z_R
  - Elbow_R
  - Wrist_Z_R
  - Wrist_Y_R
  - Wrist_X_R
  - Neck_Z
  - Neck_Y
  - A_Waist
position: [1.1984225238848012e-05, 0.17458619327953784, 1.1984225238848012e-05, -0.08726655158086488, -3.5952675716544036e-05, -5.932784771706937e-08, -1.1865569543413874e-07, 0.0, -0.39993756467083585, -0.4999938611899779, -5.992112619424006e-08, 0.0, 0.0, -1.779835431512081e-07, 1.1865569543413874e-07, 0.0, 3.9947417462826706e-05]
velocity: [0.0003141592741012573, -0.0007330383062362671, 0.00010471975803375244, -9.383648362017994e-05, 0.00041887903213500975, 8.798319816441388e-05, -4.253806681313874e-05, -0.0010471975803375243, 0.002617993950843811, 0.00041887903213500975, 1.2343751996013454e-05, 0.00041887903213500975, 0.00011657922076404131, -5.102194903667966e-05, -3.025720233570538e-06, -2.7172154254417772e-05, 0.019973708731413353]
effort: [-0.5671744998190978, 2.618777306218922, 0.09673953043275081, -0.557714769084589, 0.009540912109074918, -0.20250250389724286, 0.16814606763127612, -0.03338217067486051, -5.839410041480182, -0.012935906014924305, -1.138389479459973, -0.018093309450686476, -0.35837300519526144, -0.3231379588661109, 0.0, -0.09736851808821359, 2.07883569299655e-17]
```


3、把右臂关节数据存放到`./wave_and_say_hello.yaml`，并填写其他`/zj_humanoid/upperlimb/movej_by_path/right_arm`所需参数

4、关闭示教模式`rosservice call /zj_humanoid/upperlimb/teach_mode/exit "arm_type: 2" `

5、执行`python3 wave_and_say_hello.py wave_and_say_hello.yaml`

# right_arm_movej.py程序设计
><a href="/navi_sdk_documents/demos/right_arm_movej.py" download="right_arm_movej.py">right_arm_movej.py</a>的执行效果是机器人右臂先回到初始位置，然后运动一小段距离

1、开启示教模式（`rosservice call /zj_humanoid/upperlimb/teach_mode/enter "arm_type: 2"`，这里只开启右臂示教）

2、拖动右臂，订阅`/zj_humanoid/upperlimb/joint_states`话题查看实时关节数据

3、把右臂关节数据存放到`right_arm_movej.yaml`中

4、关闭示教模式`rosservice call /zj_humanoid/upperlimb/teach_mode/exit "arm_type: 2" `

5、执行`python3 right_arm_movej.py right_arm_movej.yaml`

# navi_sdk_ros
浙江人形机器人创新中心推出的领航者2号(Navi02)机器人的ROS1 SDK；主要介绍Navi02 基于ROS1的API接口；

[Online Docs](https://zj-humanoid.github.io/navi_sdk_documents/)

## Project Structure

```
.
├── dist                                            # .run包 一键安装所有的msg和srv文件
├── docs                                            # 相关文档
│   ├── zj_humanoid.md                              # msg和srv说明文档
│   └── zj_humanoid_ros_interface.md                # 接口文档
├── navi_demos                                      # 相关demo功能包
│   ├── CMakeLists.txt
│   ├── launch
│   ├── package.xml
│   ├── python
│   └── src
└── README.md
```

## Supported sub-items
```mermaid
  root((我的项目))
    🚀 使用
      安装
      运行
    🧩 架构
      后端
        接口
        数据库
      前端
        组件
        路由
    📌 计划
      v1.0
      v1.1
```



## How To Use .run

```
Help:
  ./zj_humanoid_types_25_R3.run                      # Install all .deb files in the current directory
  ./zj_humanoid_types_25_R3.run -- --uninstall       # Uninstall all .deb files in the current directory
  ./zj_humanoid_types_25_R3.run -- --version         # Show verison
  ./zj_humanoid_types_25_R3.run -- --changelog       # Show changelog
  ./zj_humanoid_types_25_R3.run -- --help            # Show the help info
```
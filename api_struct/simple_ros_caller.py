#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简化的 ROS 服务调用脚本

用法:
    python simple_ros_caller.py <service_name> <yaml_file>
"""

import sys
import os
import yaml
import json
from pathlib import Path

def load_yaml_data(yaml_file):
    """加载 YAML 数据"""
    try:
        with open(yaml_file, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
        return data
    except Exception as e:
        print(f"❌ 加载 YAML 文件失败: {e}")
        return None

def find_yaml_file(service_name, yaml_filename):
    """查找 YAML 文件"""
    # 从服务名称构建路径
    if service_name.startswith('/'):
        service_name = service_name[1:]
    
    # 可能的路径
    possible_paths = [
        yaml_filename,  # 当前目录
        os.path.join(service_name, yaml_filename),  # 服务目录
    ]
    
    for path in possible_paths:
        if os.path.exists(path):
            return path
    
    # 递归搜索
    for root, dirs, files in os.walk('.'):
        if yaml_filename in files:
            full_path = os.path.join(root, yaml_filename)
            return full_path
    
    return None

def call_ros_service(service_name, yaml_file):
    """调用 ROS 服务"""
    print(f"🚀 调用 ROS 服务: {service_name}")
    print(f"📄 数据文件: {yaml_file}")
    
    # 查找文件
    yaml_path = find_yaml_file(service_name, yaml_file)
    if not yaml_path:
        print(f"❌ 未找到文件: {yaml_file}")
        return False
    
    print(f"📂 使用文件: {yaml_path}")
    
    # 加载数据
    data = load_yaml_data(yaml_path)
    if data is None:
        return False
    
    print(f"📊 数据内容:")
    print(json.dumps(data, indent=2, ensure_ascii=False))
    
    # 这里添加实际的 ROS 服务调用逻辑
    try:
        import rospy
        import rosservice
        
        # 初始化节点
        rospy.init_node('simple_caller', anonymous=True)
        
        # 检查服务是否存在
        available_services = rosservice.get_service_list()
        if service_name not in available_services:
            print(f"⚠️  服务 {service_name} 不可用")
            print("可用服务列表:")
            for svc in available_services:
                if 'zj_humanoid' in svc:
                    print(f"  - {svc}")
            return False
        
        # 获取服务类型
        service_type = rosservice.get_service_type(service_name)
        print(f"🔧 服务类型: {service_type}")
        
        print("✅ 服务调用模拟成功")
        return True
        
    except ImportError:
        print("⚠️  ROS 未安装，使用模拟模式")
        print("✅ 模拟调用成功")
        return True
    except Exception as e:
        print(f"❌ 调用失败: {e}")
        return False

def main():
    if len(sys.argv) != 3:
        print("用法: python simple_ros_caller.py <service_name> <yaml_file>")
        print("示例: python simple_ros_caller.py /zj_humanoid/hand/joint_switch/left left_hand_joint_reset.yaml")
        sys.exit(1)
    
    service_name = sys.argv[1]
    yaml_file = sys.argv[2]
    
    success = call_ros_service(service_name, yaml_file)
    
    if not success:
        sys.exit(1)

if __name__ == '__main__':
    main()

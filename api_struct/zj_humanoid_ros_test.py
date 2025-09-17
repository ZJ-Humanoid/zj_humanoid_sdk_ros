#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ZJ Humanoid ROS API 测试脚本 (无模拟模式版本)

用法:
    python zj_humanoid_ros_test.py <service_name> <yaml_file>
    
示例:
    python zj_humanoid_ros_test.py /zj_humanoid/hand/joint_switch/left left_hand_joint_reset.yaml
    python zj_humanoid_ros_test.py /zj_humanoid/audio/listen hello_world.yaml
"""

import sys
import os
import json
import yaml
import argparse
from pathlib import Path
import time

# 检查 ROS 是否可用，不可用时直接退出
try:
    import rospy
    import rosservice
    from std_srvs.srv import Empty
except ImportError:
    print("❌ ROS 库未安装，请先安装 ROS 环境")
    print("💡 安装命令: sudo apt install ros-noetic-desktop-full")
    sys.exit(1)

class ZJHumanoidROSTester:
    def __init__(self, interfaces_file="zj_humanoid_interfaces.json"):
        """
        初始化 ROS 测试器
        
        Args:
            interfaces_file (str): 接口定义文件路径
        """
        self.interfaces_file = interfaces_file
        self.services_info = {}
        self.topics_info = {}
        
        # 加载接口定义
        self._load_interfaces()
        
        # 初始化 ROS 节点
        try:
            rospy.init_node('zj_humanoid_tester', anonymous=True)
            self.ros_initialized = True
            print("✅ ROS 节点初始化成功")
        except Exception as e:
            print(f"❌ ROS 节点初始化失败: {e}")
            sys.exit(1)
    
    def _load_interfaces(self):
        """加载接口定义文件"""
        try:
            # 尝试不同的编码方式
            encodings = ['utf-8-sig', 'utf-8', 'gbk', 'cp1252']
            data = None
            
            for encoding in encodings:
                try:
                    with open(self.interfaces_file, 'r', encoding=encoding) as f:
                        data = json.load(f)
                    break
                except (UnicodeDecodeError, json.JSONDecodeError):
                    continue
            
            if data is None:
                raise Exception("无法解析 JSON 文件")
            
            # 构建服务名称到信息的映射
            for service in data.get('services', []):
                name = service.get('name', '')
                if name:
                    self.services_info[name] = service
            
            # 构建话题名称到信息的映射
            for topic in data.get('topics', []):
                name = topic.get('name', '')
                if name:
                    self.topics_info[name] = topic
                    
            print(f"📚 已加载 {len(self.services_info)} 个服务和 {len(self.topics_info)} 个话题定义")
            
        except FileNotFoundError:
            print(f"❌ 接口定义文件未找到: {self.interfaces_file}")
            sys.exit(1)
        except Exception as e:
            print(f"❌ 加载接口定义失败: {e}")
            sys.exit(1)
    
    def _load_yaml_data(self, yaml_file):
        """
        加载 YAML 数据文件
        
        Args:
            yaml_file (str): YAML 文件路径
            
        Returns:
            dict: 解析后的数据
        """
        try:
            with open(yaml_file, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
            return data
        except FileNotFoundError:
            print(f"❌ YAML 文件未找到: {yaml_file}")
            return None
        except yaml.YAMLError as e:
            print(f"❌ YAML 解析错误: {e}")
            return None
        except Exception as e:
            print(f"❌ 读取 YAML 文件失败: {e}")
            return None
    
    def _find_yaml_file(self, service_name, yaml_filename):
        """
        根据服务名称查找对应的 YAML 文件路径
        优化查找逻辑：
        1. 优先从服务名称构建可能的路径
        2. 如果找不到，在脚本所在的当前目录继续寻找
        3. 如果还没有才报错
        
        Args:
            service_name (str): 服务名称
            yaml_filename (str): YAML 文件名
            
        Returns:
            str: 完整的文件路径，如果未找到则返回 None
        """
        print(f"   🔍 开始查找 YAML 文件: {yaml_filename}")
        
        # 第一步：从服务名称构建可能的路径
        if service_name.startswith('/'):
            service_name_clean = service_name[1:]  # 移除开头的 '/'
        else:
            service_name_clean = service_name
        
        # 构建基于服务名称的可能路径
        service_based_paths = [
            # 完整服务路径
            os.path.join(service_name_clean, yaml_filename),
            # zj_humanoid 目录下的服务路径
            os.path.join('zj_humanoid', service_name_clean.replace('zj_humanoid/', ''), yaml_filename),
        ]
        
        print(f"   📂 步骤1：检查服务名称相关路径...")
        for path in service_based_paths:
            print(f"      尝试路径: {path}")
            if os.path.exists(path):
                print(f"   ✅ 在服务路径中找到: {path}")
                return path
        
        # 第二步：在脚本所在的当前目录查找
        print(f"   📂 步骤2：在当前目录查找...")
        current_dir_path = yaml_filename
        print(f"      尝试路径: {current_dir_path}")
        if os.path.exists(current_dir_path):
            print(f"   ✅ 在当前目录找到: {current_dir_path}")
            return current_dir_path
        
        # 第三步：在当前目录的子目录中递归搜索
        print(f"   📂 步骤3：在当前目录及子目录中递归搜索...")
        current_script_dir = os.path.dirname(os.path.abspath(__file__))
        
        for root, dirs, files in os.walk(current_script_dir):
            if yaml_filename in files:
                full_path = os.path.join(root, yaml_filename)
                relative_path = os.path.relpath(full_path, current_script_dir)
                print(f"      找到文件: {relative_path}")
                
                # 如果文件路径包含服务名称的部分，优先返回
                if service_name_clean.replace('/', os.sep) in full_path.replace('\\', '/'):
                    print(f"   ✅ 在递归搜索中找到匹配的文件: {relative_path}")
                    return full_path
                else:
                    # 记录找到的文件，但继续寻找更匹配的
                    print(f"      记录备选文件: {relative_path}")
        
        # 第四步：如果上面都没找到，再次遍历找到的任何同名文件
        print(f"   📂 步骤4：使用任何找到的同名文件...")
        for root, dirs, files in os.walk(current_script_dir):
            if yaml_filename in files:
                full_path = os.path.join(root, yaml_filename)
                relative_path = os.path.relpath(full_path, current_script_dir)
                print(f"   ⚠️  使用找到的文件（可能不完全匹配）: {relative_path}")
                return full_path
        
        # 如果所有步骤都没找到文件
        print(f"   ❌ 所有查找步骤都未找到文件")
        return None
    
    def _check_service_availability(self, service_name):
        """
        检查 ROS 服务是否可用
        
        Args:
            service_name (str): 服务名称
            
        Returns:
            bool: 服务是否可用
        """
        try:
            available_services = rosservice.get_service_list()
            return service_name in available_services
        except Exception as e:
            print(f"❌ 检查服务可用性失败: {e}")
            return False
    
    def _get_service_type(self, service_name):
        """
        获取服务类型
        
        Args:
            service_name (str): 服务名称
            
        Returns:
            str: 服务类型
        """
        if service_name in self.services_info:
            return self.services_info[service_name].get('type', 'Unknown')
        return 'Unknown'
    
    def call_service(self, service_name, yaml_file):
        """
        调用 ROS 服务
        
        Args:
            service_name (str): 服务名称
            yaml_file (str): YAML 数据文件路径
        """
        print(f"🚀 准备调用服务...")
        print(f"   📡 服务名称: {service_name}")
        print(f"   📄 数据文件: {yaml_file}")
        
        # 查找 YAML 文件
        yaml_path = self._find_yaml_file(service_name, yaml_file)
        if not yaml_path:
            print(f"❌ 未找到 YAML 文件: {yaml_file}")
            print("💡 提示: 请确保文件存在于以下位置之一:")
            print("   1. 服务对应的目录中")
            print("   2. 脚本所在的当前目录")
            print("   3. 当前目录的任何子目录中")
            return False
        
        print(f"   📂 找到文件: {yaml_path}")
        
        # 加载 YAML 数据
        data = self._load_yaml_data(yaml_path)
        if data is None:
            return False
        
        print(f"   📊 数据内容: {json.dumps(data, indent=2, ensure_ascii=False)}")
        
        # 检查服务信息
        if service_name not in self.services_info:
            print(f"⚠️  服务 {service_name} 不在接口定义中")
        else:
            service_info = self.services_info[service_name]
            print(f"   📝 服务描述: {service_info.get('description', 'N/A')}")
            print(f"   🔧 服务类型: {service_info.get('type', 'N/A')}")
        
        # 检查服务是否可用
        if not self._check_service_availability(service_name):
            print(f"❌ ROS 服务 {service_name} 当前不可用")
            print("💡 请检查服务是否已启动")
            return False
        
        try:
            # 获取服务类型
            service_type = rosservice.get_service_type(service_name)
            print(f"   🔍 检测到服务类型: {service_type}")
            
            # 这里需要根据实际的服务类型来动态调用
            # 由于我们不知道具体的服务定义，这里使用通用方法
            print("⚠️  实际的服务调用需要根据具体的消息类型实现")
            print("💡 请根据服务类型实现具体的调用逻辑")
            
            # TODO: 在这里添加实际的服务调用代码
            # 例如:
            # service_proxy = rospy.ServiceProxy(service_name, service_type)
            # response = service_proxy(data)
            
        except Exception as e:
            print(f"❌ 服务调用失败: {e}")
            return False
        
        return True


def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description='ZJ Humanoid ROS API 测试工具 (无模拟模式)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例:
  %(prog)s /zj_humanoid/hand/joint_switch/left left_hand_joint_reset.yaml
  %(prog)s /zj_humanoid/audio/listen hello_world.yaml
        """
    )
    
    parser.add_argument('service_name', help='ROS 服务名称')
    parser.add_argument('yaml_file', help='YAML 数据文件')
    parser.add_argument('--interfaces', '-i', default='zj_humanoid_interfaces.json',
                       help='接口定义文件路径 (默认: zj_humanoid_interfaces.json)')
    
    args = parser.parse_args()
    
    # 检查接口文件是否存在
    if not os.path.exists(args.interfaces):
        print(f"❌ 接口定义文件不存在: {args.interfaces}")
        print("💡 请确保在正确的目录中运行脚本")
        sys.exit(1)
    
    # 创建测试器实例
    tester = ZJHumanoidROSTester(args.interfaces)
    
    # 调用服务
    print("🎯 ZJ Humanoid ROS API 测试工具")
    print("=" * 50)
    
    success = tester.call_service(args.service_name, args.yaml_file)
    
    if success:
        print("\n✅ 测试完成")
    else:
        print("\n❌ 测试失败")
        sys.exit(1)

if __name__ == '__main__':
    main()
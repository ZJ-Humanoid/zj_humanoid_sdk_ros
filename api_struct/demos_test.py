#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ZJ Humanoid ROS API 测试工具
用于测试 ZJ Humanoid ROS API 的 Python 脚本

作者: ZJ Humanoid SDK Team
版本: 1.0.0
"""

import os
import sys
import json
import yaml
import argparse
import glob
from pathlib import Path
from typing import Dict, Any, Optional, List, Tuple, Callable


class ROSServiceCaller:
    """封装 ROS 服务调用相关逻辑"""

    def __init__(
        self,
        rospy_module: Optional[Any] = None,
        rosservice_module: Optional[Any] = None,
        load_yaml_func: Optional[Callable[[str], Dict[str, Any]]] = None,
    ) -> None:
        self.rospy = rospy_module
        self.rosservice = rosservice_module
        self.load_yaml_func = load_yaml_func

    def update_dependencies(
        self,
        rospy_module: Optional[Any],
        rosservice_module: Optional[Any],
        load_yaml_func: Optional[Callable[[str], Dict[str, Any]]] = None,
    ) -> None:
        """更新依赖模块"""

        self.rospy = rospy_module
        self.rosservice = rosservice_module
        if load_yaml_func is not None:
            self.load_yaml_func = load_yaml_func

    def call_from_yaml(
        self,
        service_name: str,
        msg_type: str,
        yaml_file: str,
        load_yaml_func: Optional[Callable[[str], Dict[str, Any]]] = None,
    ) -> bool:
        """根据 YAML 配置调用 ROS 服务"""

        if self.rospy is None or self.rosservice is None:
            print("   - 状态: ❌ ROS 环境未初始化")
            return False

        loader = load_yaml_func or self.load_yaml_func
        if loader is None:
            print("   - 状态: ❌ 未提供 YAML 数据加载方法")
            return False

        try:
            services = self.rosservice.get_service_list()
            if service_name not in services:
                print(f"   - 状态: ❌ 服务 {service_name} 不在线")
                print(f"   - 可用服务: {len(services)} 个")
                return False
        except Exception as e:
            print(f"   - 状态: ❌ 无法检查服务状态: {e}")
            return False

        try:
            data = loader(yaml_file)
        except Exception as e:
            print(f"❌ 数据加载失败: {e}")
            return False

        try:
            msg_type_info = self.rosservice.get_service_type(service_name)
            if msg_type_info != msg_type:
                print(f"   - 状态: ❌ 消息类型不一致: {msg_type_info} != {msg_type}")
                return False
        except Exception as e:
            print(f"   - 状态: ⚠️  无法获取消息类型: {e}")

        print(f"   - 请求数据: {json.dumps(data, ensure_ascii=False, indent=2)}")
        return self._call_ros_service(service_name, msg_type, data)

    def _call_ros_service(self, service_name: str, msg_type: str, data: Dict[str, Any]) -> bool:
        """实际调用 ROS 服务"""

        if self.rospy is None:
            print("   - 状态: ❌ ROS 环境未初始化")
            return False

        ServiceException = getattr(self.rospy, "ServiceException", Exception)
        ROSException = getattr(self.rospy, "ROSException", Exception)

        try:
            self.rospy.wait_for_service(service_name, timeout=5.0)
            print("   - ✅ 服务已就绪")

            service_class, request_class = self._get_service_class(service_name, msg_type)
            if service_class is None:
                return False

            service_proxy = self.rospy.ServiceProxy(service_name, service_class)

            if request_class is not None:
                request = self._build_request_message(request_class, data)
                if request is None:
                    return False
                print("   - 调用服务...")
                response = service_proxy.call(request)
            else:
                if data:
                    print("   - 使用关键字参数调用服务...")
                    response = service_proxy(**data)
                else:
                    print("   - 使用默认参数调用服务...")
                    response = service_proxy()

            print(f"   - ✅ 响应数据: {response}")
            return True

        except ServiceException as e:
            print(f"   - 状态: ❌ 服务调用异常: {e}")
            return False
        except ROSException as e:
            print(f"   - 状态: ❌ ROS异常: {e}")
            return False
        except Exception as e:
            print(f"   - 状态: ❌ 服务调用失败: {e}")
            return False

    def _get_service_class(self, service_name: str, msg_type: str) -> Tuple[Optional[Any], Optional[Any]]:
        """根据消息类型动态导入服务类"""

        if not msg_type:
            print(f"   - 状态: ❌ 服务 {service_name} 缺少消息类型定义")
            return None, None

        try:
            if '/' not in msg_type:
                print(f"   - 状态: ❌ 服务 {service_name} 消息类型格式无效: {msg_type}")
                return None, None

            package_name, service_type_name = msg_type.split('/', 1)
            if not package_name or not service_type_name:
                print(f"   - 状态: ❌ 服务 {service_name} 消息类型格式无效: {msg_type}")
                return None, None

            if service_type_name.endswith('Request'):
                service_type_name = service_type_name[:-7]

            module_path = f"{package_name}.srv._{service_type_name}"
            try:
                service_module = __import__(module_path, fromlist=['*'])
            except ImportError as inner:
                print(f"   - 状态: ❌ 导入服务模块失败: {module_path}: {inner}")
                return None, None

            public_module = __import__(f"{package_name}.srv", fromlist=[service_type_name])
            service_class = getattr(public_module, service_type_name, None)
            if service_class is None:
                print(f"   - 状态: ❌ 在 {package_name}.srv 中找不到服务类 {service_type_name}")
                return None, None

            request_class = getattr(service_module, f"{service_type_name}Request", None)

            if request_class is None and hasattr(service_class, '_request_class'):
                request_class = service_class._request_class

            if request_class is None:
                print(f"   - ⚠️ 找不到请求类: {service_type_name}Request")

            print(f"   - 成功导入服务类: {module_path}.{service_type_name}")
            return service_class, request_class

        except ImportError as e:
            print(f"   - 导入失败: {e}")
            print("   - 尝试使用std_srvs/Empty作为默认服务类型")
            try:
                from std_srvs.srv import Empty
                request_class = Empty._request_class if hasattr(Empty, '_request_class') else None
                return Empty, request_class
            except ImportError:
                return None, None
        except Exception as e:
            print(f"   - 获取服务类失败: {e}")
            return None, None

    def _build_request_message(self, request_class: Any, data: Dict[str, Any]) -> Optional[Any]:
        """构建服务请求消息"""

        try:
            request = request_class()
        except Exception as e:
            print(f"   - 状态: ❌ 无法实例化请求消息: {e}")
            return None

        if not data:
            return request

        for key, value in data.items():
            self._assign_request_field(request, key, value)

        return request

    def _assign_request_field(self, request: Any, key: str, value: Any) -> None:
        slots = getattr(request, '__slots__', [])
        if key not in slots:
            print(f"     - ⚠️ 请求消息没有字段: {key}")
            return
        try:
            slot_types = getattr(type(request), '_slot_types', [])
            field_value = self._normalize_field_value(slot_types, slots, key, value)
            setattr(request, key, field_value)
            print(f"     - {key}: {field_value}")
        except Exception as e:
            raise ValueError(f"设置字段 {key} 失败: {e}")

    def _normalize_field_value(
        self,
        slot_types: List[str],
        slots: List[str],
        key: str,
        value: Any,
    ) -> Any:
        try:
            index = slots.index(key)
        except ValueError:
            return value

        field_type = slot_types[index] if index < len(slot_types) else None

        if field_type is None:
            return value

        if field_type.endswith('[]'):
            element_type = field_type[:-2]
            if not isinstance(value, (list, tuple)):
                raise ValueError(f"字段 {key} 需要列表类型数据")
            return [self._normalize_scalar_value(element_type, v) for v in value]

        return self._normalize_scalar_value(field_type, value)

    def _normalize_scalar_value(self, field_type: str, value: Any) -> Any:
        if field_type in ('float32', 'float64'):
            return float(value)
        if field_type in (
            'int8', 'int16', 'int32', 'int64',
            'uint8', 'uint16', 'uint32', 'uint64',
        ):
            return int(value)
        if field_type == 'bool':
            if isinstance(value, bool):
                return value
            if isinstance(value, str):
                return value.strip().lower() in ('1', 'true', 't', 'yes', 'y', 'on')
            return bool(value)
        if field_type == 'string':
            return str(value)

        return value


class ZJHumanoidROSTester:
    """ZJ Humanoid ROS API 测试器"""
    
    def __init__(self, interfaces_file: str = "zj_humanoid_interfaces.json"):
        """
        初始化测试器
        
        Args:
            interfaces_file: 接口定义文件路径
        """
        self.interfaces_file = interfaces_file
        self.interfaces_data = self._load_interfaces()
        self.ros_available = self._check_ros_environment()
        self.service_caller = ROSServiceCaller()
        
        # 尝试导入ROS相关模块
        if self.ros_available:
            try:
                import rospy
                import rosservice
                from std_srvs.srv import Empty
                self.rospy = rospy
                self.rosservice = rosservice
                self.Empty = Empty
                self.service_caller.update_dependencies(rospy, rosservice, self.load_yaml_data)
                print("✅ ROS 环境检测成功")
            except ImportError as e:
                print(f"⚠️  ROS 模块导入失败: {e}")
                self.ros_available = False
        else:
            print("❌  ROS 环境不可用，请确保ROS环境正确配置")
            self.service_caller.update_dependencies(None, None)
    
    def _load_interfaces(self) -> Dict[str, Any]:
        """加载接口定义文件"""
        try:
            if os.path.exists(self.interfaces_file):
                # 尝试不同的编码方式处理BOM
                for encoding in ['utf-8-sig', 'utf-8']:
                    try:
                        with open(self.interfaces_file, 'r', encoding=encoding) as f:
                            return json.load(f)
                    except UnicodeDecodeError:
                        continue
                # 如果都失败了，使用默认方式
                with open(self.interfaces_file, 'r', encoding='utf-8') as f:
                    return json.load(f)
            else:
                print(f"⚠️  接口定义文件 {self.interfaces_file} 不存在，使用空数据")
                return {}
        except Exception as e:
            print(f"⚠️  加载接口定义文件失败: {e}")
            return {}
    
    def _check_ros_environment(self) -> bool:
        """检查ROS环境是否可用"""
        try:
            # 检查ROS环境变量
            if 'ROS_MASTER_URI' not in os.environ:
                return False
            
            # 检查ROS master是否运行
            import subprocess
            result = subprocess.run(['rosnode', 'list'], 
                                  capture_output=True, text=True, timeout=5)
            return result.returncode == 0
        except:
            return False
    
    def find_yaml_file(self, yaml_path: str) -> Optional[str]:
        """
        智能查找YAML文件
        
        Args:
            yaml_path: YAML文件路径
            
        Returns:
            找到的完整文件路径，如果未找到返回None
        """
        search_paths = [
            # 当前目录
            yaml_path,
            # 相对于当前目录
            os.path.join(os.getcwd(), yaml_path),
            # 相对于脚本目录
            os.path.join(os.path.dirname(__file__), yaml_path),
            # 在zj_humanoid目录中递归搜索
            os.path.join(os.path.dirname(__file__), "zj_humanoid", "**", os.path.basename(yaml_path))
        ]
        
        for search_path in search_paths:
            if os.path.exists(search_path):
                return os.path.abspath(search_path)
            
            # 处理通配符搜索
            if "**" in search_path:
                matches = glob.glob(search_path, recursive=True)
                if matches:
                    return os.path.abspath(matches[0])
        
        return None
    
    def load_yaml_data(self, yaml_file: str) -> Dict[str, Any]:
        """
        加载YAML数据
        
        Args:
            yaml_file: YAML文件路径
            
        Returns:
            YAML数据字典
        """
        try:
            with open(yaml_file, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
                if data is None:
                    return {}
                return data
        except yaml.YAMLError as e:
            raise ValueError(f"YAML 格式错误: {e}")
        except Exception as e:
            raise ValueError(f"读取YAML文件失败: {e}")
    
    def extract_service_info(self, yaml_file: str, explicit_service: Optional[str] = None) -> Tuple[str, str, str]:
        """
        从YAML文件路径提取服务信息
        
        Args:
            yaml_file: YAML文件路径
            explicit_service: 显式指定的服务名
            
        Returns:
            (service_name, msg_type, description)
        """
        # 如果显式指定了服务名，直接使用
        if explicit_service:
            service_name = explicit_service
        else:
            # 从文件路径推断服务名
            # 例如: zj_humanoid/hand/joint_switch/left/left_hand_joint_reset.yaml
            # 服务名: /zj_humanoid/hand/joint_switch/left
            path_parts = yaml_file.replace('\\', '/').split('/')
            if 'zj_humanoid' in path_parts:
                zj_index = path_parts.index('zj_humanoid')
                service_parts = path_parts[zj_index:-1]  # 排除文件名
                service_name = '/' + '/'.join(service_parts)
            else:
                raise ValueError("无法从文件路径推断服务名，请使用 --service 参数显式指定")
        
        # 从interfaces_file中查找服务信息
        if self.interfaces_data:
            # 在services中查找
            if 'services' in self.interfaces_data:
                for service in self.interfaces_data['services']:
                    if service.get('name') == service_name:
                        msg_type = service.get('type', 'unknown')
                        description = service.get('description', '无描述')
                        return service_name, msg_type, description, "service"
            
            # 在topics中查找
            if 'topics' in self.interfaces_data:
                for topic in self.interfaces_data['topics']:
                    if topic.get('name') == service_name:
                        msg_type = topic.get('type', 'unknown')
                        description = topic.get('description', '无描述')
                        return service_name, msg_type, description, "topic"
        
        # 如果interfaces_file中没有找到，尝试从service.yaml文件获取（作为备用方案）
        # service_yaml_path = os.path.join(os.path.dirname(yaml_file), "service.yaml")
        # if os.path.exists(service_yaml_path):
        #     try:
        #         service_info = self.load_yaml_data(service_yaml_path)
        #         if 'service' in service_info:
        #             msg_type = service_info['service'].get('type', 'unknown')
        #             description = service_info['service'].get('description', '无描述')
        #             return service_name, msg_type, description
        #     except Exception as e:
        #         print(f"⚠️  读取service.yaml失败: {e}")
        
        return service_name, "unknown", "无描述", "unknown"
    
    
    def service_call_yaml(self, service_name: str, msg_type: str, yaml_file: str) -> bool:
        """兼容旧接口，使用 `ROSServiceCaller` 完成调用"""

        return self.service_caller.call_from_yaml(service_name, msg_type, yaml_file)
    

    def run_test(self, yaml_path: str, explicit_service: Optional[str] = None) -> bool:
        """
        运行测试
        
        Args:
            yaml_path: YAML文件路径
            explicit_service: 显式指定的服务名
            
        Returns:
            测试是否成功
        """
        print("=" * 60)
        print("🤖 ZJ Humanoid ROS API 测试工具")
        print("=" * 60)
        
        # 1. 查找YAML文件
        # print(f"\n🔍 查找YAML文件: {yaml_path}")
        yaml_file = self.find_yaml_file(yaml_path)
        
        if not yaml_file:
            print(f"❌ 文件未找到: {yaml_path}")
            print("\n💡 建议:")
            print("   - 检查文件路径是否正确")
            print("   - 确保文件存在于以下位置之一:")
            print("     * 当前目录")
            print("     * zj_humanoid/ 目录及其子目录")
            return False
        
        print(f"✅ 找到文件: {yaml_file}")
        
        # 3. 提取服务信息
        try:
            service_name, msg_type, description, service_type = self.extract_service_info(yaml_file, explicit_service)
            print(f"   - 服务名: {service_name}")
            print(f"   - 消息类型: {msg_type}")
            print(f"   - 描述: {description}")
        except Exception as e:
            print(f"❌ 服务信息提取失败: {e}")
            return False
        
        # 5. 执行服务调用
        if service_type == "service":
            success = self.service_call_yaml(service_name, msg_type, yaml_file)
        # else:
        #     success = self.real_topic_call(service_name, service_type, data)
        
        # 6. 输出结果
        print(f"\n" + "=" * 60)
        if success:
            print("🎉 测试完成 - 成功")
        else:
            print("💥 测试完成 - 失败")
        print("=" * 60)
        
        return success


def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description="ZJ Humanoid ROS API 测试工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例用法:
  # 调用左手手掌控制服务
  python demos_test.py zj_humanoid/hand/joint_switch/left/left_hand_joint_reset.yaml

  # 调用左手手臂控制服务
python3 demos_test.py zj_humanoid/upperlimb/movej/left_arm/left_arm_t_case1.yaml

  # 调用音频服务
  python demos_test.py zj_humanoid/audio/listen/hello_world.yaml
  
  # 显式指定服务名
  python demos_test.py zj_humanoid/audio/LLM_chat/hello_world.yaml --service /custom/service/name
        """
    )
    
    parser.add_argument(
        'yaml_file',
        help='YAML文件路径，包含服务调用参数'
    )
    
    parser.add_argument(
        '--service',
        help='显式指定服务名，覆盖从文件路径推断的服务名'
    )
    
    parser.add_argument(
        '--interfaces',
        default='zj_humanoid_interfaces.json',
        help='接口定义文件路径 (默认: zj_humanoid_interfaces.json)'
    )
    
    args = parser.parse_args()
    
    # 创建测试器并运行测试
    tester = ZJHumanoidROSTester(args.interfaces)
    success = tester.run_test(args.yaml_file, args.service)
    
    # 设置退出码
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()

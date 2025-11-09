#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
脚本功能：遍历 api_struct/zj_humanoid 目录，读取所有的 service.yaml、topic.yaml 和 action.yaml 文件，
然后将它们合并成一个大的 YAML 文件 (zj_humanoid_interfaces.yaml)
"""

import os
import yaml
from datetime import datetime
from pathlib import Path


def find_yaml_files(base_path):
    """
    遍历目录，查找所有的 service.yaml、topic.yaml 和 action.yaml 文件
    
    Args:
        base_path: 基础路径 (api_struct/zj_humanoid)
    
    Returns:
        包含三个列表的字典: services, topics, actions
    """
    services = []
    topics = []
    actions = []
    
    base_path = Path(base_path)
    
    # 遍历所有子目录
    for root, dirs, files in os.walk(base_path):
        root_path = Path(root)
        
        # 检查是否有 service.yaml
        if 'service.yaml' in files:
            service_file = root_path / 'service.yaml'
            try:
                with open(service_file, 'r', encoding='utf-8') as f:
                    data = yaml.safe_load(f)
                    if data and 'service' in data:
                        service_info = data['service']
                        # 处理 demos 字段
                        if 'demos' in service_info:
                            demos = service_info['demos']
                            if isinstance(demos, str):
                                service_info['demos'] = [demos] if demos else []
                            elif demos is None:
                                service_info['demos'] = []
                        else:
                            service_info['demos'] = []
                        services.append(service_info)
            except Exception as e:
                print(f"Error reading {service_file}: {e}")
        
        # 检查是否有 topic.yaml
        if 'topic.yaml' in files:
            topic_file = root_path / 'topic.yaml'
            try:
                with open(topic_file, 'r', encoding='utf-8') as f:
                    data = yaml.safe_load(f)
                    if data and 'topic' in data:
                        topic_info = data['topic']
                        # 处理 demos 字段
                        if 'demos' in topic_info:
                            demos = topic_info['demos']
                            if isinstance(demos, str):
                                topic_info['demos'] = [demos] if demos else []
                            elif demos is None:
                                topic_info['demos'] = []
                        else:
                            topic_info['demos'] = []
                        topics.append(topic_info)
            except Exception as e:
                print(f"Error reading {topic_file}: {e}")
        
        # 检查是否有 action.yaml
        if 'action.yaml' in files:
            action_file = root_path / 'action.yaml'
            try:
                with open(action_file, 'r', encoding='utf-8') as f:
                    data = yaml.safe_load(f)
                    if data and 'action' in data:
                        action_info = data['action']
                        # 处理 demos 字段
                        if 'demos' in action_info:
                            demos = action_info['demos']
                            if isinstance(demos, str):
                                action_info['demos'] = [demos] if demos else []
                            elif demos is None:
                                action_info['demos'] = []
                        else:
                            action_info['demos'] = []
                        actions.append(action_info)
            except Exception as e:
                print(f"Error reading {action_file}: {e}")
    
    # 按名称排序
    services.sort(key=lambda x: x.get('name', ''))
    topics.sort(key=lambda x: x.get('name', ''))
    actions.sort(key=lambda x: x.get('name', ''))
    
    return {
        'services': services,
        'topics': topics,
        'actions': actions
    }


def load_config(config_file):
    """
    加载配置文件
    
    Args:
        config_file: 配置文件路径
    
    Returns:
        配置字典
    """
    with open(config_file, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def generate_whole_yaml(base_path, output_file, config_file):
    """
    生成完整的 YAML 文件
    
    Args:
        base_path: api_struct/zj_humanoid 目录路径
        output_file: 输出文件路径
        config_file: 配置文件路径
    """
    # 加载配置
    config = load_config(config_file)
    project_config = config.get('project', {})
    
    # 查找所有 YAML 文件
    result = find_yaml_files(base_path)
    
    # 构建输出数据结构
    output_data = {
        'metadata': {
            'generated_at': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'description': project_config.get('description', 'ZJ Humanoid ROS1 APIs'),
            'version': project_config.get('version', 'v1.0.0')
        },
        'services': result['services'],
        'topics': result['topics']
    }
    
    # 只有在有 actions 时才添加 actions 字段
    if result['actions']:
        output_data['actions'] = result['actions']
    
    # 写入 YAML 文件
    with open(output_file, 'w', encoding='utf-8') as f:
        yaml.dump(output_data, f, allow_unicode=True, default_flow_style=False, sort_keys=False)
    
    print(f"Generated {output_file}")
    print(f"  Services: {len(result['services'])}")
    print(f"  Topics: {len(result['topics'])}")
    print(f"  Actions: {len(result['actions'])}")


if __name__ == '__main__':
    # api_struct所在路径
    script_dir = Path(__file__).parent.parent
    
    # 设置路径
    base_path = script_dir / 'zj_humanoid'
    output_file = script_dir / 'generated' / 'zj_humanoid_interfaces.yaml'
    config_file = script_dir / 'config.yaml'
    
    # 确保输出目录存在
    output_file.parent.mkdir(parents=True, exist_ok=True)
    
    # 生成 YAML 文件
    generate_whole_yaml(base_path, output_file, config_file)

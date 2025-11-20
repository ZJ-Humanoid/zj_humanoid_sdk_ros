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


def extract_subsystem(name):
    """
    从接口名称中提取子系统名称
    
    Args:
        name: 接口完整名称，如 /zj_humanoid/audio/LLM_chat
    
    Returns:
        子系统名称，如 audio
    """
    if not name or not name.startswith('/zj_humanoid/'):
        return 'other'
    
    parts = name.split('/')
    if len(parts) >= 3:
        return parts[2]  # /zj_humanoid/audio/... -> audio
    return 'other'


def find_yaml_files(base_path):
    """
    遍历目录，查找所有的 service.yaml、topic.yaml 和 action.yaml 文件
    按子系统分组
    
    Args:
        base_path: 基础路径 (api_struct/zj_humanoid)
    
    Returns:
        包含按子系统分组的字典: services, topics, actions
    """
    services_by_subsystem = {}
    topics_by_subsystem = {}
    actions_by_subsystem = {}
    
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
                        
                        # 提取子系统名称
                        subsystem = extract_subsystem(service_info.get('name', ''))
                        if subsystem not in services_by_subsystem:
                            services_by_subsystem[subsystem] = []
                        
                        # 检查是否已存在相同名称的 service（避免重复）
                        service_name = service_info.get('name', '')
                        existing_service = next(
                            (s for s in services_by_subsystem[subsystem] if s.get('name') == service_name),
                            None
                        )
                        if existing_service:
                            print(f"警告: 发现重复的 service 名称 '{service_name}'")
                            print(f"  已存在: {existing_service.get('description', 'N/A')[:50]}")
                            print(f"  新文件: {service_info.get('description', 'N/A')[:50]} (来自 {service_file})")
                            print(f"  跳过新文件，保留已存在的定义")
                        else:
                            services_by_subsystem[subsystem].append(service_info)
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
                        
                        # 提取子系统名称
                        subsystem = extract_subsystem(topic_info.get('name', ''))
                        if subsystem not in topics_by_subsystem:
                            topics_by_subsystem[subsystem] = []
                        
                        # 检查是否已存在相同名称的 topic（避免重复）
                        topic_name = topic_info.get('name', '')
                        existing_topic = next(
                            (t for t in topics_by_subsystem[subsystem] if t.get('name') == topic_name),
                            None
                        )
                        if existing_topic:
                            print(f"警告: 发现重复的 topic 名称 '{topic_name}'")
                            print(f"  已存在: {existing_topic.get('description', 'N/A')[:50]}")
                            print(f"  新文件: {topic_info.get('description', 'N/A')[:50]} (来自 {topic_file})")
                            print(f"  跳过新文件，保留已存在的定义")
                        else:
                            topics_by_subsystem[subsystem].append(topic_info)
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
                        
                        # 提取子系统名称
                        subsystem = extract_subsystem(action_info.get('name', ''))
                        if subsystem not in actions_by_subsystem:
                            actions_by_subsystem[subsystem] = []
                        actions_by_subsystem[subsystem].append(action_info)
            except Exception as e:
                print(f"Error reading {action_file}: {e}")
    
    # 对每个子系统内的接口按名称排序
    for subsystem in services_by_subsystem:
        services_by_subsystem[subsystem].sort(key=lambda x: x.get('name', ''))
    for subsystem in topics_by_subsystem:
        topics_by_subsystem[subsystem].sort(key=lambda x: x.get('name', ''))
    for subsystem in actions_by_subsystem:
        actions_by_subsystem[subsystem].sort(key=lambda x: x.get('name', ''))
    
    return {
        'services': services_by_subsystem,
        'topics': topics_by_subsystem,
        'actions': actions_by_subsystem
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
    
    # 查找所有 YAML 文件（按子系统分组）
    result = find_yaml_files(base_path)
    
    # 构建输出数据结构（按子系统分组）
    output_data = {
        'metadata': {
            'description': project_config.get('description', 'ZJ Humanoid ROS1 APIs'),
            'version': project_config.get('version', 'v1.0.0')
        }
    }
    
    # 添加 services（按子系统分组）
    if result['services']:
        output_data['services'] = {}
        # 按子系统名称排序
        for subsystem in sorted(result['services'].keys()):
            output_data['services'][subsystem] = result['services'][subsystem]
    
    # 添加 topics（按子系统分组）
    if result['topics']:
        output_data['topics'] = {}
        # 按子系统名称排序
        for subsystem in sorted(result['topics'].keys()):
            output_data['topics'][subsystem] = result['topics'][subsystem]
    
    # 只有在有 actions 时才添加 actions 字段
    if result['actions']:
        output_data['actions'] = {}
        # 按子系统名称排序
        for subsystem in sorted(result['actions'].keys()):
            output_data['actions'][subsystem] = result['actions'][subsystem]
    
    # 写入 YAML 文件
    with open(output_file, 'w', encoding='utf-8') as f:
        yaml.dump(output_data, f, allow_unicode=True, default_flow_style=False, sort_keys=False)
    
    # 统计数量
    total_services = sum(len(v) for v in result['services'].values())
    total_topics = sum(len(v) for v in result['topics'].values())
    total_actions = sum(len(v) for v in result['actions'].values())
    
    print(f"Generated {output_file}")
    print(f"\n按子系统分组统计:")
    print(f"  Subsystems: {len(result['services'])} (services), {len(result['topics'])} (topics)")
    print(f"  Total Services: {total_services}")
    print(f"  Total Topics: {total_topics}")
    print(f"  Total Actions: {total_actions}")
    
    print(f"\n各子系统接口数量:")
    all_subsystems = set(result['services'].keys()) | set(result['topics'].keys()) | set(result['actions'].keys())
    for subsystem in sorted(all_subsystems):
        s_count = len(result['services'].get(subsystem, []))
        t_count = len(result['topics'].get(subsystem, []))
        a_count = len(result['actions'].get(subsystem, []))
        print(f"  {subsystem}: {s_count} services, {t_count} topics, {a_count} actions")


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

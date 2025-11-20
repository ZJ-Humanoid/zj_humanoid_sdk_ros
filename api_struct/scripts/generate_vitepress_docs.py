#!/usr/bin/env python3
"""
Generate VitePress documentation from zj_humanoid_interfaces.yaml

This script generates VitePress-compatible Markdown files for the API documentation.
"""

import os
import json
from pathlib import Path
import yaml
from typing import Dict, List


class VitePressDocGenerator:
    # 子系统图标映射
    SUBSYSTEM_ICONS = {
        'audio': '🔊',
        'hand': '🖐️',
        'lowerlimb': '🦵',
        'manipulation': '🔧',
        'navigation': '🧭',
        'robot': '🤖',
        'sensor': '📷',
        'upperlimb': '🦾',
    }
    
    def __init__(self, yaml_file: str, output_dir: str):
        """
        Initialize the generator.
        
        Args:
            yaml_file: Path to zj_humanoid_interfaces.yaml
            output_dir: Output directory for VitePress docs
        """
        self.yaml_file = Path(yaml_file)
        self.output_dir = Path(output_dir)
        self.data = None
        
    def load_yaml(self) -> bool:
        """Load YAML data."""
        try:
            with open(self.yaml_file, 'r', encoding='utf-8') as f:
                self.data = yaml.safe_load(f)
            return True
        except Exception as e:
            print(f"Error loading YAML file: {e}")
            return False
    
    def escape_markdown(self, text: str) -> str:
        """Escape special characters for Markdown."""
        if not text:
            return ''
        return str(text).replace('|', '\\|').replace('\n', ' ')
    
    def format_type_link(self, type_name: str) -> str:
        """Format type name as a link to zj_humanoid_types.md."""
        if not type_name:
            return ''
        # 只对 zj_humanoid 相关的类型添加链接（排除标准库类型如 std_msgs, std_srvs 等）
        # 检查类型是否属于 zj_humanoid 命名空间
        zj_humanoid_namespaces = ['audio', 'hand', 'lowerlimb', 'manipulation', 'navigation', 
                                  'robot', 'sensor', 'upperlimb']
        type_namespace = type_name.split('/')[0] if '/' in type_name else ''
        
        if type_namespace in zj_humanoid_namespaces:
            # 提取类型名称的最后部分作为锚点（去掉命名空间前缀）
            # 例如：audio/LLMChat -> LLMChat
            type_anchor = type_name.split('/')[-1] if '/' in type_name else type_name
            # VitePress 的锚点格式：对于标题 `#### `BatteryInfo``，锚点通常是全部小写
            # 例如：BatteryInfo -> #batteryinfo
            # 但 VitePress 也可能保持原样，所以先尝试小写格式
            type_anchor_lower = type_anchor.lower()
            # 生成链接：在 VitePress 中，跨文档链接应使用绝对路径（相对于 base）
            # base 是 /navi_sdk_documents/，所以绝对路径是 /zj_humanoid_types
            # 使用绝对路径确保在不同页面都能正确跳转
            return f"[{type_name}](../zj_humanoid_types#{type_anchor_lower})"
        else:
            # 标准库类型或其他类型，不添加链接，直接返回类型名称
            return type_name
    
    def generate_main_page(self):
        """Generate the main API documentation page using component nesting."""
        output_file = self.output_dir / 'zj_humanoid_ros_api.md'
        
        metadata = self.data.get('metadata', {})
        services_data = self.data.get('services', {})
        topics_data = self.data.get('topics', {})
        
        total_services = sum(len(v) for v in services_data.values()) if isinstance(services_data, dict) else len(services_data)
        total_topics = sum(len(v) for v in topics_data.values()) if isinstance(topics_data, dict) else len(topics_data)
        
        with open(output_file, 'w', encoding='utf-8') as f:
            # Frontmatter
            f.write("---\n")
            f.write("layout: doc\n")
            f.write(f"title: ZJ Humanoid ROS API 接口文档\n")
            f.write(f"description: {metadata.get('description', 'ZJ Humanoid ROS1 APIs')}\n")
            f.write(f"version: {metadata.get('version', 'v1.0.0')}\n")
            f.write("---\n\n")
            
            # Title
            f.write("# ZJ Humanoid ROS API 接口文档\n\n")
            
            # Metadata
            f.write("::: info 文档信息\n")
            f.write(f"- **描述**: {metadata.get('description', 'N/A')}\n")
            f.write(f"- **版本**: {metadata.get('version', 'N/A')}\n")
            f.write(f"- **生成时间**: {metadata.get('generated_at', 'N/A')}\n")
            f.write(":::\n\n")
            
            # Statistics
            f.write("## 📊 统计信息\n\n")
            f.write(f"- **Services**: {total_services} 个服务\n")
            f.write(f"- **Topics**: {total_topics} 个话题\n")
            f.write(f"- **总计**: {total_services + total_topics} 个接口\n")
            f.write(f"- **子系统**: {len(services_data)} 个 (services), {len(topics_data)} 个 (topics)\n\n")
            
            # Table of Contents
            f.write("## 📑 目录导航\n\n")
            f.write("本文档包含所有ROS接口的详细信息，您可以通过滚动浏览所有内容，或使用右侧导航快速跳转。\n\n")
            f.write("**Services (服务):**\n")
            for subsystem in sorted(services_data.keys()):
                count = len(services_data[subsystem])
                icon = self.SUBSYSTEM_ICONS.get(subsystem, '📦')
                f.write(f"- [{icon} {subsystem.upper()}](#{subsystem.lower()}-services) ({count} services)\n")
            f.write("\n**Topics (话题):**\n")
            for subsystem in sorted(topics_data.keys()):
                count = len(topics_data[subsystem])
                icon = self.SUBSYSTEM_ICONS.get(subsystem, '📡')
                f.write(f"- [{icon} {subsystem.upper()}](#{subsystem.lower()}-topics) ({count} topics)\n")
            f.write("\n---\n\n")
            
            # Services - 使用组件嵌套各个子系统文档
            if services_data and isinstance(services_data, dict):
                f.write("## 📦 Services\n\n")
                f.write(f"共 {total_services} 个服务，分布在 {len(services_data)} 个子系统中。\n\n")
                
                for subsystem_idx, subsystem in enumerate(sorted(services_data.keys())):
                    services = services_data[subsystem]
                    if not services:
                        continue
                    
                    # 使用组件嵌套子系统文档的Services部分
                    icon = self.SUBSYSTEM_ICONS.get(subsystem, '📦')
                    f.write(f"### {icon} {subsystem.upper()} ({len(services)} services) {{#{subsystem.lower()}-services}}\n\n")
                    f.write(f"<MarkdownInclude src=\"api/{subsystem}.md\" :skip-frontmatter=\"true\" :skip-title=\"true\" section=\"services\" />\n\n")
                    
                    # 只在子系统之间添加分隔线，最后一个不添加
                    if subsystem_idx < len(services_data) - 1:
                        f.write("<div style='margin: 2rem 0; border-top: 2px solid var(--vp-c-divider);'></div>\n\n")
            
            # 添加Services和Topics之间的分隔
            f.write("\n<div style='margin: 3rem 0; padding: 2rem 0; border-top: 3px solid var(--vp-c-brand-1);'></div>\n\n")
            
            # Topics - 使用组件嵌套各个子系统文档
            if topics_data and isinstance(topics_data, dict):
                f.write("## 📡 Topics\n\n")
                f.write(f"共 {total_topics} 个话题，分布在 {len(topics_data)} 个子系统中。\n\n")
                
                for subsystem_idx, subsystem in enumerate(sorted(topics_data.keys())):
                    topics = topics_data[subsystem]
                    if not topics:
                        continue
                    
                    # 使用组件嵌套子系统文档的Topics部分
                    icon = self.SUBSYSTEM_ICONS.get(subsystem, '📡')
                    f.write(f"### {icon} {subsystem.upper()} ({len(topics)} topics) {{#{subsystem.lower()}-topics}}\n\n")
                    f.write(f"<MarkdownInclude src=\"api/{subsystem}.md\" :skip-frontmatter=\"true\" :skip-title=\"true\" section=\"topics\" />\n\n")
                    
                    # 只在子系统之间添加分隔线，最后一个不添加
                    if subsystem_idx < len(topics_data) - 1:
                        f.write("<div style='margin: 2rem 0; border-top: 2px solid var(--vp-c-divider);'></div>\n\n")
        
        print(f"✓ Generated: {output_file}")
    
    def generate_subsystem_pages(self):
        """Generate separate pages for each subsystem."""
        services_data = self.data.get('services', {})
        topics_data = self.data.get('topics', {})
        
        # 子系统文件直接输出到 api 目录（不再使用 subsystems 子目录）
        # subsystem_dir 就是 self.output_dir（即 api 目录）
        subsystem_dir = self.output_dir
        subsystem_dir.mkdir(exist_ok=True)
        
        # Get all subsystems
        all_subsystems = set()
        if isinstance(services_data, dict):
            all_subsystems.update(services_data.keys())
        if isinstance(topics_data, dict):
            all_subsystems.update(topics_data.keys())
        
        for subsystem in sorted(all_subsystems):
            services = services_data.get(subsystem, []) if isinstance(services_data, dict) else []
            topics = topics_data.get(subsystem, []) if isinstance(topics_data, dict) else []
            
            output_file = subsystem_dir / f'{subsystem}.md'
            
            with open(output_file, 'w', encoding='utf-8') as f:
                # Frontmatter
                f.write("---\n")
                f.write(f"title: {subsystem.upper()} 子系统\n")
                f.write(f"description: {subsystem.upper()} 子系统的所有ROS接口\n")
                f.write("---\n\n")
                
                # 添加 Markmap 思维导图
                f.write("## 📊 接口概览\n\n")
                
                # 生成 markmap 内容
                icon = self.SUBSYSTEM_ICONS.get(subsystem, '📦')
                markmap_lines = [f"# {icon} {subsystem.upper()} 子系统"]
                
                if services:
                    markmap_lines.append(f"## 📦 Services ({len(services)})")
                    # 按路径层级组织服务
                    service_groups = {}
                    root_services = []
                    
                    for service in services:
                        name = service.get('name', '')
                        parts = [p for p in name.split('/') if p]  # 移除空字符串
                        
                        if len(parts) >= 4:
                            # 有子路径，如 /zj_humanoid/audio/microphone/get_devices_list
                            # parts: ['zj_humanoid', 'audio', 'microphone', 'get_devices_list']
                            group = parts[2]  # microphone, speaker 等（索引2，因为parts[0]是zj_humanoid）
                            if group not in service_groups:
                                service_groups[group] = []
                            service_groups[group].append(parts[-1])  # 只保存最后一部分
                        else:
                            # 直接在子系统下的服务，如 /zj_humanoid/audio/listen
                            root_services.append(parts[-1])
                    
                    # 先显示根级别的服务
                    for item in root_services[:10]:
                        markmap_lines.append(f"- {item}")
                    if len(root_services) > 10:
                        markmap_lines.append(f"- ... 还有 {len(root_services) - 10} 个")
                    
                    # 再显示分组后的服务，分组名作为列表项，子项使用缩进
                    for group, items in sorted(service_groups.items()):
                        markmap_lines.append(f"- {group}")
                        for item in items[:10]:  # 每组最多显示8个
                            markmap_lines.append(f"  - {item}")  # 使用两个空格缩进表示子项
                        if len(items) > 10:
                            markmap_lines.append(f"  - ... 还有 {len(items) - 8} 个")
                
                if topics:
                    markmap_lines.append(f"## 📡 Topics ({len(topics)})")
                    # 按路径层级组织话题，与 Services 类似
                    topic_groups = {}
                    root_topics = []
                    
                    for topic in topics:
                        name = topic.get('name', '')
                        parts = [p for p in name.split('/') if p]  # 移除空字符串
                        
                        if len(parts) >= 4:
                            # 有子路径，如 /zj_humanoid/hand/finger_pressures/left
                            # parts: ['zj_humanoid', 'hand', 'finger_pressures', 'left']
                            group = parts[2]  # finger_pressures, wrist_force_sensor 等（索引2）
                            if group not in topic_groups:
                                topic_groups[group] = []
                            topic_groups[group].append(parts[-1])  # 只保存最后一部分
                        else:
                            # 直接在子系统下的话题，如 /zj_humanoid/hand/joint_states
                            root_topics.append(parts[-1])
                    
                    # 先显示根级别的话题
                    for item in root_topics[:10]:
                        markmap_lines.append(f"- {item}")
                    if len(root_topics) > 10:
                        markmap_lines.append(f"- ... 还有 {len(root_topics) - 10} 个")
                    
                    # 再显示分组后的话题，分组名作为列表项，子项使用缩进
                    for group, items in sorted(topic_groups.items()):
                        markmap_lines.append(f"- {group}")
                        for item in items[:10]:  # 每组最多显示8个
                            markmap_lines.append(f"  - {item}")  # 使用两个空格缩进表示子项
                        if len(items) > 10:
                            markmap_lines.append(f"  - ... 还有 {len(items) - 8} 个")
                
                markmap_content = '\n'.join(markmap_lines)
                markmap_frontmatter = """---
markmap:
  initialExpandLevel: 3
  colorFreezeLevel: 3
  maxWidth: 200
---

"""
                full_markmap_content = markmap_frontmatter + markmap_content
                
                f.write("<Markmap :content=\"markmapContent\" />\n\n")
                f.write("<script setup>\n")
                # 使用模板字符串，自然分行
                f.write("const markmapContent = `")
                f.write(full_markmap_content)
                f.write("`\n")
                f.write("</script>\n\n")
                f.write("---\n\n")
                
                # 直接开始Services部分，不包含标题和统计信息
                # Services
                if services:
                    f.write(f"## 📦 Services ({len(services)})\n\n")
                    for idx, service in enumerate(services, 1):
                        name = service.get('name', '')
                        # 去掉前两级目录（/zj_humanoid/{subsystem}/），保留后续部分
                        # 例如：/zj_humanoid/hand/finger_pressures/left/zero -> finger_pressures/left/zero
                        parts = [p for p in name.split('/') if p]  # 移除空字符串
                        if len(parts) >= 3:
                            # 去掉前两个部分（zj_humanoid 和子系统名），保留剩余部分
                            short_name = '/'.join(parts[2:])
                        else:
                            # 如果路径太短，只保留最后一部分
                            short_name = parts[-1] if parts else name
                        srv_type = service.get('type', '')
                        description = self.escape_markdown(service.get('description', ''))
                        note = self.escape_markdown(service.get('note', ''))
                        
                        f.write(f"### {idx}. `{short_name}`\n\n")
                        f.write("| 字段 | 值 |\n")
                        f.write("|------|-----|\n")
                        f.write(f"| **Service Name** | {name} |\n")
                        f.write(f"| **Type** | {self.format_type_link(srv_type)} |\n")
                        f.write(f"| **Description** | {description} |\n")
                        if note:
                            f.write(f"| **Note** | {note} |\n")
                        f.write("\n")
                
                # Topics
                if topics:
                    f.write(f"## 📡 Topics ({len(topics)})\n\n")
                    for idx, topic in enumerate(topics, 1):
                        name = topic.get('name', '')
                        # 去掉前两级目录（/zj_humanoid/{subsystem}/），保留后续部分
                        # 例如：/zj_humanoid/hand/finger_pressures/left -> finger_pressures/left
                        parts = [p for p in name.split('/') if p]  # 移除空字符串
                        if len(parts) >= 3:
                            # 去掉前两个部分（zj_humanoid 和子系统名），保留剩余部分
                            short_name = '/'.join(parts[2:])
                        else:
                            # 如果路径太短，只保留最后一部分
                            short_name = parts[-1] if parts else name
                        msg_type = topic.get('type', '')
                        direction = topic.get('direction', '')
                        description = self.escape_markdown(topic.get('description', ''))
                        note = self.escape_markdown(topic.get('note', ''))
                        
                        direction_icon = ""
                        if direction == "publish":
                            direction_icon = "📤 Publish"
                        elif direction == "subscribe":
                            direction_icon = "📥 Subscribe"
                        else:
                            direction_icon = direction
                        
                        f.write(f"### {idx}. `{short_name}`\n\n")
                        f.write("| 字段 | 值 |\n")
                        f.write("|------|-----|\n")
                        f.write(f"| **Topic Name** | {name} |\n")
                        f.write(f"| **Type** | {self.format_type_link(msg_type)} |\n")
                        f.write(f"| **Direction** | {direction_icon} |\n")
                        f.write(f"| **Description** | {description} |\n")
                        if note:
                            f.write(f"| **Note** | {note} |\n")
                        f.write("\n")
            
            print(f"✓ Generated: {output_file}")
    
    def generate_all(self):
        """Generate all documentation."""
        if not self.load_yaml():
            print("Failed to load YAML file!")
            return
        
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        print("=" * 60)
        print("Generating VitePress documentation...")
        print("=" * 60)
        print(f"Source: {self.yaml_file}")
        print(f"Output: {self.output_dir}")
        print()
        
        self.generate_main_page()
        self.generate_subsystem_pages()
        
        print()
        print("=" * 60)
        print("Generation complete!")
        print("=" * 60)


def main():
    script_dir = Path(__file__).parent.parent
    yaml_file = script_dir / 'generated' / 'zj_humanoid_interfaces.yaml'
    output_dir = script_dir.parent / 'docs' / 'api'
    
    generator = VitePressDocGenerator(str(yaml_file), str(output_dir))
    generator.generate_all()


if __name__ == '__main__':
    main()


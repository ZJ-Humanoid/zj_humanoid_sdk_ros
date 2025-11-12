#!/usr/bin/env python3
"""
Generate JSON, Markdown and HTML documentation from zj_humanoid_interfaces.yaml

This script reads the aggregated interface YAML file (grouped by subsystems) and generates:
1. zj_humanoid_interfaces.json - JSON format
2. zj_humanoid_interfaces.md - Markdown format with tables
3. zj_humanoid_interfaces.html - HTML format with styled tables

Usage:
    python3 generate_html_from_yaml.py
"""

import os
from pathlib import Path
import yaml
import json
from datetime import datetime
from typing import Dict, List


class InterfaceGenerator:
    def __init__(self, yaml_file: str = None):
        """Initialize with YAML file path."""
        if yaml_file is None:
            # Script is in api_struct/scripts/, YAML is in api_struct/generated/
            script_dir = Path(__file__).parent
            yaml_path = script_dir.parent / 'generated' / 'zj_humanoid_interfaces.yaml'
            self.yaml_file = str(yaml_path)
        else:
            self.yaml_file = yaml_file
        
        self.data = None
        self.output_dir = Path(__file__).parent.parent / 'generated'
    
    def load_yaml(self) -> bool:
        """Load data from YAML file."""
        try:
            with open(self.yaml_file, 'r', encoding='utf-8') as f:
                self.data = yaml.safe_load(f)
            return True
        except Exception as e:
            print(f"Error loading YAML file: {e}")
            return False
    
    def flatten_interfaces(self, interfaces_by_subsystem: Dict) -> List:
        """
        Flatten interfaces grouped by subsystem into a single list.
        
        Args:
            interfaces_by_subsystem: Dictionary with subsystem as key and list of interfaces as value
        
        Returns:
            Flat list of all interfaces with subsystem information added
        """
        flat_list = []
        if isinstance(interfaces_by_subsystem, dict):
            for subsystem, interfaces in interfaces_by_subsystem.items():
                for interface in interfaces:
                    # Add subsystem info to each interface
                    interface_with_subsystem = interface.copy()
                    interface_with_subsystem['subsystem'] = subsystem
                    flat_list.append(interface_with_subsystem)
        elif isinstance(interfaces_by_subsystem, list):
            # Already flat (old format)
            flat_list = interfaces_by_subsystem
        return flat_list
    
    def save_to_json(self, output_file: str = None) -> None:
        """Save data to JSON file."""
        if output_file is None:
            output_file = str(self.output_dir / 'zj_humanoid_interfaces.json')
        
        try:
            with open(output_file, 'w', encoding='utf-8') as f:
                json.dump(self.data, f, ensure_ascii=False, indent=2)
            print(f"✓ Generated: {output_file}")
        except Exception as e:
            print(f"Error writing JSON file: {e}")
    
    def save_to_markdown(self, output_file: str = None) -> None:
        """Save data to Markdown file with each interface as a separate table."""
        if output_file is None:
            output_file = str(self.output_dir / 'zj_humanoid_interfaces.md')
        
        try:
            with open(output_file, 'w', encoding='utf-8') as f:
                # Write header
                f.write("# ZJ Humanoid ROS API 接口文档\n\n")
                
                # Write metadata
                metadata = self.data.get('metadata', {})
                f.write(f"**Description**: {metadata.get('description', 'N/A')}\n")
                f.write(f"**Version**: {metadata.get('version', 'N/A')}\n")
                f.write(f"**Generated At**: {metadata.get('generated_at', 'N/A')}\n\n")
                
                # Write Services grouped by subsystem
                services_data = self.data.get('services', {})
                if services_data and isinstance(services_data, dict):
                    total_services = sum(len(v) for v in services_data.values())
                    f.write("## Services\n\n")
                    f.write(f"Total: {total_services} services in {len(services_data)} subsystems\n\n")
                    f.write("---\n\n")
                    
                    # Iterate through subsystems in sorted order
                    for subsystem in sorted(services_data.keys()):
                        services = services_data[subsystem]
                        if not services:
                            continue
                        
                        # Subsystem header
                        f.write(f"## 📦 {subsystem.upper()} ({len(services)} services)\n\n")
                        
                        for idx, service in enumerate(services, 1):
                            name = service.get('name', '')
                            srv_type = service.get('type', '')
                            description = service.get('description', '')
                            note = service.get('note', '')
                            
                            # Service header
                            f.write(f"### {subsystem}.{idx}. {name}\n\n")
                            
                            # Service table
                            f.write("| Field | Value |\n")
                            f.write("|-------|-------|\n")
                            f.write(f"| **Service Name** | `{name}` |\n")
                            f.write(f"| **Type** | `{srv_type}` |\n")
                            f.write(f"| **Description** | {description} |\n")
                            if note:
                                f.write(f"| **Note** | {note} |\n")
                            
                            f.write("\n")
                
                # Write Topics grouped by subsystem
                topics_data = self.data.get('topics', {})
                if topics_data and isinstance(topics_data, dict):
                    total_topics = sum(len(v) for v in topics_data.values())
                    f.write("## Topics\n\n")
                    f.write(f"Total: {total_topics} topics in {len(topics_data)} subsystems\n\n")
                    f.write("---\n\n")
                    
                    # Iterate through subsystems in sorted order
                    for subsystem in sorted(topics_data.keys()):
                        topics = topics_data[subsystem]
                        if not topics:
                            continue
                        
                        # Subsystem header
                        f.write(f"## 📡 {subsystem.upper()} ({len(topics)} topics)\n\n")
                        
                        for idx, topic in enumerate(topics, 1):
                            name = topic.get('name', '')
                            msg_type = topic.get('type', '')
                            direction = topic.get('direction', '')
                            description = topic.get('description', '')
                            note = topic.get('note', '')
                            
                            # Add emoji for direction
                            direction_icon = ""
                            if direction == "publish":
                                direction_icon = "📤 Publish"
                            elif direction == "subscribe":
                                direction_icon = "📥 Subscribe"
                            else:
                                direction_icon = direction
                            
                            # Topic header
                            f.write(f"### {subsystem}.{idx}. {name}\n\n")
                            
                            # Topic table
                            f.write("| Field | Value |\n")
                            f.write("|-------|-------|\n")
                            f.write(f"| **Topic Name** | `{name}` |\n")
                            f.write(f"| **Type** | `{msg_type}` |\n")
                            f.write(f"| **Direction** | {direction_icon} |\n")
                            f.write(f"| **Description** | {description} |\n")
                            if note:
                                f.write(f"| **Note** | {note} |\n")
                            
                            f.write("\n")
                
                # Write summary statistics
                f.write("---\n\n")
                f.write("## Summary\n\n")
                f.write(f"- **Total Services**: {total_services}\n")
                f.write(f"- **Total Topics**: {total_topics}\n")
                f.write(f"- **Total Interfaces**: {total_services + total_topics}\n")
                f.write(f"- **Subsystems**: {len(services_data)} (services), {len(topics_data)} (topics)\n")
            
            print(f"✓ Generated: {output_file}")
        
        except Exception as e:
            print(f"Error writing Markdown file: {e}")
    
    def save_to_html(self, output_file: str = None) -> None:
        """Save data to HTML file with styled tables."""
        if output_file is None:
            output_file = str(self.output_dir / 'zj_humanoid_interfaces.html')
        
        try:
            with open(output_file, 'w', encoding='utf-8') as f:
                # Write HTML header with styles
                f.write("""<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ZJ Humanoid ROS API 接口文档</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }
        
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', 'Noto Sans SC', 'Microsoft YaHei', sans-serif;
            line-height: 1.6;
            color: #333;
            background: #f5f5f5;
            padding: 20px;
        }
        
        .container {
            max-width: 1200px;
            margin: 0 auto;
            background: white;
            padding: 40px;
            border-radius: 8px;
            box-shadow: 0 2px 8px rgba(0,0,0,0.1);
        }
        
        h1 {
            color: #2c3e50;
            border-bottom: 3px solid #3498db;
            padding-bottom: 15px;
            margin-bottom: 30px;
            font-size: 2.5em;
        }
        
        h2 {
            color: #34495e;
            margin-top: 40px;
            margin-bottom: 20px;
            padding-bottom: 10px;
            border-bottom: 2px solid #ecf0f1;
            font-size: 2em;
        }
        
        h3 {
            color: #2980b9;
            margin-top: 30px;
            margin-bottom: 15px;
            font-size: 1.3em;
            padding: 10px;
            background: #f8f9fa;
            border-left: 4px solid #3498db;
        }
        
        .metadata {
            background: #ecf0f1;
            padding: 15px 20px;
            border-radius: 5px;
            margin-bottom: 30px;
            font-size: 0.95em;
        }
        
        .metadata p {
            margin: 5px 0;
        }
        
        .metadata strong {
            color: #2c3e50;
            min-width: 120px;
            display: inline-block;
        }
        
        .total-count {
            background: #e8f4f8;
            padding: 10px 15px;
            border-radius: 5px;
            margin-bottom: 20px;
            color: #2980b9;
            font-weight: bold;
        }
        
        table {
            width: 100%;
            border-collapse: collapse;
            margin: 15px 0 30px 0;
            background: white;
            box-shadow: 0 1px 3px rgba(0,0,0,0.1);
            border-radius: 5px;
            overflow: hidden;
        }
        
        th {
            background: #95a5a6;
            color: white;
            padding: 12px 15px;
            text-align: left;
            font-weight: 600;
            font-size: 0.9em;
            text-transform: uppercase;
            letter-spacing: 0.5px;
        }
        
        td {
            padding: 12px 15px;
            border-bottom: 1px solid #ecf0f1;
        }
        
        tr:last-child td {
            border-bottom: none;
        }
        
        tr:hover {
            background: #f8f9fa;
        }
        
        td:first-child {
            font-weight: 600;
            color: #555;
            width: 30%;
            background: #fafafa;
        }
        
        code {
            background: #f4f4f4;
            padding: 2px 6px;
            border-radius: 3px;
            font-family: 'Consolas', 'Monaco', 'Courier New', monospace;
            color: #e74c3c;
            font-size: 0.9em;
        }
        
        .direction-publish {
            color: #27ae60;
            font-weight: 600;
        }
        
        .direction-subscribe {
            color: #2980b9;
            font-weight: 600;
        }
        
        .summary {
            background: #e8f8f5;
            padding: 20px;
            border-radius: 5px;
            margin-top: 40px;
            border-left: 4px solid #27ae60;
        }
        
        .summary h2 {
            margin-top: 0;
            color: #27ae60;
            border: none;
        }
        
        .summary ul {
            list-style: none;
            padding-left: 0;
        }
        
        .summary li {
            padding: 8px 0;
            font-size: 1.1em;
        }
        
        .summary strong {
            color: #2c3e50;
        }
        
        hr {
            border: none;
            border-top: 2px solid #ecf0f1;
            margin: 30px 0;
        }
        
        @media print {
            body {
                background: white;
                padding: 0;
            }
            .container {
                box-shadow: none;
                padding: 20px;
            }
        }
    </style>
</head>
<body>
    <div class="container">
""")
                
                # Write title
                f.write("        <h1>ZJ Humanoid ROS API 接口文档</h1>\n\n")
                
                # Write metadata
                metadata = self.data.get('metadata', {})
                f.write("        <div class=\"metadata\">\n")
                f.write(f"            <p><strong>Description:</strong> {metadata.get('description', 'N/A')}</p>\n")
                f.write(f"            <p><strong>Version:</strong> {metadata.get('version', 'N/A')}</p>\n")
                f.write(f"            <p><strong>Generated At:</strong> {metadata.get('generated_at', 'N/A')}</p>\n")
                f.write("        </div>\n\n")
                
                # Write Services grouped by subsystem
                services_data = self.data.get('services', {})
                if services_data and isinstance(services_data, dict):
                    total_services = sum(len(v) for v in services_data.values())
                    f.write("        <h2>Services</h2>\n")
                    f.write(f"        <div class=\"total-count\">Total: {total_services} services in {len(services_data)} subsystems</div>\n\n")
                    
                    # Iterate through subsystems in sorted order
                    for subsystem in sorted(services_data.keys()):
                        services = services_data[subsystem]
                        if not services:
                            continue
                        
                        # Subsystem header
                        f.write(f"        <h2 style=\"color: #e67e22; border-left: 5px solid #e67e22; padding-left: 15px; margin-top: 40px;\">📦 {subsystem.upper()} ({len(services)} services)</h2>\n\n")
                        
                        for idx, service in enumerate(services, 1):
                            name = service.get('name', '').replace('<', '&lt;').replace('>', '&gt;')
                            srv_type = service.get('type', '').replace('<', '&lt;').replace('>', '&gt;')
                            description = service.get('description', '').replace('<', '&lt;').replace('>', '&gt;')
                            note = service.get('note', '').replace('<', '&lt;').replace('>', '&gt;')
                            
                            f.write(f"        <h3>{subsystem}.{idx}. {name}</h3>\n")
                            f.write("        <table>\n")
                            f.write("            <thead>\n")
                            f.write("                <tr><th>Field</th><th>Value</th></tr>\n")
                            f.write("            </thead>\n")
                            f.write("            <tbody>\n")
                            f.write(f"                <tr><td>Service Name</td><td><code>{name}</code></td></tr>\n")
                            f.write(f"                <tr><td>Type</td><td><code>{srv_type}</code></td></tr>\n")
                            f.write(f"                <tr><td>Description</td><td>{description}</td></tr>\n")
                            if note:
                                f.write(f"                <tr><td>Note</td><td>{note}</td></tr>\n")
                            f.write("            </tbody>\n")
                            f.write("        </table>\n\n")
                
                # Write Topics grouped by subsystem
                topics_data = self.data.get('topics', {})
                if topics_data and isinstance(topics_data, dict):
                    total_topics = sum(len(v) for v in topics_data.values())
                    f.write("        <h2>Topics</h2>\n")
                    f.write(f"        <div class=\"total-count\">Total: {total_topics} topics in {len(topics_data)} subsystems</div>\n\n")
                    
                    # Iterate through subsystems in sorted order
                    for subsystem in sorted(topics_data.keys()):
                        topics = topics_data[subsystem]
                        if not topics:
                            continue
                        
                        # Subsystem header
                        f.write(f"        <h2 style=\"color: #3498db; border-left: 5px solid #3498db; padding-left: 15px; margin-top: 40px;\">📡 {subsystem.upper()} ({len(topics)} topics)</h2>\n\n")
                        
                        for idx, topic in enumerate(topics, 1):
                            name = topic.get('name', '').replace('<', '&lt;').replace('>', '&gt;')
                            msg_type = topic.get('type', '').replace('<', '&lt;').replace('>', '&gt;')
                            direction = topic.get('direction', '')
                            description = topic.get('description', '').replace('<', '&lt;').replace('>', '&gt;')
                            note = topic.get('note', '').replace('<', '&lt;').replace('>', '&gt;')
                            
                            # Direction with icon and class
                            direction_html = ""
                            if direction == "publish":
                                direction_html = '<span class="direction-publish">📤 Publish</span>'
                            elif direction == "subscribe":
                                direction_html = '<span class="direction-subscribe">📥 Subscribe</span>'
                            else:
                                direction_html = direction
                            
                            f.write(f"        <h3>{subsystem}.{idx}. {name}</h3>\n")
                            f.write("        <table>\n")
                            f.write("            <thead>\n")
                            f.write("                <tr><th>Field</th><th>Value</th></tr>\n")
                            f.write("            </thead>\n")
                            f.write("            <tbody>\n")
                            f.write(f"                <tr><td>Topic Name</td><td><code>{name}</code></td></tr>\n")
                            f.write(f"                <tr><td>Type</td><td><code>{msg_type}</code></td></tr>\n")
                            f.write(f"                <tr><td>Direction</td><td>{direction_html}</td></tr>\n")
                            f.write(f"                <tr><td>Description</td><td>{description}</td></tr>\n")
                            if note:
                                f.write(f"                <tr><td>Note</td><td>{note}</td></tr>\n")
                            f.write("            </tbody>\n")
                            f.write("        </table>\n\n")
                
                # Write summary
                f.write("        <hr>\n\n")
                f.write("        <div class=\"summary\">\n")
                f.write("            <h2>Summary</h2>\n")
                f.write("            <ul>\n")
                f.write(f"                <li><strong>Total Services:</strong> {total_services}</li>\n")
                f.write(f"                <li><strong>Total Topics:</strong> {total_topics}</li>\n")
                f.write(f"                <li><strong>Total Interfaces:</strong> {total_services + total_topics}</li>\n")
                f.write(f"                <li><strong>Subsystems:</strong> {len(services_data)} (services), {len(topics_data)} (topics)</li>\n")
                f.write("            </ul>\n")
                f.write("        </div>\n\n")
                
                # Close HTML
                f.write("    </div>\n")
                f.write("</body>\n")
                f.write("</html>\n")
            
            print(f"✓ Generated: {output_file}")
        
        except Exception as e:
            print(f"Error writing HTML file: {e}")
    
    def generate_all(self) -> None:
        """Generate JSON, Markdown and HTML files."""
        if not self.load_yaml():
            print("Failed to load YAML file!")
            return
        
        print("=" * 60)
        print("Generating interface documentation...")
        print("=" * 60)
        print(f"Source: {self.yaml_file}")
        print()
        
        self.save_to_json()
        self.save_to_markdown()
        self.save_to_html()
        
        print()
        print("=" * 60)
        print("Generation complete!")
        print("=" * 60)


def main():
    generator = InterfaceGenerator()
    generator.generate_all()


if __name__ == '__main__':
    main()


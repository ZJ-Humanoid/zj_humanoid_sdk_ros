#!/usr/bin/env python3
"""
ZJ Humanoid ROS Interfaces Generator

This script reads ../zj_humanoid_interfaces.yaml and generates:
- zj_humanoid_interfaces.json: JSON format for programmatic access
- zj_humanoid_interfaces.md: Markdown format for human-readable documentation

File Structure:
    api_struct/
    ├── zj_humanoid_interfaces.yaml (input file)
    └── generated/
        ├── generate_json_from_yaml.py (this script)
        ├── zj_humanoid_interfaces.json (output)
        └── zj_humanoid_interfaces.md (output)

Usage:
    cd generated/
    python3 generate_json_from_yaml.py

Requirements:
    - Python 3.6+
    - PyYAML library (pip install PyYAML)
"""

import yaml
import json
import os
from datetime import datetime
from typing import Dict, Any


class InterfaceGenerator:
    """Generate JSON and Markdown files from YAML interface data."""
    
    def __init__(self, yaml_file: str = None):
        """
        Initialize the generator.
        
        Args:
            yaml_file: Path to the input YAML file. If None, will auto-detect.
        """
        if yaml_file is None:
            # Auto-detect the YAML file path
            if os.path.exists("../zj_humanoid_interfaces.yaml"):
                self.yaml_file = "../zj_humanoid_interfaces.yaml"
            elif os.path.exists("zj_humanoid_interfaces.yaml"):
                self.yaml_file = "zj_humanoid_interfaces.yaml"
            else:
                self.yaml_file = "zj_humanoid_interfaces.yaml"  # Default fallback
        else:
            self.yaml_file = yaml_file
        self.data = None
    
    def load_yaml_data(self) -> bool:
        """
        Load data from YAML file.
        
        Returns:
            True if successful, False otherwise
        """
        try:
            if not os.path.exists(self.yaml_file):
                print(f"Error: YAML file {self.yaml_file} not found!")
                return False
            
            with open(self.yaml_file, 'r', encoding='utf-8') as f:
                self.data = yaml.safe_load(f)
            
            print(f"Successfully loaded data from {self.yaml_file}")
            return True
            
        except yaml.YAMLError as e:
            print(f"Error parsing YAML file {self.yaml_file}: {e}")
            return False
        except Exception as e:
            print(f"Error reading file {self.yaml_file}: {e}")
            return False
    
    def save_to_json(self, output_file: str = None) -> None:
        """
        Save data to JSON files based on applicable_robot_models.
        
        Args:
            output_file: Output JSON file path (deprecated, kept for compatibility).
        """
        # Define robot models
        robot_models = ['H1', 'rx', 'wa1', 'wa2']
        
        # Create a dictionary to hold data for each robot model
        robot_data = {}
        for model in robot_models:
            robot_data[model] = {
                'metadata': self.data['metadata'].copy(),
                'services': [],
                'topics': []
            }
        
        # Filter services based on applicable_robot_models
        for service in self.data.get('services', []):
            applicable_models = service.get('applicable_robot_models', [])
            for model in robot_models:
                if model in applicable_models:
                    robot_data[model]['services'].append(service)
        
        # Filter topics based on applicable_robot_models
        for topic in self.data.get('topics', []):
            applicable_models = topic.get('applicable_robot_models', [])
            for model in robot_models:
                if model in applicable_models:
                    robot_data[model]['topics'].append(topic)
        
        # Save a JSON file for each robot model
        try:
            for model in robot_models:
                output_filename = f"zj_humanoid_interfaces_{model}.json"
                with open(output_filename, 'w', encoding='utf-8') as f:
                    json.dump(robot_data[model], f, ensure_ascii=False, indent=4)
                print(f"Successfully saved to {output_filename}")
                print(f"  - Services: {len(robot_data[model]['services'])}")
                print(f"  - Topics: {len(robot_data[model]['topics'])}")
        except Exception as e:
            print(f"Error saving to JSON files: {e}")
    
    def save_to_markdown(self, output_file: str = None) -> None:
        """
        Save data to Markdown file.
        
        Args:
            output_file: Output Markdown file path. If None, will save to generated/ directory.
        """
        if output_file is None:
            # Save to generated/ directory
            if os.path.exists("generated/"):
                output_file = "generated/zj_humanoid_interfaces.md"
            else:
                output_file = "zj_humanoid_interfaces.md"
        try:
            with open(output_file, 'w', encoding='utf-8') as f:
                # Write header
                f.write("# ZJ Humanoid ROS API 接口文档\n\n")
                f.write(f"**Description**: {self.data['metadata']['description']}\n")
                f.write(f"**Version**: {self.data['metadata']['version']}\n")
                f.write(f"**Generated At**: {self.data['metadata']['generated_at']}\n\n")
                
                # Write services section
                f.write("## Services\n\n")
                for service in self.data['services']:
                    f.write(f"### {service.get('name', '')}\n\n")
                    f.write(f"**Description**: {service.get('description', '')}\n\n")
                    f.write(f"**Type**: `{service.get('type', '')}`\n\n")
                    if service.get('note'):
                        f.write(f"**Note**: {service.get('note', '')}\n\n")
                    if service.get('demos'):
                        f.write(f"**Demos**: {', '.join(service.get('demos', []))}\n\n")
                    f.write("---\n\n")
                
                # Write topics section
                f.write("## Topics\n\n")
                for topic in self.data['topics']:
                    f.write(f"### {topic.get('name', '')}\n\n")
                    f.write(f"**Description**: {topic.get('description', '')}\n\n")
                    f.write(f"**Type**: `{topic.get('type', '')}`\n\n")
                    f.write(f"**Direction**: {topic.get('direction', '')}\n\n")
                    if topic.get('throttle_rate'):
                        f.write(f"**Throttle Rate**: {topic.get('throttle_rate')} Hz\n\n")
                    if topic.get('note'):
                        f.write(f"**Note**: {topic.get('note', '')}\n\n")
                    if topic.get('demos'):
                        f.write(f"**Demos**: {', '.join(topic.get('demos', []))}\n\n")
                    f.write("---\n\n")
                
            print(f"Successfully saved to {output_file}")
        except Exception as e:
            print(f"Error saving to Markdown file {output_file}: {e}")
    
    def generate_files(self) -> None:
        """Generate JSON and Markdown files from YAML data."""
        if not self.load_yaml_data():
            return
        
        if not self.data:
            print("No data to process!")
            return
        
        # Update metadata timestamp
        self.data['metadata']['generated_at'] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
        # Generate files
        print("\nGenerating JSON files for each robot model...")
        self.save_to_json()
        
        print("\nGenerating Markdown documentation...")
        self.save_to_markdown()
        
        print("\nGeneration completed successfully!")


def main():
    """Main function to run the interface generator."""
    print("ZJ Humanoid ROS Interfaces Generator")
    print("=" * 50)
    
    generator = InterfaceGenerator()
    generator.generate_files()


if __name__ == "__main__":
    main()

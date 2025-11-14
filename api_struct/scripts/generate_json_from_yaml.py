#!/usr/bin/env python3
"""
ZJ Humanoid ROS Interfaces Generator

This script reads zj_humanoid_interfaces.yaml from generated/ and generates:
- zj_humanoid_interfaces_*.json: JSON format for each robot model
- zj_humanoid_interfaces.md: Markdown format for human-readable documentation

File Structure:
    api_struct/
    ├── scripts/
    │   └── generate_json_from_yaml.py (this script)
    └── generated/
        ├── zj_humanoid_interfaces.yaml (input file)
        ├── zj_humanoid_interfaces_H1.json (output)
        ├── zj_humanoid_interfaces_I2.json (output)
        ├── zj_humanoid_interfaces_WA1.json (output)
        ├── zj_humanoid_interfaces_WA2.json (output)
        └── zj_humanoid_interfaces.md (output)

Usage:
    cd api_struct/scripts/
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
from pathlib import Path


class InterfaceGenerator:
    """Generate JSON and Markdown files from YAML interface data."""
    
    def __init__(self, yaml_file: str = None):
        """
        Initialize the generator.
        
        Args:
            yaml_file: Path to the input YAML file. If None, will auto-detect.
        """
        if yaml_file is None:
            # Auto-detect the YAML file path from scripts/ directory
            script_dir = Path(__file__).parent.parent
            yaml_path = script_dir / 'generated' / 'zj_humanoid_interfaces.yaml'
            self.yaml_file = str(yaml_path)
            self.output_dir = script_dir / 'generated'
            self.config_file = script_dir / 'config.yaml'
        else:
            self.yaml_file = yaml_file
            self.output_dir = Path(yaml_file).parent
            self.config_file = Path(yaml_file).parent.parent / 'config.yaml'
        self.data = None
        self.config = None
    
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
    
    def load_config(self) -> bool:
        """
        Load configuration from config.yaml file.
        
        Returns:
            True if successful, False otherwise
        """
        try:
            if not os.path.exists(self.config_file):
                print(f"Warning: Config file {self.config_file} not found! Using default configuration.")
                # Use default configuration
                self.config = {
                    'robot_models': {
                        'H1': {'exclude_modules': []},
                        'I2': {'exclude_modules': []},
                        'WA1': {'exclude_modules': []},
                        'WA2': {'exclude_modules': []}
                    }
                }
                return True
            
            with open(self.config_file, 'r', encoding='utf-8') as f:
                self.config = yaml.safe_load(f)
            
            print(f"Successfully loaded config from {self.config_file}")
            return True
            
        except yaml.YAMLError as e:
            print(f"Error parsing config file {self.config_file}: {e}")
            return False
        except Exception as e:
            print(f"Error reading config file {self.config_file}: {e}")
            return False
    
    def get_module_from_name(self, interface_name: str) -> str:
        """
        Extract module name from interface name.
        
        Args:
            interface_name: Full interface name (e.g., /zj_humanoid/audio/asr_text)
        
        Returns:
            Module name (e.g., audio) or empty string if not found
        """
        if not interface_name:
            return ''
        
        # Remove leading slash
        name = interface_name.lstrip('/')
        
        # Split by slash and get the parts
        # Expected format: zj_humanoid/module/...
        parts = name.split('/')
        if len(parts) >= 2:
            # Return the second part (module name)
            return parts[1]
        
        return ''
    
    def should_include_interface(self, interface_name: str, robot_model: str) -> bool:
        """
        Check if an interface should be included for a specific robot model.
        
        Args:
            interface_name: Full interface name
            robot_model: Robot model name (H1, I2, WA1, WA2)
        
        Returns:
            True if the interface should be included, False otherwise
        """
        module = self.get_module_from_name(interface_name)
        
        if not module:
            return True
        
        # Get exclude modules for this robot model
        robot_config = self.config.get('robot_models', {}).get(robot_model, {})
        exclude_modules = robot_config.get('exclude_modules', [])
        
        # Include if module is not in exclude list
        return module not in exclude_modules
    
    def save_to_json(self, output_file: str = None) -> None:
        """
        Save data to JSON files based on config.yaml robot models and exclude rules.
        
        Args:
            output_file: Output JSON file path (deprecated, kept for compatibility).
        """
        # Get robot models from config
        robot_models = list(self.config.get('robot_models', {}).keys())
        
        if not robot_models:
            print("Warning: No robot models found in config! Using defaults.")
            robot_models = ['H1', 'I2', 'WA1', 'WA2']
        
        # Create a dictionary to hold data for each robot model
        robot_data = {}
        for model in robot_models:
            robot_data[model] = {
                'metadata': self.data['metadata'].copy(),
                'services': [],
                'topics': []
            }
            
        # Filter services based on module exclusion rules
        # Handle nested structure: services -> module -> list of services
        services_data = self.data.get('services', {})
        if isinstance(services_data, dict):
            for module, service_list in services_data.items():
                if isinstance(service_list, list):
                    for service in service_list:
                        if isinstance(service, dict):
                            service_name = service.get('name', '')
                            for model in robot_models:
                                if self.should_include_interface(service_name, model):
                                    robot_data[model]['services'].append(service)
        
        # Filter topics based on module exclusion rules
        # Handle nested structure: topics -> module -> list of topics
        topics_data = self.data.get('topics', {})
        if isinstance(topics_data, dict):
            for module, topic_list in topics_data.items():
                if isinstance(topic_list, list):
                    for topic in topic_list:
                        if isinstance(topic, dict):
                            topic_name = topic.get('name', '')
                            for model in robot_models:
                                if self.should_include_interface(topic_name, model):
                                    robot_data[model]['topics'].append(topic)
        
        # Save a JSON file for each robot model
        try:
            # Ensure output directory exists
            self.output_dir.mkdir(parents=True, exist_ok=True)
            
            for model in robot_models:
                output_filename = self.output_dir / f"zj_humanoid_interfaces_{model}.json"
                with open(output_filename, 'w', encoding='utf-8') as f:
                    json.dump(robot_data[model], f, ensure_ascii=False, indent=4)
                print(f"Successfully saved to {output_filename}")
                print(f"  - Services: {len(robot_data[model]['services'])}")
                print(f"  - Topics: {len(robot_data[model]['topics'])}")
                
                # Print excluded modules info
                exclude_modules = self.config.get('robot_models', {}).get(model, {}).get('exclude_modules', [])
                if exclude_modules:
                    print(f"  - Excluded modules: {', '.join(exclude_modules)}")
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
            output_file = self.output_dir / "zj_humanoid_interfaces.md"
        else:
            output_file = Path(output_file)
        
        try:
            # Ensure output directory exists
            output_file.parent.mkdir(parents=True, exist_ok=True)
            with open(output_file, 'w', encoding='utf-8') as f:
                # Write header
                f.write("# ZJ Humanoid ROS API 接口文档\n\n")
                f.write(f"**Description**: {self.data['metadata']['description']}\n")
                f.write(f"**Version**: {self.data['metadata']['version']}\n")
                f.write(f"**Generated At**: {self.data['metadata']['generated_at']}\n\n")
                
                # Write services section
                f.write("## Services\n\n")
                services_data = self.data.get('services', {})
                if isinstance(services_data, dict):
                    for module, service_list in services_data.items():
                        if isinstance(service_list, list):
                            for service in service_list:
                                if isinstance(service, dict):
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
                topics_data = self.data.get('topics', {})
                if isinstance(topics_data, dict):
                    for module, topic_list in topics_data.items():
                        if isinstance(topic_list, list):
                            for topic in topic_list:
                                if isinstance(topic, dict):
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
        # Load config first
        if not self.load_config():
            print("Failed to load config! Aborting.")
            return
        
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
        
        # print("\nGenerating Markdown documentation...")
        # self.save_to_markdown()
        
        print("\nGeneration completed successfully!")


def main():
    """Main function to run the interface generator."""
    print("ZJ Humanoid ROS Interfaces Generator")
    print("=" * 50)
    
    generator = InterfaceGenerator()
    generator.generate_files()


if __name__ == "__main__":
    main()
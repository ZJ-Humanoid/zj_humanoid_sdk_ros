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
        Extract module name or module path from interface name.
        
        Supports both coarse-grained and fine-grained module extraction:
        - Coarse: /zj_humanoid/audio/asr_text → 'audio'
        - Fine: /zj_humanoid/sensor/CAM_A/camera_info → 'sensor/CAM_A'
        - Fine: /zj_humanoid/manipulation/camera_calibration → 'manipulation/camera_calibration'
        
        Args:
            interface_name: Full interface name
        
        Returns:
            Module path (e.g., 'audio', 'sensor/CAM_A', 'manipulation/grasp_teach_service') 
            or empty string if not found
        """
        if not interface_name:
            return ''
        
        # Remove leading slash
        name = interface_name.lstrip('/')
        
        # Split by slash and get the parts
        # Expected format: zj_humanoid/module/submodule/...
        parts = name.split('/')
        if len(parts) >= 2:
            # Return the second part (module name)
            module = parts[1]
            
            # Support fine-grained control: include submodule (third part) if exists
            # This allows patterns like 'sensor/CAM_A', 'manipulation/grasp_teach_service', etc.
            # Enable fine-grained control for all modules that have submodules
            if len(parts) >= 3:
                return f"{module}/{parts[2]}"
            
            return module
        
        return ''
    
    def should_include_interface(self, interface_name: str, robot_model: str) -> bool:
        """
        Check if an interface should be included for a specific robot model.
        
        Supports hierarchical module exclusion:
        - Exact match: 'sensor/CAM_A' excludes only sensor/CAM_A interfaces
        - Parent match: 'sensor' excludes all sensor/* interfaces
        
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
        
        # Check if module should be excluded
        for exclude_pattern in exclude_modules:
            # Exact match: e.g., 'sensor/CAM_A' matches 'sensor/CAM_A'
            if module == exclude_pattern:
                return False
            
            # Parent match: e.g., 'sensor' matches 'sensor/CAM_A'
            # This ensures that excluding 'sensor' will exclude all sensor submodules
            if module.startswith(exclude_pattern + '/'):
                return False
        
        return True
    
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
        
        print("\nGeneration completed successfully!")


def main():
    """Main function to run the interface generator."""
    print("ZJ Humanoid ROS Interfaces Generator")
    print("=" * 50)
    
    generator = InterfaceGenerator()
    generator.generate_files()


if __name__ == "__main__":
    main()
#!/usr/bin/env python3
"""
ZJ Humanoid ROS Test Script Generator

This script generates topic_test.py and service_test.py for each ROS interface
in the zj_humanoid/ directory structure.

Usage:
    python3 generate_ros_tests.py

The generator:
- Scans zj_humanoid/ for topic.yaml and service.yaml files
- Reads type information from zj_humanoid_interfaces.yaml
- Generates Python test scripts that accept YAML data files
- Topics publish at 10Hz, services call once
"""

import os
import sys
import yaml
from typing import Optional, Dict, List


# Path configuration
SCRIPT_DIR = os.path.abspath(os.path.dirname(__file__))
ZJ_HUMANOID_DIR = os.path.join(SCRIPT_DIR, "zj_humanoid")
INTERFACES_YAML = os.path.join(SCRIPT_DIR, "zj_humanoid_interfaces.yaml")


def load_interfaces_data() -> Optional[Dict]:
    """Load the aggregated interfaces YAML file."""
    try:
        with open(INTERFACES_YAML, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)
    except Exception as e:
        print(f"Error loading {INTERFACES_YAML}: {e}")
        return None


def build_interface_map(data: Dict) -> Dict[str, Dict]:
    """Build a map of interface_name -> interface_info."""
    interface_map = {}
    
    # Process services
    for service in data.get('services', []):
        name = service.get('name')
        if name:
            interface_map[name] = {
                'type': service.get('type', ''),
                'kind': 'service',
                'description': service.get('description', '')
            }
    
    # Process topics
    for topic in data.get('topics', []):
        name = topic.get('name')
        if name:
            interface_map[name] = {
                'type': topic.get('type', ''),
                'kind': 'topic',
                'description': topic.get('description', '')
            }
    
    return interface_map


def infer_interface_name(dir_path: str) -> str:
    """Infer the full ROS interface name from directory path."""
    rel = os.path.relpath(dir_path, SCRIPT_DIR)
    if not rel.startswith("zj_humanoid" + os.sep):
        raise ValueError(f"Directory not under zj_humanoid/: {dir_path}")
    rel_after = rel.split("zj_humanoid" + os.sep, 1)[1]
    return "/zj_humanoid/" + rel_after.replace(os.sep, "/")


# Python template for topic test
TOPIC_TEST_TEMPLATE = '''#!/usr/bin/env python3
"""
ROS Topic Test Script

Topic: {topic_name}
Type: {msg_type}
Description: {description}

Usage:
    python3 topic_test.py <data.yaml>

This script publishes the message data from a YAML file to the topic at 10Hz.
Press Ctrl+C to stop.
"""

import rospy
import yaml
import sys
from {package_name}.msg import {message_name}


def load_yaml_data(yaml_file):
    """Load message data from YAML file."""
    try:
        with open(yaml_file, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)
    except Exception as e:
        rospy.logerr(f"Failed to load YAML file: {{e}}")
        sys.exit(1)


def dict_to_msg(data, msg_class):
    """Convert dictionary to ROS message."""
    msg = msg_class()
    for key, value in data.items():
        if hasattr(msg, key):
            setattr(msg, key, value)
    return msg


def main():
    if len(sys.argv) != 2:
        print(f"Usage: {{sys.argv[0]}} <data.yaml>")
        sys.exit(1)
    
    yaml_file = sys.argv[1]
    
    # Initialize ROS node
    rospy.init_node('topic_test_{node_suffix}', anonymous=True)
    
    # Load data from YAML
    data = load_yaml_data(yaml_file)
    
    # Create publisher
    pub = rospy.Publisher('{topic_name}', {message_name}, queue_size=10)
    
    # Create message
    msg = dict_to_msg(data, {message_name})
    
    # Publish at 10Hz
    rate = rospy.Rate(10)
    
    rospy.loginfo(f"Publishing to {topic_name} at 10Hz...")
    rospy.loginfo(f"Message type: {msg_type}")
    rospy.loginfo(f"Data: {{msg}}")
    
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
'''


# Python template for service test
SERVICE_TEST_TEMPLATE = '''#!/usr/bin/env python3
"""
ROS Service Test Script

Service: {service_name}
Type: {srv_type}
Description: {description}

Usage:
    python3 service_test.py <request_data.yaml>

This script calls the service with request data from a YAML file.
"""

import rospy
import yaml
import sys
from {package_name}.srv import {service_name_class}, {service_name_class}Request


def load_yaml_data(yaml_file):
    """Load service request data from YAML file."""
    try:
        with open(yaml_file, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)
    except Exception as e:
        rospy.logerr(f"Failed to load YAML file: {{e}}")
        sys.exit(1)


def dict_to_request(data, request_class):
    """Convert dictionary to service request."""
    req = request_class()
    for key, value in data.items():
        if hasattr(req, key):
            setattr(req, key, value)
    return req


def main():
    if len(sys.argv) != 2:
        print(f"Usage: {{sys.argv[0]}} <request_data.yaml>")
        sys.exit(1)
    
    yaml_file = sys.argv[1]
    
    # Initialize ROS node
    rospy.init_node('service_test_{node_suffix}', anonymous=True)
    
    # Load request data from YAML
    data = load_yaml_data(yaml_file)
    
    # Wait for service
    rospy.loginfo(f"Waiting for service {service_name}...")
    rospy.wait_for_service('{service_name}')
    
    try:
        # Create service proxy
        service_proxy = rospy.ServiceProxy('{service_name}', {service_name_class})
        
        # Create request
        req = dict_to_request(data, {service_name_class}Request)
        
        rospy.loginfo(f"Calling service {service_name}...")
        rospy.loginfo(f"Request: {{req}}")
        
        # Call service
        response = service_proxy(req)
        
        rospy.loginfo(f"Response: {{response}}")
        print(f"\\nService call successful!")
        print(f"Response: {{response}}")
        
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {{e}}")
        sys.exit(1)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
'''


def parse_msg_type(msg_type: str) -> tuple:
    """Parse message type string into (package, message_name)."""
    if '/' in msg_type:
        parts = msg_type.split('/')
        return parts[0], parts[1]
    return '', msg_type


def generate_topic_test(dir_path: str, topic_name: str, interface_info: Dict) -> bool:
    """Generate topic_test.py for a topic."""
    msg_type = interface_info.get('type', '')
    if not msg_type:
        print(f"Warning: No type found for topic {topic_name}")
        return False
    
    package_name, message_name = parse_msg_type(msg_type)
    if not package_name or not message_name:
        print(f"Warning: Invalid message type '{msg_type}' for topic {topic_name}")
        return False
    
    # Generate node suffix from topic name
    node_suffix = topic_name.replace('/', '_').replace('-', '_').strip('_')
    
    # Fill template
    script_content = TOPIC_TEST_TEMPLATE.format(
        topic_name=topic_name,
        msg_type=msg_type,
        description=interface_info.get('description', ''),
        package_name=package_name,
        message_name=message_name,
        node_suffix=node_suffix
    )
    
    # Write to file
    output_path = os.path.join(dir_path, 'topic_test.py')
    try:
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write(script_content)
        os.chmod(output_path, 0o755)
        return True
    except Exception as e:
        print(f"Error writing {output_path}: {e}")
        return False


def generate_service_test(dir_path: str, service_name: str, interface_info: Dict) -> bool:
    """Generate service_test.py for a service."""
    srv_type = interface_info.get('type', '')
    if not srv_type:
        print(f"Warning: No type found for service {service_name}")
        return False
    
    package_name, service_type_name = parse_msg_type(srv_type)
    if not package_name or not service_type_name:
        print(f"Warning: Invalid service type '{srv_type}' for service {service_name}")
        return False
    
    # Generate node suffix from service name
    node_suffix = service_name.replace('/', '_').replace('-', '_').strip('_')
    
    # Fill template
    script_content = SERVICE_TEST_TEMPLATE.format(
        service_name=service_name,
        srv_type=srv_type,
        description=interface_info.get('description', ''),
        package_name=package_name,
        service_name_class=service_type_name,
        node_suffix=node_suffix
    )
    
    # Write to file
    output_path = os.path.join(dir_path, 'service_test.py')
    try:
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write(script_content)
        os.chmod(output_path, 0o755)
        return True
    except Exception as e:
        print(f"Error writing {output_path}: {e}")
        return False


def main():
    print("=" * 60)
    print("ZJ Humanoid ROS Test Script Generator")
    print("=" * 60)
    
    # Load interfaces data
    print(f"\nLoading interfaces from {INTERFACES_YAML}...")
    data = load_interfaces_data()
    if not data:
        print("Failed to load interfaces data!")
        sys.exit(1)
    
    # Build interface map
    interface_map = build_interface_map(data)
    print(f"Loaded {len(interface_map)} interfaces")
    
    # Scan zj_humanoid directory
    print(f"\nScanning {ZJ_HUMANOID_DIR}...")
    
    topic_count = 0
    service_count = 0
    skipped_count = 0
    
    for root, dirs, files in os.walk(ZJ_HUMANOID_DIR):
        has_topic_yaml = 'topic.yaml' in files
        has_service_yaml = 'service.yaml' in files
        
        if not (has_topic_yaml or has_service_yaml):
            continue
        
        # Infer interface name
        try:
            interface_name = infer_interface_name(root)
        except ValueError as e:
            print(f"Skipping {root}: {e}")
            skipped_count += 1
            continue
        
        # Get interface info
        interface_info = interface_map.get(interface_name)
        if not interface_info:
            print(f"Warning: No interface info found for {interface_name}")
            skipped_count += 1
            continue
        
        # Generate test scripts
        if has_topic_yaml and interface_info['kind'] == 'topic':
            if generate_topic_test(root, interface_name, interface_info):
                topic_count += 1
                print(f"✓ Generated topic_test.py for {interface_name}")
            else:
                skipped_count += 1
        
        if has_service_yaml and interface_info['kind'] == 'service':
            if generate_service_test(root, interface_name, interface_info):
                service_count += 1
                print(f"✓ Generated service_test.py for {interface_name}")
            else:
                skipped_count += 1
    
    # Summary
    print("\n" + "=" * 60)
    print("Generation Summary:")
    print(f"  Topic test scripts:   {topic_count}")
    print(f"  Service test scripts: {service_count}")
    print(f"  Skipped:              {skipped_count}")
    print(f"  Total generated:      {topic_count + service_count}")
    print("=" * 60)


if __name__ == '__main__':
    main()

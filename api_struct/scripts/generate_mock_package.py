#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Generate a mock ROS package from zj_humanoid interface definitions.

The generated package contains a single Python node (mock_server.py) that
advertises all services and topics defined in api_struct/generated/zj_humanoid_interfaces.yaml.
It allows application developers to interact with the APIs without requiring the
real robot backend.

Usage:
    cd api_struct/scripts
    python3 generate_mock_package.py

Optional arguments:
    --input <path>          Path to aggregated interfaces YAML
    --output-dir <path>     Where to generate the mock package
    --pkg-name <name>       Name of the mock ROS package (default: zj_humanoid_mock)

After generation, add the package to your catkin workspace, run `catkin_make`,
and launch the mock server with:
    rosrun <pkg-name> mock_server.py
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from textwrap import dedent
from typing import Dict, List, Tuple, Any, Set

import yaml


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent.parent
DEFAULT_INPUT = SCRIPT_DIR.parent / "generated" / "zj_humanoid_interfaces.yaml"
DEFAULT_OUTPUT = PROJECT_ROOT / "mock_packages" / "zj_humanoid_mock"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate mock ROS package for zj_humanoid interfaces.")
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT, help="Path to interfaces YAML.")
    parser.add_argument(
        "--output-dir", type=Path, default=DEFAULT_OUTPUT, help="Directory to place generated package."
    )
    parser.add_argument("--pkg-name", type=str, default="zj_humanoid_mock", help="Mock package name.")
    return parser.parse_args()


def load_interfaces(yaml_path: Path) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    with yaml_path.open("r", encoding="utf-8") as fp:
        data = yaml.safe_load(fp)

    services: List[Dict[str, Any]] = []
    for module, entries in (data.get("services") or {}).items():
        for entry in entries:
            entry = dict(entry)
            entry["module"] = module
            entry["kind"] = "service"
            services.append(entry)

    topics: List[Dict[str, Any]] = []
    for module, entries in (data.get("topics") or {}).items():
        for entry in entries:
            entry = dict(entry)
            entry["module"] = module
            entry["kind"] = "topic"
            topics.append(entry)

    return services, topics


def ensure_dirs(output_dir: Path) -> None:
    (output_dir / "scripts").mkdir(parents=True, exist_ok=True)


def dump_json_literal(data: Any) -> str:
    return json.dumps(data, indent=2, ensure_ascii=False)


def gather_dependency_packages(services: List[Dict[str, Any]], topics: List[Dict[str, Any]]) -> Set[str]:
    pkgs: Set[str] = {"rospy"}
    for entry in services + topics:
        type_str = entry.get("type") or ""
        if "/" not in type_str:
            continue
        pkg, _ = type_str.split("/", 1)
        pkgs.add(pkg)
    return pkgs


def render_package_xml(pkg_name: str, dependencies: Set[str]) -> str:
    dep_xml = "\n".join(f"  <exec_depend>{dep}</exec_depend>" for dep in sorted(dependencies))
    return dedent(
        f"""\
        <package format="2">
          <name>{pkg_name}</name>
          <version>0.1.0</version>
          <description>Mock services/topics for zj_humanoid interfaces.</description>
          <maintainer email="dev@zj-humanoid.local">ZJ Humanoid</maintainer>
          <license>Apache-2.0</license>
          <buildtool_depend>catkin</buildtool_depend>
{dep_xml}
        </package>
        """
    )


def render_cmakelists(pkg_name: str) -> str:
    return dedent(
        f"""\
        cmake_minimum_required(VERSION 3.0.2)
        project({pkg_name})

        find_package(catkin REQUIRED COMPONENTS
          rospy
        )

        catkin_package()

        catkin_install_python(PROGRAMS
          scripts/mock_server.py
          DESTINATION ${{CATKIN_PACKAGE_BIN_DESTINATION}}
        )
        """
    )


def render_readme(pkg_name: str) -> str:
    return dedent(
        f"""\
        # {pkg_name}

        This package is auto-generated from `zj_humanoid_interfaces.yaml`. It mocks every
        ROS service and topic so that higher-level applications can develop and run tests
        without connecting to the real robot.

        ## Usage

        ```bash
        # Add the package into your catkin workspace
        cp -r mock_packages/{pkg_name} ~/catkin_ws/src/
        cd ~/catkin_ws
        catkin_make

        # Run the mock server
        source devel/setup.bash
        rosrun {pkg_name} mock_server.py
        ```

        Optional arguments:
        - `--modules audio upperlimb` limit mocked subsystems
        - `--publish-rate-scale 0.5` slow down all topic publishers

        Re-run `generate_mock_package.py` whenever `zj_humanoid_interfaces.yaml` changes.
        """
    )


def render_mock_server_python(pkg_name: str, services: List[Dict[str, Any]], topics: List[Dict[str, Any]]) -> str:
    service_json = dump_json_literal(services)
    topic_json = dump_json_literal(topics)

    return f"""#!/usr/bin/env python3
# -*- coding: utf-8 -*-
\"\"\"Auto-generated mock server for {pkg_name}.\"\"\"

import argparse
import threading
from importlib import import_module
from typing import Any, Dict, List, Tuple

import rospy


SERVICE_CONFIG: List[Dict[str, Any]] = {service_json}

TOPIC_CONFIG: List[Dict[str, Any]] = {topic_json}


def load_service_type(type_name: str) -> Tuple[Any, Any]:
    pkg, srv = type_name.split("/", 1)
    module = import_module(f"{{pkg}}.srv")
    srv_cls = getattr(module, srv)
    resp_cls = getattr(module, f"{{srv}}Response")
    return srv_cls, resp_cls


def load_message_type(type_name: str) -> Any:
    pkg, msg = type_name.split("/", 1)
    module = import_module(f"{{pkg}}.msg")
    return getattr(module, msg)


def populate_header(obj: Any) -> None:
    header = getattr(obj, "header", None)
    if header is None:
        return
    header.stamp = rospy.Time.now()
    if hasattr(header, "frame_id") and not header.frame_id:
        header.frame_id = "mock"


def create_default_response(resp_cls: Any, service_name: str) -> Any:
    resp = resp_cls()
    if hasattr(resp, "success"):
        resp.success = True
    if hasattr(resp, "message"):
        resp.message = f"Mock response for {{service_name}}"
    if hasattr(resp, "status") and isinstance(resp.status, int):
        resp.status = 0
    return resp


def create_default_message(msg_cls: Any, topic_name: str) -> Any:
    msg = msg_cls()
    populate_header(msg)
    if hasattr(msg, "data"):
        try:
            msg.data = getattr(msg, "data", 0) or 0
        except Exception:
            pass
    return msg


def make_service_handler(service_name: str, resp_cls: Any):
    def handler(req):
        rospy.loginfo("[MOCK][SERVICE] %s called with %s", service_name, req)
        resp = create_default_response(resp_cls, service_name)
        return resp

    return handler


def start_publish_thread(pub, msg_cls, topic_name: str, rate_hz: float):
    def loop():
        rate = rospy.Rate(rate_hz)
        while not rospy.is_shutdown():
            msg = create_default_message(msg_cls, topic_name)
            pub.publish(msg)
            rate.sleep()

    thread = threading.Thread(target=loop, name=f"mock_pub_{{topic_name}}", daemon=True)
    thread.start()


def filter_entries(entries: List[Dict[str, Any]], modules: List[str]) -> List[Dict[str, Any]]:
    if not modules:
        return entries
    modules = [m.lower() for m in modules]
    return [entry for entry in entries if entry.get("module", "").lower() in modules]


def main():
    parser = argparse.ArgumentParser(description="Run mock zj_humanoid ROS interfaces.")
    parser.add_argument("--modules", nargs="*", help="Limit to these subsystems (audio, hand, ...)")
    parser.add_argument(
        "--publish-rate-scale",
        type=float,
        default=1.0,
        help="Scale factor for topic publish rates (default 1.0)",
    )
    args = parser.parse_args()

    rospy.init_node("{pkg_name}_mock_server")

    active_services = filter_entries(SERVICE_CONFIG, args.modules)
    active_topics = filter_entries(TOPIC_CONFIG, args.modules)

    rospy.loginfo("Starting mock server with %d services and %d topics", len(active_services), len(active_topics))

    for entry in active_services:
        type_name = entry.get("type")
        try:
            srv_cls, resp_cls = load_service_type(type_name)
        except Exception as exc:
            rospy.logwarn("Failed to load service type %s: %s", type_name, exc)
            continue
        rospy.Service(entry["name"], srv_cls, make_service_handler(entry["name"], resp_cls))
        rospy.loginfo("[MOCK][SERVICE] %s registered (%s)", entry["name"], type_name)

    for entry in active_topics:
        type_name = entry.get("type")
        direction = entry.get("direction", "publish")
        rate = float(entry.get("throttle_rate") or 1.0)
        if rate <= 0:
            rate = 1.0
        rate *= max(args.publish_rate_scale, 0.01)

        try:
            msg_cls = load_message_type(type_name)
        except Exception as exc:
            rospy.logwarn("Failed to load message type %s: %s", type_name, exc)
            continue

        if direction == "subscribe":
            rospy.Subscriber(entry["name"], msg_cls, lambda msg, name=entry["name"]: rospy.loginfo(
                "[MOCK][SUB] %s received %s", name, msg
            ))
            rospy.loginfo("[MOCK][SUB] %s listening (%s)", entry["name"], type_name)
        else:
            pub = rospy.Publisher(entry["name"], msg_cls, queue_size=10)
            start_publish_thread(pub, msg_cls, entry["name"], rate)
            rospy.loginfo("[MOCK][PUB] %s publishing at %.2f Hz (%s)", entry["name"], rate, type_name)

    rospy.loginfo("Mock server ready. Press Ctrl+C to exit.")
    rospy.spin()


if __name__ == "__main__":
    main()
"""


def write_file(path: Path, content: str) -> None:
    path.write_text(content.rstrip() + "\n", encoding="utf-8")


def main():
    args = parse_args()
    services, topics = load_interfaces(args.input)
    ensure_dirs(args.output_dir)

    dependencies = gather_dependency_packages(services, topics)
    write_file(args.output_dir / "package.xml", render_package_xml(args.pkg_name, dependencies))
    write_file(args.output_dir / "CMakeLists.txt", render_cmakelists(args.pkg_name))
    write_file(args.output_dir / "README.md", render_readme(args.pkg_name))
    mock_server_path = args.output_dir / "scripts" / "mock_server.py"
    write_file(mock_server_path, render_mock_server_python(args.pkg_name, services, topics))
    mock_server_path.chmod(0o755)

    print("Mock package generated at:", args.output_dir)
    print(f"Services: {len(services)} | Topics: {len(topics)}")


if __name__ == "__main__":
    main()


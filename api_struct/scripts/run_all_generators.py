#!/usr/bin/env python3
"""
ZJ Humanoid API 文件生成器 - 主脚本

这个脚本按正确的顺序运行所有生成脚本：
1. generate_whole_yaml.py - 从 zj_humanoid/ 生成聚合的 YAML
2. generate_json_from_yaml.py - 从 YAML 生成 JSON 和 Markdown
3. generate_ros_tests.py - 生成测试脚本

Usage:
    cd api_struct/scripts/
    python3 run_all_generators.py
"""

import subprocess
import sys
from pathlib import Path


def run_script(script_name: str, description: str) -> bool:
    """运行一个脚本并显示结果"""
    print("\n" + "=" * 70)
    print(f"▶ {description}")
    print("=" * 70)
    
    script_path = Path(__file__).parent / script_name
    
    try:
        result = subprocess.run(
            [sys.executable, str(script_path)],
            check=True,
            capture_output=False
        )
        print(f"✅ {description} - 完成")
        return True
    except subprocess.CalledProcessError as e:
        print(f"❌ {description} - 失败: {e}")
        return False
    except Exception as e:
        print(f"❌ {description} - 错误: {e}")
        return False


def main():
    """主函数"""
    print("╔" + "=" * 68 + "╗")
    print("║" + " " * 15 + "ZJ Humanoid API 文件生成器" + " " * 28 + "║")
    print("╚" + "=" * 68 + "╝")
    
    scripts = [
        ("generate_whole_yaml.py", "1/3 生成聚合 YAML 文件"),
        ("generate_json_from_yaml.py", "2/3 生成 JSON 和 Markdown 文档"),
        ("generate_ros_tests.py", "3/3 生成 ROS 测试脚本"),
    ]
    
    results = []
    for script, desc in scripts:
        success = run_script(script, desc)
        results.append((desc, success))
    
    print("\n" + "=" * 70)
    print("📊 生成总结")
    print("=" * 70)
    
    for desc, success in results:
        status = "✅ 成功" if success else "❌ 失败"
        print(f"{status} - {desc}")
    
    all_success = all(success for _, success in results)
    
    if all_success:
        print("\n🎉 所有文件生成成功！")
        print("\n生成的文件位于:")
        print("  - api_struct/generated/zj_humanoid_interfaces.yaml")
        print("  - api_struct/generated/zj_humanoid_interfaces_*.json")
        print("  - api_struct/generated/zj_humanoid_interfaces.md")
        print("  - api_struct/zj_humanoid/**/topic_test.py")
        print("  - api_struct/zj_humanoid/**/service_test.py")
        return 0
    else:
        print("\n⚠️ 部分文件生成失败，请检查错误信息")
        return 1


if __name__ == "__main__":
    sys.exit(main())

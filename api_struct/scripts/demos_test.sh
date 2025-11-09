#!/bin/sh

# 示例：sh demos_test_shell.sh ../zj_humanoid/upperlimb/movej/left_arm/left_arm_t_case1.yaml
# 路径 zj_humanoid/upperlimb/movej/left_arm 即是service_name，也是demo data数据的yaml_path
# 
# 从 api_struct/scripts/ 目录运行

if [ $# -ne 1 ]; then
    echo "Usage: $0 <yaml_path>" >&2
    echo "Example: $0 ../zj_humanoid/upperlimb/movej/left_arm/left_arm_t_case1.yaml" >&2
    exit 1
fi

# 获取脚本所在目录的父目录 (api_struct/)
SCRIPT_DIR=$(cd "$(dirname "$0")/.." && pwd)
YAML_PATH="$SCRIPT_DIR/$1"

# 根据yaml_path，推断出service_name
case "$YAML_PATH" in
    *"/zj_humanoid/"*)
        RELATIVE=${YAML_PATH#*"/zj_humanoid/"}
        SERVICE_NAME="/zj_humanoid/${RELATIVE%/*}"
        ;;
    *)
        echo "Error: unable to infer service name from path" >&2
        exit 1
        ;;
esac

PAYLOAD=$(cat "$YAML_PATH")

# 读取 generated/zj_humanoid_interfaces.yaml, 判断SERVICE_NAME是topic还是service
INTERFACES_YAML="$SCRIPT_DIR/generated/zj_humanoid_interfaces.yaml"

if [ ! -f "$INTERFACES_YAML" ]; then
    echo "Error: zj_humanoid_interfaces.yaml not found: $INTERFACES_YAML" >&2
    echo "Please run generate_whole_yaml.py first to generate the interfaces file." >&2
    exit 1
fi

# 判断SERVICE_NAME是topic还是service
# 使用grep和sed来查找
SERVICE_LINE=$(grep -n "name: $SERVICE_NAME" "$INTERFACES_YAML" | head -1 | cut -d: -f1)
TOPICS_LINE=$(grep -n "^topics:" "$INTERFACES_YAML" | cut -d: -f1)

if [ -n "$SERVICE_LINE" ] && [ -n "$TOPICS_LINE" ]; then
    if [ "$SERVICE_LINE" -lt "$TOPICS_LINE" ]; then
        IS_SERVICE="true"
        IS_TOPIC="false"
    else
        IS_SERVICE="false"
        IS_TOPIC="true"
    fi
else
    IS_SERVICE="false"
    IS_TOPIC="false"
fi

if [ "$IS_SERVICE" = "true" ]; then
    echo "Executing: rosservice call $SERVICE_NAME < $YAML_PATH" >&2
    echo "Payload: $PAYLOAD" >&2
    rosservice call "$SERVICE_NAME" "$PAYLOAD"
elif [ "$IS_TOPIC" = "true" ]; then
    echo "Executing: rostopic pub -r 10 $SERVICE_NAME < $YAML_PATH (Press Ctrl+C to stop)" >&2
    echo "Payload: $PAYLOAD" >&2
    echo "Publishing at 10Hz (100ms interval)..." >&2
    # 使用-r 10参数表示10Hz频率发送（100ms间隔）
    rostopic pub -r 10 "$SERVICE_NAME" "$PAYLOAD"
else
    echo "Error: $SERVICE_NAME not found in zj_humanoid_interfaces.yaml" >&2
    echo "Available services and topics:" >&2
    grep "name:" "$INTERFACES_YAML" | head -10 >&2
    exit 1
fi


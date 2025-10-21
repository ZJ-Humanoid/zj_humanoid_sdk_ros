#!/bin/sh

# 示例：sh demos_shell_test.sh zj_humanoid/upperlimb/movej/left_arm/left_arm_t_case1.yaml

if [ $# -ne 1 ]; then
    echo "Usage: $0 <yaml_path>" >&2
    exit 1
fi

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)

if [ "${1#/}" != "$1" ]; then
    YAML_PATH="$1"
else
    YAML_PATH="$SCRIPT_DIR/$1"
fi

if [ ! -f "$YAML_PATH" ]; then
    echo "Error: YAML file not found: $YAML_PATH" >&2
    exit 1
fi

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

echo "rosservice call $SERVICE_NAME < $YAML_PATH" >&2
rosservice call "$SERVICE_NAME" "$PAYLOAD"


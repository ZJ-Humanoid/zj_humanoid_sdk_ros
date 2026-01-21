# zj_humanoid_mock

This package is auto-generated from `zj_humanoid_interfaces.yaml`. It mocks every
ROS service and topic so that higher-level applications can develop and run tests
without connecting to the real robot.

## Usage

```bash
# Add the package into your catkin workspace
cp -r mock_packages/zj_humanoid_mock ~/catkin_ws/src/
cd ~/catkin_ws
catkin_make

# Run the mock server
source devel/setup.bash
rosrun zj_humanoid_mock mock_server.py
```

Optional arguments:
- `--modules audio upperlimb` limit mocked subsystems
- `--publish-rate-scale 0.5` slow down all topic publishers

Re-run `generate_mock_package.py` whenever `zj_humanoid_interfaces.yaml` changes.

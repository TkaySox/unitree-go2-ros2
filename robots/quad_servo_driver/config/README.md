# quad_servo_driver

ROS 2 node: CHAMP `joint_states` → Feetech bus servos (USB adapter).

## Setup

1. Edit `JOINT_ID_MAP` and `SERIAL_PORT` in
   `quad_servo_driver/champ_servo_driver.py`.
2. Install Feetech SDK (`scservo_sdk` / `STservo_sdk`) on the Pi.
3. Pose the robot in the URDF **home** pose, then first run (creates calib file):

```bash
colcon build --packages-select quad_servo_driver --symlink-install
source install/setup.bash
ros2 run quad_servo_driver champ_servo_driver
```

Calibration is saved to `~/.ros/quad_servo_calibration.yaml`.
Delete that file to force re-calibration.

## With CHAMP (hardware)

```bash
ros2 launch quad_config bringup.launch.py hardware_connected:=true
# other terminal:
ros2 run quad_servo_driver champ_servo_driver
```

Ensure CHAMP publishes `joint_states` (gazebo:=false / not sim).

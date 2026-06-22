# NAO Vuer XR Teleoperator

This package provides an XR teleoperator for controlling one NAO arm with inverse kinematics from a WebXR headset, following the same general interaction model as the SO101 Vuer teleoperator.

It uses Vuer for the XR session, Pyroki for inverse kinematics, and the NAO LeRobot robot plugin to execute joint commands on the physical robot.

An optional Viser visualization can also be enabled to inspect the IK target and the current robot configuration in a browser on localhost:8080.

## Requirements

- A working LeRobot environment
- The dependencies from `requirements.txt`
- A loadable NAO URDF and meshes
- NAOqi Python bindings available in the runtime environment
- TLS certificates for WebXR, for example `cert.pem` and `key.pem`

## Installation

```bash
cd /root/lerobot/lerobot_teleoperator_nao_vuer
pip install -r requirements.txt
pip install -e .
```

You also need the official NAO model installed locally. A typical layout is:

```text
/root/lerobot/external/nao/
  nao_meshes/
  nao_robot/
    nao_description/
      urdf/
        naoV50_generated_urdf/
          nao.urdf
```

## Usage

```bash
lerobot-teleoperate \
  --robot.type=nao_qi \
  --robot.robot_ip=127.0.0.1 \
  --robot.arm=right \
  --robot.enable_camera=true \
  --teleop.type=nao_vuer \
  --teleop.arm=right \
  --teleop.user_hand=right \
  --teleop.urdf_path=/root/lerobot/external/nao/nao_robot/nao_description/urdf/naoV50_generated_urdf/nao.urdf \
  --teleop.target_link_name=r_gripper \
  --teleop.enable_visualization=true \
  --teleop.vuer_cert=/absolute/path/to/cert.pem \
  --teleop.vuer_key=/absolute/path/to/key.pem
```

Then open the XR client from a headset browser using:

```text
https://<YOUR_IP>:8012/?ws=wss://<YOUR_IP>:8012
```

If visualization is enabled, open this page on the host machine as well:

```text
http://localhost:8080
```

## Connecting From The Headset

Once the `lerobot-teleoperate` command is running and the Vuer server has
started:

1. Make sure your computer and your headset are connected to the same Wi-Fi network.
2. Find your computer's local IP address, for example `192.168.1.50`.
3. Put on the headset, open its WebXR-capable browser, and navigate to:

```text
https://<YOUR_IP>:8012/?ws=wss://<YOUR_IP>:8012
```

4. If the browser warns that the certificate is not trusted, open the advanced options and continue anyway.
5. Stand upright, face forward, and recenter the headset view before starting XR. This matters because the teleoperator maps the tracked hand pose into a fixed torso-centered robot frame.
6. Enter VR mode from the page. Once the hand or controller stream is active, moving the tracked hand will update the NAO IK target.

### Connection Tips

- The URL must include both `https://` and the `?ws=wss://...` query string.
- If you are not launching from the directory that contains `cert.pem` and `key.pem`, pass absolute paths with `--teleop.vuer_cert` and `--teleop.vuer_key`.
- If the headset page opens but tracking does not update, first verify that the browser granted motion and XR permissions.
- If you use the provided launcher script, export `ROBOT_IP`, `CERT_PATH`, and `KEY_PATH` before sourcing it.

## Intuitive Mapping

The teleoperator uses a direct, mirror-free spatial mapping anchored at your
shoulder:

- Reach **forward** and NAO's hand goes **forward**.
- Raise your hand **up** and NAO's hand goes **up**.
- Move your hand to **your right** and NAO's hand goes to **its right**.

Position always follows your hand 1:1. The gripper also follows your wrist
orientation while your hand is open, and freezes that orientation as you close
into a grasp so the grip is not disturbed. Because the NAO arm only has 5 joints,
orientation is best-effort (position is weighted more heavily in the IK).

If position feels right but the gripper points in a slightly wrong direction,
nudge it with `--teleop.wrist_offset_euler_deg="(roll,pitch,yaw)"` (degrees), or
turn orientation tracking off entirely with `--teleop.track_orientation=false`.

## Opening And Closing The Hand

Just open and close your hand naturally — NAO's hand follows.

- With **hand tracking**, the whole-hand open/close is read from your finger
  joints. It self-calibrates: open and close your hand fully once at the start so
  it learns your range.
- With **motion controllers**, the trigger/squeeze controls the hand instead.

Related flags:

- `--teleop.hand_control_source=auto|fist|pinch` — `auto` (default) uses
  whole-hand tracking and falls back to pinch/trigger; `fist` forces whole-hand
  only; `pinch` forces the pinch/trigger.
- `--teleop.invert_hand=true` — flip the direction if the robot hand closes when
  you open yours.
- `--teleop.hand_smoothing=0.5` — exponential smoothing in `[0, 1)`; higher is
  smoother but laggier, `0.0` disables it.

## Notes

- The current implementation is intentionally one-arm only.
- The default right-arm IK target link for the official NAO V5 URDF is `r_gripper`.
- Hand opening is mapped to the configured `hand_joint_name`, which defaults to `RHand`.
- If you switch to the left arm, you should also override `--teleop.target_link_name=l_gripper` and `--teleop.hand_joint_name=LHand`.
- Visualization is disabled by default and can be enabled with `--teleop.enable_visualization=true`.

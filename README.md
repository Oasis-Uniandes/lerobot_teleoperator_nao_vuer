# NAO Vuer XR Teleoperator

This package provides an XR teleoperator for controlling one or both NAO arms with inverse kinematics from a WebXR headset, following the same general interaction model as the SO101 Vuer teleoperator.

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

## Choosing An Arm

Select which arm(s) to control with `--teleop.arm`. Valid values are `left`,
`right`, and `both`. The arm choice drives every arm-dependent setting, so you
normally only need to set `--teleop.arm`:

| Setting | `left` | `right` |
| --- | --- | --- |
| `target_link_name` | `l_gripper` | `r_gripper` |
| `hand_joint_name` | `LHand` | `RHand` |
| `user_hand` (which hand tracks) | `left` | `right` |

These are derived automatically; override them only for a non-standard setup
(e.g. pass `--teleop.user_hand=left` to drive the right NAO arm with your left
hand). The `--robot.arm` value must match `--teleop.arm`.

With `--teleop.arm=both`, the teleoperator controls both arms at once: your
**left hand drives NAO's left arm and your right hand drives its right arm**,
each with its own IK target (`l_gripper` / `r_gripper`) and hand (`LHand` /
`RHand`). The rest of the body stays locked. Use `--robot.arm=both` so the robot
accepts both arms' joints.

## Usage

Single arm (right):

```bash
lerobot-teleoperate \
  --robot.type=nao_qi \
  --robot.robot_ip=127.0.0.1 \
  --robot.arm=right \
  --robot.enable_camera=true \
  --teleop.type=nao_vuer \
  --teleop.arm=right \
  --teleop.urdf_path=/root/lerobot/external/nao/nao_robot/nao_description/urdf/naoV50_generated_urdf/nao.urdf \
  --teleop.enable_visualization=true \
  --teleop.vuer_cert=/absolute/path/to/cert.pem \
  --teleop.vuer_key=/absolute/path/to/key.pem
```

Both arms:

```bash
lerobot-teleoperate \
  --robot.type=nao_qi \
  --robot.robot_ip=127.0.0.1 \
  --robot.arm=both \
  --robot.enable_camera=true \
  --teleop.type=nao_vuer \
  --teleop.arm=both \
  --teleop.urdf_path=/root/lerobot/external/nao/nao_robot/nao_description/urdf/naoV50_generated_urdf/nao.urdf \
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

Your hand displacement (measured from your shoulder) is scaled down to NAO's
reach before it becomes the IK target, because NAO is a small robot (~0.22 m
arm reach) and your hands move at human scale (~0.6 m). The default scale is
`0.4`; lower it for finer/closer control or raise it toward `1.0` for a bigger
workspace:

- `--teleop.target_position_scale=0.4` — fraction of your real reach mapped onto
  NAO. Without scaling the targets would constantly saturate at the joint limits.

Orientation follows **where your fingers point**: NAO's gripper `+X` axis (the
red arrow you see in viser) is aimed along your fingers. Point forward and the
gripper points forward; point up and it points up. The frame is anchored to
world-up, so simply rolling your wrist does not twist the gripper — only the
pointing direction matters. This is computed from your finger-joint geometry
(wrist → middle knuckle), so it is robust even when you curl into a fist; with
motion controllers it falls back to the controller's forward ray.

The gripper holds that orientation while your hand is open and freezes it as you
close into a grasp so the grip is not disturbed. Because the NAO arm only has 5
joints, orientation is best-effort (position is weighted more heavily in the IK).

If the gripper is consistently rotated from where you expect, nudge it with
`--teleop.wrist_offset_euler_deg="(roll,pitch,yaw)"` (degrees), or turn
orientation tracking off entirely with `--teleop.track_orientation=false`.

## Orientation Gizmos (Matching Your Hand To NAO's)

The scene draws two coordinate-axis pointers just in front of each tracked hand
(offset along your pointing direction so they are not buried in the rendered
hand mesh):

- **Short axes** — the direction *your* hand points (its red `+X` follows your
  fingers).
- **Long axes** (same origin, bigger arrow heads) — the orientation NAO's
  gripper is *actually* achieving, computed from forward kinematics on the IK
  solution and mapped back into your VR space.

When the two red axes line up, your fingers and NAO's gripper point the same way.
Because the NAO arm only has 5 joints, the long axes won't always track
perfectly — that gap is the real, reachable orientation, so you can see at a
glance how closely NAO can follow. If the gripper is consistently offset, dial in
`--teleop.wrist_offset_euler_deg="(roll,pitch,yaw)"` until the long axes match
the short ones.

Tune placement/visibility with `--teleop.gizmo_forward_offset=0.12` (metres in
front of the hand) or turn the pointers off with
`--teleop.show_orientation_gizmos=false`. In `both` mode each arm gets its own
pair.

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

## Seeing NAO's Camera In VR

You can view NAO's head-camera feed as a fixed screen floating in front of you
while in immersive VR (the same approach as the SO101 Vuer teleop). The
teleoperator pulls the image and streams it into the Vuer scene by default
(`--teleop.enable_camera_feed=true`); the only requirement is that the **robot**
publishes its camera:

- `--robot.enable_camera=true` — the NAO robot publishes its head camera. If
  this is off there is no image to show, so the screen stays blank.

The screen is placed in front of the headset at
`[camera_lateral_offset, user_height + camera_height_offset, camera_screen_z]`,
`camera_distance_to_user` metres away. The defaults (`0.0`, `-0.6`, `-3.0`,
`1.0`) put it at a comfortable eye-level position straight ahead. Tune them if
the screen sits too high/low or too close:

- `--teleop.camera_height_offset=-0.6` — vertical offset from your eye height.
- `--teleop.camera_lateral_offset=0.0` — left/right shift.
- `--teleop.camera_screen_z=-3.0` — how far ahead the screen plane is.
- `--teleop.camera_distance_to_user=1.0` — HUD distance from the camera.

## Notes

- Single-arm (`left`/`right`) and dual-arm (`both`) modes are supported.
- NAO's head-camera feed can be shown in VR with `--robot.enable_camera=true --teleop.enable_camera_feed=true`.
- The IK target link and hand joint are derived from `--teleop.arm` (`r_gripper`/`RHand` for the right arm, `l_gripper`/`LHand` for the left); switching to the left arm no longer needs manual `--teleop.target_link_name`/`--teleop.hand_joint_name` overrides.
- Hand opening is mapped to the per-arm hand joint (`RHand`/`LHand`).
- Visualization is disabled by default and can be enabled with `--teleop.enable_visualization=true`. In `both` mode it shows one IK gizmo per arm.

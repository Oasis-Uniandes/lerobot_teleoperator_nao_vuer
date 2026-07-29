import asyncio
import gc
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import pyroki as pk
import viser
from lerobot.teleoperators.teleoperator import Teleoperator
from robot_descriptions.loaders.yourdfpy import load_robot_description
from scipy.spatial.transform import Rotation as R
from vuer import Vuer, VuerSession
from vuer.schemas import CoordsMarker, DefaultScene, Hands, ImageBackground, MotionControllers
from viser.extras import ViserUrdf
import yourdfpy
from yourdfpy import URDF

from .config_nao_vuer_teleop import (
    ARM_HAND_JOINT_NAME,
    ARM_INITIAL_TARGET_POSITION,
    ARM_TARGET_LINK_NAME,
    NaoVuerTeleopConfig,
    sides_for_arm,
)
from .pyroki_snippets import solve_ik

# Axis remap from VR (+X=right, +Y=up, -Z=forward) to NAO base
# (+X=forward, +Y=left, +Z=up). Orthonormal, so its transpose maps back.
R_VR_TO_ROBOT = np.array([
    [0, 0, -1],
    [-1, 0, 0],
    [0, 1, 0],
], dtype=float)


@dataclass
class _SideState:
    """Per-arm configuration plus the live VR target it tracks.

    The runtime fields (target_*, curl_*, viz_*) are mutated under the
    teleoperator's lock.
    """

    side: str  # NAO arm: 'left' or 'right'
    user_hand: str  # which of the user's hands drives this arm
    arm_joint_names: tuple[str, ...]
    target_link_name: str
    hand_joint_name: str
    joint_mask: np.ndarray
    target_pos: np.ndarray
    target_wxyz: np.ndarray
    gizmo: Any = None  # viser transform controls handle (visualization only)
    target_link_index: int = 0  # index into robot.links.names for FK
    # Gripper rest position (robot frame) at the default config. Hand motion is
    # added on top of this, so zero hand displacement keeps the IK target on the
    # robot instead of jumping to the base origin.
    rest_pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    target_hand: float = 1.0  # start with an open hand
    # Auto-calibration range for whole-hand fist tracking (per user hand).
    curl_min: float | None = None
    curl_max: float | None = None
    # Your hand's pose in VR (for the "hand" orientation gizmo).
    viz_pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    viz_rot: np.ndarray = field(default_factory=lambda: np.zeros(3))
    # NAO's achieved gripper orientation mapped back into VR space (for the
    # "nao" orientation gizmo, drawn at the same hand position).
    nao_viz_rot: np.ndarray = field(default_factory=lambda: np.zeros(3))


class NaoVuerTeleop(Teleoperator):
    config_class = NaoVuerTeleopConfig
    name = "nao_vuer"

    # WebXR 25-joint hand model. Fingertip joints and a palm-size reference
    # (middle-finger metacarpal) used to estimate how open the hand is.
    _FINGERTIP_JOINTS = (4, 9, 14, 19, 24)  # thumb, index, middle, ring, pinky tips
    _PALM_REF_JOINT = 10  # middle-finger metacarpal

    def __init__(self, config: NaoVuerTeleopConfig):
        super().__init__(config)
        self.config = config
        self._is_connected = False
        self._ik_thread = None
        self._vuer_thread = None
        self._cam_thread = None
        self._lock = threading.Lock()

        self._sides: list[_SideState] = []
        self._side_by_user_hand: dict[str, _SideState] = {}
        self._latest_q_sol = None
        self._latest_frame = None
        self._latest_action = {}
        self._prev_cfg = None
        self.viser_server = None
        self.urdf_vis = None

    def configure(self) -> None:
        arm = self.config.arm.lower()
        if arm not in {"left", "right", "both"}:
            raise ValueError("arm must be 'left', 'right' or 'both'.")
        if arm != "both" and (self.config.user_hand or "").lower() not in {"left", "right"}:
            raise ValueError("user_hand must be 'left' or 'right'.")
        if not self.config.urdf_path and not self.config.urdf_name:
            raise ValueError("nao_vuer requires urdf_path or urdf_name.")
        if self.config.speed_fraction <= 0.0 or self.config.speed_fraction > 1.0:
            raise ValueError("speed_fraction must be in the interval (0, 1].")
        if not 0.0 <= self.config.stiffness <= 1.0:
            raise ValueError("stiffness must be within [0, 1].")

    def _load_urdf(self):
        if self.config.urdf_path:
            urdf_path = Path(self.config.urdf_path).expanduser().resolve()
            nao_description_root = urdf_path.parents[2]
            package_roots = {
                "nao_description": nao_description_root,
                "nao_meshes": nao_description_root.parent.parent / "nao_meshes",
            }

            def filename_handler(fname: str) -> str:
                if fname.startswith("package://"):
                    package_name, _, rel_path = fname.removeprefix("package://").partition("/")
                    package_root = package_roots.get(package_name)
                    if package_root is not None:
                        return str((package_root / rel_path).resolve())
                return yourdfpy.filename_handler_magic(fname, dir=urdf_path.parent)

            urdf = URDF.load(urdf_path, filename_handler=filename_handler)
            self._ensure_joint_velocity_limits(urdf)
            return urdf
        urdf = load_robot_description(self.config.urdf_name)
        self._ensure_joint_velocity_limits(urdf)
        return urdf

    @staticmethod
    def _ensure_joint_velocity_limits(urdf: URDF, default_velocity: float = np.pi) -> None:
        for joint in urdf.joint_map.values():
            if joint.type in {"fixed", "floating", "planar"}:
                continue
            if joint.limit is None:
                joint.limit = yourdfpy.urdf.Limit(
                    effort=0.0,
                    velocity=default_velocity,
                    lower=None,
                    upper=None,
                )
                continue
            if joint.limit.velocity is None:
                joint.limit.velocity = default_velocity

    def _arm_joint_names_for(self, side: str) -> tuple[str, ...]:
        if side == "left":
            return self.config.left_arm_joint_names
        return self.config.right_arm_joint_names

    def _make_joint_mask(self, arm_joint_names: tuple[str, ...], hand_joint_name: str) -> np.ndarray:
        selected = set(arm_joint_names)
        if hand_joint_name:
            selected.add(hand_joint_name)
        return np.array([
            1.0 if joint_name in selected else 0.0
            for joint_name in self.robot.joints.actuated_names
        ])

    def _side_action(self, state: _SideState, q_sol: np.ndarray, hand_value: float) -> dict[str, float]:
        action = {}
        for joint_name in state.arm_joint_names:
            if joint_name in self.robot.joints.actuated_names:
                idx = self.robot.joints.actuated_names.index(joint_name)
                action[f"{joint_name}.pos"] = float(q_sol[idx])
        if state.hand_joint_name and state.hand_joint_name in self.robot.joints.actuated_names:
            idx = self.robot.joints.actuated_names.index(state.hand_joint_name)
            lower = float(self.robot.joints.lower_limits[idx])
            upper = float(self.robot.joints.upper_limits[idx])
            action[f"{state.hand_joint_name}.pos"] = lower + hand_value * (upper - lower)
        return action

    def _torso_origin_vr(self, user_hand: str) -> np.ndarray:
        """Reference point (the user's shoulder) in VR space.

        The hand is measured relative to this point, so reaching the same way
        relative to your shoulder always maps to the same target relative to
        NAO's shoulder.
        """
        head_pos = np.array([0.0, self.config.user_height, 0.0])
        origin_pos = head_pos + np.array([
            self.config.shoulder_forward_offset,
            0.0,
            0.10,
        ])

        right_vec = np.array([1.0, 0.0, 0.0])
        shoulder_offset = self.config.shoulder_lateral_offset
        if user_hand == "left":
            origin_pos -= right_vec * shoulder_offset
        else:
            origin_pos += right_vec * shoulder_offset

        if self.config.target_coord_sys == "headset":
            origin_pos[1] = self.config.user_height
        elif self.config.target_coord_sys == "floor":
            origin_pos[1] = 0.0
        elif self.config.target_coord_sys == "ribs":
            origin_pos[1] = self.config.user_height - (self.config.shoulder_vertical_offset * 0.5)
        elif self.config.target_coord_sys == "hip":
            origin_pos[1] = self.config.user_height - self.config.shoulder_vertical_offset
        return origin_pos

    def compute_robot_target_matrix(
        self,
        hand_matrix_vr: np.ndarray,
        user_hand: str,
        R_hand_vr: np.ndarray | None = None,
        position_anchor: np.ndarray | None = None,
    ) -> np.ndarray:
        # Position: hand displacement from your shoulder, scaled down to NAO's
        # much smaller reach, then remapped from VR axes to the robot base.
        #   VR:    +X = right, +Y = up,   -Z = forward
        #   Robot: +X = forward, +Y = left, +Z = up
        rel_vr = (hand_matrix_vr[:3, 3] - self._torso_origin_vr(user_hand))
        rel_vr = rel_vr * self.config.target_position_scale
        pos_robot = R_VR_TO_ROBOT @ rel_vr
        # Anchor the displacement on the gripper's actual rest position so the
        # target starts on the robot (no constant offset) instead of at the base.
        if position_anchor is not None:
            pos_robot = pos_robot + position_anchor

        # Orientation: use the supplied VR hand frame (built so its +X points
        # where your fingers point), falling back to the raw wrist rotation.
        if R_hand_vr is None:
            R_hand_vr = hand_matrix_vr[:3, :3]
        # Optional local fine-tune in the gripper frame (config.wrist_offset_euler_deg).
        R_offset = R.from_euler("xyz", np.deg2rad(self.config.wrist_offset_euler_deg)).as_matrix()
        R_robot = R_VR_TO_ROBOT @ R_hand_vr @ R_offset

        T = np.eye(4)
        T[:3, :3] = R_robot
        T[:3, 3] = pos_robot
        return T

    def _hand_forward_vr(self, hand_data) -> np.ndarray | None:
        """Direction the fingers point, in VR world space, from WebXR joints.

        Uses wrist -> middle-finger knuckle (metacarpal), which stays stable even
        when the fingers curl into a fist. Returns None without joint data (e.g.
        motion controllers).
        """
        if hand_data is None or len(hand_data) < 25 * 16:
            return None
        joints = np.array(hand_data[: 25 * 16], dtype=float).reshape(25, 16)
        # Column-major 4x4 matrices: translation is elements 12, 13, 14.
        positions = joints[:, 12:15]
        forward = positions[self._PALM_REF_JOINT] - positions[0]  # middle MCP - wrist
        norm = np.linalg.norm(forward)
        if norm < 1e-6:
            return None
        return forward / norm

    def _hand_frame_vr(self, hand_data, hand_matrix_vr: np.ndarray) -> np.ndarray:
        """Right-handed VR frame whose +X axis points where the fingers point.

        +X is the accurate finger-pointing direction (from joint geometry when
        available). The remaining axes are anchored to the hand's *own* up vector
        (the wrist matrix +Y), so rolling your wrist rolls the gripper too. A
        world-up anchor would instead discard roll; using the hand's own up keeps
        full orientation (roll and pitch) while still putting red (+X) on your
        fingers.
        """
        forward = self._hand_forward_vr(hand_data)
        if forward is None:
            forward = -hand_matrix_vr[:3, 2]  # device -Z is "forward"
        norm = np.linalg.norm(forward)
        if norm < 1e-6:
            return hand_matrix_vr[:3, :3]
        x = forward / norm
        # Roll reference: the hand's own up axis (rolls with the wrist).
        up_ref = hand_matrix_vr[:3, 1]
        y = up_ref - np.dot(up_ref, x) * x
        if np.linalg.norm(y) < 1e-6:  # up_ref parallel to x: fall back to world-up
            up_ref = np.array([0.0, 1.0, 0.0])
            y = up_ref - np.dot(up_ref, x) * x
        y /= np.linalg.norm(y)
        z = np.cross(x, y)
        return np.column_stack([x, y, z])

    def _hand_open_from_joints(self, state: _SideState, hand_data) -> float | None:
        """Estimate how open the hand is (0 = fist, 1 = open) from WebXR finger joints.

        Returns None when full finger-joint data is not available (e.g. when
        using motion controllers) or before enough range has been observed to
        auto-calibrate. The metric is the mean fingertip-to-wrist distance,
        normalised by palm size so it is independent of hand size and tracking
        scale, then auto-scaled to [0, 1] from the open/closed extremes seen so far.
        """
        if hand_data is None or len(hand_data) < 25 * 16:
            return None
        joints = np.array(hand_data[: 25 * 16], dtype=float).reshape(25, 16)
        # Each 4x4 matrix is column-major, so the translation is elements 12, 13, 14.
        positions = joints[:, 12:15]
        wrist = positions[0]
        palm_scale = float(np.linalg.norm(positions[self._PALM_REF_JOINT] - wrist))
        if palm_scale < 1e-6:
            return None
        tip_dist = np.linalg.norm(positions[list(self._FINGERTIP_JOINTS)] - wrist, axis=1)
        curl = float(np.clip(tip_dist.mean() / palm_scale, 0.0, 5.0))

        if state.curl_min is None:
            state.curl_min = state.curl_max = curl
        state.curl_min = min(state.curl_min, curl)
        state.curl_max = max(state.curl_max, curl)
        span = state.curl_max - state.curl_min
        if span < 1e-3:
            return None  # need to see at least one open/close to calibrate
        return float(np.clip((curl - state.curl_min) / span, 0.0, 1.0))

    def _compute_hand_open(self, state: _SideState, grip_close: float, hand_data) -> float:
        """Resolve the target hand-open value in [0, 1] from the configured source."""
        open_frac = None
        if self.config.hand_control_source in ("auto", "fist"):
            open_frac = self._hand_open_from_joints(state, hand_data)
        if open_frac is None and self.config.hand_control_source != "fist":
            # Fall back to pinch (hand tracking) / trigger (controllers):
            # closing the pinch/trigger closes the hand.
            open_frac = 1.0 - grip_close
        if open_frac is None:
            open_frac = state.target_hand  # keep last command if fist not yet calibrated
        if self.config.invert_hand:
            open_frac = 1.0 - open_frac
        return float(np.clip(open_frac, 0.0, 1.0))

    def _update_target_from_vr(
        self, state: _SideState, hand_matrix_vr: np.ndarray, grip_close: float, hand_data=None
    ) -> None:
        # Build a hand frame whose +X points where the fingers point (so NAO's
        # gripper +X, the red axis in viser, follows your fingers) while keeping
        # the wrist roll, so rolling your hand rolls the gripper.
        R_hand_vr = self._hand_frame_vr(hand_data, hand_matrix_vr)

        # Show the orientation gizmo a little out in front of the hand (along the
        # pointing axis) so it is not buried inside the rendered hand mesh.
        viz_pos = hand_matrix_vr[:3, 3] + R_hand_vr[:, 0] * self.config.gizmo_forward_offset
        viz_euler = R.from_matrix(R_hand_vr).as_euler("xyz")
        T_robot = self.compute_robot_target_matrix(
            hand_matrix_vr, state.user_hand, R_hand_vr, position_anchor=state.rest_pos
        )
        pos = T_robot[:3, 3]
        quat_xyzw = R.from_matrix(T_robot[:3, :3]).as_quat()
        quat_wxyz = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])

        hand_open = self._compute_hand_open(state, grip_close, hand_data)
        grip_amount = 1.0 - hand_open  # 0 = fully open, 1 = fully closed
        alpha = float(np.clip(self.config.hand_smoothing, 0.0, 0.99))

        with self._lock:
            state.target_pos = pos
            state.viz_pos = viz_pos
            state.viz_rot = viz_euler
            # Track wrist orientation while the hand is open; freeze it as you
            # close into a grasp so the grip does not get disturbed.
            if self.config.track_orientation and grip_amount < self.config.pinch_deadzone:
                state.target_wxyz = quat_wxyz
            state.target_hand = alpha * state.target_hand + (1.0 - alpha) * hand_open

    def _vuer_worker(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        app = Vuer(
            host=self.config.vuer_host,
            port=self.config.vuer_port,
            cert=self.config.vuer_cert,
            key=self.config.vuer_key,
        )

        @app.add_handler("HAND_MOVE")
        async def on_hand_move(event, session):
            # Update every controlled arm from the matching user hand.
            for user_hand, state in self._side_by_user_hand.items():
                hand_data = event.value.get(user_hand)
                if not hand_data or len(hand_data) < 16:
                    continue
                hand_matrix_vr = np.array(hand_data[:16]).reshape(4, 4).T
                hand_state = event.value.get(f"{user_hand}State", {})
                pinch_val = hand_state.get("pinch", hand_state.get("pinchStrength", 0.0))
                self._update_target_from_vr(state, hand_matrix_vr, float(pinch_val), hand_data=hand_data)

        @app.add_handler("CONTROLLER_MOVE")
        async def on_controller_move(event, session):
            for user_hand, state in self._side_by_user_hand.items():
                controller_data = event.value.get(user_hand)
                if not controller_data or len(controller_data) < 16:
                    continue
                hand_matrix_vr = np.array(controller_data[:16]).reshape(4, 4).T

                # Apply controller offset to align with wrist pose
                T_offset = np.eye(4)
                T_offset[:3, :3] = R.from_euler("x", self.config.controller_pitch_offset_deg, degrees=True).as_matrix()
                T_offset[2, 3] = self.config.controller_z_offset_m
                hand_matrix_vr = hand_matrix_vr @ T_offset

                ctrl_state = event.value.get(f"{user_hand}State", {})
                trigger_val = ctrl_state.get("triggerValue", ctrl_state.get("squeezeValue", 0.0))
                # Controllers have no finger joints, so fall back to the trigger.
                self._update_target_from_vr(state, hand_matrix_vr, float(trigger_val), hand_data=None)

        @app.spawn(start=True)
        async def main(session: VuerSession):
            session.set @ DefaultScene()
            session.upsert(Hands(stream=True, key="hands", showLeft=True, showRight=True), to="bgChildren")
            session.upsert(MotionControllers(stream=True, key="motionControllers", left=True, right=True), to="bgChildren")
            while self._is_connected:
                with self._lock:
                    current_img = None if self._latest_frame is None else self._latest_frame.copy()
                    viz = [
                        (s.side, s.viz_pos.copy(), s.viz_rot.copy(), s.nao_viz_rot.copy())
                        for s in self._sides
                    ]

                if self.config.show_orientation_gizmos:
                    for side, viz_pos, viz_rot, nao_viz_rot in viz:
                        # Short axes: the orientation of your hand.
                        session.upsert(
                            CoordsMarker(
                                position=viz_pos.tolist(),
                                rotation=viz_rot.tolist(),
                                scale=0.12,
                                headScale=1.0,
                                key=f"hand_gizmo_{side}",
                            ),
                            to="bgChildren",
                        )
                        # Long axes (same origin): NAO's actual gripper orientation.
                        # Line them up with the short axes for the most natural grip;
                        # tune --teleop.wrist_offset_euler_deg if they stay twisted.
                        session.upsert(
                            CoordsMarker(
                                position=viz_pos.tolist(),
                                rotation=nao_viz_rot.tolist(),
                                scale=0.22,
                                headScale=2.0,
                                key=f"nao_gizmo_{side}",
                            ),
                            to="bgChildren",
                        )

                if current_img is not None:
                    # Fixed screen anchored in front of the headset (HUD), matching
                    # the SO101 Vuer placement so it is visible in immersive VR.
                    session.upsert(
                        ImageBackground(
                            current_img,
                            format="jpeg",
                            quality=50,
                            fixed=True,
                            distanceToCamera=self.config.camera_distance_to_user,
                            key="camera_feed",
                            position=[
                                self.config.camera_lateral_offset,
                                self.config.user_height + self.config.camera_height_offset,
                                self.config.camera_screen_z,
                            ],
                        ),
                        to="bgChildren",
                    )
                await asyncio.sleep(0.033)

        app.run()

    def _ik_worker(self):
        while self._is_connected:
            cfg = np.array(self._prev_cfg, copy=True)
            action = {}
            for state in self._sides:
                with self._lock:
                    target_pos = state.target_pos.copy()
                    target_quat = state.target_wxyz.copy()
                    hand_value = state.target_hand

                q_sol = solve_ik(
                    robot=self.robot,
                    target_link_name=state.target_link_name,
                    target_position=target_pos,
                    target_wxyz=target_quat,
                    joint_mask=state.joint_mask,
                    prev_cfg=cfg,
                )

                # Keep joints outside this arm at their previous values so each
                # arm only moves its own joints.
                for idx, mask_value in enumerate(state.joint_mask):
                    if mask_value == 0.0:
                        q_sol[idx] = cfg[idx]
                cfg = np.array(q_sol, copy=True)

                action.update(self._side_action(state, cfg, hand_value))
                if state.gizmo is not None:
                    state.gizmo.position = target_pos
                    state.gizmo.wxyz = target_quat

            self._prev_cfg = cfg
            if self.urdf_vis is not None:
                self.urdf_vis.update_cfg(self._prev_cfg)

            # NAO's actually-achieved gripper orientation per arm, mapped back
            # into VR space so it can be drawn next to your hand for comparison.
            nao_rots = {}
            if self.config.show_orientation_gizmos:
                link_poses = np.array(self.robot.forward_kinematics(cfg))
                for state in self._sides:
                    w, x, y, z = link_poses[state.target_link_index, :4]
                    R_robot = R.from_quat([x, y, z, w]).as_matrix()
                    R_vr = R_VR_TO_ROBOT.T @ R_robot
                    nao_rots[state.side] = R.from_matrix(R_vr).as_euler("xyz")

            with self._lock:
                self._latest_q_sol = np.array(cfg, copy=True)
                self._latest_action = action
                for state in self._sides:
                    if state.side in nao_rots:
                        state.nao_viz_rot = nao_rots[state.side]

            time.sleep(0.01)

    def _camera_worker(self):
        from lerobot.robots.robot import Robot as BaseRobot

        active_robot = None
        while self._is_connected and self.config.enable_camera_feed:
            try:
                if active_robot is None:
                    for obj in gc.get_objects():
                        if isinstance(obj, BaseRobot) and getattr(obj, "is_connected", False):
                            active_robot = obj
                            break
                if active_robot is not None and hasattr(active_robot, "get_observation"):
                    obs = active_robot.get_observation()
                    # Prefer the NAO head camera, but fall back to any image-shaped
                    # observation so the feed works regardless of the exact key.
                    image = obs.get("nao_head_rgb")
                    if not (isinstance(image, np.ndarray) and image.ndim == 3):
                        image = next(
                            (v for v in obs.values() if isinstance(v, np.ndarray) and v.ndim == 3),
                            None,
                        )
                    if isinstance(image, np.ndarray) and image.ndim == 3:
                        if image.dtype != np.uint8:
                            image = (np.clip(image, 0, 1) * 255).astype(np.uint8)
                        image = cv2.resize(image.copy(), (320, 240))
                        with self._lock:
                            self._latest_frame = image
            except Exception:
                pass
            time.sleep(0.033)

    def _build_sides(self) -> None:
        """Create one _SideState (target + mask + optional gizmo) per controlled arm."""
        self._sides = []
        self._side_by_user_hand = {}
        single_arm = self.config.arm.lower() != "both"
        initial_wxyz = np.array(self.config.initial_target_wxyz, dtype=float)
        # Gripper rest poses at the default config, so each target can be anchored
        # on the robot instead of jumping to the base origin at startup.
        rest_link_poses = np.array(self.robot.forward_kinematics(self._prev_cfg))

        for side in sides_for_arm(self.config.arm):
            if single_arm:
                user_hand = self.config.user_hand
                target_link_name = self.config.target_link_name
                hand_joint_name = self.config.hand_joint_name
                initial_position = self.config.initial_target_position
            else:
                # left arm <- left hand, right arm <- right hand.
                user_hand = side
                target_link_name = ARM_TARGET_LINK_NAME[side]
                hand_joint_name = ARM_HAND_JOINT_NAME[side]
                initial_position = ARM_INITIAL_TARGET_POSITION[side]

            arm_joint_names = self._arm_joint_names_for(side)
            joint_mask = self._make_joint_mask(arm_joint_names, hand_joint_name)
            target_link_index = self.robot.links.names.index(target_link_name)
            rest_pos = np.array(rest_link_poses[target_link_index, 4:7], dtype=float)

            gizmo = None
            if self.config.enable_visualization and self.viser_server is not None:
                gizmo = self.viser_server.scene.add_transform_controls(
                    f"/ik_target_{side}",
                    scale=0.1,
                    position=rest_pos,
                    wxyz=self.config.initial_target_wxyz,
                )

            state = _SideState(
                side=side,
                user_hand=user_hand,
                arm_joint_names=arm_joint_names,
                target_link_name=target_link_name,
                hand_joint_name=hand_joint_name,
                joint_mask=joint_mask,
                target_pos=rest_pos.copy(),
                target_wxyz=initial_wxyz.copy(),
                gizmo=gizmo,
                target_link_index=target_link_index,
                rest_pos=rest_pos,
                viz_pos=np.array(initial_position, dtype=float),
            )
            self._sides.append(state)
            self._side_by_user_hand[user_hand] = state

            # Warm up the JAX solver for this arm so the worker loop is responsive.
            solve_ik(
                robot=self.robot,
                target_link_name=target_link_name,
                target_position=np.array(initial_position, dtype=float),
                target_wxyz=initial_wxyz,
                joint_mask=joint_mask,
                prev_cfg=self._prev_cfg,
            )

    def connect(self) -> None:
        self.configure()
        self.urdf = self._load_urdf()
        self.robot = pk.Robot.from_urdf(self.urdf)
        self._prev_cfg = np.array(self.robot.joint_var_cls(0).default_factory(), copy=True)

        if self.config.enable_visualization:
            self.viser_server = viser.ViserServer(port=self.config.viser_port)
            self.viser_server.scene.add_grid("/ground", width=2, height=2)
            self.urdf_vis = ViserUrdf(self.viser_server, self.urdf, root_node_name="/nao")
            self.urdf_vis.update_cfg(self._prev_cfg)

        self._build_sides()

        self._is_connected = True
        self._ik_thread = threading.Thread(target=self._ik_worker, daemon=True)
        self._vuer_thread = threading.Thread(target=self._vuer_worker, daemon=True)
        self._ik_thread.start()
        self._vuer_thread.start()

        if self.config.enable_camera_feed:
            self._cam_thread = threading.Thread(target=self._camera_worker, daemon=True)
            self._cam_thread.start()

    def disconnect(self) -> None:
        self._is_connected = False
        if self.viser_server:
            self.viser_server.stop()
        if self._ik_thread:
            self._ik_thread.join(timeout=1.0)

    def get_action(self) -> dict[str, float]:
        with self._lock:
            return self._latest_action.copy()

    @property
    def action_features(self) -> dict:
        features = {}
        for side in sides_for_arm(self.config.arm):
            for joint in self._arm_joint_names_for(side):
                features[f"{joint}.pos"] = float
            if self.config.arm.lower() == "both":
                hand_joint_name = ARM_HAND_JOINT_NAME[side]
            else:
                hand_joint_name = self.config.hand_joint_name
            if hand_joint_name:
                features[f"{hand_joint_name}.pos"] = float
        return features

    @property
    def feedback_features(self) -> dict:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        pass

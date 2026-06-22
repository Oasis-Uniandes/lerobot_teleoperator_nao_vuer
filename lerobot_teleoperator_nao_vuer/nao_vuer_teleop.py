import asyncio
import gc
import threading
import time
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
from vuer.schemas import CoordsMarker, Hands, ImageBackground, MotionControllers, Scene
from viser.extras import ViserUrdf
import yourdfpy
from yourdfpy import URDF

from .config_nao_vuer_teleop import NaoVuerTeleopConfig
from .pyroki_snippets import solve_ik


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

        self._target_pos = np.array(config.initial_target_position, dtype=float)
        self._target_wxyz = np.array(config.initial_target_wxyz, dtype=float)
        self._target_hand = 1.0  # start with an open hand
        # Auto-calibration range for whole-hand fist tracking.
        self._curl_min = None
        self._curl_max = None
        self._viz_pos = np.array([0.0, 0.0, 0.0])
        self._viz_rot = np.array([0.0, 0.0, 0.0])
        self._latest_q_sol = None
        self._latest_frame = None
        self._latest_action = {}
        self._prev_cfg = None
        self.viser_server = None
        self.urdf_vis = None
        self.ik_web_target = None

    def configure(self) -> None:
        if self.config.arm.lower() not in {"left", "right"}:
            raise ValueError("arm must be 'left' or 'right'.")
        if self.config.user_hand.lower() not in {"left", "right"}:
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

    def _arm_joint_names(self) -> tuple[str, ...]:
        if self.config.arm.lower() == "left":
            return self.config.left_arm_joint_names
        return self.config.right_arm_joint_names

    def _make_joint_mask(self) -> np.ndarray:
        selected = set(self._arm_joint_names())
        if self.config.hand_joint_name:
            selected.add(self.config.hand_joint_name)
        return np.array([
            1.0 if joint_name in selected else 0.0
            for joint_name in self.robot.joints.actuated_names
        ])

    def _solution_to_action(self, q_sol: np.ndarray, hand_value: float) -> dict[str, float]:
        action = {}
        for joint_name in self._arm_joint_names():
            if joint_name in self.robot.joints.actuated_names:
                idx = self.robot.joints.actuated_names.index(joint_name)
                action[f"{joint_name}.pos"] = float(q_sol[idx])
        if self.config.hand_joint_name in self.robot.joints.actuated_names:
            idx = self.robot.joints.actuated_names.index(self.config.hand_joint_name)
            lower = float(self.robot.joints.lower_limits[idx])
            upper = float(self.robot.joints.upper_limits[idx])
            action[f"{self.config.hand_joint_name}.pos"] = lower + hand_value * (upper - lower)
        return action

    def _torso_origin_vr(self) -> np.ndarray:
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
        if self.config.user_hand == "left":
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

    def compute_robot_target_matrix(self, hand_matrix_vr: np.ndarray) -> np.ndarray:
        T_torso_vr = np.eye(4)
        T_torso_vr[:3, 3] = self._torso_origin_vr()
        T_hand_torso = np.linalg.inv(T_torso_vr) @ hand_matrix_vr

        # Pure axis remap from VR to the robot torso frame:
        #   VR:    +X = right, +Y = up,   -Z = forward
        #   Robot: +X = forward, +Y = left, +Z = up
        # This is what makes teleop intuitive: hand forward -> gripper forward,
        # hand up -> gripper up, hand to your right -> gripper to NAO's right.
        # (The previous code mixed an extra global yaw into this, which made
        # "hand forward" come out as "gripper to the side".)
        R_vr_to_robot = np.array([
            [0, 0, -1],
            [-1, 0, 0],
            [0, 1, 0],
        ])
        T_vr_to_robot = np.eye(4)
        T_vr_to_robot[:3, :3] = R_vr_to_robot
        T_hand_robot = T_vr_to_robot @ T_hand_torso

        # Local alignment offset so the gripper link lines up with the natural
        # grip of your hand. Position is unaffected by this (it is a rotation in
        # the gripper's own frame); tune via config.wrist_offset_euler_deg.
        offset = R.from_euler(
            "xyz", np.deg2rad(self.config.wrist_offset_euler_deg)
        ).as_matrix()
        T_offset = np.eye(4)
        T_offset[:3, :3] = offset
        return T_hand_robot @ T_offset

    def _hand_open_from_joints(self, hand_data) -> float | None:
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

        if self._curl_min is None:
            self._curl_min = self._curl_max = curl
        self._curl_min = min(self._curl_min, curl)
        self._curl_max = max(self._curl_max, curl)
        span = self._curl_max - self._curl_min
        if span < 1e-3:
            return None  # need to see at least one open/close to calibrate
        return float(np.clip((curl - self._curl_min) / span, 0.0, 1.0))

    def _compute_hand_open(self, grip_close: float, hand_data) -> float:
        """Resolve the target hand-open value in [0, 1] from the configured source."""
        open_frac = None
        if self.config.hand_control_source in ("auto", "fist"):
            open_frac = self._hand_open_from_joints(hand_data)
        if open_frac is None and self.config.hand_control_source != "fist":
            # Fall back to pinch (hand tracking) / trigger (controllers):
            # closing the pinch/trigger closes the hand.
            open_frac = 1.0 - grip_close
        if open_frac is None:
            open_frac = self._target_hand  # keep last command if fist not yet calibrated
        if self.config.invert_hand:
            open_frac = 1.0 - open_frac
        return float(np.clip(open_frac, 0.0, 1.0))

    def _update_target_from_vr(self, hand_matrix_vr: np.ndarray, grip_close: float, hand_data=None) -> None:
        viz_pos = hand_matrix_vr[:3, 3]
        viz_euler = R.from_matrix(hand_matrix_vr[:3, :3]).as_euler("xyz")
        T_robot = self.compute_robot_target_matrix(hand_matrix_vr)
        pos = T_robot[:3, 3]
        quat_xyzw = R.from_matrix(T_robot[:3, :3]).as_quat()
        quat_wxyz = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])

        hand_open = self._compute_hand_open(grip_close, hand_data)
        grip_amount = 1.0 - hand_open  # 0 = fully open, 1 = fully closed
        alpha = float(np.clip(self.config.hand_smoothing, 0.0, 0.99))

        with self._lock:
            self._target_pos = pos
            self._viz_pos = viz_pos
            self._viz_rot = viz_euler
            # Track wrist orientation while the hand is open; freeze it as you
            # close into a grasp so the grip does not get disturbed.
            if self.config.track_orientation and grip_amount < self.config.pinch_deadzone:
                self._target_wxyz = quat_wxyz
            self._target_hand = alpha * self._target_hand + (1.0 - alpha) * hand_open

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
            hand_data = event.value.get(self.config.user_hand)
            if not hand_data or len(hand_data) < 16:
                return
            hand_matrix_vr = np.array(hand_data[:16]).reshape(4, 4).T
            hand_state = event.value.get(f"{self.config.user_hand}State", {})
            pinch_val = hand_state.get("pinch", hand_state.get("pinchStrength", 0.0))
            self._update_target_from_vr(hand_matrix_vr, float(pinch_val), hand_data=hand_data)

        @app.add_handler("CONTROLLER_MOVE")
        async def on_controller_move(event, session):
            controller_data = event.value.get(self.config.user_hand)
            if not controller_data or len(controller_data) < 16:
                return
            hand_matrix_vr = np.array(controller_data[:16]).reshape(4, 4).T
            state = event.value.get(f"{self.config.user_hand}State", {})
            trigger_val = state.get("triggerValue", state.get("squeezeValue", 0.0))
            # Controllers have no finger joints, so fall back to the trigger.
            self._update_target_from_vr(hand_matrix_vr, float(trigger_val), hand_data=None)

        @app.spawn(start=True)
        async def main(session: VuerSession):
            session.set(Scene())
            session.upsert(Hands(stream=True, key="hands", showLeft=True, showRight=True), to="bgChildren")
            session.upsert(MotionControllers(stream=True, key="motionControllers", left=True, right=True), to="bgChildren")
            while self._is_connected:
                with self._lock:
                    current_img = None if self._latest_frame is None else self._latest_frame.copy()
                    viz_pos = self._viz_pos.copy()
                    viz_rot = self._viz_rot.copy()

                session.upsert(
                    CoordsMarker(
                        position=viz_pos.tolist(),
                        rotation=viz_rot.tolist(),
                        scale=0.15,
                        key="ik_gizmo",
                    ),
                    to="bgChildren",
                )

                if current_img is not None:
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
                                self.config.camera_height_offset,
                                -self.config.camera_distance_to_user,
                            ],
                        ),
                        to="bgChildren",
                    )
                await asyncio.sleep(0.033)

        app.run()

    def _ik_worker(self):
        while self._is_connected:
            with self._lock:
                target_pos = self._target_pos.copy()
                target_quat = self._target_wxyz.copy()
                hand_value = self._target_hand

            q_sol = solve_ik(
                robot=self.robot,
                target_link_name=self.config.target_link_name,
                target_position=target_pos,
                target_wxyz=target_quat,
                joint_mask=self.joint_mask,
                prev_cfg=self._prev_cfg,
            )

            for idx, mask_value in enumerate(self.joint_mask):
                if mask_value == 0.0:
                    q_sol[idx] = self._prev_cfg[idx]

            self._prev_cfg = np.array(q_sol, copy=True)
            if self.urdf_vis is not None:
                self.urdf_vis.update_cfg(self._prev_cfg)
            if self.ik_web_target is not None:
                self.ik_web_target.position = target_pos
                self.ik_web_target.wxyz = target_quat
            with self._lock:
                self._latest_q_sol = np.array(q_sol, copy=True)
                self._latest_action = self._solution_to_action(self._latest_q_sol, hand_value)

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
                    image = obs.get("nao_head_rgb")
                    if isinstance(image, np.ndarray) and image.ndim == 3:
                        if image.dtype != np.uint8:
                            image = (np.clip(image, 0, 1) * 255).astype(np.uint8)
                        image = cv2.resize(image.copy(), (320, 240))
                        with self._lock:
                            self._latest_frame = image
            except Exception:
                pass
            time.sleep(0.033)

    def connect(self) -> None:
        self.configure()
        self.urdf = self._load_urdf()
        self.robot = pk.Robot.from_urdf(self.urdf)
        self.joint_mask = self._make_joint_mask()
        self._prev_cfg = np.array(self.robot.joint_var_cls(0).default_factory(), copy=True)

        if self.config.enable_visualization:
            self.viser_server = viser.ViserServer(port=self.config.viser_port)
            self.viser_server.scene.add_grid("/ground", width=2, height=2)
            self.urdf_vis = ViserUrdf(self.viser_server, self.urdf, root_node_name="/nao")
            self.urdf_vis.update_cfg(self._prev_cfg)
            self.ik_web_target = self.viser_server.scene.add_transform_controls(
                "/ik_target",
                scale=0.1,
                position=self.config.initial_target_position,
                wxyz=self.config.initial_target_wxyz,
            )

        solve_ik(
            robot=self.robot,
            target_link_name=self.config.target_link_name,
            target_position=np.array(self.config.initial_target_position, dtype=float),
            target_wxyz=np.array(self.config.initial_target_wxyz, dtype=float),
            joint_mask=self.joint_mask,
            prev_cfg=self._prev_cfg,
        )

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
        features = {f"{joint}.pos": float for joint in self._arm_joint_names()}
        if self.config.hand_joint_name:
            features[f"{self.config.hand_joint_name}.pos"] = float
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

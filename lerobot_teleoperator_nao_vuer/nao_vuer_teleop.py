import asyncio
import gc
import threading
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import pyroki as pk
from lerobot.teleoperators.teleoperator import Teleoperator
from robot_descriptions.loaders.yourdfpy import load_robot_description
from scipy.spatial.transform import Rotation as R
from vuer import Vuer, VuerSession
from vuer.schemas import CoordsMarker, Hands, ImageBackground, MotionControllers, Scene
import yourdfpy
from yourdfpy import URDF

from .config_nao_vuer_teleop import NaoVuerTeleopConfig
from .pyroki_snippets import solve_ik


class NaoVuerTeleop(Teleoperator):
    config_class = NaoVuerTeleopConfig
    name = "nao_vuer"

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
        self._target_hand = 0.0
        self._viz_pos = np.array([0.0, 0.0, 0.0])
        self._viz_rot = np.array([0.0, 0.0, 0.0])
        self._latest_q_sol = None
        self._latest_frame = None
        self._latest_action = {}
        self._prev_cfg = None

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

    def compute_robot_target_matrix(self, hand_matrix_vr: np.ndarray) -> np.ndarray:
        head_pos = np.array([0.0, self.config.user_height, 0.0])
        origin_pos = head_pos + np.array([0.0, 0.0, 0.10])

        right_vec = np.array([1.0, 0.0, 0.0])
        shoulder_offset = 0.16
        if self.config.user_hand == "left":
            origin_pos -= right_vec * shoulder_offset
        else:
            origin_pos += right_vec * shoulder_offset

        if self.config.target_coord_sys == "headset":
            origin_pos[1] = self.config.user_height
        elif self.config.target_coord_sys == "floor":
            origin_pos[1] = 0.0
        elif self.config.target_coord_sys == "ribs":
            origin_pos[1] = self.config.user_height - 0.35
        elif self.config.target_coord_sys == "hip":
            origin_pos[1] = self.config.user_height - 0.65

        T_torso_vr = np.eye(4)
        T_torso_vr[:3, 3] = origin_pos
        T_hand_torso = np.linalg.inv(T_torso_vr) @ hand_matrix_vr

        R_vr_to_robot = np.array([
            [0, 0, -1],
            [-1, 0, 0],
            [0, 1, 0],
        ])
        yaw_offset = -np.pi / 2 if self.config.arm.lower() == "right" else np.pi / 2
        R_yaw = np.array([
            [np.cos(yaw_offset), -np.sin(yaw_offset), 0],
            [np.sin(yaw_offset), np.cos(yaw_offset), 0],
            [0, 0, 1],
        ])
        T_vr_to_robot = np.eye(4)
        T_vr_to_robot[:3, :3] = R_yaw @ R_vr_to_robot

        T_hand_robot = T_vr_to_robot @ T_hand_torso
        hand_offset = R.from_euler("z", -np.pi / 2).as_matrix()
        T_offset = np.eye(4)
        T_offset[:3, :3] = hand_offset
        return T_hand_robot @ T_offset

    def _update_target_from_vr(self, hand_matrix_vr: np.ndarray, pinch_val: float) -> None:
        viz_pos = hand_matrix_vr[:3, 3]
        viz_euler = R.from_matrix(hand_matrix_vr[:3, :3]).as_euler("xyz")
        T_robot = self.compute_robot_target_matrix(hand_matrix_vr)
        pos = T_robot[:3, 3]
        quat_xyzw = R.from_matrix(T_robot[:3, :3]).as_quat()
        quat_wxyz = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])

        with self._lock:
            self._target_pos = pos
            self._viz_pos = viz_pos
            self._viz_rot = viz_euler
            if pinch_val < self.config.pinch_deadzone:
                self._target_wxyz = quat_wxyz
            self._target_hand = float(np.clip(1.0 - pinch_val, 0.0, 1.0))

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
            self._update_target_from_vr(hand_matrix_vr, float(pinch_val))

        @app.add_handler("CONTROLLER_MOVE")
        async def on_controller_move(event, session):
            controller_data = event.value.get(self.config.user_hand)
            if not controller_data or len(controller_data) < 16:
                return
            hand_matrix_vr = np.array(controller_data[:16]).reshape(4, 4).T
            state = event.value.get(f"{self.config.user_hand}State", {})
            pinch_val = state.get("triggerValue", state.get("squeezeValue", 0.0))
            self._update_target_from_vr(hand_matrix_vr, float(pinch_val))

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
                            distanceToCamera=1,
                            key="camera_feed",
                            position=[0, self.config.user_height - 0.6, -3],
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

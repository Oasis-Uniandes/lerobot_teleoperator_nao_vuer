from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("nao_vuer")
@dataclass
class NaoVuerTeleopConfig(TeleoperatorConfig):
    arm: str = "right"
    urdf_name: str = ""
    urdf_path: str = ""
    target_link_name: str = "r_gripper"
    hand_joint_name: str = "RHand"

    robot_ip: str = "127.0.0.1"
    robot_port: int = 9559
    app_name: str = "lerobot_nao_vuer"

    disable_autonomous_life: bool = True
    use_startup_posture: bool = True
    startup_posture: str = "StandInit"
    startup_posture_speed: float = 0.2
    startup_settle_time_s: float = 1.5
    stiffness: float = 1.0
    speed_fraction: float = 0.15
    connect_timeout_s: float = 10.0

    initial_target_position: tuple[float, float, float] = (0.18, -0.15, 0.35)
    initial_target_wxyz: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)

    user_hand: str = "right"
    target_coord_sys: str = "hip"
    user_height: float = 1.40
    shoulder_lateral_offset: float = 0.20
    shoulder_vertical_offset: float = 0.70
    shoulder_forward_offset: float = 0.10
    vuer_host: str = "0.0.0.0"
    vuer_port: int = 8012
    vuer_cert: str = "./cert.pem"
    vuer_key: str = "./key.pem"
    enable_visualization: bool = False
    viser_port: int = 8080
    enable_camera_feed: bool = False
    camera_distance_to_user: float = 0.8
    camera_height_offset: float = -0.15
    camera_lateral_offset: float = 0.0
    pinch_deadzone: float = 0.2

    # --- Wrist / gripper orientation ---
    # When True, the gripper follows the orientation of your tracked hand.
    # Position always follows your hand directly (see compute_robot_target_matrix).
    track_orientation: bool = True
    # Local alignment offset (degrees, applied in the gripper frame) so the NAO
    # gripper lines up with the natural grip of your hand. Tune this if the
    # gripper points in a slightly wrong direction while position feels correct.
    wrist_offset_euler_deg: tuple[float, float, float] = (0.0, 0.0, 0.0)

    # --- Hand open / close ---
    # "auto"  : use whole-hand fist tracking when available, else pinch/trigger.
    # "fist"  : only use whole-hand fist tracking (no fallback).
    # "pinch" : only use the pinch (hand tracking) or trigger (controllers).
    hand_control_source: str = "pinch"
    # Flip the open/close direction if your robot hand closes when you open yours.
    invert_hand: bool = False
    # Exponential smoothing for the hand command in [0, 1). Higher is smoother
    # but laggier; 0.0 disables smoothing.
    hand_smoothing: float = 0.5

    right_arm_joint_names: tuple[str, ...] = (
        "RShoulderPitch",
        "RShoulderRoll",
        "RElbowYaw",
        "RElbowRoll",
        "RWristYaw",
    )
    left_arm_joint_names: tuple[str, ...] = (
        "LShoulderPitch",
        "LShoulderRoll",
        "LElbowYaw",
        "LElbowRoll",
        "LWristYaw",
    )

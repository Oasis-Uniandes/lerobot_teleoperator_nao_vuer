from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig

# Arm-dependent defaults, keyed by side. These must match the joint names the
# downstream NAO robot expects for the corresponding arm.
ARM_TARGET_LINK_NAME = {"left": "l_gripper", "right": "r_gripper"}
ARM_HAND_JOINT_NAME = {"left": "LHand", "right": "RHand"}
# Per-side default position for the IK target (NAO frame, metres). The left arm
# sits on +y, the right arm on -y.
ARM_INITIAL_TARGET_POSITION = {
    "left": (0.18, 0.15, 0.35),
    "right": (0.18, -0.15, 0.35),
}

VALID_ARMS = ("left", "right", "both")


def sides_for_arm(arm: str) -> tuple[str, ...]:
    """Return the individual arm sides controlled for a given `arm` setting."""
    arm = arm.lower()
    if arm == "both":
        return ("left", "right")
    return (arm,)


@TeleoperatorConfig.register_subclass("nao_vuer")
@dataclass
class NaoVuerTeleopConfig(TeleoperatorConfig):
    # 'left', 'right' or 'both'.
    arm: str = "right"
    urdf_name: str = ""
    urdf_path: str = ""
    # Arm-dependent fields. Leave as None to derive them from `arm`; set
    # explicitly to override (single-arm only). For 'both' they are resolved
    # per-side in the teleoperator.
    target_link_name: str | None = None
    hand_joint_name: str | None = None

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

    # NAO is a small robot (~0.22 m arm reach) but VR hands move at human scale
    # (~0.6 m reach). This factor shrinks the hand displacement measured from
    # your shoulder down to NAO's workspace, so your full range of motion maps
    # into the arm's reachable range instead of saturating at the joint limits.
    target_position_scale: float = 0.4

    # Which of the user's hands drives the arm. Leave as None to follow `arm`
    # (left arm <- left hand, right arm <- right hand). Ignored for 'both',
    # where each NAO arm is driven by the matching user hand.
    user_hand: str | None = None
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
    # Draw orientation pointer gizmos in the VR scene: short axes for your hand
    # and long axes for NAO's achieved gripper orientation, both just in front of
    # your hand. The red (+X) axis points where your fingers point.
    show_orientation_gizmos: bool = True
    # How far in front of the hand (metres, along the pointing axis) to place the
    # gizmos so they are not hidden inside the rendered hand mesh.
    gizmo_forward_offset: float = 0.12
    enable_camera_feed: bool = True
    # NAO head-camera feed shown as a fixed screen in front of you in VR (same
    # placement as the SO101 Vuer teleop). The screen sits at
    # [lateral, user_height + height_offset, screen_z], distanceToCamera metres away.
    camera_distance_to_user: float = 1.0
    camera_height_offset: float = -0.6
    camera_lateral_offset: float = 0.0
    camera_screen_z: float = -3.0
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

    def __post_init__(self) -> None:
        arm = self.arm.lower()
        if arm not in VALID_ARMS:
            raise ValueError(
                f"Unsupported arm '{self.arm}'. Expected 'left', 'right' or 'both'."
            )
        # Resolve arm-dependent fields from `arm` unless explicitly overridden.
        # For 'both', target_link_name / hand_joint_name / user_hand stay
        # per-side (resolved in the teleoperator) and are left unset here.
        if arm != "both":
            if self.target_link_name is None:
                self.target_link_name = ARM_TARGET_LINK_NAME[arm]
            if self.hand_joint_name is None:
                self.hand_joint_name = ARM_HAND_JOINT_NAME[arm]
            if self.user_hand is None:
                self.user_hand = arm

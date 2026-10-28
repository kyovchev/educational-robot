from dataclasses import dataclass, field
from .constants import POS_CENTER, NUM_JOINTS, MIN_ANGLE, MAX_ANGLE, SPEED, ACC


@dataclass
class JointConfig:
    servo_id:    int = 1
    zero_offset: int = POS_CENTER
    min_angle:   float = MIN_ANGLE
    max_angle:   float = MAX_ANGLE
    home_angle:  float = 0.0
    speed:       int = SPEED
    acc:         int = ACC


@dataclass
class RobotConfig:
    joints: list = field(
        default_factory=lambda: [JointConfig(
            servo_id=i + 1) for i in range(NUM_JOINTS)]
    )
    port:     str = ""
    baud:     int = 1_000_000
    tcp_host: str = "192.168.1.100"
    tcp_port: int = 8888


@dataclass
class Waypoint:
    name:     str
    angles:   list    # logical degrees for all joints — the only thing saved to disk
    duration: float = 1.0

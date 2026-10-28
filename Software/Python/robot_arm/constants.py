# ─────────────────────────────────────────────
#  STS3215 Protocol Constants
# ─────────────────────────────────────────────
STS_HEADER = 0xFF
STS_BROADCAST_ID = 0xFE

INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03
INST_SYNC_WRITE = 0x83

REG_ID = 0x05
REG_EEPROM_MIN_ANGLE = 0x09   # EEPROM min angle limit (2 bytes)
REG_EEPROM_MAX_ANGLE = 0x0B   # EEPROM max angle limit (2 bytes)
# EEPROM max speed limit (2 bytes) — persists across reboots
REG_EEPROM_MAX_SPEED = 0x0F
# decimal 40 — also used for one-key calibration (write 128)
REG_TORQUE_ENABLE = 0x28
REG_LOCK = 0x37   # decimal 55 — RAM lock: 0=EEPROM writes allowed, 1=locked
# decimal 41 — acceleration (1 byte); WritePos starts here
REG_GOAL_ACC = 0x29
# decimal 42 — goal position (2 bytes, little-endian)
REG_GOAL_POSITION = 0x2A
REG_GOAL_SPEED = 0x2E   # decimal 46 — goal speed   (2 bytes, little-endian)
REG_PRESENT_POSITION = 0x38   # decimal 56
REG_MOVING = 0x3A   # decimal 58 — 1=moving, 0=reached target
REG_PRESENT_TEMP = 0x3F   # decimal 63
REG_PRESENT_VOLTAGE = 0x3E   # decimal 62

# One-key calibration: write 128 to REG_TORQUE_ENABLE (after EEPROM unlock)
CALIBRATE_CMD = 128

POS_MIN = 0
POS_MAX = 4095
DEG_MAX = 360.0
POS_CENTER = 2047   # maps to 0° logical
MIN_ANGLE = -130
MAX_ANGLE = 130
SPEED = 1500
ACC = 200

NUM_ARM_JOINTS = 6   # revolute joints used in FK/IK
JOINT_NAMES = [f"Joint {i+1}" for i in range(6)] + ["Gripper"]
NUM_JOINTS = len(JOINT_NAMES)  # 7 total

# ─────────────────────────────────────────────
#  Theme palettes
# ─────────────────────────────────────────────
THEMES = {
    "dark": {
        "BG":         "#1a1a2e",
        "PANEL":      "#16213e",
        "ACCENT":     "#0f3460",
        "ORANGE":     "#e94560",
        "GREEN":      "#4ecca3",
        "TEXT":       "#eaeaea",
        "SUBTEXT":    "#a0a0b0",
        "ENTRY":      "#0d2137",
        "DANGER_BTN": "#8b0000",
        "GREEN_BTN":  "#1a6b4a",
        "SEL_BG":     "#0f3460",
    },
    "light": {
        "BG":         "#f0f2f5",
        "PANEL":      "#dde1ea",
        "ACCENT":     "#4a90d9",
        "ORANGE":     "#d0392b",
        "GREEN":      "#217a52",
        "TEXT":       "#1a1a2a",
        "SUBTEXT":    "#555577",
        "ENTRY":      "#ffffff",
        "DANGER_BTN": "#c0392b",
        "GREEN_BTN":  "#217a52",
        "SEL_BG":     "#4a90d9",
    },
}

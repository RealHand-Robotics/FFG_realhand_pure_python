"""
RealForce G20 hand-shape mapping module - pure Python version
Supports precise mapping based on calibration data
v2.8.0The mapper algorithm has been upgraded
"""
import numpy as np
import copy
from ..realhand_core import RealHandCore as HandCore
from ..realforce_config.g20_config import FINGER_CONFIGS, MAPPING_ORDER, ROBOT_OPOSE_RIGHT, ROBOT_OPOSE_LEFT, ROBOT_ORIGINAL_LEFT, ROBOT_ORIGINAL_RIGHT, ROBOT_FIST_LEFT, ROBOT_FIST_RIGHT, MULTI_SEGMENT_CONFIG, MULTI_SEGMENT_CONFIG_FROZEN, MOTOR_CONSTRAINTS
from typing import List
from ..realhand_core_ex import DynamicWeightMultiStateLinearMapper,MultiStateLinearMapper

def _resolve_version_config(configs: dict, version: str) -> dict:
    """
    Parse version configuration, Convert the dictionaryFormatof weights/reverse_motion Convert to concrete values
    """
    resolved = copy.deepcopy(configs)
    for finger_name, config in resolved.items():
        if 'weights' in config and isinstance(config['weights'], dict):
            config['weights'] = config['weights'].get(version, config['weights'].get('v2', [0.5, 0, 0.5]))
        if 'reverse_motion' in config and isinstance(config['reverse_motion'], dict):
            config['reverse_motion'] = config['reverse_motion'].get(version, config['reverse_motion'].get('v2', False))
    return resolved


def _clamp_command(value: float) -> int:
    return int(max(0, min(255, round(value))))


def _apply_ros2_manual_g20_qpos(qpos, joint_arc):
    """Fill qpos using the ROS2 LinkerForce G20 no-calibration mapping."""
    qpos[20] = joint_arc[4] * 2.2
    qpos[17] = joint_arc[2] * -2.5
    qpos[1] = joint_arc[6] * 0.1 + joint_arc[8] * 0.7
    qpos[9] = joint_arc[10] * 0.1 + joint_arc[12] * 0.7
    qpos[13] = joint_arc[14] * 0.1 + joint_arc[16] * 0.7
    qpos[5] = joint_arc[18] * 0.1 + joint_arc[20] * 0.7


class _AdaptiveG20FallbackMapper:
    """Direct 21-value RealForce glove to legacy 20-command fallback.

    The calibrated mapper is still the preferred path. This fallback is used
    when no calibration file is loaded. It keeps a live neutral baseline from
    the first frame and maps sensor deltas directly into the 20-value G20/L20
    command layout consumed by l20_sdk_controller.py.
    """

    BASE_SOURCES = [18, 1, 9, 13, 5]
    ABD_SOURCES = [16, 0, 8, 12, 4]
    TIP_SOURCES = [19, 2, 10, 14, 6]
    THUMB_YAW_SOURCE = 17

    def __init__(self, side: str) -> None:
        self.side = side
        self.baseline: list[float] | None = None
        self.flex_gain = 175.0
        self.tip_gain = 190.0
        self.abd_gain = 80.0
        self.thumb_yaw_gain = 90.0

    def map(self, joint_arc: List[float]) -> list[int]:
        values = [float(v) for v in joint_arc]
        if len(values) < 21:
            values.extend([0.0] * (21 - len(values)))

        if self.baseline is None:
            self.baseline = list(values)
            return [255] * 20

        command = [255] * 20
        for i, (base_idx, abd_idx, tip_idx) in enumerate(
            zip(self.BASE_SOURCES, self.ABD_SOURCES, self.TIP_SOURCES)
        ):
            base_delta = abs(values[base_idx] - self.baseline[base_idx])
            tip_delta = abs(values[tip_idx] - self.baseline[tip_idx])
            flex_delta = max(base_delta, tip_delta * 0.55)

            command[i] = _clamp_command(255 - flex_delta * self.flex_gain)
            command[15 + i] = _clamp_command(
                255 - max(tip_delta, base_delta * 0.35) * self.tip_gain
            )

            abd_delta = values[abd_idx] - self.baseline[abd_idx]
            if self.side == "left":
                abd_delta = -abd_delta
            command[5 + i] = _clamp_command(127 + abd_delta * self.abd_gain)

        thumb_yaw_delta = abs(
            values[self.THUMB_YAW_SOURCE] - self.baseline[self.THUMB_YAW_SOURCE]
        )
        command[10] = _clamp_command(255 - thumb_yaw_delta * self.thumb_yaw_gain)
        return command


class RightHand:
    def __init__(self, handcore: HandCore, length=20, is_debug: bool = False):
        self.handcore = handcore
        self.g_jointpositions = [255] * length
        self.g_jointvelocity = [255] * length
        self.last_jointpositions = [255] * length
        self.last_jointvelocity = [255] * length
        self.g_jointpositions_arc = [0] * length
        self.g_jointvelocity_arc = [0] * length
        self.handstate = [0] * length
        self.calibrationoriginal = None    # five-finger-open calibration value (corresponds to255)
        self.calibrationfistpose = None    # fist calibration value (corresponds to0)
        self.calibrationopose = None       # O-pose calibration value (corresponds to the intermediate value)
        self.glove_version = 'v2'

        # ========== Smoothing-filter parameters ==========
        self.smooth_enabled = True
        self.smooth_alpha = 0.5  # smoothing coefficient: smaller is smoother; range 0.05-0.3
        self.smooth_positions = [255.0] * length  # smoothed position (float)
        self.max_step = 20  # maximum change per frame to prevent jumps
        self.fallback_mapper = _AdaptiveG20FallbackMapper("right")

        # Target robot-hand preset pose; values are obtained from the URDF dataset,
        # Opening the hand corresponds to the minimum angle,
        # Making a fist corresponds to the maximum angle
        # For the O pose, use a tool to drive the URDF and target robot hand to the desired pose; these parameters can also be adjusted to better reach the desired physical angle
        # Other gestures are similar; additional gestures can be added for a multimodal mapper (under continued development)
        self.robot_original = ROBOT_ORIGINAL_RIGHT
        self.robot_opose = ROBOT_OPOSE_RIGHT
        self.robot_fist = ROBOT_FIST_RIGHT

        finger_configs = _resolve_version_config(FINGER_CONFIGS, self.glove_version)
        self.multi_state_mapper = DynamicWeightMultiStateLinearMapper(finger_configs, MAPPING_ORDER, is_debug=is_debug)

        for config_name, config in FINGER_CONFIGS.items():
            if config.get('dynamic_weight'):
                self.multi_state_mapper.set_dynamic_weight_config(config_name, config['dynamic_weight'])

        self.motor_constraints = MOTOR_CONSTRAINTS['right']

    def _apply_motor_constraints(self):
        for i, constraint in enumerate(self.motor_constraints):
            if constraint.get('enabled', False):
                min_val = constraint.get('min', 0)
                max_val = constraint.get('max', 255)
                self.g_jointpositions[i] = int(max(min_val, min(max_val, self.g_jointpositions[i])))

    def set_glove_version(self, version: str):
        if not version:
            return

        major_version = version.split('.')[0]
        version_key = f'v{major_version}'

        if version_key == self.glove_version:
            return

        self.glove_version = version_key

        for finger_name, config in FINGER_CONFIGS.items():
            if 'weights' in config and isinstance(config['weights'], dict):
                if version_key in config['weights']:
                    self.multi_state_mapper.finger_configs[finger_name]['weights'] = config['weights'][version_key]

            if 'reverse_motion' in config and isinstance(config['reverse_motion'], dict):
                if version_key in config['reverse_motion']:
                    self.multi_state_mapper.finger_configs[finger_name]['reverse_motion'] = config['reverse_motion'][version_key]

    def initialize_mapper(self) -> bool:
        """
        Initialize the mapper

        Load three human-hand and three robot-hand calibration datasets into the mapper
        They are original, opose, and fist

        The human hand uses the glove_ prefix and the robot hand uses the robot_ prefix
        """

        glove_original = self._to_list(self.calibrationoriginal)
        glove_fist = self._to_list(self.calibrationfistpose)
        glove_opose = self._to_list(self.calibrationopose)

        self.multi_state_mapper.add_state('original', glove_original, self.robot_original)
        self.multi_state_mapper.add_state('opose', glove_opose, self.robot_opose)
        self.multi_state_mapper.add_state('fist', glove_fist, self.robot_fist)

        self.multi_state_mapper.set_state_order(list(MULTI_SEGMENT_CONFIG_FROZEN))

        state_info = self.multi_state_mapper.get_state_info()

    def _to_list(self, data):
        """Convert to a list"""
        if hasattr(data, 'tolist'):
            return data.tolist()
        elif isinstance(data, np.ndarray):
            # print(111)
            return data.tolist()
        else:
            return list(data)

    # V2.8.0 This function is obsolete
    # def _linear_map_diff(self, current_diff, fist_diff, extend_ratio=1.2):
    #     """
    #     Linear mapping to 0-255 based on the difference

    #     Note: joint_update receives the difference (current value - open value)

    #     Parameters:
    #         current_diff: current sensor difference (current value - open value)
    #         fist_diff: difference when making a fist (fist value - open value)
    #         extend_ratio: scaling ratio; >1.0 makes the mapping reach the 0/255 bounds more easily

    #     Mapping logic:
    #         - difference of 0 (open) → 255
    #         - difference of fist_diff (fist) → 0
    #     """
    #     if abs(fist_diff) < 0.01:
    #         return 128  # change is too small; return the middle value

    #     # Reduce fist_diff to reach the 0 bound more easily
    #     effective_fist_diff = fist_diff / extend_ratio

    #     # Calculate ratio: difference 0 → ratio 0; difference fist_diff → ratio 1
    #     ratio = current_diff / effective_fist_diff
    #     ratio = max(0.0, min(1.0, ratio))  # Limit to the 0-1 range

    #     # Mapping: ratio 0 → 255; ratio 1 → 0
    #     return int((1 - ratio) * 255)

    def _apply_smooth(self, raw_positions):
        """
        Apply smoothing to motor output to prevent abrupt jumps

        Use an exponential moving average (EMA) plus a maximum-step limit
        """
        if not self.smooth_enabled:
            return raw_positions

        smoothed = []
        for i, raw in enumerate(raw_positions):
            # exponential moving average
            target = self.smooth_alpha * raw + (1 - self.smooth_alpha) * self.smooth_positions[i]

            # maximum-step limit to prevent large jumps
            diff = target - self.smooth_positions[i]
            if abs(diff) > self.max_step:
                target = self.smooth_positions[i] + (self.max_step if diff > 0 else -self.max_step)

            self.smooth_positions[i] = target
            smoothed.append(int(round(target)))

        return smoothed

    def joint_update(self, joint_arc):
        """
        Right-hand mapping - completed by a mapper based on calibration data and expected robot-hand movement
        """
        qpos = np.zeros(25)
        # ========== Use the mapper for precise mapping ==========
        if self.calibrationoriginal is not None and self.calibrationfistpose is not None and self.calibrationopose is not None:
            arc_value = self.multi_state_mapper.map_glove_to_robot(joint_arc)
        # ========== Use manual mapping when calibration data is unavailable ==========
        else:
            arc_value = None

        if arc_value is not None:
            qpos[16] = self.g_jointpositions_arc[0] = arc_value[0]
            qpos[17] = self.g_jointpositions_arc[1] = arc_value[1]
            qpos[18] = self.g_jointpositions_arc[2] = arc_value[2]
            qpos[19] = self.g_jointpositions_arc[3] = arc_value[3]

            qpos[0] = self.g_jointpositions_arc[5] = arc_value[5]
            qpos[1] = self.g_jointpositions_arc[6] = arc_value[6]
            qpos[2] = self.g_jointpositions_arc[7] = arc_value[7]
            qpos[3] = self.g_jointpositions_arc[8] = arc_value[8]

            qpos[4] = self.g_jointpositions_arc[17] = arc_value[17]
            qpos[5] = self.g_jointpositions_arc[18] = arc_value[18]
            qpos[6] = self.g_jointpositions_arc[19] = arc_value[19]
            qpos[7] = self.g_jointpositions_arc[4] = arc_value[20]

            qpos[8] = self.g_jointpositions_arc[9] = arc_value[9]
            qpos[9] = self.g_jointpositions_arc[10] = arc_value[10]
            qpos[10] = self.g_jointpositions_arc[11] = arc_value[11]
            qpos[11] = self.g_jointpositions_arc[12] = arc_value[12]

            qpos[12] = self.g_jointpositions_arc[13] = arc_value[13]
            qpos[13] = self.g_jointpositions_arc[14] = arc_value[14]
            qpos[14] = self.g_jointpositions_arc[15] = arc_value[15]
            qpos[15] = self.g_jointpositions_arc[16] = arc_value[16]
        else:
            _apply_ros2_manual_g20_qpos(qpos, joint_arc)

        # ========== Apply smoothing filter ==========
        self.g_jointpositions = self.handcore.trans_to_motor_right(qpos)
        self._apply_motor_constraints()
        self.g_jointpositions = self._apply_smooth(self.g_jointpositions)

    def speed_update(self):
        for i in range(len(self.g_jointpositions)):
            lastpos = self.last_jointpositions[i]
            position_error = int(abs(self.g_jointpositions[i] - lastpos))
            position_derict = 1 if self.g_jointpositions[i] - lastpos > 0 else -1
            slow_limit = 4
            fast_limit = 10
            max_vel = int(self.last_jointvelocity[i] * 2)
            mid_vel = int(self.last_jointvelocity[i] * 0.7)
            min_vel = int(self.last_jointvelocity[i] * 0.5)
            target_vel = self.last_jointvelocity[i]
            if self.handstate[i] == 0:  # stop
                if 0 < position_error:
                    target_vel = position_error * 5 + 30
                    self.handstate[i] = 1
            elif self.handstate[i] == 1:  # slow
                if position_error >= fast_limit:
                    target_vel = position_error * 5 + 50
                    if target_vel > mid_vel:
                        target_vel = mid_vel
                    self.handstate[i] = 2
                elif position_error == 0:
                    self.handstate[i] = 0
                    target_vel = position_error * 5 + 100
                else:
                    target_vel = position_error * 5 + 100
            else:  # fast
                if position_error >= fast_limit:
                    target_vel = position_error * 5 + 90
                    if target_vel > max_vel:
                        target_vel = max_vel
                elif slow_limit < position_error < fast_limit:
                    target_vel = position_error * 5 + 60
                    if target_vel < mid_vel:
                        target_vel = mid_vel
                    self.handstate[i] = 3
                elif 0 < position_error <= slow_limit:
                    target_vel = position_error * 5 + 40
                    if target_vel < min_vel:
                        target_vel = min_vel
                    self.handstate[i] = 1
            self.g_jointvelocity[i] = int(target_vel * 1)
            if self.g_jointvelocity[i] > 255:
                self.g_jointvelocity[i] = 255
            self.g_jointvelocity[i] = 255
            self.last_jointvelocity[i] = self.g_jointvelocity[i]
            self.last_jointpositions[i] = self.g_jointpositions[i]


class LeftHand:
    def __init__(self, handcore: HandCore, length=20, is_debug: bool = False):
        self.handcore = handcore
        self.g_jointpositions = [255] * length
        self.g_jointvelocity = [255] * length
        self.last_jointpositions = [255] * length
        self.last_jointvelocity = [255] * length
        self.g_jointpositions_arc = [0] * length
        self.g_jointvelocity_arc = [0] * length
        self.handstate = [0] * length
        self.calibrationoriginal = None
        self.calibrationfistpose = None
        self.calibrationopose = None
        self.glove_version = 'v2'

        self.smooth_enabled = True
        self.smooth_alpha = 0.5
        self.smooth_positions = [255.0] * length
        self.max_step = 20
        self.fallback_mapper = _AdaptiveG20FallbackMapper("left")

        self.robot_original = ROBOT_ORIGINAL_LEFT
        self.robot_opose = ROBOT_OPOSE_LEFT
        self.robot_fist = ROBOT_FIST_LEFT

        finger_configs = _resolve_version_config(FINGER_CONFIGS, self.glove_version)
        self.multi_state_mapper = DynamicWeightMultiStateLinearMapper(finger_configs, MAPPING_ORDER, is_debug=is_debug)

        for config_name, config in FINGER_CONFIGS.items():
            if config.get('dynamic_weight'):
                self.multi_state_mapper.set_dynamic_weight_config(config_name, config['dynamic_weight'])

        self.motor_constraints = MOTOR_CONSTRAINTS['left']

    def _apply_motor_constraints(self):
        for i, constraint in enumerate(self.motor_constraints):
            if constraint.get('enabled', False):
                min_val = constraint.get('min', 0)
                max_val = constraint.get('max', 255)
                self.g_jointpositions[i] = int(max(min_val, min(max_val, self.g_jointpositions[i])))

    def set_glove_version(self, version: str):
        if not version:
            return

        major_version = version.split('.')[0]
        version_key = f'v{major_version}'

        if version_key == self.glove_version:
            return

        self.glove_version = version_key

        for finger_name, config in FINGER_CONFIGS.items():
            if 'weights' in config and isinstance(config['weights'], dict):
                if version_key in config['weights']:
                    self.multi_state_mapper.finger_configs[finger_name]['weights'] = config['weights'][version_key]

            if 'reverse_motion' in config and isinstance(config['reverse_motion'], dict):
                if version_key in config['reverse_motion']:
                    self.multi_state_mapper.finger_configs[finger_name]['reverse_motion'] = config['reverse_motion'][version_key]

    def initialize_mapper(self) -> bool:
        """
        Initialize the mapper

        Load three human-hand and three robot-hand calibration datasets into the mapper
        They are original, opose, and fist

        The human hand uses the glove_ prefix and the robot hand uses the robot_ prefix
        """

        glove_original = self._to_list(self.calibrationoriginal)
        glove_fist = self._to_list(self.calibrationfistpose)
        glove_opose = self._to_list(self.calibrationopose)

        self.multi_state_mapper.add_state('original', glove_original, self.robot_original)
        self.multi_state_mapper.add_state('opose', glove_opose, self.robot_opose)
        self.multi_state_mapper.add_state('fist', glove_fist, self.robot_fist)

        self.multi_state_mapper.set_state_order(list(MULTI_SEGMENT_CONFIG_FROZEN))

        state_info = self.multi_state_mapper.get_state_info()

    def _to_list(self, data):
        """Convert to a list"""
        if hasattr(data, 'tolist'):
            return data.tolist()
        elif isinstance(data, np.ndarray):
            # print(111)
            return data.tolist()
        else:
            return list(data)

    # V2.8.0 This function is obsolete
    # def _linear_map_diff(self, current_diff, fist_diff, extend_ratio=1.2):
    #     """
    #     Linear mapping to 0-255 based on the difference

    #     Note: joint_update receives the difference (current value - open value)

    #     Parameters:
    #         current_diff: current sensor difference (current value - open value)
    #         fist_diff: difference when making a fist (fist value - open value)
    #         extend_ratio: scaling ratio; >1.0 makes the mapping reach the 0/255 bounds more easily

    #     Mapping logic:
    #         - difference of 0 (open) → 255
    #         - difference of fist_diff (fist) → 0
    #     """
    #     if abs(fist_diff) < 0.01:
    #         return 128  # change is too small; return the middle value

    #     # Reduce fist_diff to reach the 0 bound more easily
    #     effective_fist_diff = fist_diff / extend_ratio

    #     # Calculate ratio: difference 0 → ratio 0; difference fist_diff → ratio 1
    #     ratio = current_diff / effective_fist_diff
    #     ratio = max(0.0, min(1.0, ratio))  # Limit to the 0-1 range

    #     # Mapping: ratio 0 → 255; ratio 1 → 0
    #     return int((1 - ratio) * 255)

    def _apply_smooth(self, raw_positions):
        """
        Apply smoothing to motor output to prevent abrupt jumps

        Use an exponential moving average (EMA) plus a maximum-step limit
        """
        if not self.smooth_enabled:
            return raw_positions

        smoothed = []
        for i, raw in enumerate(raw_positions):
            # exponential moving average
            target = self.smooth_alpha * raw + (1 - self.smooth_alpha) * self.smooth_positions[i]

            # maximum-step limit to prevent large jumps
            diff = target - self.smooth_positions[i]
            if abs(diff) > self.max_step:
                target = self.smooth_positions[i] + (self.max_step if diff > 0 else -self.max_step)

            self.smooth_positions[i] = target
            smoothed.append(int(round(target)))

        return smoothed

    def joint_update(self, joint_arc):
        """
        Right-hand mapping - completed by a mapper based on calibration data and expected robot-hand movement
        """
        qpos = np.zeros(25)
        # ========== Use the mapper for precise mapping ==========
        if self.calibrationoriginal is not None and self.calibrationfistpose is not None and self.calibrationopose is not None:
            arc_value = self.multi_state_mapper.map_glove_to_robot(joint_arc)
        # ========== Use manual mapping when calibration data is unavailable ==========
        else:
            arc_value = None

        if arc_value is not None:
            qpos[16] = self.g_jointpositions_arc[0] = arc_value[0]
            qpos[17] = self.g_jointpositions_arc[1] = arc_value[1]
            qpos[18] = self.g_jointpositions_arc[2] = arc_value[2]
            qpos[19] = self.g_jointpositions_arc[3] = arc_value[3]

            qpos[0] = self.g_jointpositions_arc[5] = arc_value[5]
            qpos[1] = self.g_jointpositions_arc[6] = arc_value[6]
            qpos[2] = self.g_jointpositions_arc[7] = arc_value[7]
            qpos[3] = self.g_jointpositions_arc[8] = arc_value[8]

            qpos[4] = self.g_jointpositions_arc[17] = arc_value[17]
            qpos[5] = self.g_jointpositions_arc[18] = arc_value[18]
            qpos[6] = self.g_jointpositions_arc[19] = arc_value[19]
            qpos[7] = self.g_jointpositions_arc[4] = arc_value[20]

            qpos[8] = self.g_jointpositions_arc[9] = arc_value[9]
            qpos[9] = self.g_jointpositions_arc[10] = arc_value[10]
            qpos[10] = self.g_jointpositions_arc[11] = arc_value[11]
            qpos[11] = self.g_jointpositions_arc[12] = arc_value[12]

            qpos[12] = self.g_jointpositions_arc[13] = arc_value[13]
            qpos[13] = self.g_jointpositions_arc[14] = arc_value[14]
            qpos[14] = self.g_jointpositions_arc[15] = arc_value[15]
            qpos[15] = self.g_jointpositions_arc[16] = arc_value[16]
        else:
            _apply_ros2_manual_g20_qpos(qpos, joint_arc)

        self.g_jointpositions = self.handcore.trans_to_motor_left(qpos)

        # print(qpos[4],arc_value[17],self.g_jointpositions[9])
        self._apply_motor_constraints()

    def speed_update(self):
        for i in range(len(self.g_jointpositions)):
            lastpos = self.last_jointpositions[i]
            position_error = int(abs(self.g_jointpositions[i] - lastpos))
            position_derict = 1 if self.g_jointpositions[i] - lastpos > 0 else -1
            slow_limit = 4
            fast_limit = 10
            max_vel = int(self.last_jointvelocity[i] * 2)
            mid_vel = int(self.last_jointvelocity[i] * 0.7)
            min_vel = int(self.last_jointvelocity[i] * 0.5)
            target_vel = self.last_jointvelocity[i]
            if self.handstate[i] == 0:  # stop
                if 0 < position_error:
                    target_vel = position_error * 5 + 30
                    self.handstate[i] = 1
            elif self.handstate[i] == 1:  # slow
                if position_error >= fast_limit:
                    target_vel = position_error * 5 + 50
                    if target_vel > mid_vel:
                        target_vel = mid_vel
                    self.handstate[i] = 2
                elif position_error == 0:
                    self.handstate[i] = 0
                    target_vel = position_error * 5 + 100
                else:
                    target_vel = position_error * 5 + 100
            else:  # fast
                if position_error >= fast_limit:
                    target_vel = position_error * 5 + 90
                    if target_vel > max_vel:
                        target_vel = max_vel
                elif slow_limit < position_error < fast_limit:
                    target_vel = position_error * 5 + 60
                    if target_vel < mid_vel:
                        target_vel = mid_vel
                    self.handstate[i] = 3
                elif 0 < position_error <= slow_limit:
                    target_vel = position_error * 5 + 40
                    if target_vel < min_vel:
                        target_vel = min_vel
                    self.handstate[i] = 1
            self.g_jointvelocity[i] = int(target_vel * 1)
            if self.g_jointvelocity[i] > 255:
                self.g_jointvelocity[i] = 255
            self.g_jointvelocity[i] = 255
            self.last_jointvelocity[i] = self.g_jointvelocity[i]
            self.last_jointpositions[i] = self.g_jointpositions[i]

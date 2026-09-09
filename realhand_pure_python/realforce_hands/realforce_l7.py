"""
RealForce L7 hand-shape mapping module - pure Python version
"""
import numpy as np
import copy
from ..realhand_core import RealHandCore as HandCore
from ..realforce_config.o7_config import FINGER_CONFIGS, MAPPING_ORDER, ROBOT_OPOSE_RIGHT, ROBOT_OPOSE_LEFT, ROBOT_ORIGINAL_RIGHT, ROBOT_ORIGINAL_LEFT, ROBOT_FIST_RIGHT, ROBOT_FIST_LEFT, MULTI_SEGMENT_CONFIG, MULTI_SEGMENT_CONFIG_FROZEN, MOTOR_CONSTRAINTS
from typing import List
from ..realhand_core_ex import DynamicWeightMultiStateLinearMapper,MultiStateLinearMapper


class RightHand:
    def __init__(self, handcore: HandCore, length=7, is_debug: bool = False):
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

        # Target robot-hand preset pose; values are obtained from the URDF dataset,
        # Opening the hand corresponds to the minimum angle,
        # Making a fist corresponds to the maximum angle
        # For the O pose, use a tool to drive the URDF and target robot hand to the desired pose; these parameters can also be adjusted to better reach the desired physical angle
        # Other gestures are similar; additional gestures can be added for a multimodal mapper (under continued development)
        self.robot_original = ROBOT_ORIGINAL_RIGHT
        self.robot_opose = ROBOT_OPOSE_RIGHT
        self.robot_fist = ROBOT_FIST_RIGHT
        self.robot_fist[0] = 0.8

        # Additional corrections for undesired self.robot_fist values can be made here,
        # Because the robot hand reaches its maximum value, there are regions with undesired values that can be corrected here,
        # This likewise requires a URDF-driving toolkit


        # Mapper (specific to v2.8.0); see l6_config.py for details
        self.multi_state_mapper = DynamicWeightMultiStateLinearMapper(FINGER_CONFIGS, MAPPING_ORDER, is_debug=is_debug)

        # Set dynamic-weight configuration (added in v2.8.2)
        for config_name, config in FINGER_CONFIGS.items():
            if config.get('dynamic_weight'):
                self.multi_state_mapper.set_dynamic_weight_config(config_name, config['dynamic_weight'])

        # Motor output constraints
        self.motor_constraints = MOTOR_CONSTRAINTS['right']

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
        """Initialize the mapper"""
        glove_original = self._to_list(self.calibrationoriginal)
        glove_fist = self._to_list(self.calibrationfistpose)
        glove_opose = self._to_list(self.calibrationopose)

        self.multi_state_mapper.add_state('original', glove_original, self.robot_original)
        self.multi_state_mapper.add_state('opose', glove_opose, self.robot_opose)
        self.multi_state_mapper.add_state('fist', glove_fist, self.robot_fist)

        self.multi_state_mapper.set_state_order(list(MULTI_SEGMENT_CONFIG_FROZEN))

    def _to_list(self, data):
        """Convert to a list"""
        if hasattr(data, 'tolist'):
            return data.tolist()
        elif isinstance(data, np.ndarray):
            return data.tolist()
        else:
            return list(data)

    def _apply_motor_constraints(self, positions):
        """applyMotor output constraints"""
        if not hasattr(self, 'motor_constraints') or self.motor_constraints is None:
            return positions
        result = []
        for i, pos in enumerate(positions):
            if i < len(self.motor_constraints) and self.motor_constraints[i].get('enabled', False):
                result.append(max(self.motor_constraints[i]['min'], min(pos, self.motor_constraints[i]['max'])))
            else:
                result.append(pos)
        return result

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
            # arc_value = ROBOT_OPOSE_RIGHT
            qpos[17] = self.g_jointpositions_arc[1] = arc_value[1]
            qpos[20] = self.g_jointpositions_arc[0] = arc_value[2]
            qpos[16] = self.g_jointpositions_arc[6] = arc_value[0]

            qpos[1] = self.g_jointpositions_arc[2] = arc_value[3]

            qpos[9] = self.g_jointpositions_arc[3] = arc_value[4]

            qpos[13] = self.g_jointpositions_arc[4] = arc_value[5]

            qpos[5] = self.g_jointpositions_arc[5] = arc_value[6]
        else:
            # Thumb handling
            qpos[16] = joint_arc[1] * 1.2
            qpos[17] = joint_arc[0] * -2 + joint_arc[1] * 1.2
            qpos[20] = joint_arc[4] * 0.5 + joint_arc[2] * 0.8

            qpos[1] = joint_arc[6] * 0.1 + joint_arc[8] * 0.7
            qpos[9] = joint_arc[10] * 0.1 + joint_arc[12] * 0.7
            qpos[13] = joint_arc[14] * 0.1 + joint_arc[16] * 0.7
            qpos[5] = joint_arc[18] * 0.1 + joint_arc[20] * 0.7
        # ========== Apply motor constraints ==========
        self.g_jointpositions = self.handcore.trans_to_motor_right(qpos)
        self.g_jointpositions = self._apply_motor_constraints(self.g_jointpositions)
        # ========== Apply smoothing filter ==========
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


# LeftHand similar corrections for this class
class LeftHand:
    def __init__(self, handcore: HandCore, length=7, is_debug: bool = False):
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

        # Target robot-hand preset pose; values are obtained from the URDF dataset,
        # Opening the hand corresponds to the minimum angle,
        # Making a fist corresponds to the maximum angle
        # For the O pose, use a tool to drive the URDF and target robot hand to the desired pose; these parameters can also be adjusted to better reach the desired physical angle
        # Other gestures are similar; additional gestures can be added for a multimodal mapper (under continued development)
        self.robot_original = ROBOT_ORIGINAL_LEFT
        self.robot_opose = ROBOT_OPOSE_LEFT
        self.robot_fist = ROBOT_FIST_LEFT
        self.robot_fist[0] = 0.8

        # Additional corrections for undesired self.robot_fist values can be made here,
        # Because the robot hand reaches its maximum value, there are regions with undesired values that can be corrected here,
        # This likewise requires a URDF-driving toolkit


        # Mapper (specific to v2.8.0); see l6_config.py for details
        self.multi_state_mapper = DynamicWeightMultiStateLinearMapper(FINGER_CONFIGS, MAPPING_ORDER, is_debug=is_debug)

        # Set dynamic-weight configuration (added in v2.8.2)
        for config_name, config in FINGER_CONFIGS.items():
            if config.get('dynamic_weight'):
                self.multi_state_mapper.set_dynamic_weight_config(config_name, config['dynamic_weight'])

        # Motor output constraints
        self.motor_constraints = MOTOR_CONSTRAINTS['left']

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
        """Initialize the mapper"""
        glove_original = self._to_list(self.calibrationoriginal)
        glove_fist = self._to_list(self.calibrationfistpose)
        glove_opose = self._to_list(self.calibrationopose)

        self.multi_state_mapper.add_state('original', glove_original, self.robot_original)
        self.multi_state_mapper.add_state('opose', glove_opose, self.robot_opose)
        self.multi_state_mapper.add_state('fist', glove_fist, self.robot_fist)

        self.multi_state_mapper.set_state_order(list(MULTI_SEGMENT_CONFIG_FROZEN))

    def _to_list(self, data):
        """Convert to a list"""
        if hasattr(data, 'tolist'):
            return data.tolist()
        elif isinstance(data, np.ndarray):
            return data.tolist()
        else:
            return list(data)

    def _apply_motor_constraints(self, positions):
        """applyMotor output constraints"""
        if not hasattr(self, 'motor_constraints') or self.motor_constraints is None:
            return positions
        result = []
        for i, pos in enumerate(positions):
            if i < len(self.motor_constraints) and self.motor_constraints[i].get('enabled', False):
                result.append(max(self.motor_constraints[i]['min'], min(pos, self.motor_constraints[i]['max'])))
            else:
                result.append(pos)
        return result

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
            # arc_value = ROBOT_OPOSE_RIGHT
            qpos[17] = self.g_jointpositions_arc[1] = arc_value[1]
            qpos[20] = self.g_jointpositions_arc[0] = arc_value[2]
            qpos[16] = self.g_jointpositions_arc[6] = arc_value[0]

            qpos[1] = self.g_jointpositions_arc[2] = arc_value[3]

            qpos[9] = self.g_jointpositions_arc[3] = arc_value[4]

            qpos[13] = self.g_jointpositions_arc[4] = arc_value[5]

            qpos[5] = self.g_jointpositions_arc[5] = arc_value[6]
        else:
            # Thumb handling
            qpos[16] = joint_arc[1] * 1.2
            qpos[17] = joint_arc[0] * -2 + joint_arc[1] * 1.2
            qpos[20] = joint_arc[4] * 0.5 + joint_arc[2] * 0.8

            qpos[1] = joint_arc[6] * 0.1 + joint_arc[8] * 0.7
            qpos[9] = joint_arc[10] * 0.1 + joint_arc[12] * 0.7
            qpos[13] = joint_arc[14] * 0.1 + joint_arc[16] * 0.7
            qpos[5] = joint_arc[18] * 0.1 + joint_arc[20] * 0.7
        # ========== Apply motor constraints ==========
        self.g_jointpositions = self.handcore.trans_to_motor_right(qpos)
        self.g_jointpositions = self._apply_motor_constraints(self.g_jointpositions)
        # ========== Apply smoothing filter ==========
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


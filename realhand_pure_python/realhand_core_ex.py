"""
Multi-state linear mapper
Supports any number of states
"""
import numpy as np
try:
    from colorama import Fore, init
except ImportError:
    class _PlainColor:
        RED = ""
        GREEN = ""
        YELLOW = ""
        BLUE = ""
        CYAN = ""
        RESET = ""
        RESET_ALL = ""

    Fore = _PlainColor()

    def init(*args, **kwargs):
        return None
from typing import List, Dict, Tuple
from .realhand_filters import MultiChannelLCFilter, MultiChannelSavitzkyGolayFilter, MultiChannelKalmanFilter

class MultiStateLinearMapper:
    """
    Multi-state linear mapper
    Supports any number of gesture states
    """

    def __init__(self,FINGER_CONFIGS,MAPPING_ORDER,is_debug = False):
        self.finger_configs = FINGER_CONFIGS.copy()
        self.mapping_order = MAPPING_ORDER.copy()

        # State storage
        self.glove_states = {}  # {state name: glove-angle array}
        self.robot_states = {}  # {state name: robot-hand-angle array}
        self.state_order = []   # state-order list
        self.debug_value = [0.0] * 20  # debug buffer data with length 20
        self.isdebug = is_debug
        self.debug_fingers = None  # None=all, []=all, ["finger_name"]=specified finger

        # self.filters = MultiChannelLCFilter(num_channels=11, alpha=0.1)
        num_joints = 21

        # Create a multi-channel Savitzky-Golay filter
        self.filters = MultiChannelKalmanFilter(
            num_channels=num_joints,
            process_variance=1e-5,
            measurement_variance=0.0005,
            initial_values=[0.0] * num_joints
        )

        # self.filters = MultiChannelSavitzkyGolayFilter(
        #     num_channels=num_joints,
        #     window_length=,
        #     polyorder=3
        # )

        # Filter parameters
        # self.filter_params = {
        #     'window_length': 7,
        #     'polyorder': 2,
        #     'filter_type': 'Savitzky-Golay'
        # }

        # History (for debugging and visualization)
        self.raw_history = []
        self.filtered_history = []

    def add_state(self, state_name: str,
                  glove_angles: List[float],
                  robot_angles: List[float]):
        """
        Add a gesture state

        Parameters:
            state_name: state namecalled, such as 'original', 'opose', 'fist'etc.
            glove_angles: glove angle (21dimensions)
            robot_angles: robot-hand angle (11dimensions)
        """
        self.glove_states[state_name] = np.array(glove_angles)
        self.robot_states[state_name] = np.array(robot_angles)

        if state_name not in self.state_order:
            self.state_order.append(state_name)

    def remove_state(self, state_name: str):
        """Remove a state"""
        if state_name in self.glove_states:
            del self.glove_states[state_name]
            del self.robot_states[state_name]
            if state_name in self.state_order:
                self.state_order.remove(state_name)

    def set_state_order(self, state_order: List[str]):
        """
        Set state order (from original to most flexed)

        Example:
            ['original', 'opose', 'fist']
        """
        # Verify that all states exist
        for state in state_order:
            if state not in self.glove_states:
                raise ValueError(f"state '{state}' is undefined")

        self.state_order = state_order

    def map_glove_to_robot(self, glove_current):
        """
        Dynamic-weight mapping
        Adjust weights according to other finger states during mapping
        """

        if isinstance(glove_current, np.ndarray):
            glove_current = glove_current.tolist()
        elif isinstance(glove_current, list):
            glove_current = glove_current
        else:
            glove_current = list(glove_current)

        if len(self.state_order) < 2:
            raise ValueError("Set at least two states")

        if 'original' not in self.glove_states:
            raise ValueError("must include 'original' state as the reference")

        glove_current_arr = np.array(glove_current)
        robot_angles = self.robot_states['original'].copy()

        for config_name in self.mapping_order:
            config = self.finger_configs[config_name]
            angle = self._map_finger_multi_state(glove_current_arr, config)
            robot_angles[config['robot_idx']] = angle

        # self.debug_value[config['robot_idx']] = angle



        return robot_angles

    def _map_finger_multi_state(self, glove_current: np.ndarray,
                               config: dict) -> float:
        """
        Multi-state finger mapping
        """
        joints = config['joints']
        weights = self._normalize_weights(config['weights'])
        robot_idx = config['robot_idx']

        # Calculate the current fused value
        current_fused = self._calculate_fused_value(
            glove_current, joints, weights
        )
        # self.debug_value[robot_idx] = current_fused

        # Calculate fused values for all states
        state_fused_values = {}
        for state_name in self.state_order:
            fused = self._calculate_reference_fused(
                joints, weights, self.glove_states[state_name]
            )
            state_fused_values[state_name] = fused

        # Get angles for all states
        state_angles = {}
        for state_name in self.state_order:
            state_angles[state_name] = self.robot_states[state_name][robot_idx]

        # Piecewise linear interpolation
        result_angle = self._multi_state_interpolation(
            current_fused, state_fused_values, state_angles
        )

        # Handle reverse motion
        if config.get('reverse_motion', True):
            # Find the minimum and maximum angles
            min_angle = min(state_angles.values())
            max_angle = max(state_angles.values())

            result_angle = max_angle - (result_angle - min_angle)
            # print("Trigger reverse motion")

        return result_angle

    def _calculate_fused_value(self, data: np.ndarray,
                            joints: List[int],
                            weights) -> float:
        """
        Complete fused-value calculation, handling values outside the limits
        """
        # Ensure that an original state exists
        if 'original' not in self.glove_states:
            return 0.0

        original = self.glove_states['original']
        weights = np.array(weights)

        # Normalize weights
        if np.sum(weights) > 0:
            weights = weights / np.sum(weights)

        fused = 0.0

        for i, idx in enumerate(joints):
            # Get the current and original values
            current = data[idx]
            orig = original[idx]

            # step1: Find this joint's minimum and maximum values across all states
            all_vals = [orig]
            for state_data in self.glove_states.values():
                all_vals.append(state_data[idx])

            min_val = min(all_vals)
            max_val = max(all_vals)

            # step2: Clamp the current value to the[min_val, max_val]range
            clamped = np.clip(current, min_val, max_val)

            # step3: Calculate the normalized position
            if abs(max_val - min_val) < 1e-6:
                normalized_diff = 0.0
            else:
                orig_norm = (orig - min_val) / (max_val - min_val)
                clamped_norm = (clamped - min_val) / (max_val - min_val)
                normalized_diff = abs(clamped_norm - orig_norm)

            fused += weights[i] * normalized_diff
        return fused

    def _calculate_reference_fused(self, joints: List[int],
                                  weights: np.ndarray,
                                  reference_data: np.ndarray) -> float:
        """
        Calculate the reference fused value
        """
        return self._calculate_fused_value(reference_data, joints, weights)

    def _multi_state_interpolation(self, current_fused: float,
                                  state_fused_values: Dict[str, float],
                                  state_angles: Dict[str, float]) -> float:
        """
        multi-statePiecewise linear interpolation
        """
        # Ensure the state order is correct
        if not self.state_order:
            return 0.0

        # Handle boundary cases
        if current_fused <= state_fused_values[self.state_order[0]]:
            return state_angles[self.state_order[0]]

        if current_fused >= state_fused_values[self.state_order[-1]]:
            return state_angles[self.state_order[-1]]

        # Find the interval containing the current fused value
        for i in range(len(self.state_order) - 1):
            state1 = self.state_order[i]
            state2 = self.state_order[i + 1]

            fused1 = state_fused_values[state1]
            fused2 = state_fused_values[state2]

            # Ensure the interval is valid
            if fused1 <= current_fused <= fused2:
                if fused2 - fused1 > 1e-6:
                    t = (current_fused - fused1) / (fused2 - fused1)
                else:
                    t = 0.0

                angle1 = state_angles[state1]
                angle2 = state_angles[state2]
                return angle1 + t * (angle2 - angle1)

        # If no interval is found (which should not occur), return the nearest state angle
        min_diff = float('inf')
        nearest_angle = 0.0
        for state_name in self.state_order:
            diff = abs(current_fused - state_fused_values[state_name])
            if diff < min_diff:
                min_diff = diff
                nearest_angle = state_angles[state_name]

        return nearest_angle

    def _normalize_weights(self, weights: List[float]) -> List[float]:
        """
        Normalize weights
        """
        if hasattr(weights, 'tolist'):
            # If it is a NumPy array
            weight_list = weights.tolist()
        elif isinstance(weights, list):
            # If it is already a list
            weight_list = weights
        else:
            # Otherwise, attempt conversion
            weight_list = list(weights)
        total = np.sum(weight_list)
        if total > 0:
            result_array = weight_list / total
        else:
            result_array = weight_list

        # Important: convert back to a list
        return result_array.tolist()

    def get_state_info(self) -> Dict:
        """
        Get state information
        """
        # Basic information
        info = {
            'states': list(self.glove_states.keys()),
            'state_order': self.state_order,
            'has_original': 'original' in self.glove_states
        }

        return info


    def clear_states(self):
        """Clear all states"""
        self.glove_states.clear()
        self.robot_states.clear()
        self.state_order.clear()

    def set_debug(self, enabled):
        """
        Set debug mode

        Args:
            enabled: bool or list
                - True: Enable debugging and display all fingers
                - False: Disable debugging
                - []: Enable debugging and display all fingers
                - ["finger_name", ...]: Enable debugging and display only the specified fingers
        """
        if isinstance(enabled, bool):
            self.isdebug = enabled
            self.debug_fingers = None
        elif isinstance(enabled, list):
            self.isdebug = True
            self.debug_fingers = enabled if enabled else None
        else:
            self.isdebug = bool(enabled)
            self.debug_fingers = None

    def _should_debug(self, finger_name: str) -> bool:
        """Check whether to output debug information for this finger"""
        if not self.isdebug:
            return False
        if self.debug_fingers is None:
            return True
        return finger_name in self.debug_fingers


class DynamicWeightMultiStateLinearMapper(MultiStateLinearMapper):
    """
    Dynamic-weight multi-state linear mapper
    inherits fromMultiStateLinearMapper, adddynamic weight adjustment
    addextended linear mapping: based onopen/oposelinear mapping, can continue extending
    """

    def __init__(self, FINGER_CONFIGS, MAPPING_ORDER,is_debug=False):
        super().__init__(FINGER_CONFIGS, MAPPING_ORDER,is_debug)

        # Dynamic-weight configuration
        self.dynamic_weight_configs = {}

        # Extended-mapping configuration
        self.extended_mapping_enabled = {}
        self.scale_factors = {}
        self.exp_factors = {}
        # self.isdebug = is_debug
        # Cache computed joint mapping values
        self.cached_mapped_values = {}

        # Initialize from the configuration tableextended mapping
        self._init_extended_mapping_from_config()

    def _init_extended_mapping_from_config(self):
        """Initialize extended-mapping settings from the configuration table"""
        for finger_name, config in self.finger_configs.items():
            if config.get('dynamic_weight'):
                self.set_dynamic_weight_config(finger_name, config['dynamic_weight'])
            ext_config = config.get('extended_mapping')
            if ext_config and ext_config.get('enabled', False):
                self.extended_mapping_enabled[finger_name] = True

                # Set the scale factor
                scale_factor = ext_config.get('scale_factor', 1.0)
                if scale_factor != 1.0:
                    self.scale_factors[finger_name] = scale_factor
                exp_factor = ext_config.get('extended_exp_factor', 1.0)
                if exp_factor != 1.0:
                    self.exp_factors[finger_name] = exp_factor

    def set_dynamic_weight_config(self, finger_name: str, config: Dict):
        """
        setDynamic-weight configuration
        """
        self.dynamic_weight_configs[finger_name] = config

    def set_extended_mapping(self, finger_name: str, enabled: bool = True,
                            scale_factor: float = 1.0):
        """
        Manually set extended mapping

        Parameters:
            finger_name: finger name
            enabled: whether extended mapping is enabled
            scale_factor: scale factor; >1 speeds up mapping and <1 slows it down
        """
        self.extended_mapping_enabled[finger_name] = enabled
        if scale_factor != 1.0:
            self.scale_factors[finger_name] = scale_factor

    def fit_exp_factor(self, finger_name: str, current_fused: float,
                       fused_open: float, fused_opose: float,
                       angle_open: float, angle_opose: float, angle_fist: float) -> float:
        """
        Automatically fit an extension factor from the current fist value

        Goal: map current_fused mappingto angle_fist

        Formula: extension = slope * t * (1 + (exp-1) * t)
        where slope = angle_opose - angle_open, t = normalized - 1

        Parameters:
            finger_name: finger name
            current_fused: current fused value for the fist
            fused_open: fused value when open
            fused_opose: Ofused value for the O pose
            angle_open: robot-hand angle when open
            angle_opose: Orobot-hand angle for the O pose
            angle_fist: robot-hand angle at the fist limit

        Returns:
            computed extension factor
        """
        if abs(fused_opose - fused_open) < 1e-6:
            return 1.0

        normalized = (current_fused - fused_open) / (fused_opose - fused_open)

        if normalized <= 1.0:
            return 1.0

        t = normalized - 1.0

        slope = angle_opose - angle_open
        target_extension = angle_fist - angle_opose

        if abs(slope * t) < 1e-6 or abs(target_extension) < 1e-6:
            return 1.0

        base_extension = slope * t
        ratio = target_extension / base_extension

        exp_factor = (ratio - 1.0) / t + 1.0

        return max(1.0, min(100.0, exp_factor))

    def _apply_scale_factor(self, fused_value: float,
                        fused_open: float, fused_opose: float,
                        finger_name: str) -> float:
        """
        Apply the scale factor based on the normalized [0, 1] range

        Parameters:
            fused_value: original fused value
            fused_open: openstate fused value (mapped to 0)
            fused_opose: oposestate fused value (mapped to 1)
            finger_name: finger name
        """
        scale_factor = self.scale_factors.get(finger_name, 1.0)

        if scale_factor == 1.0:
            return fused_value

        # Normalize the original fused value to the [0, 1] range
        # Fused-value range [fused_open, fused_opose] -> [0, 1]
        if abs(fused_opose - fused_open) < 1e-6:
            normalized = 0.0
        else:
            normalized = (fused_value - fused_open) / (fused_opose - fused_open)

        # If already at the opose position, do not apply scaling
        if abs(normalized - 1.0) < 1e-6:
            return fused_value

        # Apply the scale factor to the normalized value
        scaled_normalized = normalized * scale_factor

        # Convert the scaled normalized value back to the original fused-value range
        scaled_fused = fused_open + scaled_normalized * (fused_opose - fused_open)

        return scaled_fused

    def _get_max_angle(self, robot_idx: int) -> float:
        """
        Get the joint's maximum angle
        If a fist state exists, use its angle as the maximum angle
        Otherwise, use the default maximum angle
        """
        # if presentfiststate, usefiststate angle
        if 'fist' in self.robot_states:
            return self.robot_states['fist'][robot_idx]

        # Default maximum angle (adjust as needed)
        return 1.57  # Default: 90 degrees

    def map_glove_to_robot(self, source_current):
        """
        Dynamic-weight mapping
        Adjust weights according to other finger states during mapping
        """
        self.debug_value[3] = source_current[1]

        glove_current = self.filters.update(source_current)
        # Apply Savitzky-Golay filtering
        # filtered_angles = self.filters.update(robot_angles)

        # Record history (for debugging and analysis)
        self.raw_history.append(source_current.copy())
        self.filtered_history.append(glove_current.copy())

        # filtered_angles = self.filters.update(robot_angles)
        # Limit history length
        max_history = 100
        if len(self.raw_history) > max_history:
            self.raw_history = self.raw_history[-max_history:]
            self.filtered_history = self.filtered_history[-max_history:]

        self.debug_value[4] = glove_current[1]

        if isinstance(glove_current, np.ndarray):
            glove_current = glove_current.tolist()
        elif isinstance(glove_current, list):
            glove_current = glove_current
        else:
            glove_current = list(glove_current)

        if len(self.state_order) < 2:
            raise ValueError("Set at least two states")

        if 'original' not in self.glove_states:
            raise ValueError("must include 'original' state as the reference")

        # Reset the cache
        self.cached_mapped_values = {}

        glove_current_arr = np.array(glove_current)
        robot_angles = self.robot_states['original'].copy()

        # First pass: calculate finger mapping values needed for trigger decisions
        for config_name in self.mapping_order:
            if config_name in self.dynamic_weight_configs:
                trigger_finger = self.dynamic_weight_configs[config_name]['trigger_finger']
                # First calculate the trigger finger's mapping value
                if trigger_finger not in self.cached_mapped_values:
                    trigger_value = self._calculate_trigger_value(
                        glove_current_arr, trigger_finger
                    )
                    self.cached_mapped_values[trigger_finger] = trigger_value


        i = 0
        # Second pass: map using dynamic weights
        for config_name in self.mapping_order:
            # Get dynamic configuration, if present
            dynamic_config = self.dynamic_weight_configs.get(config_name)

            if dynamic_config:
                # Map using dynamic weights
                config = self.finger_configs[config_name]
                angle = self._map_finger_dynamic_weight(
                    glove_current_arr, config_name, dynamic_config, config
                )
            else:
                # Map with the multi-state method (supports extended mapping)
                config = self.finger_configs[config_name]
                angle = self._map_finger_multi_state(glove_current_arr, config)

            robot_idx = self.finger_configs[config_name]['robot_idx']
            robot_angles[robot_idx] = angle


        return robot_angles

    def _calculate_trigger_value(self, glove_current: np.ndarray,
                               trigger_finger: str) -> float:
        """
        Calculate the trigger finger's normalized mapping value (range 0-1)

        Returns:
            Normalized mapping value: 0 represents the original state and 1 the most flexed state
        """
        if trigger_finger not in self.finger_configs:
            raise ValueError(f"trigger-finger configuration '{trigger_finger}' does not exist")

        config = self.finger_configs[trigger_finger]

        # Calculate the current fused value
        joints = config['joints']
        weights = self._normalize_weights(config['weights'])

        current_fused = self._calculate_fused_value(
            glove_current, joints, weights
        )

        # Calculate fused values for all states
        state_fused_values = {}
        for state_name in self.state_order:
            fused = self._calculate_reference_fused(
                joints, weights, self.glove_states[state_name]
            )
            state_fused_values[state_name] = fused

        # normalize to0-1range
        min_fused = min(state_fused_values.values())
        max_fused = max(state_fused_values.values())

        if abs(max_fused - min_fused) < 1e-6:
            return 0.0

        normalized = (current_fused - min_fused) / (max_fused - min_fused)
        return np.clip(normalized, 0.0, 1.0)

    def _map_finger_dynamic_weight(self, glove_current: np.ndarray,
                                  finger_name: str,
                                  dynamic_config: Dict,
                                  base_config: Dict) -> float:
        """
        Map a finger using dynamic weights
        """
        # Get the trigger value
        trigger_finger = dynamic_config['trigger_finger']
        if trigger_finger not in self.cached_mapped_values:
            trigger_value = self._calculate_trigger_value(
                glove_current, trigger_finger
            )
            self.cached_mapped_values[trigger_finger] = trigger_value
        else:
            trigger_value = self.cached_mapped_values[trigger_finger]

        # Select configuration by threshold
        threshold = dynamic_config['threshold']
        temp_config = self.finger_configs[finger_name].copy()  # use the base configuration by default

        if trigger_value < threshold:
            weight_config = dynamic_config['low_weight_config']
            # Create temporary configuration

            temp_config['joints'] = weight_config['joints']
            temp_config['weights'] = weight_config['weights']
            if 'reverse_motion' in weight_config:
                temp_config['reverse_motion'] = weight_config['reverse_motion']
            else:
                temp_config['reverse_motion'] = base_config.get('reverse_motion', False)
        else:
          # Use the high-weight configuration
            weight_config = dynamic_config.get('high_weight_config', {})
            # Create temporary configuration, Merge base and high-weight configurations
            if weight_config:  # If a high-weight configuration exists
                temp_config['joints'] = weight_config.get('joints', temp_config['joints'])
                temp_config['weights'] = weight_config.get('weights', temp_config['weights'])
                # Prioritize reverse_motion from the high-weight configuration
                if 'reverse_motion' in weight_config:
                    temp_config['reverse_motion'] = weight_config['reverse_motion']

        # Map with temporary configuration (supports extended mapping)
        return self._map_finger_multi_state(glove_current, temp_config)

    def _map_finger_multi_state(self, glove_current: np.ndarray,
                               config: dict) -> float:
        """
        Main finger-mapping method
        Supports extended mapping and original multi-state mapping
        """
        # Look up the finger name
        finger_name = None
        for name, cfg in self.finger_configs.items():
            if cfg['robot_idx'] == config['robot_idx']:
                finger_name = name
                break
        # print(self.extended_mapping_enabled)
        # Check whether extended mapping is enabled
        if (finger_name and finger_name in self.extended_mapping_enabled and
            self.extended_mapping_enabled[finger_name]):
            # print("Trigger linear mapping")
            return self._map_finger_extended(glove_current, config, finger_name)
        else:
            # Use the original multi-state mapping
            return self._map_finger_original(glove_current, config, finger_name)

    def _map_finger_original(self, glove_current: np.ndarray,
                            config: dict, finger_name: str = None) -> float:
        """
        Original multi-state finger mapping
        """
        joints = config['joints']
        weights = self._normalize_weights(config['weights'])
        robot_idx = config['robot_idx']

        # Calculate the current fused value
        current_fused = self._calculate_fused_value(
            glove_current, joints, weights
        )

        # Calculate fused values for all states
        state_fused_values = {}
        for state_name in self.state_order:
            fused = self._calculate_reference_fused(
                joints, weights, self.glove_states[state_name]
            )
            state_fused_values[state_name] = fused

        # Get angles for all states
        state_angles = {}
        for state_name in self.state_order:
            state_angles[state_name] = self.robot_states[state_name][robot_idx]

        if self._should_debug(finger_name):
            print(f"\n=== {finger_name} debug information (original) ===")
            print(f"enabled states: {self.state_order}")
            print(f"weights: {config['weights']}")
            joints = config['joints']
            glove_joints_vals = {f"glove[{j}]": glove_current[j] for j in joints}
            print(f"glove data: {glove_joints_vals}")
            print(f"fused value: {current_fused:.6f}")
            print(f"state fused values: {state_fused_values}")
            print(f"state angles: {state_angles}")

        # Piecewise linear interpolation
        result_angle = self._multi_state_interpolation(
            current_fused, state_fused_values, state_angles
        )

        if self._should_debug(finger_name):
            print(f"interpolation result: {result_angle:.6f}")

        # Handle reverse motion
        if config.get('reverse_motion', True):
            # Find the minimum and maximum angles
            min_angle = min(state_angles.values())
            max_angle = max(state_angles.values())

            result_angle = max_angle - (result_angle - min_angle)
            if self._should_debug(finger_name):
                print(f"reverse_motion=True, after reversal: {result_angle:.6f}")

        return result_angle

    def _map_finger_extended(self, glove_current: np.ndarray,
                            config: dict, finger_name: str) -> float:
        """
        Multi-segment mapping implementation

        Determine the number of mapping segments from state_order:
        - ['origin', 'opose', 'fist'] → Three-segment mapping, clamp to fist
        - ['origin', 'opose'] + extended_mapping.enabled=True → Two-segment mapping, extend and clamp to fist
        - ['origin', 'opose'] + extended_mapping.enabled=False → Two-segment mapping, clamp to opose
        """
        joints = config['joints']
        weights = self._normalize_weights(config['weights'])
        robot_idx = config['robot_idx']

        # Get the list of enabled states
        states = self.state_order
        num_states = len(states)

        if num_states < 2:
            print(f"Warning: Insufficient number of states, fall back to the original mapping")
            return self._map_finger_original(glove_current, config)

        # Calculate the current fused value
        current_fused_raw = self._calculate_fused_value(glove_current, joints, weights)

        # Calculate the first and last states' fused value
        fused_first = self._calculate_reference_fused(joints, weights, self.glove_states[states[0]])
        fused_last = self._calculate_reference_fused(joints, weights, self.glove_states[states[-1]])

        # Apply the scale factor
        current_fused = self._apply_scale_factor(current_fused_raw, fused_first, fused_last, finger_name)

        # Get angles of the first and last states
        angle_first = self.robot_states[states[0]][robot_idx]
        angle_last = self.robot_states[states[-1]][robot_idx]

        # Ensure the order is correct
        if angle_first > angle_last:
            angle_first, angle_last = angle_last, angle_first

        if self._should_debug(finger_name):
            print(f"\n=== {finger_name} debug information ===")
            print(f"enabled states: {states}")
            print(f"weights: {config['weights']}")
            print(f"original fused value: {current_fused_raw:.6f}")
            print(f"fused value after scaling: {current_fused:.6f}")
            print(f"Fused-value range: [{fused_first:.6f}, {fused_last:.6f}]")
            print(f"robot-hand angle range: [{angle_first:.6f}, {angle_last:.6f}]")

        # Normalize fused value
        if abs(fused_last - fused_first) < 1e-6:
            normalized_fused = 0.5
        else:
            normalized_fused = (current_fused - fused_first) / (fused_last - fused_first)

        if self._should_debug(finger_name):
            print(f"normalized fused value: {normalized_fused:.6f}")

        # Determine whether extension is needed (only when original and opose are enabled).
        extrapolation_enabled = self.extended_mapping_enabled.get(finger_name, False)
        use_extrapolation = extrapolation_enabled and states == ['original', 'opose']

        if num_states >= 3:
            result_angle = self._multi_state_map(joints, weights, robot_idx, current_fused_raw, finger_name)
        elif use_extrapolation:
            result_angle = self._extrapolate_to_fist(
                current_fused, fused_first, fused_last,
                angle_first, angle_last, robot_idx, finger_name, joints, weights
            )
        else:
            result_angle = self._two_state_map(
                current_fused, fused_first, fused_last,
                angle_first, angle_last, finger_name
            )

        if config.get('reverse_motion', False):
            min_angle = min(angle_first, angle_last)
            max_angle = max(angle_first, angle_last)
            clamped = np.clip(result_angle, min_angle, max_angle)
            result_angle = max_angle - (clamped - min_angle)

        return result_angle

    def _multi_state_map(self, joints, weights, robot_idx, current_fused, finger_name):
        """Multi-segment mapping: Use all enabled states for piecewise interpolation"""
        state_fused_values = {}
        state_angles = {}

        for state_name in self.state_order:
            fused = self._calculate_reference_fused(joints, weights, self.glove_states[state_name])
            state_fused_values[state_name] = fused
            state_angles[state_name] = self.robot_states[state_name][robot_idx]

        result_angle = self._multi_state_interpolation(current_fused, state_fused_values, state_angles)

        if self._should_debug(finger_name):
            print(f"Multi-segment mapping result: {result_angle:.6f}")

        return result_angle

    def _extrapolate_to_fist(self, current_fused, fused_first, fused_last,
                             angle_first, angle_last, robot_idx, finger_name, joints, weights):
        """Two-segment mapping + extended mapping, clamp to fist angle"""
        if abs(fused_last - fused_first) < 1e-6:
            normalized = 0.5
        else:
            normalized = (current_fused - fused_first) / (fused_last - fused_first)

        if self._should_debug(finger_name):
            print(f"normalized fused value: {normalized:.6f}")

        exp_factor = self.exp_factors.get(finger_name, 1.0)
        slope = angle_last - angle_first

        if normalized <= 0:
            result_angle = angle_first
            if self._should_debug(finger_name):
                print(f"normalized value<=0: result_angle={result_angle:.6f}")
        elif normalized <= 1:
            result_angle = angle_first + normalized * slope
            if self._should_debug(finger_name):
                print(f"normalized value is within [0, 1]: result_angle={result_angle:.6f}")
        else:
            t = normalized - 1.0
            extension = slope * t * (1.0 + (exp_factor - 1.0) * t)
            result_angle = angle_last + extension

            if self._should_debug(finger_name):
                print(f"extend: normalized={normalized:.6f}, t={t:.4f}, exp_factor={exp_factor:.2f}, result={result_angle:.6f}")

        if 'fist' in self.robot_states:
            angle_fist = self.robot_states['fist'][robot_idx]
            if slope > 0:
                result_angle = min(result_angle, angle_fist)
            else:
                result_angle = max(result_angle, angle_fist)
            if self._should_debug(finger_name):
                print(f"clamp to fist: angle_fist={angle_fist:.6f}, result={result_angle:.6f}")

        return result_angle

    def _two_state_map(self, current_fused, fused_first, fused_last,
                       angle_first, angle_last, finger_name):
        """Two-segment mapping: Linearly interpolate and clamp to the last state"""
        if abs(fused_last - fused_first) < 1e-6:
            normalized = 0.5
        else:
            normalized = (current_fused - fused_first) / (fused_last - fused_first)

        # clamp to [0, 1]
        normalized = max(0.0, min(1.0, normalized))

        result_angle = angle_first + normalized * (angle_last - angle_first)

        if self._should_debug(finger_name):
            print(f"Two-segment mapping with clamping: normalized={normalized:.6f}, result={result_angle:.6f}")

        return result_angle

        return result_angle

    def get_mapping_info(self, finger_name: str = None) -> Dict:
        """
        Get mapping information
        """
        if finger_name:
            return self._get_finger_info(finger_name)
        else:
            return {name: self._get_finger_info(name) for name in self.finger_configs}

    def _get_finger_info(self, finger_name: str) -> Dict:
        """Get information for one finger"""
        if finger_name not in self.finger_configs:
            return {}

        robot_idx = self.finger_configs[finger_name]['robot_idx']
        max_angle = self._get_max_angle(robot_idx)

        info = {
            'name': self.finger_configs[finger_name]['name'],
            'robot_idx': robot_idx,
            'has_dynamic_weight': finger_name in self.dynamic_weight_configs,
            'has_extended_mapping': self.extended_mapping_enabled.get(finger_name, False),
            'scale_factor': self.scale_factors.get(finger_name, 1.0),
            'max_angle': max_angle
        }

        # if presentopenandoposestate, display relevant information
        if 'open' in self.robot_states and 'opose' in self.robot_states:
            open_angle = self.robot_states['open'][robot_idx]
            opose_angle = self.robot_states['opose'][robot_idx]
            info.update({
                'open_angle': open_angle,
                'opose_angle': opose_angle,
                'available_extension': max_angle - opose_angle
            })

        return info

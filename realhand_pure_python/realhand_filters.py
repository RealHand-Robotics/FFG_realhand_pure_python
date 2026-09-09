import numpy as np
from typing import List, Optional
from collections import deque

class LCFilter:
    """
    First-order LC low-pass filter.
    Discrete-time implementation commonly used for signal smoothing.
    """

    def __init__(self, alpha: float = 0.1, initial_value: float = 0.0):
        """
        Initialize the LC filter.

        Parameters:
            alpha: filter coefficient (0 < alpha <= 1)
                   Smaller alpha: stronger filtering (smoother).
                   Larger alpha: faster response (more sensitive).
            initial_value: initial value
        """
        if alpha <= 0 or alpha > 1:
            raise ValueError("alpha must be within the range (0, 1]")

        self.alpha = alpha
        self.filtered_value = initial_value
        self.previous_raw = initial_value
        self.previous_filtered = initial_value

        # history (optional, for debugging)
        self.history_raw = []
        self.history_filtered = []

    def update(self, new_value: float) -> float:
        """
        Update the filter and return the filtered value

        Formula: y[n] = α * x[n] + (1-α) * y[n-1]
        where x[n] is the current input and y[n-1] is the previous output.

        Parameters:
            new_value: new input value
        Returns:
            filtered value
        """
        # Save historical values
        self.previous_raw = new_value
        self.previous_filtered = self.filtered_value

        # LC filter equation
        self.filtered_value = self.alpha * new_value + (1 - self.alpha) * self.filtered_value

        # Record history (optional)
        self.history_raw.append(new_value)
        self.history_filtered.append(self.filtered_value)

        return self.filtered_value

    def update_array(self, new_values: List[float]) -> List[float]:
        """
        Batch-update an array

        Parameters:
            new_values: new input-value list
        Returns:
            filtered-value list
        """
        filtered_values = []
        for value in new_values:
            filtered = self.update(value)
            filtered_values.append(filtered)
        return filtered_values

    def reset(self, initial_value: float = 0.0):
        """Reset filter state"""
        self.filtered_value = initial_value
        self.previous_raw = initial_value
        self.previous_filtered = initial_value
        self.history_raw = []
        self.history_filtered = []

    def get_state(self):
        """Get the current state"""
        return {
            'filtered_value': self.filtered_value,
            'alpha': self.alpha,
            'history_length': len(self.history_raw)
        }


class MultiChannelLCFilter:
    """
    Multi-channel LC filter.
    Filter multiple signals simultaneously
    """

    def __init__(self, num_channels: int, alpha: float = 0.1,
                 initial_values: Optional[List[float]] = None):
        """
        Initialize the multi-channel filter

        Parameters:
            num_channels: channel count
            alpha: filter coefficient
            initial_values: Initial-value list; its length must equal num_channels.
        """
        self.num_channels = num_channels
        self.alpha = alpha

        if initial_values is None:
            initial_values = [0.0] * num_channels
        elif len(initial_values) != num_channels:
            raise ValueError(f"initial-value length must equal the channel count {num_channels}")

        # Create a filter for each channel
        self.filters = [LCFilter(alpha, initial_values[i]) for i in range(num_channels)]

    def update(self, new_values: List[float]) -> List[float]:
        """
        Update all channels

        Parameters:
            new_values: New input-value list; its length must equal num_channels.
        Returns:
            filtered-value list
        """
        if len(new_values) != self.num_channels:
            raise ValueError(f"input-value length must equal the channel count {self.num_channels}")

        filtered_values = []
        for i in range(self.num_channels):
            filtered = self.filters[i].update(new_values[i])
            filtered_values.append(filtered)

        return filtered_values

    def update_channel(self, channel_idx: int, new_value: float) -> float:
        """
        Update one channel

        Parameters:
            channel_idx: channel index (0-based)
            new_value: new input value
        Returns:
            filtered value
        """
        if channel_idx < 0 or channel_idx >= self.num_channels:
            raise ValueError(f"channel index must be within [0, {self.num_channels - 1}]")

        return self.filters[channel_idx].update(new_value)

    def reset(self, initial_values: Optional[List[float]] = None):
        """Reset all channels"""
        if initial_values is None:
            initial_values = [0.0] * self.num_channels

        for i in range(self.num_channels):
            self.filters[i].reset(initial_values[i])

    def get_state(self):
        """Get the state of all channels"""
        states = []
        for i, filter_obj in enumerate(self.filters):
            state = filter_obj.get_state()
            state['channel'] = i
            states.append(state)
        return states


class AdaptiveLCFilter(LCFilter):
    """
    Adaptive LC filter.
    Automatically adjusts the alpha value according to signal changes.
    """

    def __init__(self, alpha_min: float = 0.05, alpha_max: float = 0.3,
                 change_threshold: float = 0.1, initial_value: float = 0.0):
        """
        Initialize the adaptive filter

        Parameters:
            alpha_min: Minimum alpha value used when the signal is stable.
            alpha_max: Maximum alpha value used when the signal changes rapidly.
            change_threshold: change threshold, above this threshold, the signal is considered to change rapidly
            initial_value: initial value
        """
        super().__init__(alpha_max, initial_value)  # Initially use the maximum alpha.
        self.alpha_min = alpha_min
        self.alpha_max = alpha_max
        self.change_threshold = change_threshold

    def update(self, new_value: float) -> float:
        """
        Adaptively update the filter

        Strategy: use a larger alpha for rapid response when the signal changes
        significantly; use a smaller alpha for smoothing when it is stable.
        """
        # Calculate the signal change
        change_amount = abs(new_value - self.previous_raw)

        # Adapt alpha.
        if change_amount > self.change_threshold:
            # Signal changes rapidly: use a large alpha for rapid response.
            self.alpha = self.alpha_max
        else:
            # Signal is stable: use a small alpha for smoothing.
            self.alpha = self.alpha_min

        # Call the parent-class update method
        return super().update(new_value)


def apply_lc_filter(data: List[float], alpha: float = 0.1) -> List[float]:
    """
    Apply LC filtering to data (functional version).

    Parameters:
        data: input-data list
        alpha: filter coefficient
    Returns:
        filtered-data list
    """
    if alpha <= 0 or alpha > 1:
        raise ValueError("alpha must be within the range (0, 1]")

    if not data:
        return []

    filtered = [data[0]]  # Use the first value directly

    for i in range(1, len(data)):
        # LCfilter equation
        y = alpha * data[i] + (1 - alpha) * filtered[i-1]
        filtered.append(y)

    return filtered

class KalmanFilter:
    """
    Kalman filter (simplified version)
    for filtering one-dimensional signals
    """

    def __init__(self,
                 process_variance: float = 1e-5,
                 measurement_variance: float = 0.1,
                 initial_value: float = 0.0,
                 initial_estimate_error: float = 1.0):
        """
        Initialize the Kalman filter

        Parameters:
            process_variance: process-noise variance (Q, system uncertainty)
            measurement_variance: measurement-noise variance (R, sensor noise)
            initial_value: initial state estimate
            initial_estimate_error: initial estimate-error covariance
        """
        # system model (simple one-dimensional model)
        self.process_variance = process_variance  # Q
        self.measurement_variance = measurement_variance  # R

        # state estimation
        self.x_hat = initial_value  # state estimate
        self.p = initial_estimate_error  # estimate-error covariance

        # History (optional)
        self.history_measurement = []
        self.history_estimate = []
        self.history_kalman_gain = []

    def update(self, measurement: float) -> float:
        """
        Kalman-filter update steps

        Parameters:
            measurement: measurement value
        Returns:
            filtered estimate value
        """
        # 1. Prediction step
        # For a simple one-dimensional model, Assume state is unchanged
        x_hat_minus = self.x_hat  # A priori state estimate
        p_minus = self.p + self.process_variance  # A priori estimate error

        # 2. Update steps
        # Calculate the Kalman gain
        k = p_minus / (p_minus + self.measurement_variance)  # Kalman gain

        # Update state estimate
        self.x_hat = x_hat_minus + k * (measurement - x_hat_minus)

        # Update estimate-error covariance
        self.p = (1 - k) * p_minus

        # Record history
        self.history_measurement.append(measurement)
        self.history_estimate.append(self.x_hat)
        self.history_kalman_gain.append(k)

        return self.x_hat

    def update_batch(self, measurements: List[float]) -> List[float]:
        """
        Batch update

        Parameters:
            measurements: measurement-value list
        Returns:
            filtered estimate-value list
        """
        estimates = []
        for measurement in measurements:
            estimate = self.update(measurement)
            estimates.append(estimate)
        return estimates

    def reset(self,
              initial_value: float = 0.0,
              initial_estimate_error: float = 1.0):
        """
        Reset filter state
        """
        self.x_hat = initial_value
        self.p = initial_estimate_error
        self.history_measurement = []
        self.history_estimate = []
        self.history_kalman_gain = []

    def get_state(self) -> dict:
        """
        Get the current state
        """
        return {
            'estimate': self.x_hat,
            'error_covariance': self.p,
            'process_variance': self.process_variance,
            'measurement_variance': self.measurement_variance
        }


class MultiChannelKalmanFilter:
    """
    Multi-channel Kalman filter
    Filter multiple independent signals simultaneously
    """

    def __init__(self,
                 num_channels: int,
                 process_variance: float = 1e-5,
                 measurement_variance: float = 0.1,
                 initial_values: Optional[List[float]] = None):
        """
        Initialize the multi-channel Kalman filter

        Parameters:
            num_channels: channel count
            process_variance: process-noise variance
            measurement_variance: measurement-noise variance
            initial_values: initial-value list
        """
        self.num_channels = num_channels

        if initial_values is None:
            initial_values = [0.0] * num_channels
        elif len(initial_values) != num_channels:
            raise ValueError(f"initial-value length must equal the channel count {num_channels}")

        # Create an independent Kalman filter for each channel
        self.filters = [
            KalmanFilter(
                process_variance=process_variance,
                measurement_variance=measurement_variance,
                initial_value=initial_values[i],
                initial_estimate_error=1.0
            ) for i in range(num_channels)
        ]

    def update(self, measurements: List[float]) -> List[float]:
        """
        Update all channels

        Parameters:
            measurements: measurement-value list, length must equalnum_channels
        Returns:
            filtered estimate-value list
        """
        if len(measurements) != self.num_channels:
            raise ValueError(f"measurement length must equal the channel count {self.num_channels}")

        estimates = []
        for i in range(self.num_channels):
            estimate = self.filters[i].update(measurements[i])
            estimates.append(estimate)

        return estimates

    def update_channel(self, channel_idx: int, measurement: float) -> float:
        """
        Update one channel

        Parameters:
            channel_idx: channel index
            measurement: measurement value
        Returns:
            filtered estimate value
        """
        if channel_idx < 0 or channel_idx >= self.num_channels:
            raise ValueError(f"channel index must be within[0, {self.num_channels-1}]rangewithin")

        return self.filters[channel_idx].update(measurement)

    def reset(self, initial_values: Optional[List[float]] = None):
        """
        Reset all channels
        """
        if initial_values is None:
            initial_values = [0.0] * self.num_channels

        for i in range(self.num_channels):
            self.filters[i].reset(
                initial_value=initial_values[i],
                initial_estimate_error=1.0
            )

    def get_state(self, channel_idx: Optional[int] = None) -> dict:
        """
        Get state information
        """
        if channel_idx is not None:
            if channel_idx < 0 or channel_idx >= self.num_channels:
                raise ValueError(f"channel index must be within[0, {self.num_channels-1}]rangewithin")
            return self.filters[channel_idx].get_state()
        else:
            states = []
            for i, filter_obj in enumerate(self.filters):
                state = filter_obj.get_state()
                state['channel'] = i
                states.append(state)
            return {'channels': states}


class AdaptiveKalmanFilter(KalmanFilter):
    """
    Adaptive Kalman filter
    Automatically adjust parameters according to measurement noise
    """

    def __init__(self,
                 min_process_variance: float = 1e-6,
                 max_process_variance: float = 1e-3,
                 initial_measurement_variance: float = 0.1,
                 adaptation_rate: float = 0.01,
                 initial_value: float = 0.0):
        """
        Initialize the adaptive Kalman filter

        Parameters:
            min_process_variance: minimum process-noise variance
            max_process_variance: maximum process-noise variance
            initial_measurement_variance: initial measurement-noise variance
            adaptation_rate: adaptive adjustment rate
        """
        super().__init__(
            process_variance=(min_process_variance + max_process_variance) / 2,
            measurement_variance=initial_measurement_variance,
            initial_value=initial_value
        )

        self.min_process_variance = min_process_variance
        self.max_process_variance = max_process_variance
        self.adaptation_rate = adaptation_rate
        self.measurement_history = []

    def update(self, measurement: float) -> float:
        """
        Adaptive update
        """
        # Save measurement history
        self.measurement_history.append(measurement)
        if len(self.measurement_history) > 10:
            self.measurement_history.pop(0)

        # Calculate recent measurement noise
        if len(self.measurement_history) >= 5:
            recent_std = np.std(self.measurement_history[-5:])
            # Adjust process-noise variance according to noise level
            if recent_std > 0.1:
                # high noise, addprocess-noise variance
                self.process_variance = min(
                    self.process_variance * (1 + self.adaptation_rate),
                    self.max_process_variance
                )
            else:
                # low noise, decreaseprocess-noise variance
                self.process_variance = max(
                    self.process_variance * (1 - self.adaptation_rate),
                    self.min_process_variance
                )

        # Call the parent-class update method
        return super().update(measurement)


class SavitzkyGolayFilter:
    """
    Savitzky-Golay filter (real-time version)
    smoothing that preserves waveform features
    """

    def __init__(self, window_length: int = 7, polyorder: int = 2,
                 deriv: int = 0, delta: float = 1.0):
        """
        initializeSavitzky-Golay filter

        Parameters:
            window_length: window length (must be odd, and greater thanpolyorder)
            polyorder: polynomial order
            deriv: derivative order (0represents smoothing, 1represents a first derivative, etc.)
            delta: sampling interval
        """
        if window_length % 2 == 0:
            raise ValueError("window_lengthmust be odd")
        if window_length <= polyorder:
            raise ValueError("window_lengthmust be greater thanpolyorder")

        self.window_length = window_length
        self.polyorder = polyorder
        self.deriv = deriv
        self.delta = delta

        # data buffer
        self.buffer = deque(maxlen=window_length)

        # Calculate filter coefficients
        self.coefficients = self._compute_coefficients()

        # history
        self.history_input = []
        self.history_output = []

    def _compute_coefficients(self) -> np.ndarray:
        """
        Calculate Savitzky-Golay filter coefficients

        Returns:
            filter-coefficient array
        """
        # simple implementation: Use sliding-window polynomial fitting
        # For real-time applications, We need only the coefficients for the center point
        half_window = self.window_length // 2

        # Construct the Vandermonde matrix
        x = np.arange(-half_window, half_window + 1, dtype=float)
        A = np.vander(x, self.polyorder + 1, increasing=True)

        # Solve coefficients using least squares
        # ForSavitzky-Golay, We need only the fitted value at the center point
        # This is equivalent to takingA's first pseudoinverse row
        coeff = np.linalg.pinv(A)[self.deriv]

        # Account for derivative order and sampling interval
        if self.deriv > 0:
            for i in range(self.deriv):
                coeff = np.polyder(coeff)
            coeff = coeff / (self.delta ** self.deriv)

        return coeff

    def update(self, new_value: float) -> float:
        """
        Update the filter and return the filtered value

        Parameters:
            new_value: new input value
        Returns:
            filtered value
        """
        # Add to the buffer
        self.buffer.append(new_value)

        # If the buffer is not full, Return the original value directly
        if len(self.buffer) < self.window_length:
            self.history_input.append(new_value)
            self.history_output.append(new_value)
            return new_value

        # Apply Savitzky-Golay filtering
        # Convert the buffer to an array
        window_data = np.array(self.buffer)

        # Convolve using precomputed coefficients
        filtered_value = np.dot(window_data, self.coefficients)

        # Record history
        self.history_input.append(new_value)
        self.history_output.append(filtered_value)

        # limit history length
        max_history = 1000
        if len(self.history_input) > max_history:
            self.history_input = self.history_input[-max_history:]
            self.history_output = self.history_output[-max_history:]

        return filtered_value

    def update_batch(self, new_values: List[float]) -> List[float]:
        """
        Batch update

        Parameters:
            new_values: new input-value list
        Returns:
            filtered-value list
        """
        filtered_values = []
        for value in new_values:
            filtered = self.update(value)
            filtered_values.append(filtered)
        return filtered_values

    def reset(self):
        """Reset filter state"""
        self.buffer.clear()
        self.history_input = []
        self.history_output = []

    def get_state(self) -> dict:
        """Get the current state"""
        return {
            'window_length': self.window_length,
            'polyorder': self.polyorder,
            'deriv': self.deriv,
            'buffer_size': len(self.buffer),
            'coefficients': self.coefficients.tolist()
        }


class MultiChannelSavitzkyGolayFilter:
    """
    Multi-channel Savitzky-Golay filter
    """

    def __init__(self, num_channels: int,
                 window_length: int = 7, polyorder: int = 2,
                 initial_values: Optional[List[float]] = None):
        """
        Initialize the multi-channel filter

        Parameters:
            num_channels: channel count
            window_length: window length
            polyorder: polynomial order
            initial_values: initial-value list
        """
        self.num_channels = num_channels

        if initial_values is None:
            initial_values = [0.0] * num_channels
        elif len(initial_values) != num_channels:
            raise ValueError(f"initial-value length must equal the channel count {num_channels}")

        # Create filters for each channel
        self.filters = []
        for i in range(num_channels):
            filter_obj = SavitzkyGolayFilter(
                window_length=window_length,
                polyorder=polyorder
            )
            # Fill the buffer with the initial value
            for _ in range(window_length // 2):
                filter_obj.update(initial_values[i])
            self.filters.append(filter_obj)

    def update(self, new_values: List[float]) -> List[float]:
        """
        Update all channels

        Parameters:
            new_values: new input-value list
        Returns:
            filtered-value list
        """
        if len(new_values) != self.num_channels:
            raise ValueError(f"input-value length must equal the channel count {self.num_channels}")

        filtered_values = []
        for i in range(self.num_channels):
            filtered = self.filters[i].update(new_values[i])
            filtered_values.append(filtered)

        return filtered_values

    def reset(self, initial_values: Optional[List[float]] = None):
        """Reset all channels"""
        if initial_values is None:
            initial_values = [0.0] * self.num_channels

        for i in range(self.num_channels):
            self.filters[i].reset()
            # Warm up with the initial value
            for _ in range(self.filters[i].window_length // 2):
                self.filters[i].update(initial_values[i])


class AdaptiveSavitzkyGolayFilter:
    """
    adaptiveSavitzky-Golay filter
    Automatically adjust parameters according to signal characteristics
    """

    def __init__(self,
                 min_window: int = 5,
                 max_window: int = 15,
                 base_polyorder: int = 2,
                 noise_threshold: float = 0.05,
                 initial_value: float = 0.0):
        """
        Initialize the adaptive filter

        Parameters:
            min_window: minimum window length
            max_window: maximum window length
            base_polyorder: base polynomial order
            noise_threshold: noise threshold
            initial_value: initial value
        """
        self.min_window = min_window
        self.max_window = max_window
        self.base_polyorder = base_polyorder
        self.noise_threshold = noise_threshold

        # current filter
        self.current_filter = SavitzkyGolayFilter(
            window_length=(min_window + max_window) // 2,
            polyorder=base_polyorder
        )

        # signal-characteristic tracking
        self.signal_buffer = deque(maxlen=20)
        self.current_noise_level = 0.0

    def update(self, new_value: float) -> float:
        """
        Adaptive update
        """
        # Update the signal buffer
        self.signal_buffer.append(new_value)

        # Calculate signal characteristics (noise level)
        if len(self.signal_buffer) >= 10:
            recent_data = np.array(self.signal_buffer)
            self.current_noise_level = np.std(recent_data)

        # Adjust window size according to noise level
        if len(self.signal_buffer) >= 5:
            if self.current_noise_level > self.noise_threshold * 2:
                # high noise, use a large window for strong filtering
                new_window = self.max_window
            elif self.current_noise_level > self.noise_threshold:
                # medium noise, use a medium window
                new_window = (self.min_window + self.max_window) // 2
            else:
                # low noise, use a small window to retain detail
                new_window = self.min_window

            # If the window size needs to change, Create a new filter
            if new_window != self.current_filter.window_length:
                # Use the current filter output as the initial state of the new filter
                current_output = self.current_filter.update(new_value)

                # Create a new filter
                self.current_filter = SavitzkyGolayFilter(
                    window_length=new_window,
                    polyorder=min(self.base_polyorder, new_window - 1)
                )

                # Warm up the new filter with the current output
                for _ in range(new_window // 2):
                    self.current_filter.update(current_output)

                return current_output

        # Use the current filter
        return self.current_filter.update(new_value)

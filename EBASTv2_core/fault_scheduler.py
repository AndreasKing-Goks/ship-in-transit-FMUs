"""
Fault scheduler for Event-Based Adaptive Stress-Testing
"""

import numpy as np

class StratifiedFaultScheduler:

    def __init__(
        self,
        min_episode_time=3000.0,
        step_size=5.0,
        fault_duration=150.0,
        n_strata=10,
        timing_resolution=30.0,
        jitter_slots=1,
        seed=None,
    ):
        # Random number generator
        self.rng = np.random.default_rng(seed)

        # Simulator step size
        self.step_size = step_size
        # Length of fault duration
        self.fault_duration = fault_duration
        # Strata division count of the allowed fault injection time period
        self.n_strata = n_strata
        # Measured in timing resolution slots. Controls how far injection selection is
        self.jitter_slots = jitter_slots

        # Latest permissible injection time
        max_injection_time = min_episode_time - fault_duration

        if max_injection_time < 0:
            raise ValueError(
                "Fault duration exceeds minimum episode time."
            )

        # Convert timing to simulation step indices
        max_step = int(np.floor(max_injection_time / step_size))

        # Adjust the timing to match the simulator time step size
        grid_steps = round(timing_resolution / step_size)

        if grid_steps < 1:
            raise ValueError("Timing resolution is too small.")

        # Coarse candidate injection times
        # List of available time sample
        candidate_steps = np.arange(
            0,
            max_step + 1,
            grid_steps,
            dtype=int,
        )

        # Divide possible injection times into strata
        self.strata = np.array_split(
            candidate_steps,
            n_strata,
        )

        if any(len(s) == 0 for s in self.strata):
            raise ValueError("Too many strata for the timing grid.")

        # One randomized permutation per block
        self.stratum_order = []
        self.current_index = 0

    def sample(self):

        # Generate a new permutation after all strata are used
        if self.current_index >= len(self.stratum_order):
            # Create an order to ensure all strata are explored
            self.stratum_order = self.rng.permutation(
                self.n_strata
            )
            # Reset the index
            self.current_index = 0

        # Select the next stratum
        stratum_idx = self.stratum_order[self.current_index]

        # Advance the index
        self.current_index += 1

        # Get the strata candidate
        candidates = self.strata[stratum_idx]

        # Find central candidate (floor division)
        center_idx = len(candidates) // 2

        # Apply a small amount of jitter
        low = max(0, center_idx - self.jitter_slots)

        high = min(
            len(candidates),
            center_idx + self.jitter_slots + 1
        )

        # Pick one time between low and high
        injection_step = self.rng.choice(
            candidates[low:high]
        )

        # Convert from timing resolution to actual simulation time
        injection_time_start    = injection_step * self.step_size
        injection_time_end      = injection_time_start + self.fault_duration

        return float(injection_time_start), float(injection_time_end)
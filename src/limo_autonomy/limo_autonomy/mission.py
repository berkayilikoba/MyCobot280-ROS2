"""Mission state machine and configuration loading for LIMO autonomy.

Pure Python module with no ROS dependencies.
"""

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import yaml


@dataclass(frozen=True)
class Waypoint:
    """A target waypoint with name, 2D coordinates and heading."""

    name: str
    x: float
    y: float
    yaw_deg: float


class MissionPlanner:
    """State machine governing autonomous patrol missions."""

    VALID_OUTCOMES = {'succeeded', 'failed', 'timeout', 'canceled'}

    def __init__(
        self,
        waypoints: List[Waypoint],
        home: Optional[Waypoint] = None,
        loop: bool = False,
        max_laps: int = 1,
        max_retries: int = 1,
        max_consecutive_failures: int = 2,
    ) -> None:
        """Initialize the mission planner."""
        if not waypoints:
            raise ValueError("Waypoints list cannot be empty.")

        self.waypoints = list(waypoints)
        self.home = home
        self.loop = loop
        self.max_laps = max_laps
        self.max_retries = max_retries
        self.max_consecutive_failures = max_consecutive_failures

        self._started = False
        self._done = False
        self._current_idx = 0
        self._attempts = 0
        self._consecutive_failures = 0
        self._pending_return_home: Optional[str] = None
        self._last_action: Optional[Tuple[str, Optional[Waypoint]]] = None
        self._stats: Dict[str, Any] = {
            'succeeded': 0,
            'failed': 0,
            'retries': 0,
            'skipped': 0,
            'laps': 0,
            'end_reason': None,
        }

    def start(self) -> Tuple[str, Waypoint]:
        """Start the mission and return the first action."""
        self._started = True
        self._done = False
        self._current_idx = 0
        self._attempts = 1
        self._last_action = ('goto', self.waypoints[0])
        return self._last_action

    def request_return_home(self, reason: str) -> None:
        """Request returning home on the next reported outcome."""
        self._pending_return_home = reason

    def stats(self) -> Dict[str, Any]:
        """Return a copy of the mission statistics."""
        return dict(self._stats)

    def report(self, outcome: str) -> Tuple[str, Optional[Waypoint]]:
        """Report outcome of the previous action and transition to the next action."""
        if not self._started:
            raise RuntimeError("Cannot report outcome before calling start().")

        if outcome not in self.VALID_OUTCOMES:
            raise ValueError(f"Invalid outcome '{outcome}'. Expected one of {self.VALID_OUTCOMES}.")

        if self._done:
            return ('done', None)

        if self._last_action is not None and self._last_action[0] == 'home':
            self._done = True
            self._last_action = ('done', None)
            return ('done', None)

        # The last action was a 'goto' waypoint
        if outcome == 'succeeded':
            self._stats['succeeded'] += 1
            self._consecutive_failures = 0
            if self._pending_return_home is not None:
                self._stats['end_reason'] = self._pending_return_home
                self._pending_return_home = None
                return self._go_home()
            return self._advance()

        # Failed, timeout, or canceled
        self._stats['failed'] += 1
        if self._pending_return_home is not None:
            self._stats['end_reason'] = self._pending_return_home
            self._pending_return_home = None
            return self._go_home()

        if self._attempts < self.max_retries + 1:
            self._attempts += 1
            self._stats['retries'] += 1
            self._last_action = ('goto', self.waypoints[self._current_idx])
            return self._last_action

        self._stats['skipped'] += 1
        self._consecutive_failures += 1
        if self._consecutive_failures >= self.max_consecutive_failures:
            self._stats['end_reason'] = 'too_many_failures'
            return self._go_home()

        return self._advance()

    def _go_home(self) -> Tuple[str, Optional[Waypoint]]:
        """Transition to home waypoint or mark mission done if no home is configured."""
        if self.home is not None:
            self._last_action = ('home', self.home)
            return self._last_action
        self._done = True
        self._last_action = ('done', None)
        return self._last_action

    def _advance(self) -> Tuple[str, Optional[Waypoint]]:
        """Advance to the next waypoint or complete the mission/lap."""
        self._current_idx += 1
        if self._current_idx >= len(self.waypoints):
            self._stats['laps'] += 1
            if self.loop and (self.max_laps == 0 or self._stats['laps'] < self.max_laps):
                self._current_idx = 0
                self._attempts = 1
                self._last_action = ('goto', self.waypoints[0])
                return self._last_action
            self._stats['end_reason'] = 'completed'
            return self._go_home()

        self._attempts = 1
        self._last_action = ('goto', self.waypoints[self._current_idx])
        return self._last_action


def load_mission(path: str) -> Dict[str, Any]:
    """Load and validate mission configuration from a YAML file."""
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)

    if not isinstance(data, dict):
        raise ValueError(f"Invalid YAML content in '{path}': expected a dictionary mapping.")

    if 'waypoints' not in data or not isinstance(data['waypoints'], list):
        raise ValueError(f"Mission configuration in '{path}' must contain a 'waypoints' list.")

    waypoints: List[Waypoint] = []
    for idx, wp in enumerate(data['waypoints']):
        if not isinstance(wp, dict):
            raise ValueError(f"Waypoint at index {idx} in '{path}' must be a dictionary.")
        if 'x' not in wp or 'y' not in wp:
            raise ValueError(f"Waypoint at index {idx} in '{path}' is missing required 'x' or 'y'.")
        name = str(wp.get('name', f'wp_{idx}'))
        try:
            x = float(wp['x'])
            y = float(wp['y'])
        except (TypeError, ValueError) as err:
            raise ValueError(f"Coordinates 'x' and 'y' for waypoint '{name}' must be numeric.") from err
        yaw = float(wp.get('yaw', wp.get('yaw_deg', 0.0)))
        waypoints.append(Waypoint(name, x, y, yaw))

    home: Optional[Waypoint] = None
    if 'home' in data and data['home'] is not None:
        home_data = data['home']
        if not isinstance(home_data, dict):
            raise ValueError(f"Home configuration in '{path}' must be a dictionary.")
        if 'x' not in home_data or 'y' not in home_data:
            raise ValueError(f"Home configuration in '{path}' is missing required 'x' or 'y'.")
        try:
            hx = float(home_data['x'])
            hy = float(home_data['y'])
        except (TypeError, ValueError) as err:
            raise ValueError(f"Coordinates 'x' and 'y' for home must be numeric.") from err
        hyaw = float(home_data.get('yaw', home_data.get('yaw_deg', 0.0)))
        home = Waypoint('home', hx, hy, hyaw)

    loop = bool(data.get('loop', False))
    max_laps = int(data.get('max_laps', 1))
    max_retries = int(data.get('max_retries', 1))
    max_consecutive_failures = int(data.get('max_consecutive_failures', 2))

    return {
        'waypoints': waypoints,
        'home': home,
        'loop': loop,
        'max_laps': max_laps,
        'max_retries': max_retries,
        'max_consecutive_failures': max_consecutive_failures,
    }

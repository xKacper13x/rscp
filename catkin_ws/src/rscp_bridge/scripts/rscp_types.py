from dataclasses import dataclass
from enum import Enum


@dataclass
class GPSCoordinate:
    latitude: float     # In degrees.
    longitude: float    # In degrees.
    altitude: float     # In meters.


@dataclass
class BatteryState:
    voltage: float          # Battery voltage in volts.
    current: float          # Battery current in amperes.
    state_of_charge: float  # Value between 0.0 and 1.0.


@dataclass
class MeasuredDistance:
    distance: float


class RoverState(Enum):
    DISARMED = 0
    AUTONOMOUS = 1
    MANUAL = 2


@dataclass
class RoverStatus:
    state: RoverState               # The rover's current state.
    coordinate: GPSCoordinate       # The rover's current GPS coordinate.
    heading: float                  # The rover's compass heading.
    battery_state: BatteryState     # The rover's battery state.

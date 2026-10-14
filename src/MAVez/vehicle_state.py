# vehicle_state.py
# version: 1.0.0
# Original Author: Taiyo Molessa
# Creation Date: 2026-10-14
# Last Modified: 2026-10-14
# Organization: PSU UAS

""" Data hierarchy for the current state of the vehicle """

from dataclasses import dataclass

@dataclass(frozen=True)
class Position:
    """
        Position data in meters in north, east, down notation based on a local reference point.

        north (float): distance north in meters from reference
        east (float): distance east in meters from reference
        down (float): distance down in meters from reference (positive down, negative up)
    """
    north: float
    east: float
    down: float

@dataclass(frozen=True)
class Velocity:
    """
        Velocity data in meters per second in north, east, down notation

        north (float): velocity north in m/s
        east (float): velocity east in m/s
        down (float): velocity down in m/s
    """
    north: float
    east: float
    down: float

@dataclass(frozen=True)
class Attitude:
    """
        Attitude data in radians

        roll (float): roll in radians
        pitch (float): pitch in radians
        yaw (float): yaw in radians
    """
    roll: float
    pitch: float
    yaw: float

@dataclass(frozen=True)
class AngularVelocity:
    """
        Angular velocity data in radians per second

        roll_rate (float): roll rate in rad/s
        pitch_rate (float): pitch rate in rad/s
        yaw_rate (float): yaw rate in rad/s
    """
    roll_rate: float
    pitch_rate: float
    yaw_rate: float

@dataclass(frozen=True)
class GlobalPosition:
    """
        Latitude, longitude, altitude

        latitude (float): in degrees
        longitude (float): in degrees
        altitude (float): in meters above mean sea level
    """
    latitude: float
    longitude: float
    altitude: float

@dataclass(frozen=True)
class VehicleState:
    """
        Represents the current state of the vehicle.

        position (Position): NED in meters
        velocity (Velocity): NED in m/s
        attitude (Attitude): roll, pitch, yaw in rad
        angular_velocity (AngularVelocity): roll_rate, pitch_rate, yaw_rate in rad/s
        global_position (GlobalPosition): latitude/longitude in degrees and altitude in meters above MSL
        timestamp (float): Time estimate of the associated data
    """
    position: Position | None = None
    velocity: Velocity | None = None
    attitude: Attitude | None = None
    angular_velocity: AngularVelocity | None = None
    global_position: GlobalPosition | None = None
    timestamp: float | None = None
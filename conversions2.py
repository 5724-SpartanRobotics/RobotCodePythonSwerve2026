# conversions.py

import math
from wpimath import units

from constants import DriveConstants, TwoPI


class Conversions:
    """
    Conversion utilities for swerve drive calculations.
    """

    @staticmethod
    def vortex_to_radians(counts: float) -> float:
        """
        Converts Vortex motor rotations to radians.
        Specific to the steering motor.
        """
        return counts * (TwoPI / DriveConstants.turnGearRatio)

    @staticmethod
    def vortex_to_degrees(counts: float) -> float:
        """
        Converts steering motor rotations to wheel degrees.
        """
        return counts * (360.0 / DriveConstants.turnGearRatio)

    @staticmethod
    def radians_to_vortex(radians: float) -> float:
        """
        Converts radians to Vortex motor rotations.
        """
        return radians / (TwoPI / DriveConstants.turnGearRatio)

    @staticmethod
    def degrees_to_vortex(degrees: float) -> float:
        """
        Converts wheel degrees to Vortex motor rotations.
        """
        return degrees / (360.0 / DriveConstants.turnGearRatio)

    @staticmethod
    def vortex_to_rpm(velocity_counts: float) -> float:
        """
        Converts motor rotations/sec to wheel RPM.

        NOTE:
        The original Java code was named falconToRPM and assumed
        Phoenix velocity units (rotations/100ms). If you are using
        REV Spark MAX/Flex with a NEO or Vortex, the encoder velocity
        is already RPM and this conversion should NOT be used.
        """
        motor_rpm = velocity_counts * 600.0
        wheel_rpm = motor_rpm / DriveConstants.driveGearRatio
        return wheel_rpm

    @staticmethod
    def vortex_to_mps(velocity_counts: float) -> float:
        """
        Converts velocity counts to wheel speed in m/s.
        """
        wheel_rpm = Conversions.vortex_to_rpm(velocity_counts)
        return (wheel_rpm * DriveConstants.wheelCircumference) / 60.0

    @staticmethod
    def vortex_to_meters(position_counts: float) -> float:
        """
        Converts motor rotations to wheel travel distance in meters.
        """
        return (
            position_counts
            / DriveConstants.driveGearRatio
            * DriveConstants.wheelCircumference
        )

    @staticmethod
    def rpm_to_mps(rpm: float) -> float:
        """
        Converts wheel RPM to meters per second.
        """
        wheel_diameter_m = units.inchesToMeters(
            DriveConstants.wheelDiameter
        )
        return rpm * math.tau * wheel_diameter_m / 60.0


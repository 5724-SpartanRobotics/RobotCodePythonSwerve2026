import math
from constants import *


class Conversions:
    
    @staticmethod
    def radiansToFalcon(radians):
        return radians / (TwoPI / (DriveConstants.turnGearRatio * 1.0))
    
    @staticmethod
    def falconToRPM(velocitycounts):
        motorRPM = velocitycounts * (600.0/1.0)
        wheelRPM = motorRPM / DriveConstants.driveGearRatio
        return wheelRPM
    
    @staticmethod
    def falconToMPS(velocitycounts):
        wheelRPM = Conversions.falconToRPM(velocitycounts)
        wheelMPS = (wheelRPM * DriveConstants.wheelCircumference) / 60
        return wheelMPS
    
    @staticmethod
    def falconToDegrees(counts):
        return counts * (360.0 / (DriveConstants.turnGearRatio * 1.0))
    
    @staticmethod
    def falconToMeters(position_counts: float) -> float:
        """
        Converts motor rotations to wheel travel distance in meters.
        """
        return (
            position_counts
            / DriveConstants.driveGearRatio
            * DriveConstants.wheelCircumference
        )
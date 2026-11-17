import math
from constants import *


class Conversions:
    
    def radiansToFalcon(self,radians):
        return radians / (TwoPI / (DriveConstants.turnGearRatio * 1.0))
    
    def falconToRPM(self,velocitycounts):
        motorRPM = velocitycounts * (600.0/1.0)
        wheelRPM = motorRPM / DriveConstants.driveGearRatio
        return wheelRPM
    
    def falconToMPS(self,velocitycounts):
        wheelRPM = self.falconToRPM(velocitycounts)
        wheelMPS = (wheelRPM * DriveConstants.wheelCircumfrence()) / 60
        return wheelMPS
    
    def falconToDegrees(counts):
        return counts * (360.0 / (DriveConstants.turnGearRatio() * 1.0))
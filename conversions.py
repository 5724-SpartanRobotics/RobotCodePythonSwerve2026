import math
import constant

class Conversions:
    
    def radiansToFalcon(self,radians):
        return radians / (constant.constants.TwoPi / (constant.driveConstants.turnGearRatio * 1.0))
    
    def falconToRPM(self,velocitycounts):
        motorRPM = velocitycounts * (600.0/1.0)
        wheelRPM = motorRPM / constant.driveConstants.driveGearRatio
        return wheelRPM
    
    def falconToMPS(self,velocitycounts):
        wheelRPM = self.falconToRPM(velocitycounts)
        wheelMPS = (wheelRPM * constant.driveConstants.wheelCircumfrence()) / 60
        return wheelMPS
    
    def falconToDegrees(counts):
        return counts * (360.0 / (constant.driveConstants.turnGearRatio() * 1.0))
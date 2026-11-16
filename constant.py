import math
import wpilib
from wpimath.units import *

class constants:
    pigeonID = 14

    def TwoPi(self):
        return math.pi * 2

class driveConstants:
    driveGearRatio = 6.75
    
    def wheelDiameter(self):
        return inchesToMeters(inches:4.125)
    
    def turnGearRatio(self):
        return 150.0 / 7.0
    
    def wheelCircumfrence(self):
        wheelDiameter = wheelDiameter()
        return wheelDiameter * math.pi
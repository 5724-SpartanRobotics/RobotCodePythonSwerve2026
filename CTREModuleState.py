import math

import wpilib
from wpimath.kinematics import SwerveModuleState
from wpimath.geometry import Rotation2d

class CTREModuleState:

    @staticmethod
    def optimize(desiredState: SwerveModuleState,currentAngle: Rotation2d):
        targetAngle = CTREModuleState.placeInAppropriate0To360Scope(currentAngle.degrees(),desiredState.angle.degrees())
        targetSpeed = desiredState.speed
        delta = targetAngle - currentAngle.degrees()
        if abs(delta) > 90:
            targetSpeed = -targetSpeed
            if delta > 90:
                targetAngle -= 180
            else:
                targetAngle += 180
        return SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle))

    @staticmethod
    def placeInAppropriate0To360Scope(scopeReference,newAngle):
        lowerBound: float
        upperBound: float
        lowerOffset = scopeReference % 360
        if lowerOffset >= 0:
            lowerBound = scopeReference - lowerOffset
            upperBound = scopeReference + (360 - lowerOffset)
        else:
            upperBound = scopeReference - lowerOffset
            lowerBound = scopeReference - (360 - lowerOffset)
        while newAngle < lowerBound:
            newAngle += 360
        while newAngle > upperBound:
            newAngle -= 360
        if newAngle - scopeReference > 180:
            newAngle -= 360
        elif newAngle - scopeReference < -180:
            newAngle += 360
        return newAngle
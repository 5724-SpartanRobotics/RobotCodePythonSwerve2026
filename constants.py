# constants.py

from enum import Enum
import math

from wpimath.geometry import Translation2d
from wpimath.units import inchesToMeters


TwoPI = math.tau
HalfPI = math.pi / 2.0


class CanIdConstants:
    LFTurnMotor = 7
    LFDriveMotor = 8
    LBTurnMotor = 6
    LBDriveMotor = 5
    RBTurnMotor = 3
    RBDriveMotor = 4
    RFTurnMotor = 2
    RFDriveMotor = 1

    LFCanID = 10
    LBCanID = 11
    RFCanID = 13
    RBCanID = 12

    PigeonID = 14
    PDHID = 50


class DriveConstants:
    defaultSpeedMod = 0.3

    maxRobotSpeedmps = 4.5
    maxRobotAutoSpeedmps = 2.5

    driveGearRatio = 6.75
    turnGearRatio = 150.0 / 7.0

    wheelDiameter = inchesToMeters(4.25)
    wheelCircumference = wheelDiameter * math.pi

    maxAngularVelocityRadps = TwoPI
    maxLinearSpeed = 15.5 / 3.281

    wheelBase = 0.52705
    trackWidth = 0.5842

    LFOff = 0.0
    RFOff = 0.0
    LBOff = 0.0
    RBOff = 0.0

    LFLocation = Translation2d(
        wheelBase / 2.0,
        trackWidth / 2.0,
    )

    RFLocation = Translation2d(
        wheelBase / 2.0,
        -trackWidth / 2.0,
    )

    LBLocation = Translation2d(
        -wheelBase / 2.0,
        trackWidth / 2.0,
    )

    RBLocation = Translation2d(
        -wheelBase / 2.0,
        -trackWidth / 2.0,
    )

    kNominalVoltage = 12
    kDriveCurrentLimit = 60
    kSteerCurrentLimit = 25


class ControllerConstants:
    joystickDeadband = 0.1
    joystickZDeadband = 0.35



class NeoConstants:
    CountsPerRevolution = 1.0


class DebugLevel (Enum):
    Off = 0
    Swerve = 1
    ArmRotate = 2
    ArmExtend = 3
    Elevator = 4
    Algae = 5
    Claw = 6
    Autonomous = 7
    Vision = 8
    Wrist = 9
    Climber = 10
    All = 11


class DebugSetting:
    TraceLevel = DebugLevel.Autonomous


class TimeDuration:
    Instant = 0.01
    HalfSecond = 0.5
    Second = 1.0
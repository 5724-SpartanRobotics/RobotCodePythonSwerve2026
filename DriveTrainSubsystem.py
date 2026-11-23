#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
import wpilib
import wpimath.kinematics
import swervemodule
from phoenix6.hardware import Pigeon2
import constants
import commands2
import wpimath.geometry

kMaxSpeed = 3.0  # 3 meters per second
kMaxAngularSpeed = math.pi  # 1/2 rotation per second


class DriveTrain(commands2.Subsystem):
    """
    Represents a swerve drive style drivetrain.
    """

    def __init__(self):
        super().__init__()
        self.lastUpdatedGyroHeading = 0.0

        self.gyro = Pigeon2(constants.CanIdConstants.PigeonID)
        self.UpdateGyro()
        self.LF = swervemodule.SwerveModule(constants.CanIdConstants.LFTurnMotor,constants.CanIdConstants.LFDriveMotor,constants.CanIdConstants.LFCanID,constants.DriveConstants.LFOff,"LF",self)
        self.RF = swervemodule.SwerveModule(constants.CanIdConstants.RFTurnMotor,constants.CanIdConstants.RFDriveMotor,constants.CanIdConstants.RFCanID,constants.DriveConstants.RFOff,"RF",self)
        self.LB = swervemodule.SwerveModule(constants.CanIdConstants.LBTurnMotor,constants.CanIdConstants.LBDriveMotor,constants.CanIdConstants.LBCanID,constants.DriveConstants.LBOff,"LB",self)
        self.RB = swervemodule.SwerveModule(constants.CanIdConstants.RBTurnMotor,constants.CanIdConstants.RBDriveMotor,constants.CanIdConstants.RBCanID,constants.DriveConstants.RBOff,"RB",self)

        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            constants.DriveConstants.LFLocation,
            constants.DriveConstants.RFLocation,
            constants.DriveConstants.LBLocation,
            constants.DriveConstants.RBLocation
        )

        self.robotPose = wpimath.geometry.Pose2d(wpimath.geometry.Translation2d(4.0,5.0),wpimath.geometry.Rotation2d())

        self.odometry = wpimath.kinematics.SwerveDrive4Odometry(
            self.kinematics,
            self.gyro.getRotation2d(),
            (
                self.LF.getPosition(),
                self.RF.getPosition(),
                self.LB.getPosition(),
                self.RB.getPosition(),
            ),
        )

        self.gyro.reset()

    def UpdateGyro(self):
        self.lastUpdatedGyroHeading = wpimath.geometry.Rotation2d.fromDegrees(-self.gyro.get_yaw().value_as_double)
    
    def drive(
        self,
        xSpeed: float,
        ySpeed: float,
        rot: float,
        fieldRelative: bool,
        periodSeconds: float,
    ) -> None:
        """
        Method to drive the robot using joystick info.
        :param xSpeed: Speed of the robot in the x direction (forward).
        :param ySpeed: Speed of the robot in the y direction (sideways).
        :param rot: Angular rate of the robot.
        :param fieldRelative: Whether the provided x and y speeds are relative to the field.
        :param periodSeconds: Time
        """
        swerveModuleStates = self.kinematics.toSwerveModuleStates(
            wpimath.kinematics.ChassisSpeeds.discretize(
                (
                    wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, self.gyro.getRotation2d()
                    )
                    if fieldRelative
                    else wpimath.kinematics.ChassisSpeeds(xSpeed, ySpeed, rot)
                ),
                periodSeconds,
            )
        )
        wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, kMaxSpeed
        )
        self.LF.setDesiredState(swerveModuleStates[0])
        self.RF.setDesiredState(swerveModuleStates[1])
        self.LB.setDesiredState(swerveModuleStates[2])
        self.RB.setDesiredState(swerveModuleStates[3])

    def updateOdometry(self) -> None:
        """Updates the field relative position of the robot."""
        self.odometry.update(
            self.gyro.getRotation2d(),
            (
                self.LF.getPosition(),
                self.RF.getPosition(),
                self.LB.getPosition(),
                self.RB.getPosition(),
            ),
        )

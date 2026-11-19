#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math

import wpilib
import wpimath.controller
import wpimath.geometry
import wpimath.kinematics
import wpimath.trajectory
import DriveTrainSubsystem
import phoenix6.hardware
from phoenix6.controls import PositionVoltage
import phoenix6.configs
import phoenix6.signals
import conversions
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState

kWheelRadius = 0.0508
kEncoderResolution = 4096
kModuleMaxAngularVelocity = math.pi
kModuleMaxAngularAcceleration = math.tau


class SwerveModule:
    def __init__(
        self,
        driveMotorID: int,
        turningMotorID: int,
        canCoderID: int,
        offSet: float,
        name: str,
        DriveTrain: DriveTrain
    ) -> None:
        """Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.

        :param driveMotorChannel:      PWM output for the drive motor.
        :param turningMotorChannel:    PWM output for the turning motor.
        :param driveEncoderChannelA:   DIO input for the drive encoder channel A
        :param driveEncoderChannelB:   DIO input for the drive encoder channel B
        :param turningEncoderChannelA: DIO input for the turning encoder channel A
        :param turningEncoderChannelB: DIO input for the turning encoder channel B
        """
        self.turnMotorOffset = offSet
        self.driveMotor = phoenix6.hardware.TalonFX(driveMotorID)
        self.turningMotor = phoenix6.hardware.TalonFX(turningMotorID)
        self.turnVoltagePosition = PositionVoltage(0)
        self.canCoder = phoenix6.hardware.CANcoder(canCoderID)
        self.canCoderName = name + str(canCoderID)
        driveConfiguration = phoenix6.configs.talon_fx_configs.TalonFXConfiguration()
        self.driveMotor.configurator.apply(driveConfiguration)
        self.driveMotor.setNeutralMode(phoenix6.signals.NeutralModeValue.BRAKE)
        self.turnConfiguration = phoenix6.configs.talon_fx_configs.TalonFXConfiguration()
        self.turnConfiguration.hardware_limit_switch.forward_limit_enable = False
        self.turnConfiguration.hardware_limit_switch.reverse_limit_enable = False
        self.turningMotor.setNeutralMode(phoenix6.signals.NeutralModeValue.BRAKE)
        self.turnConfiguration.slot0.k_v = 0
        self.turnConfiguration.slot0.k_p = 0.9
        self.turnConfiguration.slot0.k_i = 0
        self.turnConfiguration.slot0.k_d = 0
        self.turnConfiguration.voltage.peak_forward_voltage = 10
        self.turnConfiguration.voltage.peak_reverse_voltage = -10
        self.resetTurnToAbsolute()
        status = self.turningMotor.configurator.apply(self.turnConfiguration)
        if status != phoenix6.status_code.StatusCode.is_ok:
            for i in range(5):
                status = self.turningMotor.configurator.apply(self.turnConfiguration)
                if status == phoenix6.status_code.StatusCode.is_ok:
                    break

    def resetTurnToAbsolute(self):
        canCodePosInRadians = self.canCoder.get_absolute_position().value_as_double * (math.pi * 2.0)
        absPosition = conversions.Conversions.radiansToFalcon(canCodePosInRadians - self.turnMotorOffset)
        self.turningMotor.set_position(-absPosition)

    def getState(self):
        velocity = conversions.Conversions.falconToMPS(self.driveMotor.get_velocity().value_as_double())
        angle = Rotation2d.fromDegrees(conversions.Conversions.falconToDegrees(-self.turningMotor.get_position().value_as_double()))
        return SwerveModuleState(velocity, angle)
import math
from enum import Enum

import navx
import wpilib
import wpimath.controller
from wpimath.geometry import Rotation2d
import wpimath.units

import ntutil


class SwerveHeadingMode(Enum):
    DISABLED = 0
    """
    Entirely shut off the heading controller. Rotation speed will be passed
    through unmodified.
    """

    HUMAN_DRIVERS = 1
    """
    The heading controller will try to maintain the current heading, but will
    temporarily turn off if the drivers steer the robot, or if something causes
    the robot to physically rotate in an unexpected way.
    """

    FORCE_HEADING = 2
    """
    The heading controller will always attempt to steer the bot toward a
    particular heading, regardless of human control or other factors.
    """


class SwerveHeadingController:
    def __init__(self, gyro: navx.AHRS, mode: SwerveHeadingMode) -> None:
        self.gyro: navx.AHRS = gyro
        self.mode: SwerveHeadingMode = mode
        self.goal: Rotation2d = self.gyro.getRotation2d()
        self.PID = wpimath.controller.PIDController(1 / wpimath.units.degreesToRadians(15), 0, 0)
        self.PID.enableContinuousInput(-math.pi, math.pi)

        self.stateTopic = ntutil.getStringTopic("/SwerveHeading/State")
        self.goalTopic = ntutil.getStructTopic("/SwerveHeading/Goal", Rotation2d)
        self.turningTopic = ntutil.getBooleanTopic("/SwerveHeading/IsTurning")
        self.translatingTopic = ntutil.getBooleanTopic("/SwerveHeading/IsTranslating")
        self.maintainingTopic = ntutil.getBooleanTopic("/SwerveHeading/IsMaintaining")
        self.gyroRateTopic = ntutil.getFloatTopic("/SwerveHeading/GyroRate")
        self.outputTopic = ntutil.getFloatTopic("/SwerveHeading/Output")

        self.badModeAlert = wpilib.Alert("Bad mode for swerve heading controller", wpilib.Alert.AlertType.kError)

    def update(self, speed: float, rot: float) -> float:
        self.stateTopic.set(str(self.mode))
        self.goalTopic.set(self.goal)
        self.gyroRateTopic.set(self.getRateRadiansPerSecond())

        self.PID.setSetpoint(self.goal.radians())

        if self.mode == SwerveHeadingMode.DISABLED:
            output = rot
        elif self.mode == SwerveHeadingMode.HUMAN_DRIVERS:
            # TODO: Validate if 15 deg/s is a reasonable threshold for disabling the controller.
            bot_turning = abs(rot) > 0.1 or abs(self.getRateRadiansPerSecond()) > wpimath.units.degreesToRadians(15)
            bot_translating = speed > 0
            should_maintain_heading = not bot_turning and bot_translating

            self.turningTopic.set(bot_turning)
            self.translatingTopic.set(bot_translating)
            self.maintainingTopic.set(should_maintain_heading)

            if should_maintain_heading:
                # Run PID with the previously-set goal, ignoring `rot`
                output = self.PID.calculate(self.gyro.getRotation2d().radians())
            else:
                # Use `rot`, and update the goal to the current gyro angle to maintain later.
                self.goal = self.gyro.getRotation2d()
                output = rot
        elif self.mode == SwerveHeadingMode.FORCE_HEADING:
            output = self.PID.calculate(self.gyro.getRotation2d().radians())
        else:
            ntutil.logAlert(self.badModeAlert, self.mode)
            output = rot

        self.outputTopic.set(output)
        return output

    def getRateRadiansPerSecond(self) -> float:
        """
        The NavX reports its rate in degrees/second instead of radians/second.
        Please use this method instead of `self.gyro.getRate()` for our own
        sanity.

        Also, be aware that `getRate()` has a history of being totally broken:
        https://github.com/kauailabs/navxmxp/issues/69

        But this was probably fixed (probably!) for the 2025 season:
        https://github.com/Studica-Robotics/NavX/issues/5
        https://github.com/Studica-Robotics/NavX/issues/6
        """
        return wpimath.units.degreesToRadians(self.gyro.getRate())

    def setGoal(self, goal: Rotation2d):
        self.goal = goal

    def setMode(self, state: SwerveHeadingMode):
        self.mode = state

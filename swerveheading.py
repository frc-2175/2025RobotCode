import math
from enum import Enum
from typing import Callable

import wpilib
import wpimath.controller
import wpimath.units
from wpimath.geometry import Rotation2d

import constants
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


class SwerveHeadingController:
    def __init__(self, getHeading: Callable[[], Rotation2d], getRate: Callable[[], float], mode: SwerveHeadingMode) -> None:
        self.getHeading = getHeading
        self.getRate = getRate
        self.mode: SwerveHeadingMode = mode
        self.goal: Rotation2d = self.getHeading()
        self.PID = wpimath.controller.PIDController(
            constants.kHeadingControllerP,
            constants.kHeadingControllerI,
            constants.kHeadingControllerD,
        )
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
        self.gyroRateTopic.set(self.getRate())

        self.PID.setSetpoint(self.goal.radians())

        if self.mode == SwerveHeadingMode.DISABLED:
            output = rot
        elif self.mode == SwerveHeadingMode.HUMAN_DRIVERS:
            # TODO: Validate if 15 deg/s is a reasonable threshold for disabling the controller.
            bot_turning = abs(rot) > 0.1 or abs(self.getRate()) > wpimath.units.degreesToRadians(15)
            bot_translating = speed > 0
            should_maintain_heading = not bot_turning and bot_translating

            self.turningTopic.set(bot_turning)
            self.translatingTopic.set(bot_translating)
            self.maintainingTopic.set(should_maintain_heading)

            if should_maintain_heading:
                # Run PID with the previously-set goal, ignoring `rot`
                output = self.PID.calculate(self.getHeading().radians())
            else:
                # Use `rot`, and update the goal to the current gyro angle to maintain later.
                self.goal = self.getHeading()
                output = rot
        else:
            ntutil.logAlert(self.badModeAlert, self.mode)
            output = rot

        self.outputTopic.set(output)
        return output

    def setGoal(self, goal: Rotation2d):
        self.goal = goal

    def setMode(self, state: SwerveHeadingMode):
        self.mode = state

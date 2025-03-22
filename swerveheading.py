from enum import Enum
import math
import navx
import wpimath.controller
from wpimath.geometry import Rotation2d
import ntutil

class SwerveHeadingState(Enum):
    OFF = 0
    MAINTAIN = 1
    DISABLE = 2

class SwerveHeadingController:
    state: SwerveHeadingState
    goal: Rotation2d

    def __init__(self, gyro: navx.AHRS) -> None:
        self.gyro = gyro
        self.state = SwerveHeadingState.OFF
        self.shouldMaintain = False
        self.goal = self.gyro.getRotation2d()
        self.PID = wpimath.controller.PIDController(1 / math.radians(15), 0, 0)
        self.PID.enableContinuousInput(-math.pi, math.pi)

        self.turningTopic = ntutil.getBooleanTopic("/isTurning")
        self.translatingTOpic = ntutil.getBooleanTopic("/isTranslating")

    def update(self, xSpeed: float, ySpeed: float, rot: float) -> float:
        if self.state == SwerveHeadingState.DISABLE:
            return rot
        
        bot_turning = math.fabs(rot) > 0.1 or math.fabs(self.gyro.getRate()) > 1
        bot_translating = xSpeed != 0 or ySpeed != 0
        shouldChangeToMaintain = not bot_turning and bot_translating

        self.turningTopic.set(bot_turning)
        self.translatingTOpic.set(bot_translating)

        if shouldChangeToMaintain:
            self.state = SwerveHeadingState.MAINTAIN
            self.PID.setSetpoint(self.goal.radians())
        else:
            self.state = SwerveHeadingState.OFF
            self.goal = self.gyro.getRotation2d()
        
        if self.state == SwerveHeadingState.OFF:
            return rot
        else:
            return self.PID.calculate(self.gyro.getRotation2d().radians())
        
    def setGoal(self, goal: Rotation2d): # because navX is clockwise positive, we have to negate it
        self.goal = goal

    def setState(self, state: SwerveHeadingState):
        self.state = state
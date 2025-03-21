import math

import rev
import wpilib
import wpimath.geometry
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState

import constants
import ntutil


class SourceIntake:
    def __init__(self):
        self.intakeSensor = wpilib.AnalogInput(0)

        # Telemetry
        self.intakeSensorTopic = ntutil.getFloatTopic("/Intake/Sensor")

    def periodic(self):
        self.intakeSensorTopic.set(self.intakeSensor.getVoltage())

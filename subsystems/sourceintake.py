import rev
import math
import ntutil
import wpilib
import constants
from wpimath.kinematics import SwerveModuleState
import wpimath.geometry
from wpimath.geometry import Rotation2d

class SourceIntake:
    def __init__(self):
        self.intakeSensor = wpilib.AnalogInput(0)

        # Telemetry
        self.intakeSensorTopic = ntutil.getFloatTopic("/Intake/Sensor")

    def periodic(self):
        self.intakeSensorTopic.set(self.intakeSensor.getVoltage())
        pass

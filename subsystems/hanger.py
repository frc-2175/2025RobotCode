import math

import ntcore
import rev
import wpilib
import wpimath
from wpimath.filter import SlewRateLimiter
import constants
import ntutil
import utils


class Hanger:
    def __init__(self):
        # Hardware
        self.hangerMotor = rev.SparkMax(61, rev.SparkLowLevel.MotorType.kBrushless)
        self.hangerController = self.hangerMotor.getClosedLoopController()
        # self.hangerEncoder = self.hangerMotor.getAbsoluteEncoder()
        self.hangerEncoder = self.hangerMotor.getEncoder()

        self.intakeSolenoid = rev.SparkMax(62, rev.SparkLowLevel.MotorType.kBrushed)

        hangerMotorConfig = rev.SparkMaxConfig()
        (
            hangerMotorConfig
                .setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
                .smartCurrentLimit(40)
        )
        (
            hangerMotorConfig
                .closedLoop
                    .pid(0.6, 0, 0)
                    .setFeedbackSensor(rev.ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        )
        (
            hangerMotorConfig
                .encoder
                    .positionConversionFactor(constants.kHangerMotorReduction * math.pi * 2)
        )
        # (
        #     hangerMotorConfig
        #         .absoluteEncoder
        #             .positionConversionFactor(math.pi * 2)
        #             .zeroCentered(True)
        # )

        intakeSolenoidConfig = rev.SparkMaxConfig()

        (
            intakeSolenoidConfig
                .smartCurrentLimit(40)
        )

        self.hangerMotor.configure(hangerMotorConfig, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)
        self.intakeSolenoid.configure(intakeSolenoidConfig, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)

        # Telemetry
        self.setpointTopic = ntutil.getFloatTopic("/Hanger/Setpoint/Actual")
        self.slewedSetpointTopic = ntutil.getFloatTopic("/Hanger/Setpoint/Slewed")
        self.angleTopic = ntutil.getFloatTopic("/Hanger/Angle")

        # Control variables
        self.setpoint = 0
        self.setpointLimiter = SlewRateLimiter(math.pi)

    def periodic(self):
        safeSetpoint = utils.clamp(self.setpoint, constants.kHangerMinAngle, constants.kHangerMaxAngle)
        slewedHangerSetpoint = self.setpointLimiter.calculate(safeSetpoint)
        self.hangerController.setReference(slewedHangerSetpoint, rev.SparkMax.ControlType.kPosition)

        self.setpointTopic.set(self.setpoint)
        self.slewedSetpointTopic.set(slewedHangerSetpoint)
        self.angleTopic.set(self.hangerEncoder.getPosition())

    def set_position(self, setpoint: float):
        """
        Set the target position of the hanger in radians.
        """
        self.setpoint = setpoint

    def trigger_solenoid(self):
        self.intakeSolenoid.set(1)

    def release_solenoid(self):
        self.intakeSolenoid.set(0)
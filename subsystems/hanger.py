import rev
import wpimath
import math
import wpilib
import ntcore
from wpimath.filter import SlewRateLimiter
import ntutil
import utils
import constants

class Hanger:
    # Hardware
    hangerMotor = rev.SparkMax(61, rev.SparkLowLevel.MotorType.kBrushless)
    hangerController = hangerMotor.getClosedLoopController()
    hangerEncoder = hangerMotor.getAbsoluteEncoder()

    hangerMotorConfig = rev.SparkMaxConfig()
    (
        hangerMotorConfig
            .setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
    )
    (
        hangerMotorConfig
            .closedLoop
                .pid(0, 0, 0)
                .setFeedbackSensor(rev.ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
    )
    (
        hangerMotorConfig
            .absoluteEncoder
                .positionConversionFactor(math.pi * 2)
                .zeroCentered(True)
    )
    hangerMotor.configure(hangerMotorConfig, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)

    # Telemetry
    setpointTopic = ntutil.getFloatTopic("/Hanger/Setpoint/Actual")
    slewedSetpointTopic = ntutil.getFloatTopic("/Hanger/Setpoint/Slewed")

    # Control variables
    setpoint = 0
    setpointLimiter = SlewRateLimiter(math.pi)

    def periodic(self):
        safeSetpoint = utils.clamp(self.setpoint, constants.kHangerMinAngle, constants.kHangerMaxAngle)
        slewedHangerSetpoint = self.setpointLimiter.calculate(safeSetpoint)
        self.hangerController.setReference(slewedHangerSetpoint, rev.SparkMax.ControlType.kPosition)

        self.setpointTopic.set(self.setpoint)
        self.slewedSetpointTopic.set(slewedHangerSetpoint)

    def set_position(self, setpoint: float):
        """
        Set the target position of the hanger in radians.
        """
        self.setpoint = setpoint

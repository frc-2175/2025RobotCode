import rev
import wpimath
import math
import wpilib
import ntcore
from wpimath.filter import SlewRateLimiter

nt = ntcore.NetworkTableInstance.getDefault()

setpointTopic = nt.getFloatTopic("/HangerSetpoint").publish()
slewedSetpointTopic = nt.getFloatTopic("/HangerSlewedSetpoint").publish()

class Hanger:
    hangerMotor = rev.SparkMax(61, rev.SparkLowLevel.MotorType.kBrushless)

    hangerController = hangerMotor.getClosedLoopController()

    hangerEncoder = hangerMotor.getAbsoluteEncoder()

    hangerMotorConfig = rev.SparkMaxConfig()
    (
        hangerMotorConfig
            .setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
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

    setpoint = 0
    setpointLimiter = SlewRateLimiter(math.pi)

    def periodic(self):
        slewedHangerSetpoint = self.setpointLimiter.calculate(self.setpoint)
        self.hangerController.setReference(slewedHangerSetpoint, rev.SparkMax.ControlType.kPosition)

        setpointTopic.set(self.setpoint)
        slewedSetpointTopic.set(slewedHangerSetpoint)

    def set_position(self, setpoint: float):
        """
        Set the target position of the hanger in radians. If the position is outside of a safe range, it will be clamped.
        """
        self.setpoint = setpoint

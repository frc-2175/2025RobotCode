import rev
import wpimath
import math
import wpilib
import ntcore

class Hanger:
    hangerMotor = rev.SparkMax(61, rev.SparkLowLevel.MotorType.kBrushless)

    hangerController = hangerMotor.getClosedLoopController()

    hangerEncoder = hangerMotor.getAbsoluteEncoder()

    hangerMotorConfig = rev.SparkMaxConfig()
    (
        hangerMotorConfig
            .setIdleMode(rev.SparkMax.IdleMode.kBrake)
            .closedLoop
                .pid(0, 0, 0)
                .setFeedbackSensor(rev.ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
    )
    (
        hangerMotorConfig
            .absoluteEncoder
                .positionConversionFactor(math.pi * 2)
    )

    hangerMotor.configure(hangerMotorConfig, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)

    def periodic(self):
        pass

    def set_position(self, setpoint: float):
        """
        Set the target position of the hanger in radians. If the position is outside of a safe range, it will be clamped.
        """
        self.hangerController.setReference(setpoint)
        
        pass

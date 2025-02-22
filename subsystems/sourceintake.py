import rev
import math
import constants
from wpimath.kinematics import SwerveModuleState
import wpimath.geometry
from wpimath.geometry import Rotation2d

primaryWheelMotorConfig = rev.SparkMaxConfig()
primaryWheelMotorConfig.setIdleMode(rev.SparkMax.IdleMode.kBrake)
secondaryWheelMotorConfig = rev.SparkMaxConfig()
secondaryWheelMotorConfig.setIdleMode(rev.SparkMax.IdleMode.kBrake)
secondaryWheelMotorConfig.follow(51, invert=True)

class SourceIntake:
    primaryWheelMotor = rev.SparkMax(51, rev.SparkLowLevel.MotorType.kBrushless)
    secondaryWheelMotor = rev.SparkMax(52, rev.SparkLowLevel.MotorType.kBrushless)
    primaryWheelMotorConfig.smartCurrentLimit(20)
    secondaryWheelMotorConfig.smartCurrentLimit(20)
    secondaryWheelMotor.configure(secondaryWheelMotorConfig, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)
    primaryWheelMotor.configure(primaryWheelMotorConfig, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)

    def periodic(self):
        # TODO: Report encoder positions (and anything else) to NetworkTables
        pass

    def run_intake(self, speed: float):
        """
        Positive means moving coral from the funnel through to the arm. Negative is the reverse.
        Valid speeds range from -1 to 1.
        """
        self.primaryWheelMotor.set(speed * constants.kSourceIntakeSpeed)
        pass

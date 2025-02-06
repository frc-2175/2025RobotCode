import rev
import math
import constants
from wpimath.kinematics import SwerveModuleState
import wpimath.geometry

driveMotorConfig = rev.SparkMaxConfig()
driveMotorConfig.smartCurrentLimit(40)
driveMotorConfig.encoder.velocityConversionFactor(
    (math.pi *constants.kWheelDiameter / constants.kDriveMotorReduction) / 60.0
).positionConversionFactor(
    math.pi * constants.kWheelDiameter / constants.kDriveMotorReduction 
)
driveMotorConfig.closedLoop.setFeedbackSensor(rev.ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder).pidf(0.1,0,0,1 / constants.kMaxSpeed)

steerMotorConfig = rev.SparkMaxConfig()
steerMotorConfig.absoluteEncoder.positionConversionFactor(2 * math.pi).inverted(True)
steerMotorConfig.closedLoop.setFeedbackSensor(
    rev.ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder
).pid(
    1, 0.0, 0.0
).positionWrappingEnabled(True).positionWrappingInputRange(-math.pi,math.pi)

class SwerveModule:
    driveMotor: rev.SparkMax
    steerMotor: rev.SparkMax
    angleOffset: float

    driveEncoder: rev.SparkRelativeEncoder
    steerEncoder: rev.SparkAbsoluteEncoder

    drivePidController: rev.SparkClosedLoopController
    steerPidController: rev.SparkClosedLoopController

    def __init__(self, driveMotorId: int, steerMotorId: int, angleOffset: float):
        self.driveMotor = rev.SparkMax(driveMotorId, rev.SparkLowLevel.MotorType.kBrushless)
        self.steerMotor = rev.SparkMax(steerMotorId, rev.SparkLowLevel.MotorType.kBrushless)
        self.angleOffset = angleOffset

        self.driveMotor.configure(driveMotorConfig, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)
        self.steerMotor.configure(steerMotorConfig, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)
    
        self.driveEncoder = self.driveMotor.getEncoder()
        self.steerEncoder = self.steerMotor.getAbsoluteEncoder()

        self.drivePidController = self.driveMotor.getClosedLoopController()
        self.steerPidController= self.steerMotor.getClosedLoopController()
    
    def setState(self, state: SwerveModuleState):
        self.drivePidController.setReference(state.speed,rev.SparkLowLevel.ControlType.kVelocity)
        self.steerPidController.setReference(state.angle.radians()+self.angleOffset,rev.SparkLowLevel.ControlType.kPosition)

    def getState(self) -> SwerveModuleState:
        return SwerveModuleState(
            self.driveEncoder.getVelocity(),
            wpimath.geometry.Rotation2d(self.steerEncoder.getPosition())
        )

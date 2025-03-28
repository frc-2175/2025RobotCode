import math

import rev
import wpimath.geometry
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState

import constants

driveMotorConfig = rev.SparkMaxConfig()
driveMotorConfig.smartCurrentLimit(40)
driveMotorConfig.encoder.velocityConversionFactor(
    (math.pi *constants.kWheelDiameter / constants.kDriveMotorReduction) / 60.0
).positionConversionFactor(
    math.pi * constants.kWheelDiameter / constants.kDriveMotorReduction 
)
driveMotorConfig.closedLoop.setFeedbackSensor(rev.ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder).pidf(0,0,0,1 / constants.kMaxSpeed)

steerMotorConfig = rev.SparkMaxConfig()
steerMotorConfig.absoluteEncoder.positionConversionFactor(2 * math.pi).inverted(True)
steerMotorConfig.smartCurrentLimit(40)
steerMotorConfig.closedLoop.setFeedbackSensor(
    rev.ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder
).pid(
    1, 0.0, 0.0
).positionWrappingEnabled(True).positionWrappingInputRange(-math.pi,math.pi)

class SwerveModule:
    def __init__(self, driveMotorId: int, steerMotorId: int, angleOffset: float):
        self.driveMotor: rev.SparkMax = rev.SparkMax(driveMotorId, rev.SparkLowLevel.MotorType.kBrushless)
        self.steerMotor: rev.SparkMax = rev.SparkMax(steerMotorId, rev.SparkLowLevel.MotorType.kBrushless)
        self.angleOffset: float = angleOffset

        self.driveMotor.configure(driveMotorConfig, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)
        self.steerMotor.configure(steerMotorConfig, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)
    
        self.driveEncoder: rev.SparkRelativeEncoder = self.driveMotor.getEncoder()
        self.steerEncoder: rev.SparkAbsoluteEncoder = self.steerMotor.getAbsoluteEncoder()

        self.drivePidController: rev.SparkClosedLoopController = self.driveMotor.getClosedLoopController()
        self.steerPidController: rev.SparkClosedLoopController = self.steerMotor.getClosedLoopController()
    
    def setState(self, state: SwerveModuleState):
        state.angle += Rotation2d(self.angleOffset)
        encoderRotation = Rotation2d(self.steerEncoder.getPosition())
        state.optimize(encoderRotation)
        state.cosineScale(encoderRotation)
        self.drivePidController.setReference(state.speed, rev.SparkLowLevel.ControlType.kVelocity)
        self.steerPidController.setReference(state.angle.radians(), rev.SparkLowLevel.ControlType.kPosition)

    def getState(self) -> SwerveModuleState:
        return SwerveModuleState(
            self.driveEncoder.getVelocity(),
            Rotation2d(self.steerEncoder.getPosition()-self.angleOffset)
        )
    
    def getPosition(self) -> SwerveModulePosition:
        return SwerveModulePosition(
            self.driveEncoder.getPosition(),
            Rotation2d(self.steerEncoder.getPosition()-self.angleOffset)
        )

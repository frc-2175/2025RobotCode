# TODO: insert robot code here
from wpimath.geometry import Translation2d
import wpilib
import math
import rev
import wpimath
import wpimath.geometry
from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds, SwerveModuleState
import ntcore
import wpimath.units
import constants

wheelDistanceFromCenter = wpimath.units.inchesToMeters(12.375)
frontLeftLocation = Translation2d(wheelDistanceFromCenter, wheelDistanceFromCenter)
frontRightLocation = Translation2d(wheelDistanceFromCenter, -wheelDistanceFromCenter)
backLeftLocation = Translation2d(-wheelDistanceFromCenter, wheelDistanceFromCenter)
backRightLocation = Translation2d(-wheelDistanceFromCenter, -wheelDistanceFromCenter)

kinematics = SwerveDrive4Kinematics(frontLeftLocation,frontRightLocation,backLeftLocation,backRightLocation)
nt= ntcore.NetworkTableInstance.getDefault()
desiredSwerveStatesTopic=nt.getStructArrayTopic("/DesiredSwerveStates", SwerveModuleState).publish()
actualSwerveStatesTopic=nt.getStructArrayTopic("/ActualSwerveStates", SwerveModuleState).publish()

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


class MyRobot(wpilib.TimedRobot):
    leftStick = wpilib.Joystick(0)
    rightStick = wpilib.Joystick(1)
    gamePad = wpilib.XboxController(2)

    frontLeftDriveMotor = rev.SparkMax(25, rev.SparkLowLevel.MotorType.kBrushless)
    frontLeftSteerMotor = rev.SparkMax(21, rev.SparkLowLevel.MotorType.kBrushless)
    frontRightDriveMotor = rev.SparkMax(28, rev.SparkLowLevel.MotorType.kBrushless)
    frontRightSteerMotor = rev.SparkMax(22, rev.SparkLowLevel.MotorType.kBrushless)
    backLeftDriveMotor = rev.SparkMax(26, rev.SparkLowLevel.MotorType.kBrushless)
    backLeftSteerMotor = rev.SparkMax(24, rev.SparkLowLevel.MotorType.kBrushless)
    backRightDriveMotor = rev.SparkMax(27, rev.SparkLowLevel.MotorType.kBrushless)
    backRightSteerMotor = rev.SparkMax(23, rev.SparkLowLevel.MotorType.kBrushless)

    frontLeftDriveMotor.configure(driveMotorConfig, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)
    frontRightDriveMotor.configure(driveMotorConfig, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)
    backLeftDriveMotor.configure(driveMotorConfig, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)
    backRightDriveMotor.configure(driveMotorConfig, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)

    frontLeftSteerMotor.configure(steerMotorConfig, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)
    frontRightSteerMotor.configure(steerMotorConfig, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)
    backLeftSteerMotor.configure(steerMotorConfig, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)
    backRightSteerMotor.configure(steerMotorConfig, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)

    frontLeftDriveEncoder = frontLeftDriveMotor.getEncoder()
    frontLeftSteerEncoder = frontLeftSteerMotor.getAbsoluteEncoder()
    frontRightDriveEncoder = frontRightDriveMotor.getEncoder()
    frontRightSteerEncoder = frontRightSteerMotor.getAbsoluteEncoder()
    backLeftDriveEncoder = backLeftDriveMotor.getEncoder()
    backLeftSteerEncoder = backLeftSteerMotor.getAbsoluteEncoder()
    backRightDriveEncoder = backRightDriveMotor.getEncoder()
    backRightSteerEncoder = backRightSteerMotor.getAbsoluteEncoder()

    frontLeftDrivePidController= frontLeftDriveMotor.getClosedLoopController()
    frontRightDrivePidController= frontRightDriveMotor.getClosedLoopController()
    backLeftDrivePidController= backLeftDriveMotor.getClosedLoopController()
    backRightDrivePidController= backRightDriveMotor.getClosedLoopController()   
    

    
    frontLeftSteerPidController= frontLeftSteerMotor.getClosedLoopController()
    frontRightSteerPidController= frontRightSteerMotor.getClosedLoopController()
    backLeftSteerPidController= backLeftSteerMotor.getClosedLoopController()
    backRightSteerPidController= backRightSteerMotor.getClosedLoopController()

    # TODO: Rev seems to have removed this method in 2025.
    # frontLeftSteerEncoder.setInverted(True)
    # frontRightSteerEncoder.setInverted(True)
    # backLeftSteerEncoder.setInverted(True)
    # backRightSteerEncoder.setInverted(True)

    def __setattr__(self, name, value):
        self.frontLeftSteerEncoder.getVelocity()
    
    def robotPeriodic(self):
        actualFrontLeft = SwerveModuleState(
            self.frontLeftDriveEncoder.getVelocity(),
            wpimath.geometry.Rotation2d(self.frontLeftSteerEncoder.getPosition())
        )
        actualFrontRight = SwerveModuleState(
            self.frontRightDriveEncoder.getVelocity(),
            wpimath.geometry.Rotation2d(self.frontRightSteerEncoder.getPosition())
        )
        actualBackLeft = SwerveModuleState(
            self.backLeftDriveEncoder.getVelocity(),
            wpimath.geometry.Rotation2d(self.backLeftSteerEncoder.getPosition())
        )
        actualBackRight = SwerveModuleState(
            self.backRightDriveEncoder.getVelocity(),
            wpimath.geometry.Rotation2d(self.backRightSteerEncoder.getPosition())
        )
        actualSwerveStatesTopic.set([actualFrontLeft,actualFrontRight,actualBackLeft,actualBackRight])


    def teleopPeriodic(self) -> None:
        xSpeed = wpimath.applyDeadband(-self.leftStick.getY(), 0.2)
        ySpeed = wpimath.applyDeadband(-self.leftStick.getX(), 0.2)
        turnSpeed = wpimath.applyDeadband(-self.rightStick.getX(), 0.2)
        speeds= ChassisSpeeds(xSpeed, ySpeed, turnSpeed)
        frontLeft, frontRight,backLeft,backRight= kinematics.toSwerveModuleStates(speeds)
        desiredSwerveStatesTopic.set([frontLeft,frontRight,backLeft,backRight])
        self.frontLeftDrivePidController.setReference(frontLeft.speed,rev.SparkLowLevel.ControlType.kVelocity)
        self.frontRightDrivePidController.setReference(frontRight.speed,rev.SparkLowLevel.ControlType.kVelocity)
        self.backLeftDrivePidController.setReference(backLeft.speed,rev.SparkLowLevel.ControlType.kVelocity)
        self.backRightDrivePidController.setReference(backRight.speed,rev.SparkLowLevel.ControlType.kVelocity)
        self.frontRightSteerPidController.setReference(frontRight.angle.radians(),rev.SparkLowLevel.ControlType.kPosition)
        self.frontLeftSteerPidController.setReference(frontLeft.angle.radians()+(3*math.pi/2),rev.SparkLowLevel.ControlType.kPosition)
        self.backLeftSteerPidController.setReference(backLeft.angle.radians()+(math.pi),rev.SparkLowLevel.ControlType.kPosition)
        self.backRightSteerPidController.setReference(backRight.angle.radians()+(math.pi/2),rev.SparkLowLevel.ControlType.kPosition)
        pass


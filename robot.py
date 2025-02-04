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
driveMotorConfig.encoder.velocityConversionFactor(
    (math.pi *constants.kWheelDiameter / constants.kDriveMotorReduction) / 60.0
).positionConversionFactor(
    math.pi * constants.kWheelDiameter / constants.kDriveMotorReduction 
)

steerMotorConfig = rev.SparkMaxConfig()
steerMotorConfig.absoluteEncoder.positionConversionFactor(2 * math.pi).inverted(True)

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
        speeds= ChassisSpeeds(-self.leftStick.getY(), -self.leftStick.getX(), -self.rightStick.getX())
        frontLeft, frontRight,backLeft,backRight= kinematics.toSwerveModuleStates(speeds)
        desiredSwerveStatesTopic.set([frontLeft,frontRight,backLeft,backRight])
        pass


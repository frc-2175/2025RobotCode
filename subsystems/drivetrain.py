import navx
import rev
import constants
import math
import wpimath
from wpimath.geometry import Translation2d, Rotation2d
from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds, SwerveModuleState
import ntcore
from swervemodule import SwerveModule

wheelDistanceFromCenter = wpimath.units.inchesToMeters(12.375)
frontLeftLocation = Translation2d(wheelDistanceFromCenter, wheelDistanceFromCenter)
frontRightLocation = Translation2d(wheelDistanceFromCenter, -wheelDistanceFromCenter)
backLeftLocation = Translation2d(-wheelDistanceFromCenter, wheelDistanceFromCenter)
backRightLocation = Translation2d(-wheelDistanceFromCenter, -wheelDistanceFromCenter)

kinematics = SwerveDrive4Kinematics(frontLeftLocation,frontRightLocation,backLeftLocation,backRightLocation)
nt= ntcore.NetworkTableInstance.getDefault()
desiredSwerveStatesTopic=nt.getStructArrayTopic("/DesiredSwerveStates", SwerveModuleState).publish()
actualSwerveStatesTopic=nt.getStructArrayTopic("/ActualSwerveStates", SwerveModuleState).publish()

gyroTopic = nt.getStructTopic("/Gyro", Rotation2d).publish()

class Drivetrain:
    frontLeftSwerveModule = SwerveModule(25, 21, 3*math.pi/2)
    frontRightSwerveModule = SwerveModule(28, 22, 0)
    backLeftSwerveModule = SwerveModule(26, 24, math.pi)
    backRightSwerveModule = SwerveModule(27, 23, math.pi/2)

    gyro = navx.AHRS.create_spi()

    def periodic(self):
        actualFrontLeft = self.frontLeftSwerveModule.getState()
        actualFrontRight = self.frontRightSwerveModule.getState()
        actualBackLeft = self.backLeftSwerveModule.getState()
        actualBackRight = self.backRightSwerveModule.getState()
        actualSwerveStatesTopic.set([actualFrontLeft,actualFrontRight,actualBackLeft,actualBackRight])

        gyroTopic.set(self.gyro.getRotation2d())

    def drive(self, xSpeed: float, ySpeed: float, turnSpeed: float):
        speeds= ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed, self.gyro.getRotation2d())
        frontLeft, frontRight,backLeft,backRight= kinematics.toSwerveModuleStates(speeds)
        desiredSwerveStatesTopic.set([frontLeft,frontRight,backLeft,backRight])

        self.frontLeftSwerveModule.setState(frontLeft)
        self.frontRightSwerveModule.setState(frontRight)
        self.backLeftSwerveModule.setState(backLeft)
        self.backRightSwerveModule.setState(backRight)

    def reset_heading(self):
        self.gyro.reset()

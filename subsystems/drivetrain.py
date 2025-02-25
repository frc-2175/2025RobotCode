import navx
import rev
import constants
import math
import wpimath
from wpimath.geometry import Translation2d, Rotation2d
from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds, SwerveModuleState
from wpimath.filter import SlewRateLimiter
import ntcore
from swervemodule import SwerveModule
from utils import RotationSlewRateLimiter

wheelDistanceFromCenter = wpimath.units.inchesToMeters(12.375)
frontLeftLocation = Translation2d(wheelDistanceFromCenter, wheelDistanceFromCenter)
frontRightLocation = Translation2d(wheelDistanceFromCenter, -wheelDistanceFromCenter)
backLeftLocation = Translation2d(-wheelDistanceFromCenter, wheelDistanceFromCenter)
backRightLocation = Translation2d(-wheelDistanceFromCenter, -wheelDistanceFromCenter)

kinematics = SwerveDrive4Kinematics(frontLeftLocation,frontRightLocation,backLeftLocation,backRightLocation)
nt= ntcore.NetworkTableInstance.getDefault()
desiredSwerveStatesTopic=nt.getStructArrayTopic("/DesiredSwerveStates", SwerveModuleState).publish()
actualSwerveStatesTopic=nt.getStructArrayTopic("/ActualSwerveStates", SwerveModuleState).publish()
desiredChassisSpeedsTopic=nt.getStructTopic("/DesiredChassisSpeeds", ChassisSpeeds).publish()
currentChassisSpeedsTopic=nt.getStructTopic("/CurrentChassisSpeeds", ChassisSpeeds).publish()

gyroTopic = nt.getStructTopic("/Gyro", Rotation2d).publish()

class Drivetrain:
    frontLeftSwerveModule = SwerveModule(25, 21, 3*math.pi/2)
    frontRightSwerveModule = SwerveModule(28, 22, 0)
    backLeftSwerveModule = SwerveModule(26, 24, math.pi)
    backRightSwerveModule = SwerveModule(27, 23, math.pi/2)

    gyro = navx.AHRS.create_spi()

    speedLimiter = SlewRateLimiter(1) #m/s
    rotationLimiter = SlewRateLimiter(math.pi) #rad/s
    directionLimiter = RotationSlewRateLimiter(math.pi) #rad/s

    desiredChassisSpeeds = ChassisSpeeds(0, 0, 0)
    currentChassisSpeeds = ChassisSpeeds(0, 0, 0)
    
    def periodic(self):
        desiredSpeed = math.sqrt(self.desiredChassisSpeeds.vx**2 + self.desiredChassisSpeeds.vy**2)
        newSpeed = self.speedLimiter.calculate(desiredSpeed)

        desiredDirection = math.atan2(self.desiredChassisSpeeds.vy, self.desiredChassisSpeeds.vx)
        newDirection = self.directionLimiter.calculate(desiredDirection)
        newTurnSpeed = self.rotationLimiter.calculate(self.desiredChassisSpeeds.omega)

        newVX = newSpeed*math.cos(newDirection)
        newVY = newSpeed*math.sin(newDirection)
        self.currentChassisSpeeds = ChassisSpeeds(newVX, newVY, newTurnSpeed)

        frontLeft, frontRight,backLeft,backRight= kinematics.toSwerveModuleStates(self.currentChassisSpeeds)
        desiredSwerveStatesTopic.set([frontLeft,frontRight,backLeft,backRight])

        desiredChassisSpeedsTopic.set(self.desiredChassisSpeeds)
        currentChassisSpeedsTopic.set(self.currentChassisSpeeds)

        self.frontLeftSwerveModule.setState(frontLeft)
        self.frontRightSwerveModule.setState(frontRight)
        self.backLeftSwerveModule.setState(backLeft)
        self.backRightSwerveModule.setState(backRight)

        actualFrontLeft = self.frontLeftSwerveModule.getState()
        actualFrontRight = self.frontRightSwerveModule.getState()
        actualBackLeft = self.backLeftSwerveModule.getState()
        actualBackRight = self.backRightSwerveModule.getState()
        actualSwerveStatesTopic.set([actualFrontLeft,actualFrontRight,actualBackLeft,actualBackRight])

        gyroTopic.set(self.gyro.getRotation2d())
        

    def drive(self, xSpeed: float, ySpeed: float, turnSpeed: float):
        self.desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed, self.gyro.getRotation2d())

    def reset_heading(self):
        self.gyro.reset()
import navx
import rev
import constants
import math
import wpimath
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds, SwerveModuleState, SwerveDrive4Odometry
from wpimath.filter import SlewRateLimiter
import ntcore
from swervemodule import SwerveModule
import utils
from utils import RotationSlewRateLimiter
from chassisspeeds import ChassisSpeeds2175
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import RobotConfig, PIDConstants
from wpilib import DriverStation


wheelDistanceFromCenter = constants.kWheelDistanceFromCenter
frontLeftLocation = Translation2d(wheelDistanceFromCenter, wheelDistanceFromCenter)
frontRightLocation = Translation2d(wheelDistanceFromCenter, -wheelDistanceFromCenter)
backLeftLocation = Translation2d(-wheelDistanceFromCenter, wheelDistanceFromCenter)
backRightLocation = Translation2d(-wheelDistanceFromCenter, -wheelDistanceFromCenter)

kinematics = SwerveDrive4Kinematics(frontLeftLocation,frontRightLocation,backLeftLocation,backRightLocation)

nt = ntcore.NetworkTableInstance.getDefault()
desiredSwerveStatesTopic = nt.getStructArrayTopic("/DesiredSwerveStates", SwerveModuleState).publish()
actualSwerveStatesTopic = nt.getStructArrayTopic("/ActualSwerveStates", SwerveModuleState).publish()
desiredChassisSpeedsTopic = nt.getStructTopic("/DesiredChassisSpeeds", ChassisSpeeds).publish()
currentChassisSpeedsTopic = nt.getStructTopic("/CurrentChassisSpeeds", ChassisSpeeds).publish()
robotPoseTopic = nt.getStructTopic("/RobotPose", Pose2d).publish()

gyroTopic = nt.getStructTopic("/Gyro", Rotation2d).publish()

class Drivetrain:
    frontLeftSwerveModule = SwerveModule(25, 21, 3 * math.pi/2)
    frontRightSwerveModule = SwerveModule(28, 22, 0)
    backLeftSwerveModule = SwerveModule(26, 24, math.pi)
    backRightSwerveModule = SwerveModule(27, 23, math.pi/2)

    gyro = navx.AHRS.create_spi()

    speedLimiter = SlewRateLimiter(constants.kSpeedSlewRate) #m/s
    rotationLimiter = SlewRateLimiter(constants.kRotationSlewRate) #rad/s
    directionLimiter = RotationSlewRateLimiter(constants.kDirectionSlewRate) #rad/s

    desiredChassisSpeeds = ChassisSpeeds2175(0, 0, 0)
    currentChassisSpeeds = ChassisSpeeds2175(0, 0, 0)

    odometry = SwerveDrive4Odometry(
        kinematics,
        gyro.getRotation2d(),
        (
            frontLeftSwerveModule.getPosition(),
            frontRightSwerveModule.getPosition(),
            backLeftSwerveModule.getPosition(),
            backRightSwerveModule.getPosition()
        ),
        # Initial pose of the robot, TODO, set from vision
        Pose2d(0, 0, gyro.getRotation2d())
    )

    def __init__(self):
        pass
        # Add PathPlanner or Choreo init here
    
    def periodic(self):
        currentDirection = self.currentChassisSpeeds.direction
        currentSpeed = self.currentChassisSpeeds.speed

        desiredDirection = self.desiredChassisSpeeds.direction
        desiredSpeed = self.desiredChassisSpeeds.speed

        newDirection = None
        newSpeed = None

        if currentSpeed == 0:
            newDirectionSlewRate = 500 # arbitarily huge number; effectively instantaneous
        else:
            percentOfMaxSpeed = currentSpeed / constants.kMaxSpeed
            newDirectionSlewRate = constants.kDirectionSlewRate / percentOfMaxSpeed
        self.directionLimiter.setRateLimit(newDirectionSlewRate)

        angleDiff = utils.angleDifference(desiredDirection, currentDirection)
        if angleDiff < 0.45 * math.pi:
            # Angle is close to the correct angle.
            # Do normal slew limiting for both direction and speed.
            newDirection = self.directionLimiter.calculate(desiredDirection)
            newSpeed = self.speedLimiter.calculate(desiredSpeed)
        elif angleDiff > 0.85 * math.pi:
            # Angle is close to 180 degrees off.
            # Decelerate and then change direction.
            if currentSpeed > 1e-4:
                # Moving at speed in the wrong direction.
                # Keep current direction and decelerate.
                newDirection = currentDirection
                newSpeed = self.speedLimiter.calculate(0)
            else:
                # Stopped. Switch to new direction.
                newDirection = self.directionLimiter.calculate(desiredDirection, force=True)
                newSpeed = self.speedLimiter.calculate(desiredSpeed)
        else:
            # Angle is very wrong. Decelerate and steer wheels to correct direction.
            newDirection = self.directionLimiter.calculate(desiredDirection)
            newSpeed = self.speedLimiter.calculate(0)

        newTurnSpeed = self.rotationLimiter.calculate(self.desiredChassisSpeeds.omega)

        self.currentChassisSpeeds = ChassisSpeeds2175(newDirection, newSpeed, newTurnSpeed)

        frontLeft, frontRight, backLeft, backRight = kinematics.toSwerveModuleStates(self.currentChassisSpeeds.toWPILibChassisSpeeds())
        desiredSwerveStatesTopic.set([frontLeft,frontRight,backLeft,backRight])

        desiredChassisSpeedsTopic.set(self.desiredChassisSpeeds.toWPILibChassisSpeeds())
        currentChassisSpeedsTopic.set(self.currentChassisSpeeds.toWPILibChassisSpeeds())

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

        self.odometry.update(
            self.gyro.getRotation2d(),
            (
                self.frontLeftSwerveModule.getPosition(),
                self.frontRightSwerveModule.getPosition(),
                self.backLeftSwerveModule.getPosition(),
                self.backRightSwerveModule.getPosition()
            )
        )

        robotPoseTopic.set(self.odometry.getPose())

    def reset_pose(self):
        pass

    def get_pose(self) -> Pose2d:
        return self.odometry.getPose()

    def get_heading(self) -> Rotation2d:
        return self.gyro.getRotation2d()
    
    def get_wpi_chassis_speeds(self) -> ChassisSpeeds:
        return self.currentChassisSpeeds.toWPILibChassisSpeeds()

    def drive(self, xSpeed: float, ySpeed: float, turnSpeed: float):
        newSpeeds = ChassisSpeeds2175.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed, self.gyro.getRotation2d())
        # The speeds will actually be zero because these are the products of the deadbanded joystick values
        if xSpeed == 0 and ySpeed == 0:
            # Preserve the old direction instead of going to angle 0.
            newSpeeds.direction = self.currentChassisSpeeds.direction
        if newSpeeds.speed > constants.kMaxSpeed: 
            newSpeeds.speed = constants.kMaxSpeed
        self.desiredChassisSpeeds = newSpeeds

    def reset_heading(self):
        self.gyro.reset()

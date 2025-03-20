import navx
import constants
import math
from wpimath.geometry import Translation2d, Rotation2d, Pose2d, Pose3d
from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds, SwerveModuleState, SwerveDrive4Odometry
from wpimath.filter import SlewRateLimiter
from wpimath.controller import PIDController
from swervemodule import SwerveModule
import utils
from utils import RotationSlewRateLimiter
from chassisspeeds import ChassisSpeeds2175
from wpilib import DriverStation
import ntutil
import choreo
from photonlibpy import PhotonCamera, PhotonPoseEstimator, PoseStrategy
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField

class Drivetrain:
    def __init__(self):
        # Hardware
        self.frontLeftSwerveModule = SwerveModule(25, 21, 3 * math.pi/2)
        self.frontRightSwerveModule = SwerveModule(28, 22, 0)
        self.backLeftSwerveModule = SwerveModule(26, 24, math.pi)
        self.backRightSwerveModule = SwerveModule(27, 23, math.pi/2)

        self.gyro = navx.AHRS.create_spi()

        self.desiredChassisSpeeds = ChassisSpeeds2175(0, 0, 0)
        self.currentChassisSpeeds = ChassisSpeeds2175(0, 0, 0)

        # Kinematics and odometry
        frontLeftLocation = Translation2d(constants.kWheelDistanceFromCenter, constants.kWheelDistanceFromCenter)
        frontRightLocation = Translation2d(constants.kWheelDistanceFromCenter, -constants.kWheelDistanceFromCenter)
        backLeftLocation = Translation2d(-constants.kWheelDistanceFromCenter, constants.kWheelDistanceFromCenter)
        backRightLocation = Translation2d(-constants.kWheelDistanceFromCenter, -constants.kWheelDistanceFromCenter)
        self.kinematics = SwerveDrive4Kinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation)

        self.odometry = SwerveDrive4Odometry(
            self.kinematics,
            self.gyro.getRotation2d(),
            (
                self.frontLeftSwerveModule.getPosition(),
                self.frontRightSwerveModule.getPosition(),
                self.backLeftSwerveModule.getPosition(),
                self.backRightSwerveModule.getPosition(),
            ),
            # Initial pose of the robot, TODO, set from vision
            Pose2d(0, 0, self.gyro.getRotation2d())
        )

        # Telemetry
        self.desiredSwerveStatesTopic = ntutil.getStructArrayTopic("/SwerveStates/Desired", SwerveModuleState)
        self.actualSwerveStatesTopic = ntutil.getStructArrayTopic("/SwerveStates/Actual", SwerveModuleState)
        self.desiredChassisSpeedsTopic = ntutil.getStructTopic("/ChassisSpeeds/Desired", ChassisSpeeds)
        self.currentChassisSpeedsTopic = ntutil.getStructTopic("/ChassisSpeeds/Current", ChassisSpeeds)
        self.gyroTopic = ntutil.getStructTopic("/Gyro", Rotation2d)
        self.robotPoseTopic = ntutil.getStructTopic("/RobotPose", Pose2d)
        self.visionPoseTopic = ntutil.getStructTopic("/VisionPose", Pose3d)

        # Control variables
        self.speedLimiter = SlewRateLimiter(constants.kSpeedSlewRate) #m/s
        self.rotationLimiter = SlewRateLimiter(constants.kRotationSlewRate) #rad/s
        self.directionLimiter = RotationSlewRateLimiter(constants.kDirectionSlewRate) #rad/s

        # Choreo PID controllers
        self.x_controller = PIDController(0, 0, 0)
        self.y_controller = PIDController(0, 0, 0) 
        self.heading_controller = PIDController(0, 0, 0)

        self.heading_controller.enableContinuousInput(-math.pi, math.pi)

        # PhotonVision
        # https://docs.photonvision.org/en/latest/docs/programming/photonlib/robot-pose-estimator.html#apriltags-and-photonposeestimator
        # self.camera = PhotonCamera("vision_camera")

        # self.cameraPoseEst = PhotonPoseEstimator(
        #     AprilTagFieldLayout.loadField(AprilTagField.k2025ReefscapeWelded),
        #     PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        #     self.camera,
        #     constants.kRobotToCam
        # )
        

    def periodic(self):
        currentDirection = self.currentChassisSpeeds.direction
        currentSpeed = self.currentChassisSpeeds.speed

        desiredDirection = self.desiredChassisSpeeds.direction
        desiredSpeed = self.desiredChassisSpeeds.speed

        newDirection = None
        newSpeed = None
        newTurnSpeed = self.rotationLimiter.calculate(self.desiredChassisSpeeds.omega)

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
        self.currentChassisSpeeds = ChassisSpeeds2175(newDirection, newSpeed, newTurnSpeed)
        self.desiredChassisSpeedsTopic.set(self.desiredChassisSpeeds.toWPILibChassisSpeeds())
        self.currentChassisSpeedsTopic.set(self.currentChassisSpeeds.toWPILibChassisSpeeds())

        frontLeft, frontRight, backLeft, backRight = self.kinematics.toSwerveModuleStates(self.currentChassisSpeeds.toWPILibChassisSpeeds())
        self.desiredSwerveStatesTopic.set([frontLeft, frontRight, backLeft, backRight])

        self.frontLeftSwerveModule.setState(frontLeft)
        self.frontRightSwerveModule.setState(frontRight)
        self.backLeftSwerveModule.setState(backLeft)
        self.backRightSwerveModule.setState(backRight)

        actualFrontLeft = self.frontLeftSwerveModule.getState()
        actualFrontRight = self.frontRightSwerveModule.getState()
        actualBackLeft = self.backLeftSwerveModule.getState()
        actualBackRight = self.backRightSwerveModule.getState()
        self.actualSwerveStatesTopic.set([actualFrontLeft, actualFrontRight, actualBackLeft, actualBackRight])

        self.gyroTopic.set(self.gyro.getRotation2d())

        self.odometry.update(
            self.gyro.getRotation2d(),
            (
                self.frontLeftSwerveModule.getPosition(),
                self.frontRightSwerveModule.getPosition(),
                self.backLeftSwerveModule.getPosition(),
                self.backRightSwerveModule.getPosition()
            )
        )

        self.robotPoseTopic.set(self.odometry.getPose())
        try:
            self.visionPoseTopic.set(self.cameraPoseEst.update().estimatedPose)
        except:
            pass

    def reset_pose(self, pose: Pose2d):
        self.odometry.resetPose(pose)
        pass

    def get_pose(self) -> Pose2d:
        return self.odometry.getPose()

    def get_heading(self) -> Rotation2d:
        return self.gyro.getRotation2d()

    def drive(self, xSpeed: float, ySpeed: float, turnSpeed: float):
        newSpeeds = ChassisSpeeds2175.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed, self.gyro.getRotation2d())

        # If stopping the robot, preserve the old direction instead of going to
        # angle 0. The speeds will indeed be exactly equal to zero in teleop
        # because xSpeed and ySpeed are the deadbanded joystick values.
        if xSpeed == 0 and ySpeed == 0:
            newSpeeds.direction = self.currentChassisSpeeds.direction

        # Limit the overall max speed.
        if newSpeeds.speed > constants.kMaxSpeed:
            newSpeeds.speed = constants.kMaxSpeed

        self.desiredChassisSpeeds = newSpeeds

    def follow_choreo_trajectory(self, sample: choreo.SwerveSample):
        pose = self.get_pose()

        # Not currently controlling robot heading
        self.drive(
            sample.vx + self.x_controller.calculate(pose.X(), sample.x),
            sample.vy + self.y_controller.calculate(pose.Y(), sample.y),
            sample.omega, # TODO: Add heading controller - see Getting Started docs for Choreo
        )
        
    def reset_heading(self):
        self.gyro.reset()

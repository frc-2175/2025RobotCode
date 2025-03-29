import math

import choreo.trajectory
import navx
import wpilib
import wpimath.units
from photonlibpy import PhotonCamera, PhotonPoseEstimator, PoseStrategy
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from wpilib import DriverStation
from wpimath.controller import PIDController
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Pose2d, Pose3d, Rotation2d, Rotation3d, Translation2d, Transform3d
from wpimath.kinematics import (ChassisSpeeds, SwerveDrive4Kinematics,
                                SwerveDrive4Odometry, SwerveModuleState)
from wpimath.estimator import SwerveDrive4PoseEstimator

import constants
import ntutil
import utils
from chassisspeeds import ChassisSpeeds2175
from swerveheading import SwerveHeadingController, SwerveHeadingMode
from swervemodule import SwerveModule
from utils import RotationSlewRateLimiter


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

        self.odometry = SwerveDrive4PoseEstimator(
            self.kinematics,
            self.gyro.getRotation2d(),
            (
                (
                self.frontLeftSwerveModule.getPosition(),
                self.frontRightSwerveModule.getPosition(),
                self.backLeftSwerveModule.getPosition(),
                self.backRightSwerveModule.getPosition(),
                )
            ),
            Pose2d(0, 0, self.gyro.getRotation2d())
        )

        # Choreo PID controllers
        self.choreoXController = PIDController(constants.kChoreoTranslationP, constants.kChoreoTranslationI, constants.kChoreoTranslationD)
        self.choreoYController = PIDController(constants.kChoreoTranslationP, constants.kChoreoTranslationI, constants.kChoreoTranslationD)
        self.choreoHeadingController = PIDController(constants.kChoreoRotationP, constants.kChoreoRotationI, constants.kChoreoRotationD)
        self.choreoHeadingController.enableContinuousInput(-math.pi, math.pi)

        # PhotonVision
        # https://docs.photonvision.org/en/latest/docs/programming/photonlib/robot-pose-estimator.html#apriltags-and-photonposeestimator
        self.camera = PhotonCamera("front")

        self.cameraPoseEst = PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagField.k2025ReefscapeWelded),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            self.camera,
            constants.kRobotToCam
        )

        # Telemetry
        self.desiredSwerveStatesTopic = ntutil.getStructArrayTopic("/SwerveStates/Desired", SwerveModuleState)
        self.actualSwerveStatesTopic = ntutil.getStructArrayTopic("/SwerveStates/Actual", SwerveModuleState)
        self.desiredChassisSpeedsTopic = ntutil.getStructTopic("/ChassisSpeeds/Desired", ChassisSpeeds)
        self.currentChassisSpeedsTopic = ntutil.getStructTopic("/ChassisSpeeds/Current", ChassisSpeeds)
        self.gyroTopic = ntutil.getStructTopic("/Gyro", Rotation2d)
        self.robotPoseTopic = ntutil.getStructTopic("/RobotPose", Pose2d)
        self.visionPoseTopic = ntutil.getStructTopic("/VisionPose", Pose3d)
        self.photonTagTransformsTopic = ntutil.getStructArrayTopic("/PhotonTagTransforms", Transform3d)

        self.badChoreoModeAlert = wpilib.Alert("Choreo requires SwerveHeadingMode.DISABLED", wpilib.Alert.AlertType.kError)

        # Control variables
        self.speedLimiter = SlewRateLimiter(constants.kSpeedSlewRate) #m/s
        self.rotationLimiter = SlewRateLimiter(constants.kRotationSlewRate) #rad/s
        self.directionLimiter = RotationSlewRateLimiter(constants.kDirectionSlewRate) #rad/s
        self.headingController = SwerveHeadingController(
            getHeading=self.get_heading,
            getRate=self.get_heading_rate,
            mode=SwerveHeadingMode.HUMAN_DRIVERS,
        )

        self.visionPose = Pose3d()
        

    def periodic(self):
        currentDirection = self.currentChassisSpeeds.direction
        currentSpeed = self.currentChassisSpeeds.speed

        desiredDirection = self.desiredChassisSpeeds.direction
        desiredSpeed = self.desiredChassisSpeeds.speed

        newDirection = None
        newSpeed = None
        newTurnSpeed = self.rotationLimiter.calculate(self.desiredChassisSpeeds.omega)

        # Calculate a new direction/speed for the bot based on the current and desired
        # directions/speeds. This smooths out jittery controls and ensures that we
        # accelerate and decelerate as gracefully as possible.
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

        # Potentially override the turn speed via the heading controller.
        newTurnSpeed = self.headingController.update(newSpeed, newTurnSpeed)

        # Update our current chassis speeds to the new values.
        self.currentChassisSpeeds = ChassisSpeeds2175(newDirection, newSpeed, newTurnSpeed)
        self.desiredChassisSpeedsTopic.set(self.desiredChassisSpeeds.toWPILibChassisSpeeds())
        self.currentChassisSpeedsTopic.set(self.currentChassisSpeeds.toWPILibChassisSpeeds())

        # Do kinematics and update each swerve module's setpoints.
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

        self.robotPoseTopic.set(self.odometry.getEstimatedPosition())

        try:
            self.photonTagTransformsTopic.set(self.get_photon_targets())
            visionUpdate = self.cameraPoseEst.update()
            if visionUpdate:
                self.visionPose = visionUpdate.estimatedPose
                self.visionPoseTopic.set(visionUpdate.estimatedPose)
                self.odometry.addVisionMeasurement(visionUpdate.estimatedPose.toPose2d(), visionUpdate.timestampSeconds, (4, 4, 8))
        except:
            ntutil.log("Failed to retrieve any tags from PhotonVision")

    def reset_pose(self, pose: Pose2d):
        self.odometry.resetPose(pose)
        pass

    def get_pose(self) -> Pose2d:
        return self.odometry.getEstimatedPosition()

    def get_heading(self) -> Rotation2d:
        return self.odometry.getEstimatedPosition().rotation()

    def get_heading_rate(self) -> float:
        """
        The NavX reports its rate in degrees/second instead of radians/second.
        Please use this method instead of `self.gyro.getRate()` for our own
        sanity.

        Also, be aware that `getRate()` has a history of being totally broken:
        https://github.com/kauailabs/navxmxp/issues/69

        But this was probably fixed (probably!) for the 2025 season:
        https://github.com/Studica-Robotics/NavX/issues/5
        https://github.com/Studica-Robotics/NavX/issues/6
        """
        return wpimath.units.degreesToRadians(self.gyro.getRate())

    def drive_common(self, newSpeeds: ChassisSpeeds2175):
        """
        DO NOT CALL THIS DIRECTLY. Call drive_robot_relative or drive_field_relative.
        """

        # If stopping the robot, preserve the old direction instead of going to
        # angle 0. The speeds will indeed be exactly equal to zero in teleop
        # because xSpeed and ySpeed are the deadbanded joystick values.
        if newSpeeds.speed == 0:
            newSpeeds.direction = self.currentChassisSpeeds.direction

        # Limit the overall max speed.
        if newSpeeds.speed > constants.kMaxSpeed:
            newSpeeds.speed = constants.kMaxSpeed

        self.desiredChassisSpeeds = newSpeeds

    def drive_robot_relative(self, xSpeed: float, ySpeed: float, turnSpeed: float):
        self.drive_common(ChassisSpeeds2175.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed, Rotation2d(0)))

    def drive_field_relative(self, xSpeed: float, ySpeed: float, turnSpeed: float):
        """
        Drives the robot in the given direction IN FIELD COORDINATES. Note that because we use the
        "always blue origin" convention, as described in the WPILib docs, this means that
        speeds must be flipped when humans are driving on the red alliance side of the field.

        https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#always-blue-origin
        """
        self.drive_common(ChassisSpeeds2175.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed, self.get_heading()))

    def follow_choreo_trajectory(self, sample: choreo.trajectory.SwerveSample):
        pose = self.get_pose()

        self.drive_field_relative(
            sample.vx + self.choreoXController.calculate(pose.X(), sample.x),
            sample.vy + self.choreoYController.calculate(pose.Y(), sample.y),
            sample.omega + self.choreoHeadingController.calculate(pose.rotation().radians(), sample.heading),
        )

        self.badChoreoModeAlert.set(self.headingController.mode != SwerveHeadingMode.DISABLED)
        self.headingController.setGoal(Rotation2d(sample.heading))
    
    def set_heading_controller_to_teleop(self):
        self.headingController.setMode(SwerveHeadingMode.HUMAN_DRIVERS)

    def set_heading_controller_to_autonomous(self):
        self.headingController.setMode(SwerveHeadingMode.DISABLED)

    def reset_heading(self, angle: float):
        self.odometry.resetRotation(Rotation2d(angle))

    def get_photon_targets(self):
        photon_detections = self.camera.getLatestResult().targets
        target_transforms = []

        for detection in photon_detections:
            target_transforms.append(detection.bestCameraToTarget)
        
        return target_transforms

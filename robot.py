import math
import os
import os.path
from typing import Any, Callable, Dict, Iterable, List

import choreo
import choreo.trajectory
import wpilib
import wpimath
import wpimath.geometry
import wpimath.units
from wpilib import Alert, DriverStation, SmartDashboard
from wpimath.geometry import Pose2d, Pose3d, Rotation2d, Translation2d
from wpimath.kinematics import (ChassisSpeeds, SwerveDrive4Kinematics,
                                SwerveModuleState)
from wpilib.cameraserver import CameraServer

from wpilib.cameraserver import CameraServer

import constants
import ntutil
import utils
from gentools import doneable
from sim.simswerve import SwerveDriveSim
from subsystems.drivetrain import Drivetrain
from subsystems.elevatorandarm import ElevatorAndArm
from subsystems.hanger import Hanger
from swerveheading import SwerveHeadingMode

# from urcl import URCL


class MyRobot(wpilib.TimedRobot):
    def init(self):
        """
        A hacky replacement for __init__ that is called from robotInit to
        satisfy the needs of robotpy tests.
        """

        # Joysticks and input
        self.leftStick = wpilib.Joystick(0)
        self.rightStick = wpilib.Joystick(1)
        self.gamePad = wpilib.XboxController(2)

        # Subsystems
        self.drivetrain = Drivetrain()

        # Simulated subsystems
        self.simDrivetrain = SwerveDriveSim(self.drivetrain)

        # Alerts
        self.badTrajectoryAlert = Alert("Choreo path not found", Alert.AlertType.kError)
        self.noAutoAlert = Alert("No autonomous trajectory", Alert.AlertType.kWarning)
        self.noAutoSampleAlert = Alert("No sample for autonomous trajectory; stopping bot", Alert.AlertType.kWarning)
        self.scoringModeImproperValue = Alert("Variable scoringMode improper value (expected kCoralMode or kAlgaeMode)", Alert.AlertType.kError)
        self.tooFarAwayFromAutoStartAlert = Alert("Robot is not in the right place to begin auto! Double-check which auto you have selected.", Alert.AlertType.kWarning)
        self.wrongWayForAutoAlert = Alert("Robot is not facing the right way to begin auto!", Alert.AlertType.kWarning)

        # Control state
        self.previousAutoTime: float = 0

        # Controls telemetry
        self.scoringModeTopic = ntutil.getStringTopic("/Controls/ScoringMode")

        # Auto telemetry
        self.autoTimerTopic = ntutil.getFloatTopic("/Auto/Timer")
        self.autoTrajectoryTopic = ntutil.getStructArrayTopic("/Auto/Trajectory", Pose2d)
        self.autoChassisSpeedsTopic = ntutil.getStructTopic("/Auto/ChassisSpeeds", ChassisSpeeds)
        self.autoPoseTopic = ntutil.getStructTopic("/Auto/Pose", Pose2d)

        CameraServer().launch()


    def robotInit(self):
        wpilib.DataLogManager.start()
        wpilib.DriverStation.startDataLog(wpilib.DataLogManager.getLog())
        # URCL.start()

        self.init()

        CameraServer().launch()


    def robotPeriodic(self):
        self.drivetrain.periodic()
        if self.isSimulation():
            self.simDrivetrain.simulationPeriodic(0.02)


    def disabledInit(self):
        self.drivetrain.drive_field_relative(0, 0, 0)

    def disabledPeriodic(self):
        pass


    def testInit(self):
        pass

    def testPeriodic(self):
        pass

    def autonomousInit(self) -> None:
        pass

    def autonomousPeriodic(self) -> None:
        pass

    def teleopInit(self) -> None:
        self.drivetrain.set_heading_controller_to_teleop()

    def teleopPeriodic(self) -> None:
        # Get raw speeds from joysticks (to be converted to field/robot relative)
        joystickX = constants.kMaxSpeed * wpimath.applyDeadband(self.leftStick.getX(), 0.1)
        joystickY = constants.kMaxSpeed * wpimath.applyDeadband(self.leftStick.getY(), 0.1)

        # Turn speed is the same regardless of field/robot relative
        turnSpeed = wpimath.applyDeadband(-self.rightStick.getX(), 0.1)

        # Precision mode
        if self.leftStick.getRawButton(1) or self.rightStick.getRawButton(1):
            joystickX *= 0.5
            joystickY *= 0.5
            turnSpeed *= 0.5

        # Drive
        doRobotRelative = self.leftStick.getRawButton(3) or self.rightStick.getRawButton(3)
        if doRobotRelative:
            self.drivetrain.drive_robot_relative(-joystickY, -joystickX, turnSpeed)
        else:
            # Flip to red alliance if necessary
            xSpeed = -joystickY
            ySpeed = -joystickX
            if utils.isRedAlliance():
                xSpeed *= -1
                ySpeed *= -1
            self.drivetrain.drive_field_relative(xSpeed, ySpeed, turnSpeed * constants.kMaxTurnSpeed)

        if self.leftStick.getRawButtonPressed(8):
            self.drivetrain.reset_heading(utils.driverForwardAngle())

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
        self.elevatorandarm = ElevatorAndArm()
        self.hanger = Hanger()

        # Simulated subsystems
        self.simDrivetrain = SwerveDriveSim(self.drivetrain)

        # Auto and commands
        self.autoTimer = wpilib.Timer()
        self.trajectoryChooser = wpilib.SendableChooser()
        self.trajectory: choreo.trajectory.SwerveTrajectory | None = None
        self.trajectoryAlerts: List[Alert] = [] # keep things from getting garbage collected
        SmartDashboard.putData("Auto Trajectory", self.trajectoryChooser)

        self.autoEvents: Dict[str, Callable[[], None]] = {
            "c1": lambda: self.elevatorandarm.go_to_coral_preset(level=1),
            "c2": lambda: self.elevatorandarm.go_to_coral_preset(level=2),
            "c3": lambda: self.elevatorandarm.go_to_coral_preset(level=3),
            "c4": lambda: self.elevatorandarm.go_to_coral_preset(level=4),
        }

        # Alerts
        self.badTrajectoryAlert = Alert("Choreo path not found", Alert.AlertType.kError)
        self.noAutoAlert = Alert("No autonomous trajectory", Alert.AlertType.kWarning)
        self.noAutoSampleAlert = Alert("No sample for autonomous trajectory; stopping bot", Alert.AlertType.kWarning)
        self.scoringModeImproperValue = Alert("Variable scoringMode improper value (expected kCoralMode or kAlgaeMode)", Alert.AlertType.kError)
        self.redAllianceWhenTestingAlert = Alert("Do not use red alliance in testing! Controls will be inverted.", Alert.AlertType.kWarning)

        # Control state
        self.scoringMode = constants.kCoralMode
        self.previousAutoTime: float = 0

        # Controls telemetry
        self.scoringModeTopic = ntutil.getStringTopic("/Controls/ScoringMode")

        # Auto telemetry
        self.autoTimerTopic = ntutil.getFloatTopic("/Auto/Timer")
        self.autoTrajectoryTopic = ntutil.getStructArrayTopic("/Auto/Trajectory", Pose2d)
        self.autoChassisSpeedsTopic = ntutil.getStructTopic("/Auto/ChassisSpeeds", ChassisSpeeds)
        self.autoPoseTopic = ntutil.getStructTopic("/Auto/Pose", Pose2d)


    def robotInit(self):
        wpilib.DataLogManager.start()
        wpilib.DriverStation.startDataLog(wpilib.DataLogManager.getLog())
        # URCL.start()

        self.init()
        self.loadChoreoTrajectories()

        # Final initialization
        self.elevatorandarm.go_to_coral_preset(1)


    def robotPeriodic(self):
        self.drivetrain.periodic()
        self.elevatorandarm.periodic()
        self.hanger.periodic()
        if self.isSimulation():
            self.simDrivetrain.simulationPeriodic(0.02)

        if self.scoringMode == constants.kCoralMode:
            self.scoringModeTopic.set("Coral")
        elif self.scoringMode == constants.kAlgaeMode:
            self.scoringModeTopic.set("Algae")
        else:
            self.scoringModeTopic.set("???")
        
        self.redAllianceWhenTestingAlert.set(utils.isRedAlliance() and wpilib.DriverStation.isDSAttached() and not wpilib.DriverStation.isFMSAttached())


    def disabledInit(self):
        self.drivetrain.drive_field_relative(0, 0, 0)
        self.autoTrajectoryTopic.set([])


    def testPeriodic(self):
        # self.elevatorandarm.set_elevator_pid(utils.remap(self.leftStick.getRawAxis(2), (-1, 1), (3, 0)), 0, 0)
        # self.elevatorandarm.set_elevator_position(utils.remap(self.rightStick.getRawAxis(2), (-1, 1), (1, 0)))

        self.hanger.set_position(utils.remap(self.leftStick.getRawAxis(2), (-1, 1), (math.pi / 2, -math.pi / 2)))

        pass


    def autonomousInit(self) -> None:
        self.drivetrain.set_heading_controller_to_autonomous()
        self.elevatorandarm.set_arm_nudge_amount(0)

        self.trajectory = self.trajectoryChooser.getSelected()
        if self.trajectory:
            initial_pose = self.trajectory.get_initial_pose(utils.isRedAlliance())
            if initial_pose:
                self.drivetrain.reset_pose(initial_pose)
            self.autoTimer.restart()
            self.previousAutoTime = 0

            if utils.isRedAlliance():
                self.autoTrajectoryTopic.set([s.flipped().get_pose() for s in self.trajectory.samples])
            else:
                self.autoTrajectoryTopic.set([s.get_pose() for s in self.trajectory.samples])


    def autonomousPeriodic(self) -> None:
        currentAutoTime = self.autoTimer.get()
        self.autoTimerTopic.set(currentAutoTime)

        self.noAutoAlert.set(not self.trajectory)
        if self.trajectory:
            sample = self.trajectory.sample_at(self.autoTimer.get(), utils.isRedAlliance())
            self.noAutoSampleAlert.set(not sample)
            if sample:
                self.drivetrain.follow_choreo_trajectory(sample)
                self.autoChassisSpeedsTopic.set(sample.get_chassis_speeds())
                self.autoPoseTopic.set(sample.get_pose())

                for event in self.trajectory.events:
                    if self.previousAutoTime <= event.timestamp < currentAutoTime:
                        if event.event in self.autoEvents:
                            command = self.autoEvents[event.event] # Get the command from our map
                            command() # Run it!
                        else:
                            # We are ok to just log this here because we produce alerts during robotInit.
                            ntutil.log(f"Autonomous event not recognized; skipping: {event.event}")
            else:
                # We should always have samples. If not, stop the bot.
                self.drivetrain.drive_field_relative(0, 0, 0)
        
        self.previousAutoTime = currentAutoTime


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

        gamePieceSpeed = self.gamePad.getRightTriggerAxis()-self.gamePad.getLeftTriggerAxis()

        if self.gamePad.getRightBumper():
            self.scoringMode = constants.kCoralMode

        if self.gamePad.getLeftBumper():
            self.scoringMode = constants.kAlgaeMode

        if self.scoringMode == constants.kCoralMode:
            if self.gamePad.getAButton():
                #Set Elevator to L1/Handoff Height
                self.elevatorandarm.go_to_coral_preset(level=1)
            elif self.gamePad.getBButton():
                #Set Elevator to L2 Height
                self.elevatorandarm.go_to_coral_preset(level=2)
            elif self.gamePad.getXButton():
                #Set Elevator to L3 Height
                self.elevatorandarm.go_to_coral_preset(level=3)
            elif self.gamePad.getYButton():
                #Set Elevator to L4 Height
                self.elevatorandarm.go_to_coral_preset(level=4)

            coralSpeed = wpimath.applyDeadband(gamePieceSpeed, 0.2)
            self.elevatorandarm.move_coral_manually(coralSpeed)

        elif self.scoringMode == constants.kAlgaeMode:
            if self.gamePad.getAButton():
                self.elevatorandarm.go_to_algae_floor_preset()
            elif self.gamePad.getXButton() or self.gamePad.getBButton():
                self.elevatorandarm.go_to_algae_dereef_preset(high=False)
            elif self.gamePad.getYButton():
                self.elevatorandarm.go_to_algae_dereef_preset(high=True)

            self.elevatorandarm.move_algae(gamePieceSpeed)

        else:
            ntutil.logAlert(self.scoringModeImproperValue, self.scoringMode)

        self.elevatorandarm.set_arm_nudge_amount(wpimath.applyDeadband(-self.gamePad.getRightY(), 0.1))

        if self.gamePad.getStartButton():
            self.hanger.trigger_solenoid()
            print("Triggering solenoid")
        else:
            self.hanger.release_solenoid()


    # ================= SETUP METHODS =================

    def loadChoreoTrajectories(self):
        choreoDir = os.path.join(wpilib.getDeployDirectory(), "choreo")
        for idx, filename in enumerate(os.listdir(choreoDir)):
            if not os.path.isfile(os.path.join(choreoDir, filename)):
                continue
            if not filename.endswith(".traj"):
                continue
            autoName = filename.removesuffix(".traj")

            try:
                # Check the path we're trying to load actually exists before handing to Choreo
                if os.path.exists(os.path.join(wpilib.getDeployDirectory(), "choreo", autoName + ".traj")):
                    trajectory = choreo.load_swerve_trajectory(autoName)
                    if idx == 0:
                        self.trajectoryChooser.setDefaultOption(autoName, trajectory)
                    else:
                        self.trajectoryChooser.addOption(autoName, trajectory)

                    for event in trajectory.events:
                        if event.event not in self.autoEvents:
                            alert = Alert(f"Invalid event in \"{autoName}\": {event.event}", Alert.AlertType.kWarning)
                            alert.set(True)
                            self.trajectoryAlerts.append(alert)
                else:
                    ntutil.logAlert(self.badTrajectoryAlert, autoName)
            except ValueError as err:
                ntutil.logAlert(self.badTrajectoryAlert, err)

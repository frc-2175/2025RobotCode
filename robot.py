from typing import Callable
from coroutinecommand import RestartableCommand, commandify, sleep
from gentools import doneable
import ntutil
from wpimath.geometry import Translation2d
import wpilib
from wpilib import SmartDashboard, Alert
import math
import wpimath
import wpimath.geometry
from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds, SwerveModuleState
import wpimath.units
from commands2 import Command, CommandScheduler
import choreo
from wpilib import DriverStation
import os
import os.path
# from urcl import URCL
import constants
from subsystems.drivetrain import Drivetrain
from subsystems.sourceintake import SourceIntake
from subsystems.elevatorandarm import ElevatorAndArm
from subsystems.hanger import Hanger
import utils

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        wpilib.DataLogManager.start()
        wpilib.DriverStation.startDataLog(wpilib.DataLogManager.getLog())
        # URCL.start()

        # Joysticks and input
        self.leftStick = wpilib.Joystick(0)
        self.rightStick = wpilib.Joystick(1)
        self.gamePad = wpilib.XboxController(2)

        # Subsystems
        self.drivetrain = Drivetrain()
        self.sourceintake = SourceIntake()
        self.elevatorandarm = ElevatorAndArm()
        self.hanger = Hanger()

        # Auto and commands
        self.scheduler = CommandScheduler.getInstance()
        self.autoTimer = wpilib.Timer()
        self.initializeSchedulerLogging()

        # In order to play nice with PathPlanner's autos, which are Command-only,
        # we always store Commands in our auto chooser. However, since we want our
        # autos to be restartable, we use these wrappers to ensure that all our auto
        # options can be wrapped in RestartableCommand.
        self.autoChooser = wpilib.SendableChooser()
        self.trajectory = None
        choreoDir = os.path.join(wpilib.getDeployDirectory(), "choreo")
        autoFiles = [f for f in os.listdir(choreoDir) if os.path.isfile(os.path.join(choreoDir, f)) and f.endswith(".traj")]
        for filename in autoFiles:
            self.autoChooser.addOption(filename, filename)
        SmartDashboard.putData("Auto Trajectory", self.autoChooser)

        # Alerts
        self.badTrajectoryAlert = Alert("Choreo path not found", Alert.AlertType.kError)

        # Control state
        self.scoringMode = constants.kCoralMode
        self.algaeReverse = False

        # Controls telemetry
        self.scoringModeTopic = ntutil.getStringTopic("/Controls/ScoringMode")

        # Final initialization
        self.elevatorandarm.set_arm_position(constants.kElevatorL1, constants.kWristUprightAngle, constants.kCoralMode)

    def robotPeriodic(self):
        self.scheduler.run()

        self.drivetrain.periodic()
        self.sourceintake.periodic()
        self.elevatorandarm.periodic()
        self.hanger.periodic()

        if self.scoringMode == constants.kCoralMode:
            self.scoringModeTopic.set("Coral")
        elif self.scoringMode == constants.kAlgaeMode:
            self.scoringModeTopic.set("Algae")
        else:
            self.scoringModeTopic.set("???")


    def testPeriodic(self):
        # self.elevatorandarm.set_elevator_pid(utils.remap(self.leftStick.getRawAxis(2), (-1, 1), (3, 0)), 0, 0)
        # self.elevatorandarm.set_elevator_position(utils.remap(self.rightStick.getRawAxis(2), (-1, 1), (1, 0)))

        self.hanger.set_position(utils.remap(self.leftStick.getRawAxis(2), (-1, 1), (math.pi / 2, -math.pi / 2)))

        pass


    def autonomousInit(self) -> None:
        self.scheduler.cancelAll()

        try:
            selected_trajectory = self.autoChooser.getSelected()
            # Check the path we're trying to load actually exists before handing to Choreo
            if selected_trajectory and os.path.exists(os.path.join(wpilib.getDeployDirectory(), "choreo", selected_trajectory + ".traj")):
                self.trajectory = choreo.load_swerve_trajectory(self.autoChooser.getSelected())
            else:
                ntutil.log(f"Choreo path not found: {selected_trajectory}")
        except ValueError:
            self.trajectory = None
        self.badTrajectoryAlert.set(self.trajectory is None)

        if self.trajectory:
            initial_pose = self.trajectory.get_initial_pose(wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed)
            if initial_pose:
                self.drivetrain.reset_pose(initial_pose)
            self.autoTimer.restart()


    def autonomousPeriodic(self) -> None:
        if self.trajectory:
            sample = self.trajectory.sample_at(self.autoTimer.get(), (wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed))
            if sample:
                self.drivetrain.follow_choreo_trajectory(sample)


    def teleopPeriodic(self) -> None:
        xSpeed = constants.kMaxSpeed * wpimath.applyDeadband(-self.leftStick.getY(), 0.1)
        ySpeed = constants.kMaxSpeed * wpimath.applyDeadband(-self.leftStick.getX(), 0.1)
        turnSpeed = wpimath.applyDeadband(-self.rightStick.getX(), 0.1)
        self.drivetrain.drive(xSpeed, ySpeed, turnSpeed * constants.kMaxTurnSpeed)
        if self.leftStick.getRawButtonPressed(8):
            self.drivetrain.reset_heading()

        gamePieceSpeed = self.gamePad.getRightTriggerAxis()-self.gamePad.getLeftTriggerAxis()

        if self.gamePad.getRightBumper():
            self.scoringMode = constants.kCoralMode

        if self.gamePad.getLeftBumper():
            self.scoringMode = constants.kAlgaeMode
        
        if self.scoringMode == constants.kCoralMode:
            if self.gamePad.getAButton():
                #Set Elevator to L1/Handoff Height
                self.elevatorandarm.set_arm_position(constants.kElevatorL1, constants.kWristUprightAngle, constants.kCoralMode)
            elif self.gamePad.getBButton():
                #Set Elevator to L2 Height
                self.elevatorandarm.set_arm_position(constants.kElevatorL2, constants.kWristCoralScoreAngle, constants.kCoralMode)
            elif self.gamePad.getXButton():
                #Set Elevator to L3 Height
                self.elevatorandarm.set_arm_position(constants.kElevatorL3, constants.kWristCoralScoreAngle, constants.kCoralMode)
            elif self.gamePad.getYButton():
                #Set Elevator to L4 Height
                self.elevatorandarm.set_arm_position(constants.kElevatorL4, constants.kWristHighCoralScoreAngle, constants.kCoralMode)

            coralSpeed = wpimath.applyDeadband(gamePieceSpeed, 0.2)
            self.elevatorandarm.move_coral(coralSpeed)

        elif self.scoringMode == constants.kAlgaeMode:
            if self.gamePad.getAButton():
                self.elevatorandarm.set_arm_position(constants.kElevatorAlgaeGround, constants.kWristAlgaeGround, constants.kAlgaeMode)
                self.algaeReverse = False
            elif self.gamePad.getXButton() or self.gamePad.getBButton():
                self.algaeReverse = False
                self.elevatorandarm.set_arm_position(constants.kElevatorAlgaeLow, constants.kWristAlgaeDereef, constants.kAlgaeMode)
            elif self.gamePad.getYButton():
                self.elevatorandarm.set_arm_position(constants.kElevatorAlgaeHigh, constants.kWristAlgaeDereef, constants.kAlgaeMode)
                self.algaeReverse = True
            
            if self.algaeReverse == True:
                self.elevatorandarm.move_algae(gamePieceSpeed)
            elif self.algaeReverse == False:
                self.elevatorandarm.move_algae(-gamePieceSpeed)
            else:
                print(f"Variable algaeReverse improper value: {self.algaeReverse}; expected True or False")

        else:
            print(f"Variable scoringMode improper value: {self.scoringMode}; expected Coral or Algae")

    # ================= UTILITY METHODS =================

    def initializeSchedulerLogging(self):
        self.scheduler.onCommandInitialize(lambda command: ntutil.log(f"{command.getName()}: Initialized"))
        self.scheduler.onCommandInterrupt(lambda command: ntutil.log(f"{command.getName()}: Interrupted"))
        self.scheduler.onCommandFinish(lambda command: ntutil.log(f"{command.getName()}: Finished"))

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
# from urcl import URCL

from subsystems.drivetrain import Drivetrain
from subsystems.sourceintake import SourceIntake
from subsystems.elevatorandarm import ElevatorAndArm
import utils

class MyRobot(wpilib.TimedRobot):
    leftStick = wpilib.Joystick(0)
    rightStick = wpilib.Joystick(1)
    gamePad = wpilib.XboxController(2)

    drivetrain = Drivetrain()
    sourceintake = SourceIntake()
    elevatorandarm = ElevatorAndArm()

    scoringMode = "Coral"
    algaeReverse = False

    def robotInit(self):
        # wpilib.DataLogManager.start()
        # URCL.start()
        pass
    
    def robotPeriodic(self):
        self.drivetrain.periodic()
        self.sourceintake.periodic()
        self.elevatorandarm.periodic()
    
    def testPeriodic(self):
        self.elevatorandarm.set_elevator_pid(utils.remap(self.leftStick.getRawAxis(2), (-1, 1), (3, 0)), 0, 0)
        self.elevatorandarm.set_elevator_position(utils.remap(self.rightStick.getRawAxis(2), (-1, 1), (1, 0)))
        pass


    def teleopPeriodic(self) -> None:
        xSpeed = wpimath.applyDeadband(-self.leftStick.getY(), 0.2)
        ySpeed = wpimath.applyDeadband(-self.leftStick.getX(), 0.2)
        turnSpeed = wpimath.applyDeadband(-self.rightStick.getX(), 0.2)
        self.drivetrain.drive(xSpeed, ySpeed, turnSpeed * constants.kMaxTurnSpeed)
        if self.leftStick.getRawButtonPressed(8):
            self.drivetrain.reset_heading()

        gamePieceSpeed = self.gamePad.getRightTriggerAxis()-self.gamePad.getLeftTriggerAxis()

        if self.gamePad.getRightBumper():
            self.scoringMode = "Coral"

        if self.gamePad.getLeftBumper():
            self.scoringMode = "Algae"
        
        if self.scoringMode == "Coral":
            if self.gamePad.getAButton():
                self.elevatorandarm.set_wrist_position(constants.kWristUprightAngle)
                #TODO: Set Elevator to L1/Handoff Height
            elif self.gamePad.getBButton():
                self.elevatorandarm.set_wrist_position(constants.kWristCoralScoreAngle)
                #TODO: Set Elevator to L2 Height
            elif self.gamePad.getXButton():
                self.elevatorandarm.set_wrist_position(constants.kWristCoralScoreAngle)
                #TODO: Set Elevator to L3 Height
            elif self.gamePad.getYButton():
                self.elevatorandarm.set_wrist_position(constants.kWristCoralScoreAngle)
                #TODO" Set Elevator to L4 Height

            coralSpeed = wpimath.applyDeadband(gamePieceSpeed, 0.2)
            self.elevatorandarm.move_coral(coralSpeed)

            if self.elevatorandarm.get_elevator_position() <= constants.kSourceIntakeElevatorMaxHeight and self.elevatorandarm.get_wrist_position() >= constants.kSourceIntakeWristMinAngle:
                self.sourceintake.run_intake(coralSpeed)
            else:
                self.sourceintake.run_intake(0)

        elif self.scoringMode == "Algae":
            if self.gamePad.getAButton():
                self.elevatorandarm.set_wrist_position(constants.kWristAlgaeDereef)
                algaeReverse = False
                #TODO: Set Elevator to Algae Ground Height
            elif self.gamePad.getXButton or self.gamePad.getBButton():
                self.elevatorandarm.set_wrist_position(constants.kWristAlgaeDereef)
                algaeReverse = False
                #TODO Set Elevator To Algae Low DeReef Height
            elif self.gamePad.getYButton():
                self.elevatorandarm.set_wrist_position(constants.kWristAlgaeDereef)
                algaeReverse = True
                #TODO Set Elevator To Algae High DeReef Height
            
            if algaeReverse == True:
                self.elevatorandarm.move_algae(gamePieceSpeed)
            elif algaeReverse == False:
                self.elevatorandarm.move_algae(-gamePieceSpeed)
            else:
                print(f"Variable algaeReverse improper value: {algaeReverse}; expected True or False")
            
            self.sourceintake.run_intake(0)
        else:
            print(f"Variable scoringMode improper value: {scoringMode}; expected Coral or Algae")